// Cl-4 Phase-1 census — sensor/chain census (MEASUREMENT ONLY, research
// tooling; no shipped tracker behaviour).
//   --mode evidence (default): Phase-1a A3 sensor-evidence census
//     (docs/baselines/2026-07-10_harbor_truthsort_reconcile.md family;
//      docs/baselines/2026-07-11_cl4_phase1_a3_probe.md).
//   --mode chain / --mode target: Phase-1b conditional-coverage-floor probe —
//     re-detection chain statistics + per-target motion/latency
//     (docs/baselines/2026-07-11_cl4_phase1b_floor_probe.md, ticket
//      docs/superpowers/plans/2026-07-11-cl4-phase1b-coverage-floor-probe-ticket.md).
//   --mode chain --motion 1 [--gate --vcap]: Phase-1c Tier-A motion-model
//     (CV-consistent, teleport-rejecting) chainer for honest displacement +
//     smoothness (docs/baselines/2026-07-11_cl4_phase1c_smoothness_probe.md).
//     Tier B = the real PMBM via navtracker_bench_baseline --export-states-dir.
//
// For a replay scenario, classifies every truth sample as in-band (inside the
// coverage_land <50 m no-birth zone, using the tracker's OWN CoastlineModel /
// clutterPrior>0 criterion) and, per in-band scan, tests whether a clutter-free
// sensor produced a measurement the tracker would associate to that target:
//   - Position2D (lidar / radar / AIS): within R metres of the truth position;
//   - Bearing2D  (EO / IR camera): a bearing ray within `bearing_tol` rad of
//     the truth line-of-sight, in front of the sensor.
// R / bearing_tol / dt_window are fixed up front (defaults below) to the
// tracker's association gate (15 m / 0.15 rad, ReplayScenarioRun.cpp:321) so
// "evidence present" == "a measurement the tracker would gate to this target".
//
// Emits a per-target summary to stdout and (optional) a per-scan CSV.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimScenarioRun.hpp"
#include "adapters/land/GeoJsonCoastline.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/land/CoastlineModel.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {

std::string arg(int argc, char** argv, const std::string& key,
                const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i)
    if (key == argv[i]) return argv[i + 1];
  return def;
}

// Angular residual (rad) between a sensor bearing ray and the true LOS to p.
// Returns >pi (invalid) when the target is behind the sensor.
double bearingResidual(const Eigen::Vector2d& origin, double theta_enu,
                       const Eigen::Vector2d& p) {
  const Eigen::Vector2d d(std::cos(theta_enu), std::sin(theta_enu));
  const Eigen::Vector2d v = p - origin;
  if (v.dot(d) <= 0.0) return 10.0;  // behind the sensor
  const double cross = v.x() * d.y() - v.y() * d.x();
  return std::atan2(std::abs(cross), v.dot(d));
}

}  // namespace

int main(int argc, char** argv) {
  const std::string label = arg(argc, argv, "--scenario", "autoferry_scenario16");
  const double R = std::stod(arg(argc, argv, "--radius", "15.0"));
  const double btol = std::stod(arg(argc, argv, "--bearing-tol", "0.15"));
  const double dt = std::stod(arg(argc, argv, "--dt", "0.5"));
  const int min_scans = std::stoi(arg(argc, argv, "--min-scans", "3"));
  const std::string csv = arg(argc, argv, "--csv", "");

  // Locate the scenario across every replay factory (philos/haxr live in
  // defaultReplayScenarios; autoferry env-1/env-2 in defaultAutoferryScenarios).
  std::unique_ptr<ScenarioRun> run;
  std::string coast_path;
  {
    std::vector<std::unique_ptr<ScenarioRun>> all;
    for (auto& s : defaultReplayScenarios()) all.push_back(std::move(s));
    for (auto& s : defaultAutoferryScenarios()) all.push_back(std::move(s));
    for (auto& s : defaultAutoferryScenariosAnchored())
      all.push_back(std::move(s));
    for (auto& s : defaultSimScenarios()) all.push_back(std::move(s));
    for (auto& s : all) {
      if (s->descriptor().label == label) {
        coast_path = s->descriptor().coastline_geojson_path;
        run = std::move(s);
        break;
      }
    }
  }
  if (!run) {
    std::cerr << "scenario not found or fixture absent: " << label << "\n";
    return 2;
  }
  const std::uint64_t seed =
      static_cast<std::uint64_t>(std::stoll(arg(argc, argv, "--seed", "0")));
  Scenario scen = run->generate(seed);
  if (scen.truth.empty()) {
    std::cerr << "scenario has no truth (fixture absent?): " << label << "\n";
    return 2;
  }
  if (!scen.datum.has_value()) {
    std::cerr << "scenario has no datum; cannot build coastline\n";
    return 2;
  }

  // Build the coastline model EXACTLY as Sweep.cpp does for the land configs.
  // Optional: harbor_complete_truth is chart-free by design (no coastline path),
  // so the land ramp is inert there and inBand() is false everywhere — matching
  // the tracker, whose land model is never wired without a coastline.
  std::unique_ptr<CoastlineModel> land;
  if (!coast_path.empty()) {
    std::ifstream probe(coast_path);
    if (probe.good()) {
      auto geom = loadCoastlineGeoJson(coast_path, CoastlinePriorParams{});
      land = std::make_unique<CoastlineModel>(std::move(geom), *scen.datum);
    }
  }
  auto inBand = [&](const Eigen::Vector2d& p) {
    return land && land->clutterPrior(p) > 0.0;  // coverage_land no-birth zone
  };

  // ---- Phase 1b: re-detection chain census (conditional coverage floor) ----
  const std::string mode = arg(argc, argv, "--mode", "evidence");

  // Per-truth-target motion + re-detection availability, with the latency
  // constraint baked in. K1 asks, per target: does it re-detect on enough
  // scans AND move ≥ D within a ≤ lat_s window? Truth motion is authoritative
  // (no chain-labelling ambiguity); a birth-candidate detection within R of the
  // truth position counts as a re-detection on that scan.
  if (mode == "target") {
    const double R2 = std::stod(arg(argc, argv, "--radius", "25.0"));
    const double lat_s = std::stod(arg(argc, argv, "--latency", "30.0"));
    // group truth samples by id, in time order
    std::map<std::uint64_t, std::vector<std::pair<double, Eigen::Vector2d>>> tr;
    for (const auto& ts : scen.truth)
      tr[ts.truth_id].push_back({ts.time.seconds(), ts.position});
    // birth-candidate position measurements (radar+lidar+ais), time-sorted
    std::vector<std::pair<double, Eigen::Vector2d>> pos;
    for (const auto& m : scen.measurements) {
      if (m.value.size() < 2) continue;
      if (m.sensor == SensorKind::ArpaTtm || m.sensor == SensorKind::Lidar ||
          m.sensor == SensorKind::Ais)
        pos.push_back({m.time.seconds(), m.value.head<2>()});
    }
    std::cout << "=== TARGET motion+redetection: " << label
              << "  (R=" << R2 << "m, latency window=" << lat_s << "s) ===\n";
    std::cout << "id | scans | in_band | redetect_scans | truth_net_disp_m | "
                 "max_disp_in_" << (int)lat_s << "s_m | max_speed_mps\n";
    auto byTime = [](const std::pair<double, Eigen::Vector2d>& a,
                     const std::pair<double, Eigen::Vector2d>& b) {
      return a.first < b.first;
    };
    for (auto& [id, samps] : tr) {
      std::sort(samps.begin(), samps.end(), byTime);
      int inband = 0, redet = 0;
      for (auto& [t, p] : samps) {
        if (!inBand(p)) continue;  // K1 concerns the in-band (suppressed) part
        inband++;
        for (auto& [mt, mp] : pos)
          if (std::abs(mt - t) <= 0.5 && (mp - p).norm() <= R2) { redet++; break; }
      }
      // max truth net displacement over any window of duration ≤ lat_s
      double max_win_disp = 0.0, max_win_speed = 0.0;
      for (std::size_t i = 0; i < samps.size(); ++i) {
        for (std::size_t j = i + 1; j < samps.size(); ++j) {
          const double dtw = samps[j].first - samps[i].first;
          if (dtw > lat_s) break;
          const double dd = (samps[j].second - samps[i].second).norm();
          if (dd > max_win_disp) max_win_disp = dd;
          if (dtw > 1e-6) max_win_speed = std::max(max_win_speed, dd / dtw);
        }
      }
      const double net =
          samps.size() >= 2
              ? (samps.back().second - samps.front().second).norm()
              : 0.0;
      // per-consecutive-step truth speed (median / p95) — the "smoothness"
      // scale a vessel moves at, vs a structure-walk's teleport jumps.
      std::vector<double> step;
      for (std::size_t i = 1; i < samps.size(); ++i) {
        const double sdt = samps[i].first - samps[i - 1].first;
        if (sdt > 1e-3)
          step.push_back((samps[i].second - samps[i - 1].second).norm() / sdt);
      }
      std::sort(step.begin(), step.end());
      const double smed = step.empty() ? 0.0 : step[step.size() / 2];
      const double sp95 = step.empty() ? 0.0 : step[(int)(0.95 * step.size())];
      std::printf(
          "%2llu | %5zu | %7d | %14d | %16.1f | %18.1f | %.2f | step_med=%.2f "
          "p95=%.2f\n",
          (unsigned long long)id, samps.size(), inband, redet, net,
          max_win_disp, max_win_speed, smed, sp95);
    }
    return 0;
  }
  if (mode == "chain") {
    const double r_chain = std::stod(arg(argc, argv, "--chain-radius", "25.0"));
    // Absolute re-visit tolerance (s): re-detections of one object within this
    // gap AND r_chain chain up. Physically = the sensor's worst revisit period
    // (philos rotating radar re-hits a fixed structure every few s; autoferry
    // fused stream is denser). Default 5 s covers both; stated up front.
    const double max_gap_s = std::stod(arg(argc, argv, "--max-gap-s", "5.0"));
    const std::string kind_s = arg(argc, argv, "--kind", "radar");
    const bool inband_only = arg(argc, argv, "--inband-only", "0") == "1";
    const std::string ccsv = arg(argc, argv, "--csv", "");
    // Tier A (--motion 1): motion-model-consistent chaining. A chain carries a
    // CV velocity estimate; a detection extends it only if it lands within an
    // innovation gate of the CV-PREDICTED position (last + vel*dt), not merely
    // near the last point. This rejects the teleport-walk that the plain-NN
    // (Tier-0/Phase-1b) chainer allowed along the pier's 10 m-spaced points: a
    // chain that accepts one 10 m jump acquires ~20 m/s velocity, predicts 10 m
    // further, and the next static pier return then falls OUTSIDE the gate — so
    // a static structure cannot sustain a walk, while a real CV vessel does.
    const bool motion = arg(argc, argv, "--motion", "0") == "1";
    // Innovation gate (m) around the CV prediction — the tracker's own ~15 m
    // position association gate scale (ReplayScenarioRun.cpp:321).
    const double gate_m = std::stod(arg(argc, argv, "--gate", "15.0"));
    // Physical velocity cap (m/s): a step implying speed > vcap is rejected as
    // kinematically implausible (harbour craft ≪ this) — bounds the CV estimate
    // so a chain cannot acquire runaway velocity and teleport to distant points.
    const double vcap = std::stod(arg(argc, argv, "--vcap", "20.0"));
    auto kindMatch = [&](const Measurement& m) {
      if (kind_s == "radar") return m.sensor == SensorKind::ArpaTtm;
      if (kind_s == "lidar") return m.sensor == SensorKind::Lidar;
      if (kind_s == "ais") return m.sensor == SensorKind::Ais;
      // "pos" = any birth-capable Position2D sensor (radar+lidar+ais union)
      return m.sensor == SensorKind::ArpaTtm || m.sensor == SensorKind::Lidar ||
             m.sensor == SensorKind::Ais;
    };

    // Collect the birth-candidate population: Position2D measurements of the
    // chosen sensor(s), time-sorted. (These are what would seed a birth; under
    // coverage_land the in-band ones are suppressed.)
    struct P { double t; Eigen::Vector2d xy; };
    std::vector<P> pts;
    for (const auto& m : scen.measurements) {
      if (!kindMatch(m) || m.value.size() < 2) continue;
      if (inband_only && !inBand(m.value.head<2>())) continue;
      pts.push_back({m.time.seconds(), m.value.head<2>()});
    }
    std::sort(pts.begin(), pts.end(),
              [](const P& a, const P& b) { return a.t < b.t; });
    // median inter-sample dt (for reporting the scan cadence).
    // Greedy single-link NN chaining across time: a point extends the nearest
    // active chain whose last point is within r_chain and whose time gap is in
    // (0, max_gap*scan]; else it starts a new chain. Gaps up to max_gap scans
    // are tolerated (missed detections). Chain = re-detections of one object.
    struct Chain {
      double t0, t1;
      Eigen::Vector2d p0, plast;
      int n;
      double pathlen;
      double max_step;  // largest single-step speed (m/s) — flags NN jumps
      Eigen::Vector2d vel{Eigen::Vector2d::Zero()};  // CV estimate (motion mode)
      bool have_vel{false};
    };
    std::vector<Chain> chains;
    std::vector<int> active;  // indices into chains, last updated recently
    // scan interval estimate = median positive dt between consecutive samples
    std::vector<double> gaps;
    for (std::size_t i = 1; i < pts.size(); ++i) {
      const double d = pts[i].t - pts[i - 1].t;
      if (d > 1e-6) gaps.push_back(d);
    }
    double scan = 1.0;
    if (!gaps.empty()) {
      std::sort(gaps.begin(), gaps.end());
      scan = gaps[gaps.size() / 2];
    }
    const double gap_win = max_gap_s;  // absolute revisit tolerance
    for (const auto& p : pts) {
      int best = -1;
      double best_d = motion ? gate_m : r_chain;
      for (int ci : active) {
        Chain& c = chains[ci];
        const double dt_c = p.t - c.t1;
        if (dt_c <= 1e-6 || dt_c > gap_win) continue;
        // Tier A: gate on the CV-predicted position; Tier 0: on the last point.
        // reject kinematically implausible steps (motion mode): a step faster
        // than vcap cannot be this chain's re-detection.
        if (motion && (p.xy - c.plast).norm() / dt_c > vcap) continue;
        const Eigen::Vector2d ref =
            motion && c.have_vel ? (c.plast + c.vel * dt_c) : c.plast;
        const double dd = (p.xy - ref).norm();
        if (dd <= best_d) { best_d = dd; best = ci; }
      }
      if (best >= 0) {
        Chain& c = chains[best];
        const double step = (p.xy - c.plast).norm();
        const double step_dt = p.t - c.t1;
        if (step_dt > 1e-6) {
          c.max_step = std::max(c.max_step, step / step_dt);
          const Eigen::Vector2d v = (p.xy - c.plast) / step_dt;
          c.vel = c.have_vel ? (0.5 * c.vel + 0.5 * v) : v;  // light smoothing
          c.have_vel = true;
        }
        c.pathlen += step;
        c.plast = p.xy;
        c.t1 = p.t;
        c.n++;
      } else {
        Chain nc;
        nc.t0 = nc.t1 = p.t;
        nc.p0 = nc.plast = p.xy;
        nc.n = 1;
        nc.pathlen = 0.0;
        nc.max_step = 0.0;
        chains.push_back(nc);
      }
      // rebuild active list = chains whose t1 is within gap_win of current time
      active.clear();
      for (int ci = 0; ci < (int)chains.size(); ++ci)
        if (p.t - chains[ci].t1 <= gap_win) active.push_back(ci);
    }

    // Label each chain by proximity to a truth trajectory: "vessel" if its
    // start point is within r_chain of any truth sample near t0.
    auto nearTruth = [&](double t, const Eigen::Vector2d& p) {
      for (const auto& ts : scen.truth) {
        if (std::abs(ts.time.seconds() - t) > gap_win) continue;
        if ((ts.position - p).norm() <= r_chain) return (long long)ts.truth_id;
      }
      return -1LL;
    };

    std::ofstream out;
    if (!ccsv.empty()) {
      out.open(ccsv);
      out << "chain,t0_s,n_scans,duration_s,net_disp_m,path_m,mean_speed_mps,max_step_mps,p0x,p0y,label_truth_id\n";
    }
    std::cout << "=== CHAIN census: " << label << "  kind=" << kind_s
              << (inband_only ? " (in-band only)" : "") << "  scan~" << scan
              << "s  r_chain=" << r_chain << "m  max_gap_s=" << max_gap_s
              << "s ===\n";
    std::cout << "population points=" << pts.size()
              << "  chains=" << chains.size() << "\n";
    // Summary: bucket chains by whether they track a truth vessel.
    int n_vessel = 0, n_clutter = 0;
    for (std::size_t i = 0; i < chains.size(); ++i) {
      const Chain& c = chains[i];
      const double dur = c.t1 - c.t0;
      const double disp = (c.plast - c.p0).norm();
      const double spd = dur > 1e-6 ? disp / dur : 0.0;
      const long long lab = nearTruth(c.t0, c.p0);
      if (lab >= 0) n_vessel++; else n_clutter++;
      if (out.is_open())
        out << i << "," << c.t0 << "," << c.n << "," << dur << "," << disp << ","
            << c.pathlen << "," << spd << "," << c.max_step << "," << c.p0.x()
            << "," << c.p0.y() << "," << lab << "\n";
    }
    std::cout << "chains labelled vessel(near-truth)=" << n_vessel
              << "  clutter/structure=" << n_clutter << "\n";
    // Distribution helper over a predicate-selected chain set.
    auto dist = [&](const char* name, bool vessel) {
      std::vector<int> len;
      std::vector<double> dur, disp, spd;
      for (const Chain& c : chains) {
        const bool v = nearTruth(c.t0, c.p0) >= 0;
        if (v != vessel) continue;
        const double d = c.t1 - c.t0;
        const double dp = (c.plast - c.p0).norm();
        len.push_back(c.n);
        dur.push_back(d);
        disp.push_back(dp);
        spd.push_back(d > 1e-6 ? dp / d : 0.0);
      }
      if (len.empty()) { std::printf("  %-16s (none)\n", name); return; }
      auto med = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
      };
      auto mx = [](const std::vector<double>& v) {
        return *std::max_element(v.begin(), v.end());
      };
      std::vector<double> lend(len.begin(), len.end());
      std::printf(
          "  %-16s count=%zu  len[med=%.0f max=%.0f]  dur_s[med=%.1f max=%.1f]  "
          "disp_m[med=%.1f max=%.1f]  speed_mps[med=%.2f max=%.2f]\n",
          name, len.size(), med(lend), mx(lend), med(dur), mx(dur), med(disp),
          mx(disp), med(spd), mx(spd));
    };
    dist("vessel", true);
    dist("clutter/struct", false);
    return 0;
  }

  // Index measurements by sensor role for the per-scan evidence search.
  struct Meas {
    double t;
    Eigen::Vector2d xy;         // Position2D value, or sensor origin for bearing
    double theta;               // Bearing2D value (ENU), else 0
    bool is_bearing;
  };
  std::vector<Meas> lidar, radar, ais, eo, ir;
  for (const auto& m : scen.measurements) {
    const double t = m.time.seconds();
    if (m.model == MeasurementModel::Bearing2D && m.sensor == SensorKind::EoIr) {
      Meas mm{t, m.sensor_position_enu, m.value(0), true};
      if (m.source_id == "autoferry_ir") ir.push_back(mm);
      else eo.push_back(mm);  // autoferry_eo / philos_cam*
    } else if (m.value.size() >= 2) {
      Meas mm{t, m.value.head<2>(), 0.0, false};
      if (m.sensor == SensorKind::Lidar) lidar.push_back(mm);
      else if (m.sensor == SensorKind::Ais) ais.push_back(mm);
      else if (m.sensor == SensorKind::ArpaTtm) radar.push_back(mm);
    }
  }
  auto posHit = [&](const std::vector<Meas>& v, double t,
                    const Eigen::Vector2d& p) {
    for (const auto& m : v)
      if (std::abs(m.t - t) <= dt && (m.xy - p).norm() <= R) return true;
    return false;
  };
  auto bearHit = [&](const std::vector<Meas>& v, double t,
                     const Eigen::Vector2d& p) {
    for (const auto& m : v)
      if (std::abs(m.t - t) <= dt && bearingResidual(m.xy, m.theta, p) <= btol)
        return true;
    return false;
  };

  // Per-target tallies.
  std::map<std::uint64_t, int> n_scans, n_inband, ev_lidar, ev_radar, ev_ais,
      ev_eo, ev_ir, ev_cam, ev_any_cf, ev_two_sensor;
  std::ofstream out;
  if (!csv.empty()) {
    out.open(csv);
    out << "truth_id,time_s,in_band,lidar,radar,ais,eo,ir\n";
  }
  for (const auto& ts : scen.truth) {
    const auto id = ts.truth_id;
    const auto& p = ts.position;
    const double t = ts.time.seconds();
    n_scans[id]++;
    const bool band = inBand(p);
    bool l = false, r = false, a = false, e = false, i2 = false;
    if (band) {
      n_inband[id]++;
      l = posHit(lidar, t, p);
      r = posHit(radar, t, p);
      a = posHit(ais, t, p);
      e = bearHit(eo, t, p);
      i2 = bearHit(ir, t, p);
      if (l) ev_lidar[id]++;
      if (r) ev_radar[id]++;
      if (a) ev_ais[id]++;
      if (e) ev_eo[id]++;
      if (i2) ev_ir[id]++;
      const bool cam = e || i2;
      if (cam) ev_cam[id]++;
      // clutter-free = non-radar (lidar/camera/AIS)
      if (l || a || cam) ev_any_cf[id]++;
      const int kinds = (int)l + (int)r + (int)a + (int)e + (int)i2;
      if (kinds >= 2) ev_two_sensor[id]++;
    }
    if (out.is_open())
      out << id << "," << t << "," << band << "," << l << "," << r << "," << a
          << "," << e << "," << i2 << "\n";
  }

  // --- Guard-density census: of the in-band measurements from each clutter-
  // free sensor, how many are OFF-target (not within the gate of any truth) —
  // i.e. clutter/structure a sensor-typed exemption would ALSO re-admit. This
  // is the ADR's "cameras see piers/structure too" worry, measured on the one
  // workload (env-2) that actually carries camera+lidar.
  auto nearAnyTruth = [&](const Meas& m) {
    for (const auto& ts : scen.truth) {
      if (std::abs(ts.time.seconds() - m.t) > dt) continue;
      if (m.is_bearing) {
        if (bearingResidual(m.xy, m.theta, ts.position) <= btol) return true;
      } else if ((m.xy - ts.position).norm() <= R) {
        return true;
      }
    }
    return false;
  };
  auto guard = [&](const char* name, const std::vector<Meas>& v) {
    int in = 0, on = 0;
    for (const auto& m : v) {
      // A bearing ray has no single ENU location; the env-2 channel is entirely
      // in-band (targets 100% in-band), so treat every camera return as a
      // near-shore candidate. Position sensors use their own in-band test.
      const bool band = m.is_bearing ? true : inBand(m.xy);
      if (!band) continue;
      in++;
      if (nearAnyTruth(m)) on++;
    }
    std::printf("  guard %-6s in-band=%5d  on-target=%5d  OFF-target(clutter)=%5d  (%.0f%% off)\n",
                name, in, on, in - on, in ? 100.0 * (in - on) / in : 0.0);
  };
  std::cout << "--- guard-density (in-band measurements off-target = clutter an exemption re-admits) ---\n";
  guard("lidar", lidar);
  guard("radar", radar);
  guard("eo", eo);
  guard("ir", ir);
  guard("ais", ais);

  std::cout << "=== A3 evidence census: " << label
            << "  (R=" << R << "m, btol=" << btol << "rad, dt=" << dt
            << "s, revivable>=" << min_scans << " in-band evidence scans) ===\n";
  std::cout << "id | scans | in_band | ev_lidar ev_radar ev_ais ev_eo ev_ir | "
               "ev_cam ev_cf(lid/cam/ais) ev_2sensor | "
               "revive(i:AIS ii:AIS+cam+lid iii:2sensor)\n";
  for (const auto& [id, ns] : n_scans) {
    const int ib = n_inband[id];
    auto rev = [&](int c) { return c >= min_scans ? "Y" : "n"; };
    const int var_i = ev_ais[id];
    const int var_ii = ev_any_cf[id];
    const int var_iii = ev_two_sensor[id];
    std::printf(
        "%2llu | %5d | %7d | %8d %8d %6d %5d %5d | %6d %17d %10d | "
        "%s %s %s\n",
        (unsigned long long)id, ns, ib, ev_lidar[id], ev_radar[id], ev_ais[id],
        ev_eo[id], ev_ir[id], ev_cam[id], var_ii, var_iii, rev(var_i),
        rev(var_ii), rev(var_iii));
  }
  return 0;
}
