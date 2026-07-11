// Cl-4 Phase-1 A3 evidence probe — sensor census (MEASUREMENT ONLY, research
// tooling; no shipped tracker behaviour). See
// docs/superpowers/plans/2026-07-11-cl4-phase1-a3-probe-ticket.md and the
// write-up docs/baselines/2026-07-11_cl4_phase1_a3_probe.md.
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
  Scenario scen = run->generate(0);
  if (scen.truth.empty()) {
    std::cerr << "scenario has no truth (fixture absent?): " << label << "\n";
    return 2;
  }
  if (!scen.datum.has_value()) {
    std::cerr << "scenario has no datum; cannot build coastline\n";
    return 2;
  }

  // Build the coastline model EXACTLY as Sweep.cpp does for the land configs.
  std::unique_ptr<CoastlineModel> land;
  {
    std::ifstream probe(coast_path);
    if (!probe.good()) {
      std::cerr << "coastline geojson not readable: " << coast_path << "\n";
      return 2;
    }
    auto geom = loadCoastlineGeoJson(coast_path, CoastlinePriorParams{});
    land = std::make_unique<CoastlineModel>(std::move(geom), *scen.datum);
  }
  auto inBand = [&](const Eigen::Vector2d& p) {
    return land->clutterPrior(p) > 0.0;  // coverage_land no-birth zone
  };

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
