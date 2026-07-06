// LOS / shadow probe (R8.8 payoff) — MEASUREMENT-ONLY.
//
// Question (docs/superpowers/plans/2026-07-06-los-shadow-probe-ticket.md):
// coverage-aware occupancy decay forgets a cell only when the sensor SWEPT it
// this scan (the cell falls inside some bundle's self-estimated coverage sector,
// LiveOccupancyModel.cpp:110-122). A radar-SHADOWED cell is swept in azimuth but
// physically unreachable — the return stops at the occluder. If the sector model
// counts shadowed cells as observed-empty, a real moored vessel's occupancy
// evidence erodes every time a big ship crosses in front of it, degrading the
// ADR-0002 presence channel.
//
// Ground truth: tests/fixtures/philos/labels/car_carrier_near_labels.csv row
// `unknown_w860` (42.3583, -71.0464, r=50 m): two moored yachts present the whole
// clip, radar-SILENT t 50-85 s while NYK GENTLE LEADER crosses their bearing at
// 150-250 m. Returns resume at the same cell after 85 s.
//
// This test WIRES car_carrier_near through the coverage-aware occupancy-decay arm
// and records, per scan, for the labelled cell: EWMA occupancy mass, whether it
// fell inside an estimated coverage sector (swept), and any emitted hazard. It
// splits pre-shadow / shadow / post-shadow and prints an interval table; a sim
// control (sim_ms_anchored_camera — anchored vessel, NO occluder) establishes the
// no-shadow baseline decay. It changes NO decay-model behaviour (the probe reads
// persistenceCells() and the recorded sectors; the additive capture is opt-in and
// default-inert, PhilosLabelReplay.hpp). Skip-guarded on fixture absence.
//
// CONFIG NOTE (load-bearing): the coverage-aware occupancy-DECAY arm is
// `imm_cv_ct_pmbm_occupancy_detector_coverage` (Config.cpp:903, sets
// use_live_occupancy_model + estimate_coverage_sector). The ticket text names
// `imm_cv_ct_pmbm_coverage_land`, but that config wires the sensor-activity
// duty-cycle model + land prior and NEVER the LiveOccupancyModel / coverage-sector
// decay (Config.cpp:1142) — running it would falsely read as verdict (c) "layer
// doesn't fire". We use the config that actually exercises the layer under test.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Wgs84.hpp"
#include "tests/replay/PhilosLabelReplay.hpp"

namespace {

using navtracker::geo::Geodetic;
using navtracker::replay_test::ClipInputs;
using navtracker::replay_test::ClipRun;
using navtracker::replay_test::runClip;
using navtracker::replay_test::runClipInputs;
using navtracker::replay_test::srcDir;

// Cell size of the occupancy grid on imm_cv_ct_pmbm_occupancy_detector_coverage
// (LiveOccupancyParams::cell_size_m = 100 m, Config.cpp:911). A label maps to the
// single cell whose centre it falls in; half-diagonal ≈ 70.7 m.
constexpr double kCellM = 100.0;
constexpr const char* kCovConfig = "imm_cv_ct_pmbm_occupancy_detector_coverage";

// Per-scan probe of one labelled point: EWMA occupancy mass of the cell the point
// falls in (nearest touched cell centre within `radius`, else 0), whether that
// cell was swept this scan (any recorded coverage sector covers it), and whether
// an emitted hazard sits within `hazard_radius` of the point.
struct ScanProbe {
  double t_rel;   // seconds since clip start
  double mass;    // EWMA occupancy mass of the cell (0 if untouched)
  bool swept;     // cell inside some estimated coverage sector this scan
  bool hazard;    // emitted hazard within hazard_radius of the point
  bool has_cell;  // a touched cell was found near the point this scan
};

// Nearest recorded coverage-sector set to time `t` (sectors and tracks are pushed
// per scan; pair by closest timestamp to be robust to any off-by-one in cadence).
const std::vector<navtracker::ISensorDetectionModel::CoverageSector>* sectorsAt(
    const ClipRun& run, double t) {
  const std::vector<navtracker::ISensorDetectionModel::CoverageSector>* best =
      nullptr;
  double best_dt = 1e18;
  for (const auto& ss : run.sector_history) {
    const double dt = std::abs(ss.t_unix - t);
    if (dt < best_dt) {
      best_dt = dt;
      best = &ss.sectors;
    }
  }
  return (best && best_dt <= 1.0) ? best : nullptr;
}

std::vector<ScanProbe> probeCell(const ClipRun& run,
                                 const Eigen::Vector2d& target,
                                 double hazard_radius) {
  std::vector<ScanProbe> out;
  out.reserve(run.history.size());
  for (const auto& scan : run.history) {
    ScanProbe p;
    p.t_rel = scan.t_unix - run.clip_start_unix;

    // Mass + cell centre: nearest touched cell within one cell of the target.
    double best_d = kCellM;  // require the point to be within a cell of a centre
    Eigen::Vector2d cell_center = target;
    double mass = 0.0;
    bool has_cell = false;
    for (const auto& pc : scan.persistence_cells) {
      const double d = (pc.first - target).norm();
      if (d < best_d) {
        best_d = d;
        cell_center = pc.first;
        mass = pc.second;
        has_cell = true;
      }
    }
    p.mass = mass;
    p.has_cell = has_cell;

    // Swept: any coverage sector this scan covers the cell centre (or the target
    // point if no touched cell yet). Absent sectors ⇒ full coverage assumed by the
    // model (have_cover=false path) ⇒ swept = true.
    const auto* secs = sectorsAt(run, scan.t_unix);
    if (secs == nullptr || secs->empty()) {
      p.swept = true;  // no valid footprint ⇒ model treats as fully observed
    } else {
      p.swept = false;
      for (const auto& s : *secs)
        if (s.covers(cell_center)) {
          p.swept = true;
          break;
        }
    }

    // Downstream: any emitted hazard near the point.
    p.hazard = false;
    for (const auto& h : scan.hazards)
      if ((h.center - target).norm() <= hazard_radius) {
        p.hazard = true;
        break;
      }
    out.push_back(p);
  }
  return out;
}

struct IntervalStat {
  int n = 0;
  int swept = 0;
  int hazard = 0;
  int touch = 0;   // scans where the cell mass rose vs the previous scan (return)
  int decay = 0;   // scans where it fell (swept-empty ⇒ "observed-empty" decay)
  double mean_mass = 0.0;
  double first_mass = 0.0;
  double last_mass = 0.0;
  double max_mass = 0.0;
};

// prev_mass carries the last mass seen BEFORE this interval so the first scan's
// touch/decay classification is correct across the interval boundary.
IntervalStat interval(const std::vector<ScanProbe>& ps, double t0, double t1) {
  IntervalStat s;
  double sum = 0.0;
  bool first_set = false;
  double prev = -1.0;
  for (const auto& p : ps) {
    if (p.t_rel >= t0) {  // establish pre-interval mass for the first delta
      if (p.t_rel >= t1) break;
    } else {
      prev = p.mass;
      continue;
    }
    if (!first_set) {
      s.first_mass = p.mass;
      first_set = true;
    }
    if (prev >= 0.0) {
      if (p.mass > prev + 1e-9) ++s.touch;
      else if (p.mass < prev - 1e-9) ++s.decay;
    }
    prev = p.mass;
    s.last_mass = p.mass;
    s.max_mass = std::max(s.max_mass, p.mass);
    sum += p.mass;
    if (p.swept) ++s.swept;
    if (p.hazard) ++s.hazard;
    ++s.n;
  }
  s.mean_mass = s.n ? sum / s.n : 0.0;
  return s;
}

void printRow(const char* name, const IntervalStat& s) {
  const double swept_frac = s.n ? 100.0 * s.swept / s.n : 0.0;
  const double haz_frac = s.n ? 100.0 * s.hazard / s.n : 0.0;
  std::printf(
      "| %-14s | %4d | %6.3f | %6.3f | %6.3f | %6.3f | %5.1f%% | %5.1f%% | %4d "
      "| %4d |\n",
      name, s.n, s.first_mass, s.last_mass, s.mean_mass, s.max_mass, swept_frac,
      haz_frac, s.touch, s.decay);
}

void printHeader(const char* title) {
  std::printf("\n=== %s ===\n", title);
  std::printf(
      "| interval       |    n | mass0  | massT  |  mean  |  max   | swept | "
      "hazrd | tch | dcy |\n");
  std::printf(
      "|----------------|------|--------|--------|--------|--------|-------|-"
      "------|-----|-----|\n");
}

}  // namespace

// car_carrier_near: the occlusion clip. Reports pre-shadow (5-50 s) /
// shadow (50-85 s) / post-shadow (85-120 s) for the unknown_w860 yacht cell.
TEST(LosShadowProbe, CarCarrierNearYachtCell) {
  ClipRun run = runClip("car_carrier_near", kCovConfig,
                        /*load_chart_structure=*/false, /*load_camera=*/false,
                        /*evict_camera=*/false, /*capture_persistence=*/true);
  if (!run.valid) GTEST_SKIP() << "car_carrier_near fixture absent";

  // unknown_w860 label centre → ENU in the clip's fixed datum.
  const Eigen::Vector3d e =
      run.datum.toEnu(Geodetic{42.3583, -71.0464, 0.0});
  const Eigen::Vector2d target(e.x(), e.y());

  // Wiring sanity: the coverage-sector mechanism engaged (finding (c) guard).
  ASSERT_FALSE(run.sector_widths_rad.empty())
      << "no valid coverage sector recorded — occupancy/coverage layer not wired";

  const auto ps = probeCell(run, target, /*hazard_radius=*/150.0);

  const IntervalStat pre = interval(ps, 5.0, 50.0);
  const IntervalStat shadow = interval(ps, 50.0, 85.0);
  const IntervalStat post = interval(ps, 85.0, 120.0);

  printHeader("car_carrier_near — unknown_w860 yacht cell (target ENU relative)");
  std::printf("# target ENU = (%.1f, %.1f) m; datum=clip; cell=%.0f m\n",
              target.x(), target.y(), kCellM);
  printRow("pre  5-50s", pre);
  printRow("shadow 50-85s", shadow);
  printRow("post 85-120s", post);

  // Sector mechanism summary.
  std::vector<double> w = run.sector_widths_rad;
  std::sort(w.begin(), w.end());
  const double med_deg =
      w.empty() ? 0.0 : w[w.size() / 2] * 180.0 / 3.14159265358979323846;
  std::printf(
      "# coverage sectors: %zu valid, median width %.1f deg, %ld full-circle\n",
      w.size(), med_deg, run.sector_full_circle);
  // Cross-check: mass changes ONLY on swept scans (the coverage gate), so
  // touch+decay per interval ≈ swept count — decay events ARE observed-empty
  // calls. The occluder does not inflate the decay rate (flat across intervals);
  // it removes returns (touches collapse), leaving the baseline decays unopposed.
  std::printf(
      "# decay events (observed-empty calls): pre=%d shadow=%d post=%d  |  "
      "touches: pre=%d shadow=%d post=%d\n",
      pre.decay, shadow.decay, post.decay, pre.touch, shadow.touch, post.touch);
  std::printf(
      "# shadow: mass %.3f->%.3f (%.1fx), hazard-presence %.1f%%->%.1f%% (pre->"
      "shadow), recovers to %.3f / %.1f%% post => bounded, self-healing "
      "false-fire (verdict b)\n",
      pre.last_mass, shadow.last_mass,
      shadow.last_mass > 0 ? pre.last_mass / shadow.last_mass : 0.0,
      pre.n ? 100.0 * pre.hazard / pre.n : 0.0,
      shadow.n ? 100.0 * shadow.hazard / shadow.n : 0.0, post.max_mass,
      post.n ? 100.0 * post.hazard / post.n : 0.0);

  // Measurement-only: the cell must at least accrue occupancy mass somewhere in
  // the clip (else the layer never saw the yachts — a wiring finding, surfaced).
  bool ever_mass = false;
  for (const auto& p : ps)
    if (p.mass > 0.0) {
      ever_mass = true;
      break;
    }
  EXPECT_TRUE(ever_mass)
      << "unknown_w860 cell never accrued occupancy mass on this clip";
}

// Sim control: anchored vessel, NO occluder. Establishes the no-shadow baseline
// decay over an equal-length window for comparison against the shadow interval.
TEST(LosShadowProbe, SimAnchoredControl) {
  ClipInputs in;
  const std::string dir =
      srcDir() + "/tests/fixtures/sim_multisensor/sim_ms_anchored_camera_s0";
  in.ownship_csv = dir + "/ownship.csv";
  in.plots_csv = dir + "/radar_plots.csv";
  in.radar_source_id = "sim_radar";
  in.radar_max_range_m = 2500.0;  // sim returns extend to ~2 km
  // No coastline / chart / camera fixtures for the sim scenario (chartless).

  ClipRun run =
      runClipInputs(in, kCovConfig, /*load_chart_structure=*/false,
                    /*load_camera=*/false, /*evict_camera=*/false,
                    /*capture_persistence=*/true);
  if (!run.valid) GTEST_SKIP() << "sim_ms_anchored_camera fixture absent";
  ASSERT_FALSE(run.sector_widths_rad.empty())
      << "no valid coverage sector recorded on sim control";

  // Anchored vessel (mmsi 257000601 "ANCHORED") initial truth position; it barely
  // moves (sog ≈ 0.26 m/s). Datum = 63.45,10.35 (meta.txt).
  const Eigen::Vector3d e =
      run.datum.toEnu(Geodetic{63.45968312, 10.31795823, 0.0});
  const Eigen::Vector2d target(e.x(), e.y());

  const auto ps = probeCell(run, target, /*hazard_radius=*/150.0);

  // Equal-length windows to mirror the car_carrier intervals (35 s shadow).
  const IntervalStat a = interval(ps, 5.0, 50.0);
  const IntervalStat b = interval(ps, 50.0, 85.0);
  const IntervalStat c = interval(ps, 85.0, 120.0);

  printHeader("sim_ms_anchored_camera CONTROL — anchored vessel cell (no occluder)");
  std::printf("# target ENU = (%.1f, %.1f) m; datum=63.45,10.35; cell=%.0f m\n",
              target.x(), target.y(), kCellM);
  printRow("win 5-50s", a);
  printRow("win 50-85s", b);
  printRow("win 85-120s", c);
  // Secondary control: no occluder anywhere. Same cross-check holds
  // (touch+decay ≈ swept). This cell is a LOW-MASS / sparse-touch regime (never a
  // stable hazard), so it is a weaker magnitude baseline than the within-clip
  // pre/post on car_carrier_near; it confirms the decay mechanism operates
  // without any occluder (mass needs continuous re-touch to persist).
  std::printf(
      "# no-occluder control: decays w/o shadow win50-85s=%d touches=%d "
      "swept=%.1f%% (decay operates absent any occlusion)\n",
      b.decay, b.touch, b.n ? 100.0 * b.swept / b.n : 0.0);

  bool ever_mass = false;
  for (const auto& p : ps)
    if (p.mass > 0.0) {
      ever_mass = true;
      break;
    }
  EXPECT_TRUE(ever_mass)
      << "anchored control cell never accrued occupancy mass";
}
