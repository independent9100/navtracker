// Adaptive UERE bus-level sweep (Task 3 of adaptive-UERE plan).
//
// Two TESTs:
//   1. AdaptiveTracksSimInjectedSigma — for each injected sigma_gps, run
//      20 seeds with the adaptive UERE estimator opted in. After bus.run()
//      completes, sample the provider's published position_std_m as the
//      estimator's final per-run estimate. Assert mean across seeds is
//      within +/-50% of the injected sigma.
//
//   2. AdaptiveSweepClutterCrossing — mirrors BusGpsSweep.ClutterCrossing
//      but with three rows per sigma cell: R-off, R-on static (HDOP*UERE),
//      R-on adaptive. SUCCEED-only; prints the table for the eval-log.
//
// Note: in test 1, report_gps_std=false means the sim does NOT advertise
// the injected sigma via the sticky setter; only lat/lon noise reaches
// the adapter, and the estimator must infer sigma from that. The adaptive
// estimator runs inside adapter.ingest() while bus.run() drives emissions,
// so the final published value is the estimator's verdict for the last
// GGA fix. Because the own-ship in this scenario is stationary (zero
// velocity), the maneuver detector never trips.

#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

// Inline copy of runBusClutterCrossingWithGps that exposes the provider's
// final published position_std_m. We can't use the helper because it only
// returns a Scenario and the provider's lifetime ends at helper return.
double runClutterCrossingAndReportFinalSigma(std::uint32_t seed,
                                             double sigma_gps_m) {
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig own_adapter_cfg;
  own_adapter_cfg.enable_adaptive_uere = true;
  OwnShipNmeaAdapter own_adapter(provider, own_adapter_cfg);
  AisAdapter ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 30.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-200.0, 5.0), Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.gps_pos_std_m = sigma_gps_m;
  // Critical: do not advertise via sticky setter — let the adaptive
  // estimator infer sigma from observed noisy positions.
  own_cfg.report_gps_std = false;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_emitter_cfg;
  arpa_emitter_cfg.targets.push_back({1, 1});
  arpa_emitter_cfg.targets.push_back({2, 2});
  arpa_emitter_cfg.clutter_per_rotation = 8;
  bus.attachArpa(arpa_adapter, arpa_emitter_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.targets.push_back({2, 2});
  eo_emitter_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  bus.run();
  const auto p = provider.latest();
  return p ? p->position_std_m : 0.0;
}

// Standard batched (JPDA-style) tracker cell used by the sweep. Mirrors
// runBatchedCell in test_bus_gps_sweep.cpp.
RunStats runBatchedCell(navtracker::Scenario&& s, double cutoff, int confirm,
                        int del, double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev, countIdSwitches(r.steps, cutoff)};
}

struct Cell {
  double sigma_gps_m;
  const char* row;  // "R-off" | "R-on static" | "R-on adaptive"
  AggStats agg;
};

void printCellsTable(const char* scenario_name,
                     const std::vector<Cell>& cells) {
  std::fprintf(stderr,
      "\n[Bus Adaptive UERE Sweep on %s, %d seeds]\n"
      "  sigma_gps_m | row             | per-window OSPA mean   | id_sw_mean\n",
      scenario_name, kNumSeeds);
  for (const Cell& c : cells) {
    std::fprintf(stderr,
        "  %10.2f  | %-15s | %7.4f +/- %6.4f m | %.2f\n",
        c.sigma_gps_m, c.row, c.agg.mean_ospa, c.agg.std_ospa, c.agg.mean_id_sw);
  }
}

}  // namespace

TEST(BusAdaptiveUere, AdaptiveTracksSimInjectedSigma) {
  const double sigmas[] = {0.1, 1.0, 5.0};

  std::fprintf(stderr,
      "\n[Adaptive UERE tracks injected sigma, %d seeds]\n"
      "  sigma_injected (m) | mean published sigma (m) | within +/-50%%?\n",
      kNumSeeds);

  for (double sigma_inj : sigmas) {
    double sum = 0.0;
    int n = 0;
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 301u + static_cast<std::uint32_t>(k);
      const double s = runClutterCrossingAndReportFinalSigma(seed, sigma_inj);
      sum += s;
      ++n;
    }
    const double mean = n > 0 ? sum / static_cast<double>(n) : 0.0;
    const bool within = (mean >= 0.5 * sigma_inj) && (mean <= 1.5 * sigma_inj);
    std::fprintf(stderr, "  %16.4f   | %22.4f   | %s\n",
                 sigma_inj, mean, within ? "yes" : "NO");
    EXPECT_GE(mean, 0.5 * sigma_inj)
        << "sigma_injected=" << sigma_inj << " mean=" << mean;
    EXPECT_LE(mean, 1.5 * sigma_inj)
        << "sigma_injected=" << sigma_inj << " mean=" << mean;
  }
}

TEST(BusAdaptiveUere, AdaptiveSweepClutterCrossing) {
  const double sigmas[] = {0.0, 0.1, 1.0, 5.0};

  // Three rows per sigma cell:
  //   row_a: R-off            — sigma injected, R inflation off
  //   row_b: R-on static      — sigma injected, R on via static HDOP*UERE
  //   row_c: R-on adaptive    — sigma injected, R on via adaptive estimator
  struct RowSpec {
    const char* name;
    bool r_inflation_on;
    bool adaptive_uere;
    // For adaptive (row_c) we want the sim to NOT advertise via sticky
    // setter so the adaptive path is exercised end-to-end. r_inflation_on
    // gates whether OwnShipEmitter calls setPositionStd.
  };
  const RowSpec rows[] = {
      {"R-off",         false, false},
      {"R-on static",   true,  false},
      {"R-on adaptive", false, true},
  };

  std::vector<Cell> cells;
  for (double sg : sigmas) {
    for (const auto& row : rows) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        GpsSweepKnob knob;
        knob.sigma_gps_m = sg;
        knob.r_inflation_on = row.r_inflation_on;
        knob.adaptive_uere = row.adaptive_uere;
        runs.push_back(runBatchedCell(
            runBusClutterCrossingWithGps(seed, knob),
            /*cutoff=*/50.0, /*confirm=*/2, /*del=*/4, /*miss=*/30.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{sg, row.name, aggregate(runs)});
    }
  }
  printCellsTable("ClutterCrossing", cells);
  SUCCEED();
}
