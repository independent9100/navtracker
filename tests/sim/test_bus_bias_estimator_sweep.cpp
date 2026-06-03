// Sweep — §14.9 scenarios with heading-bias estimator on/off.
//
// For each of three scenarios (ClutterCrossing, BearingOnlyMoving,
// Maneuvering) we sweep sigma_h ∈ {0.0, 0.5, 1.0, 2.0} deg × 20 seeds
// (201..220) and aggregate three rows per cell:
//
//   row_a = R-off, no estimator   (baseline; cfg.heading_std_deg = 0)
//   row_b = R-on,  no estimator   (R inflation only)
//   row_c = R-on,  estimator ON   (closed-loop: estimator publishes b_hat
//                                  back through adapters via stepOnce)
//
// rows a/b use the existing `runBus*WithHeading` builders + the bus's
// `run()` -> `runScenario` path; row_c rebuilds the bus + adapters
// inline so that ARPA/EO-IR adapters can take `bias_provider=&estimator`
// and the interleaved driver in `runBusCellInterleaved` can dispatch
// per-cycle pair extraction.
//
// SUCCEED-only. Output captured for the eval-log (Task 11).
//
// Note on BearingOnlyMoving: this scenario only emits EO-IR; no AIS or
// ARPA is wired, so the AIS+ARPA pair extractor never finds a pair and
// the estimator never publishes. Row_c is therefore expected to be ≈ row_b.
// We keep the row to document the (intentional) gating fallback.

#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/bias/AisArpaPairExtractor.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
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

constexpr double kPi = 3.14159265358979323846;

// Slightly relaxed publish threshold so the estimator publishes within the
// 30 s / 15 s scenarios. Default is (0.3 deg)^2; we widen to (0.5 deg)^2.
// The estimator still self-gates on this threshold; goal is to measure the
// closed-loop benefit at modest convergence on short scenarios.
inline HeadingBiasEstimatorConfig sweepEstimatorCfg() {
  HeadingBiasEstimatorConfig cfg{};
  const double half_deg_rad = 0.5 * kPi / 180.0;
  cfg.publish_variance_threshold_rad2 = half_deg_rad * half_deg_rad;
  return cfg;
}

// -- Estimator-off rows (a/b) ------------------------------------------------
// Mirror test_bus_heading_sweep.cpp helpers so row_a and row_b use the same
// tracker config as the §14.9 sweep.

RunStats runStandardCell(navtracker::Scenario&& s, double cutoff, int confirm,
                         int del, double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev, countIdSwitches(r.steps, cutoff)};
}

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

// -- Estimator-on row (c) ----------------------------------------------------
// Inline cell builders that mirror the WithHeading scenarios but wire the
// estimator as bias_provider into ARPA/EO-IR adapters and drive the
// closed-loop interleaved runner.

RunStats runBusClutterCrossingWithBias(std::uint32_t seed,
                                       int clutter_per_rotation,
                                       const HeadingSweepKnob& knob,
                                       double ospa_cutoff) {
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);

  ArpaAdapterConfig arpa_cfg_adapter;
  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) {
    arpa_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
    eo_cfg_adapter.heading_std_deg   = knob.sigma_heading_deg;
  }

  HeadingBiasEstimator estimator(sweepEstimatorCfg());
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter, &estimator);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter,   &estimator);

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
      Eigen::Vector2d(-200.0,  5.0), Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d( 200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_emitter_cfg;
  arpa_emitter_cfg.targets.push_back({1, 1});
  arpa_emitter_cfg.targets.push_back({2, 2});
  arpa_emitter_cfg.clutter_per_rotation = clutter_per_rotation;
  bus.attachArpa(arpa_adapter, arpa_emitter_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.targets.push_back({2, 2});
  eo_emitter_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  // Tracker config matches the §14.9 sweep cell for ClutterCrossing.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(/*confirm=*/2, /*del=*/4);
  Tracker tracker(ekf, gnn, mgr, /*miss=*/30.0);

  AisArpaPairExtractorConfig extractor_cfg{};
  return runBusCellInterleaved(bus, tracker, mgr, estimator, extractor_cfg,
                               ospa_cutoff);
}

RunStats runBusBearingOnlyMovingWithBias(std::uint32_t seed,
                                         const HeadingSweepKnob& knob,
                                         double ospa_cutoff) {
  // EOIR-only — no AIS, no ARPA. The estimator never receives a pair and
  // never publishes; this row therefore approximates row_b (R-on only).
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);

  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) eo_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;

  HeadingBiasEstimator estimator(sweepEstimatorCfg());
  EoIrAdapter eo_adapter(datum, provider, eo_cfg_adapter, &estimator);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 60.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0), Eigen::Vector2d(0.0, 10.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(1500.0, 0.0), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.fov_deg = 360.0;
  eo_emitter_cfg.range_mode = sim::EoIrEmitterConfig::RangeMode::BearingOnly;
  eo_emitter_cfg.bearing_std_deg = 1.5;
  eo_emitter_cfg.dt_s = 1.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(/*confirm=*/1, /*del=*/8);
  Tracker tracker(ekf, gnn, mgr, /*miss=*/90.0);

  AisArpaPairExtractorConfig extractor_cfg{};
  return runBusCellInterleaved(bus, tracker, mgr, estimator, extractor_cfg,
                               ospa_cutoff);
}

RunStats runBusManeuveringWithBias(std::uint32_t seed,
                                   const HeadingSweepKnob& knob,
                                   double ospa_cutoff) {
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);

  ArpaAdapterConfig arpa_cfg_adapter;
  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) {
    arpa_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
    eo_cfg_adapter.heading_std_deg   = knob.sigma_heading_deg;
  }

  HeadingBiasEstimator estimator(sweepEstimatorCfg());
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter, &estimator);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter,   &estimator);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 15.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ManeuveringTrajectory>(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      /*straight=*/5.0, /*turn=*/5.0, /*omega=*/0.2,
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_emitter_cfg;
  arpa_emitter_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_emitter_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(/*confirm=*/1, /*del=*/5);
  Tracker tracker(ekf, gnn, mgr, /*miss=*/10.0);

  AisArpaPairExtractorConfig extractor_cfg{};
  return runBusCellInterleaved(bus, tracker, mgr, estimator, extractor_cfg,
                               ospa_cutoff);
}

// -- Output ------------------------------------------------------------------

struct Cell {
  const char* scenario_name;
  double sigma_heading_deg;
  const char* row_label;  // "R-off", "R-on", "R-on+est"
  AggStats agg;
};

void printCellsTable(const char* scenario_name,
                     const std::vector<Cell>& cells) {
  std::fprintf(stderr,
      "\n[Bus Bias Estimator Sweep on %s, %d seeds]\n"
      "  sigma_h_deg | row        | per-window OSPA mean   | id_sw_mean\n",
      scenario_name, kNumSeeds);
  for (const Cell& c : cells) {
    std::fprintf(stderr,
        "  %10.2f  | %-10s | %7.4f +/- %6.4f m | %.2f\n",
        c.sigma_heading_deg, c.row_label,
        c.agg.mean_ospa, c.agg.std_ospa, c.agg.mean_id_sw);
  }
}

}  // namespace

TEST(BusBiasEstimatorSweep, ClutterCrossing) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const double cutoff = 50.0;

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    std::vector<RunStats> runs_a, runs_b, runs_c;
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);

      // row_a: σ injected, R-off, no estimator (use sigma=0 to express "off")
      HeadingSweepKnob knob_a;
      knob_a.sigma_heading_deg = sh;
      knob_a.r_inflation_on = false;
      runs_a.push_back(runBatchedCell(
          runBusClutterCrossingWithHeading(seed, /*clutter=*/5, knob_a),
          cutoff, /*confirm=*/2, /*del=*/4, /*miss=*/30.0, /*q=*/0.1));

      // row_b: R-on, no estimator
      HeadingSweepKnob knob_b = knob_a;
      knob_b.r_inflation_on = true;
      runs_b.push_back(runBatchedCell(
          runBusClutterCrossingWithHeading(seed, /*clutter=*/5, knob_b),
          cutoff, /*confirm=*/2, /*del=*/4, /*miss=*/30.0, /*q=*/0.1));

      // row_c: R-on, estimator ON
      runs_c.push_back(runBusClutterCrossingWithBias(
          seed, /*clutter=*/5, knob_b, cutoff));
    }
    cells.push_back(Cell{"ClutterCrossing", sh, "R-off",    aggregate(runs_a)});
    cells.push_back(Cell{"ClutterCrossing", sh, "R-on",     aggregate(runs_b)});
    cells.push_back(Cell{"ClutterCrossing", sh, "R-on+est", aggregate(runs_c)});
  }
  printCellsTable("ClutterCrossing", cells);
  SUCCEED();
}

TEST(BusBiasEstimatorSweep, BearingOnlyMoving) {
  // EOIR-only scenario: estimator never sees AIS+ARPA pairs, never publishes.
  // R-on+est is expected to be ~ R-on; we keep the row to document gating.
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const double cutoff = 500.0;

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    std::vector<RunStats> runs_a, runs_b, runs_c;
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);

      HeadingSweepKnob knob_a;
      knob_a.sigma_heading_deg = sh;
      knob_a.r_inflation_on = false;
      runs_a.push_back(runStandardCell(
          runBusBearingOnlyMovingWithHeading(seed, knob_a),
          cutoff, /*confirm=*/1, /*del=*/8, /*miss=*/90.0, /*q=*/0.1));

      HeadingSweepKnob knob_b = knob_a;
      knob_b.r_inflation_on = true;
      runs_b.push_back(runStandardCell(
          runBusBearingOnlyMovingWithHeading(seed, knob_b),
          cutoff, /*confirm=*/1, /*del=*/8, /*miss=*/90.0, /*q=*/0.1));

      runs_c.push_back(runBusBearingOnlyMovingWithBias(seed, knob_b, cutoff));
    }
    cells.push_back(Cell{"BearingOnlyMoving", sh, "R-off",    aggregate(runs_a)});
    cells.push_back(Cell{"BearingOnlyMoving", sh, "R-on",     aggregate(runs_b)});
    cells.push_back(Cell{"BearingOnlyMoving", sh, "R-on+est", aggregate(runs_c)});
  }
  printCellsTable("BearingOnlyMoving", cells);
  SUCCEED();
}

TEST(BusBiasEstimatorSweep, Maneuvering) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const double cutoff = 100.0;

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    std::vector<RunStats> runs_a, runs_b, runs_c;
    for (int k = 0; k < kNumSeeds; ++k) {
      const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);

      HeadingSweepKnob knob_a;
      knob_a.sigma_heading_deg = sh;
      knob_a.r_inflation_on = false;
      runs_a.push_back(runStandardCell(
          runBusManeuveringWithHeading(seed, knob_a),
          cutoff, /*confirm=*/1, /*del=*/5, /*miss=*/10.0, /*q=*/0.1));

      HeadingSweepKnob knob_b = knob_a;
      knob_b.r_inflation_on = true;
      runs_b.push_back(runStandardCell(
          runBusManeuveringWithHeading(seed, knob_b),
          cutoff, /*confirm=*/1, /*del=*/5, /*miss=*/10.0, /*q=*/0.1));

      runs_c.push_back(runBusManeuveringWithBias(seed, knob_b, cutoff));
    }
    cells.push_back(Cell{"Maneuvering", sh, "R-off",    aggregate(runs_a)});
    cells.push_back(Cell{"Maneuvering", sh, "R-on",     aggregate(runs_b)});
    cells.push_back(Cell{"Maneuvering", sh, "R-on+est", aggregate(runs_c)});
  }
  printCellsTable("Maneuvering", cells);
  SUCCEED();
}
