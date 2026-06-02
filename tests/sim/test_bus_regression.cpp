#include <cstdint>
#include <memory>
#include <vector>

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
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

struct TrackerKit {
  std::shared_ptr<ConstantVelocity2D> motion;
  std::unique_ptr<EkfEstimator> est;
  std::unique_ptr<GnnAssociator> assoc;
  std::unique_ptr<TrackManager> mgr;
  std::unique_ptr<Tracker> tracker;
};

// Builds the canonical EKF+GNN+CV2D tracker used by all bus-regression
// scenarios in this file. Caller picks association gate and process noise.
TrackerKit makeTracker(double process_noise_q, double assoc_gate,
                       double tracker_gate, std::size_t init_hits,
                       std::size_t coast_misses) {
  TrackerKit k;
  k.motion = std::make_shared<ConstantVelocity2D>(process_noise_q);
  k.est = std::make_unique<EkfEstimator>(k.motion, 5.0);
  k.assoc = std::make_unique<GnnAssociator>(assoc_gate);
  k.mgr = std::make_unique<TrackManager>(init_hits, coast_misses);
  k.tracker = std::make_unique<Tracker>(*k.est, *k.assoc, *k.mgr, tracker_gate);
  return k;
}

struct TargetSpec {
  std::uint64_t truth_id;
  Eigen::Vector2d start_pos;
  Eigen::Vector2d velocity;
  std::uint32_t mmsi;
  int arpa_track_num;
  int eoir_sensor_track_id;
};

// Drives a CV-only multi-target scenario through SimulatedSensorBus with
// the full quartet (OwnShip + AIS + ARPA + EO/IR) attached. Returns the
// produced Scenario. Caller owns and feeds it to a Tracker.
Scenario runBusFullQuartet(const std::vector<TargetSpec>& targets,
                           double duration_s, std::uint32_t seed) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = duration_s;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  for (const auto& t : targets) {
    bus.addTarget(t.truth_id,
                  std::make_shared<sim::ConstantVelocityTrajectory>(
                      t.start_pos, t.velocity, Timestamp::fromSeconds(0.0)));
  }

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  for (const auto& t : targets)
    ais_cfg.targets.push_back({t.truth_id, t.mmsi, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  for (const auto& t : targets)
    arpa_cfg.targets.push_back({t.truth_id, t.arpa_track_num});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  for (const auto& t : targets)
    eo_cfg.targets.push_back({t.truth_id, t.eoir_sensor_track_id});
  eo_cfg.fov_deg = 360.0;  // disable FOV gate for fair regression
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

double baselineOspaCrossing() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, /*seed=*/11);
  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  return runScenario(s, *kit.tracker, *kit.mgr, 50.0).mean_ospa;
}

}  // namespace

TEST(BusRegression, CrossingMeanOspaWithinTolerance) {
  const double baseline = baselineOspaCrossing();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusFullQuartet(
      {{1, Eigen::Vector2d(-500.0, 10.0), Eigen::Vector2d( 25.0, 0.0), 200000001u, 1, 1},
       {2, Eigen::Vector2d( 500.0,-10.0), Eigen::Vector2d(-25.0, 0.0), 200000002u, 2, 2}},
      40.0, /*seed=*/11);
  ASSERT_GT(s.measurements.size(), 0u);

  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  const ScenarioResult r = runScenario(s, *kit.tracker, *kit.mgr, 50.0);

  // Bus injects more noise (multi-sensor cadence variation, real adapter
  // chain; ArpaEmitter / EoIrEmitter produce range-bearing rather than the
  // baseline's direct 2D position). Pre-truth-tick-OSPA-fix the metric
  // saturated and showed a ~6.66x ratio; post-fix (2026-06-02) the
  // observed ratio on seed=11 is ~4.35x. Tolerance 5.01x = ratio * 1.15
  // for headroom. The bus is allowed to be noisier; do not tune emitters
  // to chase the direct-Measurement baseline.
  EXPECT_LT(r.mean_ospa, baseline * 5.01)
      << "bus mean OSPA " << r.mean_ospa
      << " vs baseline " << baseline;
}

namespace {

double baselineOspaOvertaking() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildOvertakingScenario(
      Eigen::Vector2d(-200.0,  10.0),
      Eigen::Vector2d(  5.0,   0.0),
      Eigen::Vector2d(-400.0, -10.0),
      Eigen::Vector2d( 15.0,   0.0),
      times, 8.0, /*seed=*/11);
  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  return runScenario(s, *kit.tracker, *kit.mgr, 50.0).mean_ospa;
}

}  // namespace

TEST(BusRegression, OvertakingMeanOspaWithinTolerance) {
  const double baseline = baselineOspaOvertaking();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusFullQuartet(
      {{1, Eigen::Vector2d(-200.0, 10.0),  Eigen::Vector2d( 5.0, 0.0), 200000001u, 1, 1},
       {2, Eigen::Vector2d(-400.0,-10.0),  Eigen::Vector2d(15.0, 0.0), 200000002u, 2, 2}},
      40.0, /*seed=*/11);

  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  const ScenarioResult r = runScenario(s, *kit.tracker, *kit.mgr, 50.0);
  // Pre-truth-tick-OSPA-fix the metric saturated and showed a ~7.44x ratio;
  // post-fix (2026-06-02) the observed ratio on seed=11 is ~4.53x.
  // Tolerance 5.22x = ratio * 1.15 for headroom.
  EXPECT_LT(r.mean_ospa, baseline * 5.22)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}

namespace {

double baselineOspaParallel() {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(-500.0,  50.0),
      Eigen::Vector2d(-500.0, -50.0),
      Eigen::Vector2d(  25.0,   0.0),
      times, 8.0, /*seed=*/17);
  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  return runScenario(s, *kit.tracker, *kit.mgr, 50.0).mean_ospa;
}

}  // namespace

TEST(BusRegression, ParallelTargetsMeanOspaWithinTolerance) {
  const double baseline = baselineOspaParallel();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusFullQuartet(
      {{1, Eigen::Vector2d(-500.0, 50.0), Eigen::Vector2d(25.0, 0.0), 200000001u, 1, 1},
       {2, Eigen::Vector2d(-500.0,-50.0), Eigen::Vector2d(25.0, 0.0), 200000002u, 2, 2}},
      30.0, /*seed=*/17);

  auto kit = makeTracker(0.1, 50.0, 30.0, 2, 4);
  const ScenarioResult r = runScenario(s, *kit.tracker, *kit.mgr, 50.0);
  // Post-truth-tick-OSPA-fix (2026-06-02) the observed ratio on seed=17 is
  // ~2.77x. Tolerance 3.19x = ratio * 1.15 for headroom.
  EXPECT_LT(r.mean_ospa, baseline * 3.19)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}

namespace {

double baselineOspaBearingOnlyMoving() {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildBearingOnlyMovingSensorScenario(
      Eigen::Vector2d(1500.0,   0.0),   // target
      Eigen::Vector2d(   0.0,-300.0),   // sensor start
      Eigen::Vector2d(   0.0,  20.0),   // sensor velocity
      times, /*init_pos_std=*/300.0, /*bearing_std_rad=*/0.026,
      /*seed=*/202);
  auto kit = makeTracker(0.1, 200.0, 400.0, 2, 4);
  return runScenario(s, *kit.tracker, *kit.mgr, 400.0).mean_ospa;
}

Scenario runBusBearingOnlyMoving() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 30.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 202;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0),
      Eigen::Vector2d(0.0,   20.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(1500.0, 0.0),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;
  eo_cfg.range_mode = sim::EoIrEmitterConfig::RangeMode::BearingOnly;
  eo_cfg.bearing_std_deg = 1.5;
  eo_cfg.dt_s = 1.0;  // match baseline scan cadence
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, BearingOnlyMovingSensorMeanOspaWithinTolerance) {
  const double baseline = baselineOspaBearingOnlyMoving();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusBearingOnlyMoving();
  auto kit = makeTracker(0.1, 200.0, 400.0, 2, 4);
  const ScenarioResult r = runScenario(s, *kit.tracker, *kit.mgr, 400.0);
  // Bearing-only is high-variance. Post-truth-tick-OSPA-fix (2026-06-02)
  // the observed ratio on seed=202 is ~4.38x. Tolerance 5.04x =
  // ratio * 1.15 for headroom.
  EXPECT_LT(r.mean_ospa, baseline * 5.04)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}
