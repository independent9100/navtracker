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

double baselineOspaCrossing() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, /*seed=*/11);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  return runScenario(s, tracker, mgr, 50.0).mean_ospa;
}

Scenario runBusCrossing() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 40.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 11;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;  // disable FOV gate
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, CrossingMeanOspaWithinTolerance) {
  const double baseline = baselineOspaCrossing();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusCrossing();
  ASSERT_GT(s.measurements.size(), 0u);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);

  // Bus injects strictly more noise (multi-sensor cadence variation, real
  // adapter chain — ArpaEmitter / EoIrEmitter produce range-bearing rather
  // than the baseline's direct 2D position; ~10x more measurements at higher
  // cadence). Observed ratio ~6.66x on seed=11; tolerance 7.0x as a
  // "no catastrophic regression" guard. The bus is allowed to be noisier;
  // do not tune emitters to chase the direct-Measurement baseline.
  EXPECT_LT(r.mean_ospa, baseline * 7.0)
      << "bus mean OSPA " << r.mean_ospa
      << " vs baseline " << baseline;
}
