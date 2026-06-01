#include <memory>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

Scenario runOnce(std::uint32_t seed) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 20.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-300.0, 100.0),
      Eigen::Vector2d(10.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(SimulatedSensorBusDeterminism, TwoRunsSameSeedIdenticalOutput) {
  const Scenario a = runOnce(42);
  const Scenario b = runOnce(42);

  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].time, b.measurements[i].time);
    EXPECT_EQ(a.measurements[i].sensor, b.measurements[i].sensor);
    ASSERT_EQ(a.measurements[i].value.size(), b.measurements[i].value.size());
    for (int k = 0; k < a.measurements[i].value.size(); ++k)
      EXPECT_DOUBLE_EQ(a.measurements[i].value(k), b.measurements[i].value(k));
  }
  ASSERT_EQ(a.truth.size(), b.truth.size());
  for (std::size_t i = 0; i < a.truth.size(); ++i) {
    EXPECT_EQ(a.truth[i].time, b.truth[i].time);
    EXPECT_EQ(a.truth[i].truth_id, b.truth[i].truth_id);
    EXPECT_DOUBLE_EQ(a.truth[i].position.x(), b.truth[i].position.x());
    EXPECT_DOUBLE_EQ(a.truth[i].position.y(), b.truth[i].position.y());
  }
}

TEST(SimulatedSensorBusDeterminism, DifferentSeedsDifferOnNoise) {
  const Scenario a = runOnce(42);
  const Scenario b = runOnce(43);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  // At least one measurement value differs.
  bool any_diff = false;
  for (std::size_t i = 0; i < a.measurements.size() && !any_diff; ++i) {
    for (int k = 0; k < a.measurements[i].value.size() && !any_diff; ++k) {
      if (a.measurements[i].value(k) != b.measurements[i].value(k)) any_diff = true;
    }
  }
  EXPECT_TRUE(any_diff);
}
