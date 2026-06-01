#include "sim/SimulatedSensorBus.hpp"

#include <memory>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(SimulatedSensorBus, ProducesAscendingTimeOrderedMeasurements) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig bus_cfg;
  bus_cfg.t0 = Timestamp::fromSeconds(0.0);
  bus_cfg.duration_s = 30.0;
  bus_cfg.dt_s = 0.1;
  bus_cfg.truth_sample_dt_s = 1.0;
  bus_cfg.seed = 7;
  bus_cfg.datum = datum;

  sim::SimulatedSensorBus bus(bus_cfg);

  auto own_traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));
  bus.setOwnShip(own_traj);

  auto tgt_traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0, 200.0),
      Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0));
  bus.addTarget(/*truth_id=*/1, tgt_traj);

  bus.attachOwnShip(own_adapter, {});

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_cfg);

  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;  // disable FOV gate
  bus.attachEoIr(eo_adapter, eo_cfg);

  const Scenario s = bus.run();

  // Some measurements arrived.
  EXPECT_GT(s.measurements.size(), 0u);
  // Strictly non-decreasing time.
  for (std::size_t i = 1; i < s.measurements.size(); ++i) {
    EXPECT_LE(s.measurements[i - 1].time, s.measurements[i].time);
  }
  // Truth sampled at 1 Hz over 30 s => 31 samples.
  EXPECT_EQ(s.truth.size(), 31u);
}

TEST(SimulatedSensorBus, NoSensorsAttachedProducesEmptyMeasurements) {
  Datum datum({53.5, 8.0, 0.0});

  sim::SimulatedSensorBusConfig bus_cfg;
  bus_cfg.t0 = Timestamp::fromSeconds(0.0);
  bus_cfg.duration_s = 5.0;
  bus_cfg.dt_s = 0.5;
  bus_cfg.truth_sample_dt_s = 1.0;
  bus_cfg.seed = 1;
  bus_cfg.datum = datum;

  sim::SimulatedSensorBus bus(bus_cfg);
  auto own = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Timestamp::fromSeconds(0.0));
  bus.setOwnShip(own);

  const Scenario s = bus.run();
  EXPECT_TRUE(s.measurements.empty());
  EXPECT_EQ(s.truth.size(), 6u);
}
