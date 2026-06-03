#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Metrics.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker_test {

struct RunStats {
  double mean_ospa;        // mean of per-window OSPA means (1 s windows)
  double stddev_ospa;      // stddev across windows
  int id_switches;
};

struct AggStats {
  double mean_ospa;
  double std_ospa;
  double mean_id_sw;
};

inline AggStats aggregate(const std::vector<RunStats>& runs) {
  const std::size_t N = runs.size();
  if (N == 0) return {0.0, 0.0, 0.0};
  double sum_o = 0.0, sum_i = 0.0;
  for (const auto& r : runs) { sum_o += r.mean_ospa; sum_i += r.id_switches; }
  const double m_o = sum_o / static_cast<double>(N);
  double sse = 0.0;
  for (const auto& r : runs) sse += (r.mean_ospa - m_o) * (r.mean_ospa - m_o);
  const double s_o = N > 1 ? std::sqrt(sse / static_cast<double>(N - 1)) : 0.0;
  return {m_o, s_o, sum_i / static_cast<double>(N)};
}

// Two cooperative AIS+ARPA+EO/IR targets crossing through the origin, with
// configurable ARPA clutter. Used by JPDA and MHT bus comparisons.
inline navtracker::Scenario runBusClutterCrossing(std::uint32_t seed,
                                                  int clutter_per_rotation) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
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
      Eigen::Vector2d(-200.0,  5.0),
      Eigen::Vector2d(  15.0,  0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d( 200.0, -5.0),
      Eigen::Vector2d( -15.0,  0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  arpa_cfg.clutter_per_rotation = clutter_per_rotation;
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

// Single static target observed by a moving EO-IR sensor. Used by PF.
inline navtracker::Scenario runBusBearingOnlyMoving(std::uint32_t seed) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 60.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0),
      Eigen::Vector2d(0.0,   10.0),
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
  eo_cfg.dt_s = 1.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

// Single maneuvering target (straight-turn-straight). Used by IMM-3.
inline navtracker::Scenario runBusManeuvering(std::uint32_t seed) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

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

constexpr int kNumSeeds = 20;
constexpr double kWindowDtS = 1.0;

struct HeadingSweepKnob {
  double sigma_heading_deg{0.0};        // per-tick white noise on HDT
  double bias_deg{0.0};                 // constant offset
  double drift_deg_per_s{0.0};          // linear-in-time offset
  bool   r_inflation_on{false};         // pass σ_h through to adapter cfg
};

inline navtracker::Scenario runBusClutterCrossingWithHeading(
    std::uint32_t seed, int clutter_per_rotation,
    const HeadingSweepKnob& knob) {
  using namespace navtracker;
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
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter);

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

  return bus.run();
}

inline navtracker::Scenario runBusBearingOnlyMovingWithHeading(
    std::uint32_t seed, const HeadingSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);

  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) eo_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
  EoIrAdapter eo_adapter(datum, provider, eo_cfg_adapter);

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

  return bus.run();
}

inline navtracker::Scenario runBusManeuveringWithHeading(
    std::uint32_t seed, const HeadingSweepKnob& knob) {
  using namespace navtracker;
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
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter);

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

  return bus.run();
}

}  // namespace navtracker_test
