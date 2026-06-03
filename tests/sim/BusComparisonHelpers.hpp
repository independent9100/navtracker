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
#include "core/bias/AisArpaPairExtractor.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/scenario/Ospa.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/pipeline/Tracker.hpp"
#include "ports/IHeadingBiasProvider.hpp"
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

struct GpsSweepKnob {
  double sigma_gps_m{0.0};       // injected lat/lon noise std (m)
  bool   r_inflation_on{false};  // when true, pose advertises position_std_m
  bool   adaptive_uere{false};   // when true, adapter runs UereEstimator
};

inline navtracker::Scenario runBusClutterCrossingWithGps(
    std::uint32_t seed, const GpsSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig own_adapter_cfg;
  own_adapter_cfg.enable_adaptive_uere = knob.adaptive_uere;
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
      Eigen::Vector2d(-200.0,  5.0), Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d( 200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.gps_pos_std_m = knob.sigma_gps_m;
  own_cfg.report_gps_std = knob.r_inflation_on;
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

  return bus.run();
}

inline navtracker::Scenario runBusBearingOnlyMovingWithGps(
    std::uint32_t seed, const GpsSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig own_adapter_cfg;
  own_adapter_cfg.enable_adaptive_uere = knob.adaptive_uere;
  OwnShipNmeaAdapter own_adapter(provider, own_adapter_cfg);
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
      Eigen::Vector2d(0.0, -300.0), Eigen::Vector2d(0.0, 10.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(1500.0, 0.0), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.gps_pos_std_m = knob.sigma_gps_m;
  own_cfg.report_gps_std = knob.r_inflation_on;
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

// ---------------------------------------------------------------------------
// Bias-estimator wiring (Task 8 of heading-bias-estimator plan).
//
// Closed-loop interleaving via SimulatedSensorBus::stepOnce.
//
// `runBusCellInterleaved` is the low-level driver: it owns the per-cycle loop
// where on each outer iteration we (1) step the bus by dt, appending one
// cycle's worth of measurements and (when due) truth samples to a running
// Scenario, (2) feed the new measurements into the tracker, (3) at every
// truth-tick boundary call extractPairs() against the live track set and
// observe any AIS+ARPA pairs that completed this cycle on the estimator,
// (4) clear `recent_contributions` so the next cycle starts clean, (5)
// snapshot tracks for per-tick OSPA. Because the estimator publishes its
// b_hat through the IHeadingBiasProvider interface, the very next call to
// `bus.stepOnce` re-projects ARPA/EO-IR detections with the updated b_hat —
// this is the closed loop that the post-hoc runner in 3c39e13 lacked.
//
// `runBus{Clutter,BearingOnly,Maneuvering}*WithHeadingAndBiasCell` are
// integrated cell helpers that build the bus + adapters + tracker + manager
// + estimator with appropriate config and drive `runBusCellInterleaved`,
// returning a RunStats. The integrated form is needed because the adapters
// hold references the bus consumes per-step; keeping everything in one
// stack frame side-steps the ownership puzzle that a "build then run later"
// split would otherwise require.
// ---------------------------------------------------------------------------

struct BiasEstimatorKnob {
  bool enabled{false};
  navtracker::HeadingBiasEstimatorConfig cfg{};
  navtracker::AisArpaPairExtractorConfig extractor_cfg{};
};

// Low-level interleaved driver. Steps the bus one cycle at a time, dispatches
// each new measurement into the tracker, and at every truth-tick boundary
// runs extract+observe on the estimator before clearing recent_contributions.
// Returns a RunStats computed against per-window OSPA (window = kWindowDtS)
// and the standard id-switch count over track snapshots.
//
// Pre: bus has all emitters/adapters attached and own-ship/targets set.
//      tracker, manager, estimator are fresh (or in their desired initial
//      state). The estimator must be wired into the ARPA/EO-IR adapters
//      held by `bus` (via their `bias_provider` parameter) so that
//      adapter-side projections after estimator updates see b_hat.
inline RunStats runBusCellInterleaved(
    navtracker::sim::SimulatedSensorBus& bus,
    navtracker::Tracker& tracker,
    navtracker::TrackManager& manager,
    navtracker::HeadingBiasEstimator& estimator,
    const navtracker::AisArpaPairExtractorConfig& extractor_cfg,
    double ospa_cutoff) {
  using namespace navtracker;

  Scenario scenario;  // grows as the bus produces cycles
  ScenarioResult result;

  std::size_t mi_processed = 0;  // measurements already dispatched to tracker
  std::size_t ti_processed = 0;  // truth samples already scored

  while (bus.stepOnce(scenario)) {
    // (1) Dispatch any measurements appended this step into the tracker.
    //     Within one bus cycle, emissions are time-ordered by construction,
    //     so no sort is needed; the tracker handles same-timestamp dispatch.
    while (mi_processed < scenario.measurements.size()) {
      tracker.process(scenario.measurements[mi_processed]);
      ++mi_processed;
    }

    // (2) For every truth tick that completed this step, run estimator
    //     extract+observe and emit an OSPA sample. A single bus step adds
    //     at most one truth-sample timestamp (truth_sample_dt_s >= dt_s),
    //     but multiple truth samples may share that timestamp (one per
    //     target).
    while (ti_processed < scenario.truth.size()) {
      const Timestamp tick = scenario.truth[ti_processed].time;

      // Extract AIS+ARPA pairs visible on tracks at this cycle boundary and
      // feed them to the estimator. Then advance estimator age and clear
      // contribution buffers so the next cycle starts clean.
      const auto pairs = extractPairs(manager.tracks(), tick, extractor_cfg);
      for (const auto& p : pairs) estimator.observe(p);
      estimator.predictTo(tick);
      for (auto& tr : manager.mutableTracks()) tr.recent_contributions.clear();

      // Gather all truth samples sharing this tick.
      std::vector<Eigen::Vector2d> truth_xy;
      std::size_t tj = ti_processed;
      while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
        truth_xy.push_back(scenario.truth[tj].position);
        ++tj;
      }

      // Snapshot tracks; score OSPA.
      std::vector<Eigen::Vector2d> est_xy;
      std::vector<TrackSnapshot> snaps;
      for (const Track& tr : manager.tracks()) {
        if (tr.state.size() >= 2) {
          est_xy.emplace_back(tr.state(0), tr.state(1));
          snaps.push_back(TrackSnapshot{tr.id,
                                        Eigen::Vector2d(tr.state(0), tr.state(1))});
        }
      }

      result.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

      ScenarioStep step;
      step.time = tick;
      step.truth = std::move(truth_xy);
      step.tracks = std::move(snaps);
      result.steps.push_back(std::move(step));

      ti_processed = tj;
    }
  }

  if (!result.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : result.ospa_per_step) sum += v;
    result.mean_ospa = sum / static_cast<double>(result.ospa_per_step.size());
  }

  const PerWindowOspa pw = computePerWindowOspa(
      result, navtracker::Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev, countIdSwitches(result.steps, ospa_cutoff)};
}

}  // namespace navtracker_test
