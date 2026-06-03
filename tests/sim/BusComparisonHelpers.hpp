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

// ---------------------------------------------------------------------------
// Bias-estimator wiring (Task 8 of heading-bias-estimator plan).
//
// Knob + parallel scenario builders that wire a HeadingBiasEstimator into the
// ARPA and EO/IR adapters at construction. A small per-cycle runner
// (runScenarioWithBiasEstimator) drives the tracker measurement-by-measurement
// and, at each truth-tick boundary, calls extractPairs(...) and feeds
// AIS+ARPA pair observations to the estimator before clearing per-track
// `recent_contributions`.
//
// Path B caveat: the bus produces the full Scenario before the tracker runs,
// so adapter-side bias corrections see the estimator state at *scenario build
// time* (initially unpublished -> b=0). The estimator's published b̂ therefore
// affects the variance composition (var_b_hat folded into sigma_heading) once
// it converges within bus.run(), but the projected ENU positions for an
// already-emitted scenario are baked. T9 uses this runner to A/B compare R-on
// + estimator vs R-on no-estimator; the comparison is conservative w.r.t.
// what an interleaved implementation would deliver, but the trajectory of
// b̂ is real and is what feeds future adapter projections within the same
// bus.run() call (TTM/EOIR detections after AIS-derived updates already
// landed in the bus emission order benefit from the latest published b̂
// because the adapters query the provider per detection).
// ---------------------------------------------------------------------------

struct BiasEstimatorKnob {
  bool enabled{false};
  navtracker::HeadingBiasEstimatorConfig cfg{};
  navtracker::AisArpaPairExtractorConfig extractor_cfg{};
};

// Per-cycle runner mirroring core/scenario/Harness.cpp::runScenario but with
// a hook at every truth tick: after the tracker has consumed all measurements
// up to the tick, call extractPairs(tracks(), tick, extractor_cfg) and feed
// pairs into the estimator; then clear `recent_contributions` on every track.
inline navtracker::ScenarioResult runScenarioWithBiasEstimator(
    const navtracker::Scenario& scenario,
    navtracker::Tracker& tracker,
    navtracker::TrackManager& manager,
    double ospa_cutoff,
    navtracker::HeadingBiasEstimator& estimator,
    const navtracker::AisArpaPairExtractorConfig& extractor_cfg) {
  using namespace navtracker;
  ScenarioResult r;
  if (scenario.truth.empty()) return r;

  std::size_t mi = 0;
  std::size_t ti = 0;

  while (ti < scenario.truth.size()) {
    const Timestamp tick = scenario.truth[ti].time;

    while (mi < scenario.measurements.size() &&
           !(tick < scenario.measurements[mi].time)) {
      tracker.process(scenario.measurements[mi]);
      ++mi;
    }

    // Run bias estimator extract + observe at this cycle boundary.
    const auto pairs = extractPairs(manager.tracks(), tick, extractor_cfg);
    for (const auto& p : pairs) estimator.observe(p);
    // Predict to current tick so age tracking advances even without obs.
    estimator.predictTo(tick);
    // Clear recent_contributions so next cycle's extraction is clean.
    for (auto& tr : manager.mutableTracks()) tr.recent_contributions.clear();

    std::vector<Eigen::Vector2d> truth_xy;
    std::size_t tj = ti;
    while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
      truth_xy.push_back(scenario.truth[tj].position);
      ++tj;
    }

    std::vector<Eigen::Vector2d> est_xy;
    std::vector<TrackSnapshot> snaps;
    for (const Track& tr : manager.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
        snaps.push_back(TrackSnapshot{tr.id,
                                      Eigen::Vector2d(tr.state(0), tr.state(1))});
      }
    }

    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

    ScenarioStep step;
    step.time = tick;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));

    ti = tj;
  }

  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}

// Parallel scenario builders that take an optional bias provider. Wire it
// into the ARPA and EO/IR adapter constructors so projection-time
// corrections kick in once the estimator publishes during bus.run().

inline navtracker::Scenario runBusClutterCrossingWithHeadingAndBias(
    std::uint32_t seed, int clutter_per_rotation,
    const HeadingSweepKnob& knob,
    const navtracker::IHeadingBiasProvider* bias_provider) {
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
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter, bias_provider);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter,   bias_provider);

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

inline navtracker::Scenario runBusBearingOnlyMovingWithHeadingAndBias(
    std::uint32_t seed, const HeadingSweepKnob& knob,
    const navtracker::IHeadingBiasProvider* bias_provider) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);

  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) eo_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
  EoIrAdapter eo_adapter(datum, provider, eo_cfg_adapter, bias_provider);

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

inline navtracker::Scenario runBusManeuveringWithHeadingAndBias(
    std::uint32_t seed, const HeadingSweepKnob& knob,
    const navtracker::IHeadingBiasProvider* bias_provider) {
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
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter, bias_provider);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter,   bias_provider);

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
