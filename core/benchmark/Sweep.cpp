#include "core/benchmark/Sweep.hpp"

// Sweep — drives the (config x scenario x seed) matrix, runs BenchRunner,
// computes metrics, emits long-format rows.
//
// Math:
//   For each (config, scenario, seed) cell:
//     - Build a fresh estimator and associator from the config's factories.
//     - Construct TrackManager(min_misses, max_misses) and
//       Tracker(*estimator, *associator, manager, init_gate_m).
//     - Generate the Scenario for the seed (replays ignore the seed).
//     - Drive runBench(...) to produce a BenchResult.
//     - computeMetrics(...) -> MetricsResult.
//     - Emit one MetricRow per (config, scenario, seed, metric).
//
// Assumptions:
//   - scenarios[i]->descriptor().is_multi_seed tells whether to sweep
//     synthetic_seeds seeds (true) or a single seed=0 (false).
//   - Each call to config.build_*() returns a fresh instance, never
//     reused across runs (required for determinism).
//   - Tracker/TrackManager construction parameters are shared across all
//     configs at this stage; per-config Tracker tuning is out of scope.
//
// Rationale:
//   Single entry point keeps the bench/baseline_matrix executable in Task 15
//   trivial. Long-format output keeps the schema additive: a new metric
//   is a new value of `metric`, not a new column.
//
// Improve next:
//   - Parallel execution over (config, scenario, seed) cells; results
//     remain deterministic since each cell is independent.
//   - Per-config Tracker construction parameters when they need to vary.

#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/benchmark/Consistency.hpp"
#include "core/bias/SensorBiasPairExtractor.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {
namespace benchmark {

namespace {

const char* sensorName(SensorKind s) {
  switch (s) {
    case SensorKind::Unknown:  return "unknown";
    case SensorKind::Ais:      return "ais";
    case SensorKind::ArpaTtm:  return "arpattm";
    case SensorKind::ArpaTll:  return "arpatll";
    case SensorKind::EoIr:     return "eoir";
    case SensorKind::OwnShip:  return "ownship";
    case SensorKind::Lidar:    return "lidar";
    case SensorKind::Cooperative: return "cooperative";
  }
  return "unknown";
}

const char* modelName(MeasurementModel m) {
  switch (m) {
    case MeasurementModel::Position2D:         return "pos2d";
    case MeasurementModel::PositionVelocity2D: return "posvel2d";
    case MeasurementModel::RangeBearing2D:     return "rb2d";
    case MeasurementModel::Bearing2D:          return "b2d";
  }
  return "unknown";
}

// Compact source label "<sensor>_<model>[_<source_id>]". Used as a
// metric-name suffix so the long-format CSV stays key-value while
// still differentiating per-(SensorKind, MeasurementModel, source_id)
// breakdowns (per design spec §3 — granularity matches item 4's
// SourceKey).
std::string sourceLabel(const ConsistencySourceKey& k) {
  std::string s = sensorName(std::get<0>(k));
  s += "_";
  s += modelName(std::get<1>(k));
  const std::string& sid = std::get<2>(k);
  if (!sid.empty()) {
    s += "_";
    s += sid;
  }
  return s;
}

void emit(std::vector<MetricRow>& out,
          const SweepParams& p,
          const std::string& config,
          const std::string& scenario,
          std::uint64_t seed,
          const MetricsResult& m,
          const ConsistencyResult& c) {
  out.push_back({p.run_id, config, scenario, seed, "ospa_mean", m.ospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "ospa_p95", m.ospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_mean", m.gospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_p95", m.gospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_rms", m.gospa_rms, "m"});
  out.push_back({p.run_id, config, scenario, seed, "tgospa_raw", m.tgospa_raw_m, "m"});
  out.push_back({p.run_id, config, scenario, seed, "tgospa_smooth", m.tgospa_smooth_m, "m"});
  out.push_back({p.run_id, config, scenario, seed, "lifetime_ratio", m.lifetime_ratio, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "track_breaks", m.track_breaks, "count"});
  out.push_back({p.run_id, config, scenario, seed, "id_switches", m.id_switches, "count"});
  out.push_back({p.run_id, config, scenario, seed, "pos_rmse_m", m.pos_rmse_m, "m"});
  out.push_back({p.run_id, config, scenario, seed, "sog_rmse_mps", m.sog_rmse_mps, "m/s"});
  out.push_back({p.run_id, config, scenario, seed, "cog_rmse_deg", m.cog_rmse_deg, "deg"});
  // Per-truth-id breakdown of the kinematic + continuity metrics.
  // Suffix pattern mirrors NIS per-source: "<metric>:truth_<id>". This
  // makes it possible to see which target is dragging a scenario's
  // mean without re-running. truth_id is the canonical truth slot
  // identifier from the scenario; for replays it is the dataset's
  // ground-truth track id.
  for (const auto& [tid, pt] : m.per_truth) {
    const std::string sfx = ":truth_" + std::to_string(tid);
    out.push_back({p.run_id, config, scenario, seed,
                   "lifetime_ratio" + sfx, pt.lifetime_ratio, "ratio"});
    out.push_back({p.run_id, config, scenario, seed,
                   "track_breaks" + sfx, pt.track_breaks, "count"});
    out.push_back({p.run_id, config, scenario, seed,
                   "id_switches" + sfx, pt.id_switches, "count"});
    out.push_back({p.run_id, config, scenario, seed,
                   "pos_rmse_m" + sfx, pt.pos_rmse_m, "m"});
    out.push_back({p.run_id, config, scenario, seed,
                   "sog_rmse_mps" + sfx, pt.sog_rmse_mps, "m/s"});
    out.push_back({p.run_id, config, scenario, seed,
                   "cog_rmse_deg" + sfx, pt.cog_rmse_deg, "deg"});
    out.push_back({p.run_id, config, scenario, seed,
                   "rmse_n" + sfx, static_cast<double>(pt.rmse_n), "count"});
  }
  // NEES aggregate (position, Confirmed-only via BenchStep::tracks).
  out.push_back({p.run_id, config, scenario, seed, "nees_mean", c.nees.mean, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_median", c.nees.median, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_p95", c.nees.p95, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_p99", c.nees.p99, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_coverage_95", c.nees.coverage_95, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_beta_hat", c.nees.beta_hat, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "nees_n",
                 static_cast<double>(c.nees.n), "count"});
  out.push_back({p.run_id, config, scenario, seed, "nees_dropped_singular",
                 static_cast<double>(c.nees.dropped_singular), "count"});
  // Per-source NIS breakdown. One row per stat per source key — the
  // long-format schema absorbs any sensor mix without column churn.
  for (const auto& [key, s] : c.per_source) {
    const std::string sl = sourceLabel(key);
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_mean:" + sl, s.mean, "ratio"});
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_coverage_95:" + sl, s.coverage_95, "ratio"});
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_alpha_hat:" + sl, s.alpha_hat, "ratio"});
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_trace_ratio:" + sl, s.trace_ratio_HPH_over_R, "ratio"});
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_n:" + sl, static_cast<double>(s.n), "count"});
    out.push_back({p.run_id, config, scenario, seed,
                   "nis_dropped_singular:" + sl,
                   static_cast<double>(s.dropped_singular), "count"});
  }
}
}  // namespace

std::shared_ptr<ISensorDetectionModel> detectionModelFor(
    const ScenarioDescriptor& desc, const MhtTracker::Config& cfg,
    bool use_clutter_map) {
  if (desc.detection_table.empty()) return nullptr;
  auto model = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{cfg.probability_of_detection, cfg.clutter_density});
  for (const SensorDetectionEntry& e : desc.detection_table) {
    if (e.source_id.empty())
      model->set(e.sensor, e.model, e.params);
    else
      model->set(e.sensor, e.model, e.source_id, e.params);
  }
  if (use_clutter_map)
    return std::make_shared<ClutterMapSensorDetectionModel>(
        std::move(model), ClutterMapParams{});
  return model;
}

std::vector<MetricRow> runSweep(
    const std::vector<Config>& configs,
    const std::vector<std::unique_ptr<ScenarioRun>>& scenarios,
    const SweepParams& params) {
  std::vector<MetricRow> rows;
  // Loop order: scenario × seed × config. The Scenario for a given
  // (scenario, seed) is independent of which estimator/associator runs
  // on it, so we generate it once and replay-by-reference across every
  // config. This is essentially free for synthetic scenarios but a
  // ~9× win for the file-driven replays whose generate() reads CSVs
  // from disk and sorts them. Replays also ignore seed and are always
  // single-seed, so per-replay generate() runs exactly once total.
  // Skip a scenario entirely when generate() returns empty — this lets
  // ReplayScenarioRun absent-fixtures path turn into a graceful no-op
  // rather than throwing inside the inner loops.
  for (const auto& scenario_ptr : scenarios) {
    const auto desc = scenario_ptr->descriptor();
    const std::uint32_t seeds =
        desc.is_multi_seed ? params.synthetic_seeds : 1u;
    for (std::uint32_t seed = 0; seed < seeds; ++seed) {
      const Scenario scen = scenario_ptr->generate(seed);
      if (scen.measurements.empty()) continue;
      for (const auto& config : configs) {
        auto est = config.build_estimator();
        BenchResult result;
        NisCollector nis;
        std::map<std::uint64_t, std::vector<pmbm::TrajectoryPoint>>
            pmbm_smoothed_trajectories;
        if (config.tracker_kind == TrackerKind::Mht) {
          MhtTracker::Config cfg =
              config.mht_config ? config.mht_config() : MhtTracker::Config{};
          // The detection environment is a property of the scenario, not
          // the tracker config. Preferred: the scenario's per-sensor
          // detection table (correct units per sensor, coverage-
          // conditioned miss P_D). Legacy: a scenario-scalar clutter
          // density override when no table is declared.
          auto det = detectionModelFor(desc, cfg, config.use_clutter_map);
          if (!det) cfg.clutter_density = desc.clutter_density;
          MhtTracker tracker(*est, cfg, std::move(det));
          tracker.setInnovationSink(&nis);
          // Item 9: optional per-sensor registration bias estimator.
          // Pair extraction runs after each scan; observations feed the
          // estimator, whose published estimates are then applied to
          // subsequent scans' incoming measurements via the provider hook.
          std::shared_ptr<SensorBiasEstimator> bias_est;
          PostScanHook post_scan;
          if (config.build_sensor_bias_estimator) {
            bias_est = config.build_sensor_bias_estimator();
            // Per-scenario seeding (offline calibration hand-off):
            // e.g. AutoFerry env-2 cameras carry a known ~7° EO/IR
            // bias measured by tools/autoferry_r_calibration.py.
            // Default no-op for scenarios that don't override.
            scenario_ptr->seedSensorBiasEstimator(*bias_est);
            tracker.setSensorBiasProvider(bias_est.get());
            post_scan = [bias_est](const MhtTracker& t, Timestamp scan_t) {
              bias_est->predictTo(scan_t);
              const auto pos_pairs =
                  extractPositionPairs(t.tracks(), scan_t);
              for (const auto& p : pos_pairs) bias_est->observe(p);
              const auto brg_pairs =
                  extractBearingPairs(t.tracks(), scan_t);
              for (const auto& p : brg_pairs) bias_est->observe(p);
              // Item 13: cross-sensor anchored pairs on AIS-less tracks
              // (deployment scenario: non-cooperative targets). The
              // extractor skips tracks that the AIS-anchored path
              // already handled this cycle, so this is purely
              // additive. bias_est itself is the provider for the
              // Schmidt fold on the anchor side.
              const auto cross_pairs =
                  extractCrossSensorPositionPairs(t.tracks(), scan_t,
                                                  bias_est.get());
              for (const auto& p : cross_pairs) bias_est->observe(p);
            };
          }
          result = runBenchMht(scen, tracker, post_scan);
        } else if (config.tracker_kind == TrackerKind::Pmbm) {
          pmbm::PmbmTracker::Config cfg =
              config.pmbm_config ? config.pmbm_config()
                                  : pmbm::PmbmTracker::Config{};
          pmbm::PmbmTracker::BirthModelFn birth =
              config.pmbm_birth_model ? config.pmbm_birth_model()
                                       : pmbm::PmbmTracker::BirthModelFn{};
          // Per-sensor detection model from the scenario (same call as
          // the MHT path). When the scenario declares no table, the
          // helper returns nullptr and PMBM falls back to its Config
          // scalars.
          MhtTracker::Config carrier;
          carrier.probability_of_detection = cfg.probability_of_detection;
          carrier.clutter_density = cfg.clutter_intensity;
          auto det = detectionModelFor(desc, carrier, /*use_clutter_map=*/false);
          pmbm::PmbmTracker tracker(*est, cfg, std::move(birth));
          if (det) tracker.setSensorDetectionModel(det);

          // Same item-9 bias-estimator wiring as the MHT path:
          // per-cycle pair extraction (AIS-anchored position pairs +
          // bearing pairs + cross-sensor non-AIS pairs), bias-provider
          // applied to incoming measurements via setSensorBiasProvider.
          std::shared_ptr<SensorBiasEstimator> bias_est;
          PmbmPostScanHook pmbm_post_scan;
          if (config.build_sensor_bias_estimator) {
            bias_est = config.build_sensor_bias_estimator();
            scenario_ptr->seedSensorBiasEstimator(*bias_est);
            tracker.setSensorBiasProvider(bias_est.get());
            pmbm_post_scan =
                [bias_est](const pmbm::PmbmTracker& t, Timestamp scan_t) {
                  bias_est->predictTo(scan_t);
                  const auto pos_pairs =
                      extractPositionPairs(t.tracks(), scan_t);
                  for (const auto& p : pos_pairs) bias_est->observe(p);
                  const auto brg_pairs =
                      extractBearingPairs(t.tracks(), scan_t);
                  for (const auto& p : brg_pairs) bias_est->observe(p);
                  const auto cross_pairs =
                      extractCrossSensorPositionPairs(t.tracks(), scan_t,
                                                      bias_est.get());
                  for (const auto& p : cross_pairs) bias_est->observe(p);
                };
          }
          result = runBenchPmbm(scen, tracker, pmbm_post_scan);
          // Phase 6 measurement hook: drain + RTS-smooth the alive
          // Bernoulli trajectories. Empty when trajectory_window_scans
          // is 0; passed through to computeMetrics for tgospa_smooth.
          pmbm_smoothed_trajectories = tracker.collectSmoothedTrajectories();
        } else {
          // Per-sensor associator if the scenario has a table and the
          // config opts in; otherwise the scalar factory. Detection
          // model is built from the scenario the same way the MHT path
          // builds it. `det` must out-live the associator — JpdaAssociator
          // holds a raw pointer into it (matches the MhtTracker pattern,
          // where the tracker owns the shared_ptr).
          std::shared_ptr<ISensorDetectionModel> det;
          std::shared_ptr<IDataAssociator> asc;
          if (config.build_associator_per_sensor) {
            MhtTracker::Config carrier;
            det = detectionModelFor(desc, carrier, /*use_clutter_map=*/false);
            if (det) {
              asc = config.build_associator_per_sensor(det);
            } else {
              asc = config.build_associator();
            }
          } else {
            asc = config.build_associator();
          }
          TrackManager mgr(
              static_cast<int>(params.track_manager_min_misses),
              static_cast<int>(params.track_manager_max_misses));
          Tracker tracker(*est, *asc, mgr, params.tracker_init_gate_m);
          tracker.setInnovationSink(&nis);
          BenchSink sink;
          result = runBench(scen, tracker, mgr, sink);
        }
        const auto m = pmbm_smoothed_trajectories.empty()
            ? computeMetrics(result, params.metrics)
            : computeMetrics(result, params.metrics,
                             pmbm_smoothed_trajectories);
        const auto c =
            computeConsistency(nis, result, params.metrics.assoc_gate_m);
        emit(rows, params, config.label, desc.label,
             static_cast<std::uint64_t>(seed), m, c);
      }
    }
  }
  return rows;
}

}  // namespace benchmark
}  // namespace navtracker
