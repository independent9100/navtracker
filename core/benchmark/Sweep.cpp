#include "core/benchmark/Sweep.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

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

#include "adapters/land/GeoJsonCoastline.hpp"
#include "adapters/static/GeoJsonStaticObstacles.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/benchmark/Consistency.hpp"
#include "core/benchmark/GospaExport.hpp"
#include "core/benchmark/PmbmDiagRecorder.hpp"
#include "core/bias/SensorBiasPairExtractor.hpp"
#include "core/land/CoastlineModel.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/sensor_activity/DeclaredSensorActivity.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/tracking/TrackManager.hpp"
#include "ports/ISensorActivity.hpp"

namespace navtracker {
namespace benchmark {

namespace {

// Cl-4 endgame cliff-reprice sweep (2026-07-11): the SECOND dial. Vary the
// shoreline clutter-ramp half-widths for the GeoJSON land configs WITHOUT a
// recompile, byte-identical when unset — the same sanctioned A1 env-sweep
// method as PMBM_MIN_NEW_BERN in Config.cpp. NOT a deployment surface: no
// default is touched (CoastlinePriorParams keeps its 50/50 defaults), so the
// determinism test and the standing suite stay green with these vars unset.
// PMBM_OFFSHORE_HALFWIDTH_M narrows the blocked offshore strip W_off (the
// user's 2026-07-11 spatial-split directive); PMBM_INLAND_HALFWIDTH_M is the
// inland companion, kept for symmetry (default unchanged). The ramp math in
// CoastlineGeometry is untouched — only its two half-width params move.
CoastlinePriorParams sweepCoastlineParams() {
  CoastlinePriorParams p{};  // shipped defaults: inland=offshore=50 m
  if (const char* v = std::getenv("PMBM_OFFSHORE_HALFWIDTH_M"); v && *v)
    p.offshore_halfwidth_m = std::strtod(v, nullptr);
  if (const char* v = std::getenv("PMBM_INLAND_HALFWIDTH_M"); v && *v)
    p.inland_halfwidth_m = std::strtod(v, nullptr);
  return p;
}

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
    case SensorKind::RemoteTrack: return "remote_track";
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
          const ConsistencyResult& c,
          double wall_seconds) {
  out.push_back({p.run_id, config, scenario, seed, "wall_seconds", wall_seconds, "s"});
  out.push_back({p.run_id, config, scenario, seed, "ospa_mean", m.ospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "ospa_p95", m.ospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_mean", m.gospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_p95", m.gospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_rms", m.gospa_rms, "m"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_localization", m.gospa_localization, "m2"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_missed", m.gospa_missed, "m2"});
  out.push_back({p.run_id, config, scenario, seed, "gospa_false", m.gospa_false, "m2"});
  out.push_back({p.run_id, config, scenario, seed, "card_err_mean", m.card_err_mean, "tracks"});
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
  // (per-scan latency rows are emitted separately by emitScanTiming — they
  //  need the BenchResult, which is not threaded into this metric emitter.)
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

// Per-scan latency emitter (perf round 2). Turns the raw per-scan
// processing times captured by BenchRunner into the operator-facing
// realtime statistic: mean / p95 / p99 / max scan-processing time (ms)
// plus the median scan interval (s) and scan count. Live operation cares
// about the WORST scan, not the replay mean, so max/p99 are first-class.
// Purely additive rows — no effect on any accuracy metric.
void emitScanTiming(std::vector<MetricRow>& out, const SweepParams& p,
                    const std::string& config, const std::string& scenario,
                    std::uint64_t seed, const BenchResult& result) {
  const auto& sps = result.scan_process_seconds;
  if (sps.empty()) return;
  double sum = 0.0;
  for (double s : sps) sum += s;
  const double mean_ms = 1000.0 * sum / static_cast<double>(sps.size());
  out.push_back({p.run_id, config, scenario, seed, "scan_proc_ms_mean", mean_ms, "ms"});
  out.push_back({p.run_id, config, scenario, seed, "scan_proc_ms_p95",
                 1000.0 * percentile(sps, 0.95), "ms"});
  out.push_back({p.run_id, config, scenario, seed, "scan_proc_ms_p99",
                 1000.0 * percentile(sps, 0.99), "ms"});
  out.push_back({p.run_id, config, scenario, seed, "scan_proc_ms_max",
                 1000.0 * percentile(sps, 1.0), "ms"});
  // Scan interval: median gap between consecutive scan data-timestamps.
  // Robust to leading/trailing gaps in the feed; the honest "one scan
  // worth of wall time" that max scan-processing must fit inside for the
  // tracker to keep up in live operation.
  std::vector<double> dt;
  const auto& ts = result.scan_time_sec;
  for (std::size_t i = 1; i < ts.size(); ++i) {
    const double d = ts[i] - ts[i - 1];
    if (d > 0.0) dt.push_back(d);
  }
  const double interval_s = dt.empty() ? 0.0 : percentile(dt, 0.5);
  out.push_back({p.run_id, config, scenario, seed, "scan_interval_s", interval_s, "s"});
  out.push_back({p.run_id, config, scenario, seed, "n_scans",
                 static_cast<double>(sps.size()), "count"});
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
        // Stage 1b-i occupancy introspection: negative sentinel = layer not
        // wired for this (config, scenario); populated inside the PMBM branch
        // when a LiveOccupancyModel is constructed, emitted after the standard
        // metrics. These are truth-independent mechanism observations (did the
        // birth suppressor fire at all?), safe from any ground-truth gaming.
        double occ_peak_structures = -1.0;
        double occ_peak_persistence = -1.0;
        double occ_suppress_hits = -1.0;
        // Increment 5 (conservation/presence at bench): per-truth-id, is that
        // truth's final position covered by an emitted live-occupancy hazard's
        // keep-clear ring? 1 = emitted as a static hazard, 0 = not. Combined
        // with lifetime_ratio:truth_<id> the gate asserts presence over
        // classification — a suppressed truth object is a track OR a hazard,
        // never neither. Truth-independent of GOSPA scoring (pure geometry).
        std::map<std::uint64_t, double> occ_truth_in_hazard;
        const auto t_cell0 = std::chrono::steady_clock::now();
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
          auto det = detectionModelFor(desc, carrier, config.use_clutter_map);
          pmbm::PmbmTracker tracker(*est, cfg, std::move(birth));
          if (det) tracker.setSensorDetectionModel(det);

          // Backlog #25 PMBM localization: wire the per-scan diagnostic
          // recorder when a diag dir is set. Kept alive until after the
          // synchronous runBenchPmbm call below (the tracker holds only a raw
          // pointer). Unset → no sink → byte-identical.
          std::unique_ptr<PmbmDiagRecorder> pmbm_diag_rec;
          if (params.export_pmbm_diag_dir) {
            std::string base = *params.export_pmbm_diag_dir;
            if (!base.empty() && base.back() != '/') base.push_back('/');
            base += config.label + "__" + desc.label + "__seed" +
                    std::to_string(seed);
            pmbm_diag_rec = std::make_unique<PmbmDiagRecorder>(base);
            tracker.setDiagnosticSink(pmbm_diag_rec.get());
          }

          // Task 4: build a DeclaredSensorActivity from the scenario's
          // per-sensor detection table and wire it into the PMBM tracker.
          // The shared_ptr is kept alive until after the synchronous
          // runBenchPmbm call below (the tracker holds only a raw pointer).
          // Declared per-sensor cadence constants (decision §9a); tunable —
          // see spec roadmap §13.1 adaptive provider.
          constexpr double kArpaDutyCycleSec     = 2.5;   // typical radar rotation period
          constexpr double kEoIrDutyCycleSec     = 1.0;   // EO/IR frame period
          constexpr double kLidarDutyCycleSec    = 0.1;   // lidar scan period
          constexpr double kAisReportIntervalSec = 10.0;  // AIS/cooperative broadcast interval
          std::shared_ptr<DeclaredSensorActivity> activity;
          if (config.use_sensor_activity_model && !desc.detection_table.empty()) {
            std::vector<DeclaredSensorActivity::ChannelProfile> profiles;
            for (const auto& e : desc.detection_table) {
              DeclaredSensorActivity::ChannelProfile prof;
              prof.sensor = e.sensor;
              switch (e.sensor) {
                case SensorKind::ArpaTtm:
                case SensorKind::EoIr:
                case SensorKind::Lidar:
                  prof.kind            = ChannelKind::Surveillance;
                  prof.max_range_m     = e.params.max_range_m;
                  prof.sector_center_rad = e.params.sector_center_rad;
                  prof.sector_width_rad  = e.params.sector_width_rad;
                  prof.p_D             = e.params.probability_of_detection;
                  prof.duty_cycle_sec  =
                      (e.sensor == SensorKind::ArpaTtm) ? kArpaDutyCycleSec :
                      (e.sensor == SensorKind::EoIr)    ? kEoIrDutyCycleSec :
                                                           kLidarDutyCycleSec;
                  break;
                case SensorKind::Ais:
                case SensorKind::Cooperative:
                  prof.kind                      = ChannelKind::Cooperative;
                  prof.expected_report_interval_sec = kAisReportIntervalSec;
                  break;
                default:
                  continue;  // skip Unknown, OwnShip, ArpaTll
              }
              profiles.push_back(prof);
            }
            activity = std::make_shared<DeclaredSensorActivity>(std::move(profiles));
            tracker.setSensorActivity(activity.get());
          }

          // Task 6 (land) + Task E (synthetic): build and wire a CoastlineModel
          // when the config opts in. Prefer an in-memory synthetic coastline
          // from the ScenarioRun (synthetic shore-clutter scenarios); otherwise
          // fall back to a GeoJSON fixture path (real-data replays). The
          // shared_ptr outlives the synchronous runBenchPmbm call below (the
          // tracker holds only a raw pointer). The datum is fixed for the whole
          // run, so no datum-sink registration is needed.
          std::shared_ptr<CoastlineModel> land;
          if (config.use_land_model && scen.datum.has_value()) {
            std::optional<CoastlineGeometry> synth =
                scenario_ptr->syntheticCoastline();
            if (synth.has_value()) {
              land = std::make_shared<CoastlineModel>(std::move(*synth),
                                                      *scen.datum);
              tracker.setLandModel(land.get());
            } else if (!desc.coastline_geojson_path.empty()) {
              std::ifstream probe(desc.coastline_geojson_path);
              if (probe.good()) {
                try {
                  auto geom = loadCoastlineGeoJson(desc.coastline_geojson_path,
                                                   sweepCoastlineParams());
                  land = std::make_shared<CoastlineModel>(std::move(geom),
                                                          *scen.datum);
                  tracker.setLandModel(land.get());
                } catch (const std::exception&) {
                  // GeoJSON parse failure — proceed without land model
                }
              }
            }
          }

          // Stage 1 static-obstacle model (ADR 0002), same lifetime/datum
          // rules as the land model above. Prefer in-memory synthetic
          // obstacles; else a GeoJSON fixture path. Null → bit-identical.
          // The shared_ptr outlives the synchronous runBenchPmbm call below
          // (the tracker holds only a raw pointer). The datum is fixed for the
          // whole run, so no datum-sink registration is needed.
          std::shared_ptr<StaticObstacleModel> obstacles;
          if (config.use_static_obstacle_model && scen.datum.has_value()) {
            std::optional<std::vector<StaticObstacle>> synth =
                scenario_ptr->syntheticObstacles();
            if (synth.has_value()) {
              obstacles = std::make_shared<StaticObstacleModel>(
                  std::move(*synth), *scen.datum);
              tracker.setStaticObstacleModel(obstacles.get());
            } else if (!desc.static_obstacles_geojson_path.empty()) {
              std::ifstream probe(desc.static_obstacles_geojson_path);
              if (probe.good()) {
                try {
                  auto obs = loadStaticObstaclesGeoJson(
                      desc.static_obstacles_geojson_path);
                  obstacles = std::make_shared<StaticObstacleModel>(
                      std::move(obs), *scen.datum);
                  tracker.setStaticObstacleModel(obstacles.get());
                } catch (const std::exception&) {
                  // GeoJSON parse failure — proceed without obstacles.
                }
              }
            }
          }

          // Stage 1b live occupancy layer: one object wired both as the
          // birth-suppression model and as the per-scan occupancy feed. Fixed
          // datum per run → no datum-sink registration needed (as above). The
          // shared_ptr outlives the synchronous runBenchPmbm call. Null (no
          // datum) → identical to the base config.
          std::shared_ptr<LiveOccupancyModel> occupancy;
          if (config.use_live_occupancy_model && scen.datum.has_value()) {
            auto occ_params =
                config.live_occupancy_params.value_or(LiveOccupancyParams{});
            // Detector mode: enable the clutter-adaptive persistence bar. The
            // clutter background is estimated from the feed (median live-cell
            // persistence), so no external λ_C is passed — robust on real data.
            if (config.occupancy_adaptive_clutter_bar)
              occ_params.clutter_adaptive = true;
            occupancy =
                std::make_shared<LiveOccupancyModel>(*scen.datum, occ_params);
            tracker.setStaticObstacleModel(occupancy.get());
            tracker.setLiveOccupancyFeed(occupancy.get());
          }

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
          if (occupancy) {
            occ_peak_structures =
                static_cast<double>(occupancy->peakStructureCount());
            occ_peak_persistence = occupancy->peakPersistence();
            occ_suppress_hits =
                static_cast<double>(occupancy->suppressionHits());
            // Presence check: cover each truth's final position against the
            // emitted hazard set. Fixed datum per run ⇒ obstacle geodetic →
            // ENU about scen.datum matches the model's anchor frame exactly.
            if (scen.datum.has_value()) {
              std::map<std::uint64_t, Eigen::Vector2d> last_pos;
              for (const auto& ts : scen.truth) last_pos[ts.truth_id] = ts.position;
              std::vector<std::pair<Eigen::Vector2d, double>> rings;
              for (const auto& o : occupancy->obstacles()) {
                const Eigen::Vector3d e = scen.datum->toEnu(o.position);
                rings.emplace_back(Eigen::Vector2d(e.x(), e.y()),
                                   o.keep_clear_radius_m);
              }
              for (const auto& [tid, pos] : last_pos) {
                double in = 0.0;
                for (const auto& r : rings)
                  if ((pos - r.first).norm() <= r.second) { in = 1.0; break; }
                occ_truth_in_hazard[tid] = in;
              }
            }
          }
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
        // For PMBM runs (even when no tracks were emitted) call the
        // smoothed overload so tgospa_smooth_m is computed against
        // the (possibly empty) smoothed trajectories — empty est
        // produces a genuine cardinality penalty rather than the
        // default-0 sentinel from the scalar overload.
        const auto t_cell1 = std::chrono::steady_clock::now();
        const double wall_seconds =
            std::chrono::duration<double>(t_cell1 - t_cell0).count();
        if (params.fast_metrics) {
          // Dev-loop path: skip the accuracy/consistency scoring (its own
          // Hungarian passes over truth×track), emit only wall + latency.
          rows.push_back({params.run_id, config.label, desc.label,
                          static_cast<std::uint64_t>(seed), "wall_seconds",
                          wall_seconds, "s"});
        } else {
          const auto m = (config.tracker_kind == TrackerKind::Pmbm)
              ? computeMetrics(result, params.metrics,
                               pmbm_smoothed_trajectories)
              : computeMetrics(result, params.metrics);
          const auto c =
              computeConsistency(nis, result, params.metrics.assoc_gate_m);
          emit(rows, params, config.label, desc.label,
               static_cast<std::uint64_t>(seed), m, c, wall_seconds);
        }
        emitScanTiming(rows, params, config.label, desc.label,
                       static_cast<std::uint64_t>(seed), result);
        // D2 GOSPA cross-validation export (inert unless a dir is set). Dump
        // the exact BenchResult the metrics just consumed so an external
        // scorer re-scores identical tracks. See GospaExport.hpp.
        if (params.export_states_dir) {
          std::string base = *params.export_states_dir;
          if (!base.empty() && base.back() != '/') base.push_back('/');
          base += config.label + "__" + desc.label + "__seed" +
                  std::to_string(seed);
          writeBenchStatesCsv(result, base + ".states.csv");
          writeOurGospaCsv(result, params.metrics.gospa_cutoff_m,
                           base + ".ours_gospa.csv");
        }
        if (occ_peak_structures >= 0.0) {
          const auto s64 = static_cast<std::uint64_t>(seed);
          rows.push_back({params.run_id, config.label, desc.label, s64,
                          "occ_peak_structures", occ_peak_structures, "count"});
          rows.push_back({params.run_id, config.label, desc.label, s64,
                          "occ_peak_persistence", occ_peak_persistence, "ratio"});
          rows.push_back({params.run_id, config.label, desc.label, s64,
                          "occ_suppress_hits", occ_suppress_hits, "count"});
          for (const auto& [tid, in] : occ_truth_in_hazard) {
            rows.push_back({params.run_id, config.label, desc.label, s64,
                            "occ_truth_in_hazard:truth_" + std::to_string(tid),
                            in, "bool"});
          }
        }
      }
    }
  }
  return rows;
}

}  // namespace benchmark
}  // namespace navtracker
