#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

/**
 * One long-format metric record — the sweep's output unit. Identifies a
 * (run, config, scenario, seed) cell and carries a single named metric's
 * value + unit, so results serialize directly to one CSV row each.
 */
struct MetricRow {
  std::string run_id;
  std::string config;
  std::string scenario;
  std::uint64_t seed;
  std::string metric;
  double value;
  std::string unit;
};

/**
 * Run-wide parameters for a sweep: the run id, metric cutoffs, synthetic seed
 * count, and the Tracker / TrackManager construction knobs shared across all
 * configs.
 */
struct SweepParams {
  std::string run_id;
  MetricsParams metrics;
  std::uint32_t synthetic_seeds{10};
  // Tracker / TrackManager construction parameters — keep at the values
  // that match tests/scenario/test_crossing.cpp until a future task
  // makes them per-config.
  std::uint32_t track_manager_min_misses{2};
  std::uint32_t track_manager_max_misses{4};
  double tracker_init_gate_m{30.0};
  // D2 GOSPA cross-validation hook. When set, each run additionally writes
  // its per-scan (truth, track) states and our per-scan GOSPA to
  // <dir>/<config>__<scenario>__seed<seed>.{states,ours_gospa}.csv, for an
  // independent scorer (Stone Soup) to re-score the exact tracks the harness
  // scores. Unset (default) → no files, bit-identical behaviour. Test/tool
  // knob only — see core/benchmark/GospaExport.hpp.
  std::optional<std::string> export_states_dir;
};

/**
 * Build the per-sensor detection model a scenario declares, or null when
 * the scenario has no detection table (legacy scalar-clutter path).
 * Defaults (unlisted sensors) come from the tracker config's global
 * (P_D, λ_C) pair. With use_clutter_map the fixed table is wrapped in a
 * ClutterMapSensorDetectionModel (spatially-varying λ_C, backlog item 5);
 * the wrap is transparent until observations arrive.
 */
std::shared_ptr<ISensorDetectionModel> detectionModelFor(
    const ScenarioDescriptor& desc, const MhtTracker::Config& cfg,
    bool use_clutter_map = false);

/** Run every (config × scenario × seed) cell and collect the metric rows. */
std::vector<MetricRow> runSweep(
    const std::vector<Config>& configs,
    const std::vector<std::unique_ptr<ScenarioRun>>& scenarios,
    const SweepParams& params);

}  // namespace benchmark
}  // namespace navtracker
