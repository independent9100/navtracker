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
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {
namespace benchmark {

namespace {
void emit(std::vector<MetricRow>& out,
          const SweepParams& p,
          const std::string& config,
          const std::string& scenario,
          std::uint64_t seed,
          const MetricsResult& m) {
  out.push_back({p.run_id, config, scenario, seed, "ospa_mean", m.ospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "ospa_p95", m.ospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "lifetime_ratio", m.lifetime_ratio, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "track_breaks", m.track_breaks, "count"});
  out.push_back({p.run_id, config, scenario, seed, "id_switches", m.id_switches, "count"});
  out.push_back({p.run_id, config, scenario, seed, "pos_rmse_m", m.pos_rmse_m, "m"});
  out.push_back({p.run_id, config, scenario, seed, "sog_rmse_mps", m.sog_rmse_mps, "m/s"});
  out.push_back({p.run_id, config, scenario, seed, "cog_rmse_deg", m.cog_rmse_deg, "deg"});
}
}  // namespace

std::vector<MetricRow> runSweep(
    const std::vector<Config>& configs,
    const std::vector<std::unique_ptr<ScenarioRun>>& scenarios,
    const SweepParams& params) {
  std::vector<MetricRow> rows;
  for (const auto& config : configs) {
    for (const auto& scenario_ptr : scenarios) {
      const auto desc = scenario_ptr->descriptor();
      const std::uint32_t seeds =
          desc.is_multi_seed ? params.synthetic_seeds : 1u;
      for (std::uint32_t seed = 0; seed < seeds; ++seed) {
        const Scenario scen = scenario_ptr->generate(seed);
        auto est = config.build_estimator();
        auto asc = config.build_associator();
        TrackManager mgr(static_cast<int>(params.track_manager_min_misses),
                         static_cast<int>(params.track_manager_max_misses));
        Tracker tracker(*est, *asc, mgr, params.tracker_init_gate_m);
        BenchSink sink;
        const auto result = runBench(scen, tracker, mgr, sink);
        const auto m = computeMetrics(result, params.metrics);
        emit(rows, params, config.label, desc.label,
             static_cast<std::uint64_t>(seed), m);
      }
    }
  }
  return rows;
}

}  // namespace benchmark
}  // namespace navtracker
