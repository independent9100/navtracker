#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
std::string nowUtcIso8601() {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::gmtime(&t);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string argv_str(int argc, char** argv, const std::string& flag) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) return argv[i + 1];
  }
  return {};
}

bool has_flag(int argc, char** argv, const std::string& flag) {
  for (int i = 1; i < argc; ++i) {
    if (flag == argv[i]) return true;
  }
  return false;
}
}  // namespace

int main(int argc, char** argv) {
  if (has_flag(argc, argv, "--help") || has_flag(argc, argv, "-h")) {
    std::cout
        << "Usage: navtracker_bench_baseline [--run-id ID] [--out DIR]\n"
           "                                 [--seeds N] [--skip-replays]\n"
           "                                 [--with-haxr]\n"
           "                                 [--config-filter SUBSTR]\n"
           "                                 [--scenario-filter SUBSTR]\n"
           "                                 [--config-eq LABEL]\n"
           "                                 [--scenario-eq LABEL]\n"
           "                                 [--fast-metrics]\n"
           "\n"
           "--fast-metrics skips per-cell accuracy/consistency scoring\n"
           "(OSPA/GOSPA/T-GOSPA/RMSE/NEES/NIS) and emits only wall_seconds +\n"
           "per-scan latency (scan_proc_ms_*, scan_interval_s, n_scans). The\n"
           "tracker still runs in full; use it to cut dev turnaround and to\n"
           "separate the harness scoring tax from tracker compute.\n"
           "\n"
           "Writes <out>/<run-id>.csv containing one row per\n"
           "(config x scenario x seed x metric) plus a provenance header.\n"
           "\n"
           "Replays (philos + AutoFerry x9) run by default; --skip-replays\n"
           "omits them. The haxr full radar-hour (302k plots, ~169/scan) is\n"
           "off by default because the full-enumeration JPDA / MHT configs\n"
           "are intractable on it without cluster decomposition; pass\n"
           "--with-haxr to include it (expect long runtime / high memory).\n"
           "\n"
           "--export-states-dir DIR writes, per run, the per-scan (truth,\n"
           "track) states and our per-scan GOSPA to DIR for the D2 Stone\n"
           "Soup metric cross-check (tools/stonesoup_gospa_crosscheck.py).\n"
           "Pair with --config-eq / --scenario-eq / --seeds 1 to export a\n"
           "single run.\n"
           "\n"
           "--config-filter / --scenario-filter restrict the matrix to\n"
           "entries whose label contains the given substring. Both filters\n"
           "compose. Use for focused re-measurement against an existing\n"
           "pinned baseline (e.g. --config-filter imm_cv_ct_mht\n"
           "--scenario-filter autoferry_scenario2).\n";
    return 0;
  }
  const std::string config_filter = argv_str(argc, argv, "--config-filter");
  const std::string scenario_filter = argv_str(argc, argv, "--scenario-filter");
  // Exact-label variants: substring filters catch siblings (e.g.
  // "imm_cv_ct_pmbm_occupancy" also matches "..._occupancy_sensitive"), which
  // silently multiplies compute on a focused A/B. --config-eq / --scenario-eq
  // match the label EXACTLY. When set, they take precedence over the substring
  // filter for that dimension.
  const std::string config_eq = argv_str(argc, argv, "--config-eq");
  const std::string scenario_eq = argv_str(argc, argv, "--scenario-eq");

  const std::string run_id =
      argv_str(argc, argv, "--run-id").empty()
          ? std::string("run_") + nowUtcIso8601()
          : argv_str(argc, argv, "--run-id");
  const std::string out_dir =
      argv_str(argc, argv, "--out").empty() ? std::string("docs/baselines/")
                                            : argv_str(argc, argv, "--out");
  const std::string seeds_arg = argv_str(argc, argv, "--seeds");
  const std::uint32_t synthetic_seeds =
      seeds_arg.empty() ? 10u
                        : static_cast<std::uint32_t>(std::stoul(seeds_arg));
  const bool skip_replays = has_flag(argc, argv, "--skip-replays");
  const bool with_haxr = has_flag(argc, argv, "--with-haxr");
  // Dev-loop knob: skip per-cell accuracy/consistency scoring (OSPA/GOSPA/
  // T-GOSPA/RMSE/NEES/NIS), emit only wall + per-scan latency. See SweepParams.
  const bool fast_metrics = has_flag(argc, argv, "--fast-metrics");

  const auto t0 = std::chrono::steady_clock::now();

  auto configs = defaultConfigs();
  auto sim_scenarios = defaultSimScenarios();
  std::vector<std::unique_ptr<ScenarioRun>> replay_scenarios;
  if (!skip_replays) {
    // philos + AutoFerry are tractable across the full config matrix. haxr
    // (302k plots / ~169 per scan) is gated behind --with-haxr: the
    // full-enumeration JPDA and MHT configs OOM on it absent cluster
    // decomposition (see docs/baselines/README.md).
    for (auto& s : defaultReplayScenarios()) {
      if (!with_haxr && s->descriptor().label == "haxr") continue;
      replay_scenarios.push_back(std::move(s));
    }
    auto autoferry = defaultAutoferryScenarios();
    for (auto& s : autoferry) replay_scenarios.push_back(std::move(s));
    // Item 9 option 1: AutoFerry scenarios with a synthetic
    // truth-derived AIS anchor injected so SensorBiasEstimator has
    // pair observations. Labelled "autoferry_scenarioN_anchored" so
    // they coexist with the canonical no-anchor scenarios in the
    // bench output and are comparable side-by-side.
    auto autoferry_anchored = defaultAutoferryScenariosAnchored();
    for (auto& s : autoferry_anchored)
      replay_scenarios.push_back(std::move(s));
  }

  std::vector<std::unique_ptr<ScenarioRun>> all;
  for (auto& s : sim_scenarios) all.push_back(std::move(s));
  for (auto& s : replay_scenarios) all.push_back(std::move(s));

  // Apply optional filters (substring match on labels). Lets the user
  // re-measure a small slice against an existing pinned baseline
  // without paying for the full 47-min matrix.
  if (!config_eq.empty()) {
    configs.erase(
        std::remove_if(configs.begin(), configs.end(),
                       [&](const Config& c) { return c.label != config_eq; }),
        configs.end());
  } else if (!config_filter.empty()) {
    configs.erase(
        std::remove_if(configs.begin(), configs.end(),
                       [&](const Config& c) {
                         return c.label.find(config_filter) == std::string::npos;
                       }),
        configs.end());
  }
  if (!scenario_eq.empty()) {
    all.erase(std::remove_if(all.begin(), all.end(),
                             [&](const std::unique_ptr<ScenarioRun>& s) {
                               return s->descriptor().label != scenario_eq;
                             }),
              all.end());
  } else if (!scenario_filter.empty()) {
    all.erase(
        std::remove_if(all.begin(), all.end(),
                       [&](const std::unique_ptr<ScenarioRun>& s) {
                         return s->descriptor().label.find(scenario_filter) ==
                                std::string::npos;
                       }),
        all.end());
  }
  std::cout << "Running " << configs.size() << " configs x " << all.size()
            << " scenarios\n";

  SweepParams sp;
  sp.run_id = run_id;
  sp.synthetic_seeds = synthetic_seeds;
  sp.fast_metrics = fast_metrics;
  const std::string export_states_dir =
      argv_str(argc, argv, "--export-states-dir");
  if (!export_states_dir.empty()) sp.export_states_dir = export_states_dir;
  const auto rows = runSweep(configs, all, sp);

  const auto t1 = std::chrono::steady_clock::now();

  CsvProvenance prov;
  prov.run_id = run_id;
  prov.started_at_utc = nowUtcIso8601();
  prov.git_sha = "unknown";  // wire in via CMake-generated header later
  prov.build_type = "Release";
  prov.compiler = "unknown";
  prov.host = "unknown";
  for (std::uint32_t s = 0; s < synthetic_seeds; ++s) prov.seeds.push_back(s);
  prov.config_count = static_cast<std::uint32_t>(configs.size());
  prov.scenario_count = static_cast<std::uint32_t>(all.size());
  prov.total_runs =
      static_cast<std::uint32_t>(configs.size()) *
      (static_cast<std::uint32_t>(sim_scenarios.size()) * synthetic_seeds +
       static_cast<std::uint32_t>(replay_scenarios.size()));
  prov.elapsed_seconds = std::chrono::duration<double>(t1 - t0).count();

  std::string out_path = out_dir;
  if (!out_path.empty() && out_path.back() != '/') out_path.push_back('/');
  out_path += run_id + ".csv";
  std::ofstream os(out_path);
  if (!os) {
    std::cerr << "Cannot open output file: " << out_path << "\n";
    return 1;
  }
  writeCsv(os, prov, rows);
  std::cout << "Wrote " << rows.size() << " rows to " << out_path << "\n";
  return 0;
}
