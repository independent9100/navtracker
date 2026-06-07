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
           "\n"
           "Writes <out>/<run-id>.csv containing one row per\n"
           "(config x scenario x seed x metric) plus a provenance header.\n";
    return 0;
  }

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

  const auto t0 = std::chrono::steady_clock::now();

  auto configs = defaultConfigs();
  auto sim_scenarios = defaultSimScenarios();
  std::vector<std::unique_ptr<ScenarioRun>> replay_scenarios;
  if (!skip_replays) replay_scenarios = defaultReplayScenarios();

  std::vector<std::unique_ptr<ScenarioRun>> all;
  for (auto& s : sim_scenarios) all.push_back(std::move(s));
  for (auto& s : replay_scenarios) all.push_back(std::move(s));

  SweepParams sp;
  sp.run_id = run_id;
  sp.synthetic_seeds = synthetic_seeds;
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
