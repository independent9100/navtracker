#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/utsname.h>  // uname() for the host provenance field
#endif

#include "adapters/benchmark/RbadScenarioRun.hpp"
#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
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

// W6.3.2 baseline provenance: git sha (compile-time from CMake), compiler and
// host (compile/runtime). Best-effort; each returns "unknown" if unavailable.
std::string buildGitSha() {
#ifdef NAVTRACKER_GIT_SHA
  return NAVTRACKER_GIT_SHA;
#else
  return "unknown";
#endif
}

std::string buildCompiler() {
#if defined(__clang__)
  return "clang " + std::to_string(__clang_major__) + "." +
         std::to_string(__clang_minor__) + "." +
         std::to_string(__clang_patchlevel__);
#elif defined(__GNUC__)
  return "gcc " + std::to_string(__GNUC__) + "." +
         std::to_string(__GNUC_MINOR__) + "." +
         std::to_string(__GNUC_PATCHLEVEL__);
#else
  return "unknown";
#endif
}

std::string buildHost() {
#if defined(__unix__) || defined(__APPLE__)
  struct utsname uts;
  if (uname(&uts) == 0)
    return std::string(uts.sysname) + " " + uts.machine;  // e.g. "Linux x86_64"
#endif
  return "unknown";
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
           "                                 [--with-haxr] [--with-simms]\n"
           "                                 [--with-rbad]\n"
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
           "--with-simms adds the multi-sensor SIMULATION battery (radar+AIS\n"
           "+camera over seeded synthetic truth). Fixtures are local-only\n"
           "(tests/fixtures/sim_multisensor/, git-ignored) — generate them\n"
           "first (see that dir's README); absent fixtures are skipped. This\n"
           "is the first controlled fusion-accuracy gate (truth independent\n"
           "of every sensor by construction).\n"
           "\n"
           "--with-rbad adds the R-BAD berthing REPLAY battery (automotive-band\n"
           "mmWave FMCW, NOT marine X-band — a new sensor class, not a marine\n"
           "geography). Fixtures are local-only (tests/fixtures/rbad/,\n"
           "git-ignored); generate them first (see that dir's README); absent\n"
           "fixtures are skipped. The dataset has NO ego pose (fixed body frame)\n"
           "and its labels are the authors' own reference TRACKER, so every\n"
           "score is cross-tracker CONSISTENCY (vs_reference_tracker), NOT\n"
           "accuracy. Reality-check arm only — do not tune to it.\n"
           "\n"
           "--export-states-dir DIR writes, per run, the per-scan (truth,\n"
           "track) states and our per-scan GOSPA to DIR for the D2 Stone\n"
           "Soup metric cross-check (tools/stonesoup_gospa_crosscheck.py).\n"
           "Pair with --config-eq / --scenario-eq / --seeds 1 to export a\n"
           "single run.\n"
           "\n"
           "--export-pmbm-diag-dir DIR writes, per PMBM run, the per-scan\n"
           "MBM-internal diagnostics (per-identity existence mass, dominant-\n"
           "hyp r, claimed measurement, state divergence, prune/cap events)\n"
           "to DIR/<config>__<scenario>__seed<seed>.{pmbmscan,pmbmbern}.csv\n"
           "for backlog #25 close-pass localization (tools/pmbm_closepass_\n"
           "trace.py). Diagnostic-only; no effect on tracking or metrics.\n"
           "\n"
           "--config-filter / --scenario-filter restrict the matrix to\n"
           "entries whose label contains the given substring. Both filters\n"
           "compose. Use for focused re-measurement against an existing\n"
           "pinned baseline (e.g. --config-filter imm_cv_ct_mht\n"
           "--scenario-filter autoferry_scenario2).\n"
           "\n"
           "--no-bias-feed / --force-no-idle / --force-no-source-aware are\n"
           "F2-provenance-cycle path-isolation knobs (research tooling): they\n"
           "disable, respectively, the AIS-ARPA bias-feed loop (clears\n"
           "build_sensor_bias_estimator), idle_halflife existence decay, and the\n"
           "source_aware_misdetection miss gate on ALL configs. Applied to both\n"
           "A/B arms they attribute an ON-vs-OFF delta to a single tracking\n"
           "path; all three together must make the F2 fix byte-identical.\n";
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
  // Multi-sensor SIMULATION battery (seeded Python-generated fixtures under
  // tests/fixtures/sim_multisensor/). Off by default because the fixtures are
  // local-only (git-ignored) and must be generated first; pass --with-simms to
  // include them. This is the first controlled fusion-accuracy gate (truth is
  // independent of every sensor by construction). Absent fixtures => the
  // scenarios generate empty and are skipped by the Sweep.
  const bool with_simms = has_flag(argc, argv, "--with-simms");
  // Imazu 22 fixed-geometry encounter battery (seeded Python-generated fixtures
  // under tests/fixtures/sim_multisensor/imazu_NN_s0/, same class as --with-simms,
  // radar+AIS arm). Off by default (local-only fixtures); pass --with-imazu.
  // Absent fixtures => scenarios generate empty and are skipped by the Sweep.
  const bool with_imazu = has_flag(argc, argv, "--with-imazu");
  // R-BAD berthing REPLAY battery (automotive-band mmWave FMCW; local-only
  // fixtures under tests/fixtures/rbad/). Off by default; pass --with-rbad.
  // NEW SENSOR CLASS (not marine X-band), NO ego pose (fixed body frame), and
  // labels are the authors' reference tracker => cross-tracker CONSISTENCY only,
  // never a marine-radar accuracy number. Absent fixtures self-skip (empty
  // Scenario). See docs/algorithms/evaluation-log.md.
  const bool with_rbad = has_flag(argc, argv, "--with-rbad");
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
  // Multi-sensor simulation battery (opt-in via --with-simms; fixtures are
  // local-only). Each generate() self-skips (empty Scenario) when its fixture
  // dir is missing, so this is safe to enable without the fixtures present.
  if (with_simms) {
    for (auto& s : defaultSimMultisensorScenarios()) all.push_back(std::move(s));
  }
  // Imazu 22 fixed-geometry encounter battery (opt-in via --with-imazu;
  // local-only fixtures). Each generate() self-skips (empty Scenario) when its
  // fixture dir is missing, so this is safe to enable without the fixtures.
  if (with_imazu) {
    for (auto& s : defaultImazuScenarios()) all.push_back(std::move(s));
  }
  // R-BAD berthing replay battery (opt-in via --with-rbad; local-only fixtures).
  // Each generate() self-skips (empty Scenario) when its fixture dir is missing,
  // so this is safe to enable without the fixtures present.
  if (with_rbad) {
    for (auto& s : defaultRbadScenarios()) all.push_back(std::move(s));
  }

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
  // F2 provenance cycle — path-isolation knobs (research tooling, no effect
  // unless a flag is passed). The F2 source-touch fix changes ONLY
  // Track::recent_contributions, consumed by exactly three tracking paths:
  //   (a) source_aware_misdetection miss gate,
  //   (b) idle_halflife existence decay,
  //   (c) the AIS-ARPA bias-feed loop
  //       (build_sensor_bias_estimator -> extractPairs -> applyBiasCorrection).
  // Disabling a path on BOTH A/B arms removes it from the ON-vs-OFF delta, so
  // the delta is attributed per path. Disabling ALL THREE must make ON == OFF
  // byte-identical (the fix has no other reach) — the attribution sanity check.
  // See docs/superpowers/plans/2026-07-12-f2-provenance-cycle-ticket.md Q(a).
  const bool no_bias_feed = has_flag(argc, argv, "--no-bias-feed");
  const bool force_no_idle = has_flag(argc, argv, "--force-no-idle");
  const bool force_no_source_aware =
      has_flag(argc, argv, "--force-no-source-aware");
  if (no_bias_feed || force_no_idle || force_no_source_aware) {
    for (auto& c : configs) {
      if (no_bias_feed) c.build_sensor_bias_estimator = {};  // path (c) off
      if ((force_no_idle || force_no_source_aware) && c.pmbm_config) {
        auto orig = c.pmbm_config;
        c.pmbm_config = [orig, force_no_idle, force_no_source_aware]() {
          auto cfg = orig();
          if (force_no_idle) cfg.idle_halflife_sec = 0.0;           // path (b)
          if (force_no_source_aware)
            cfg.source_aware_misdetection = false;                  // path (a)
          return cfg;
        };
      }
    }
    std::cout << "[path-isolation] no_bias_feed=" << no_bias_feed
              << " force_no_idle=" << force_no_idle
              << " force_no_source_aware=" << force_no_source_aware << "\n";
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
  const std::string export_pmbm_diag_dir =
      argv_str(argc, argv, "--export-pmbm-diag-dir");
  if (!export_pmbm_diag_dir.empty())
    sp.export_pmbm_diag_dir = export_pmbm_diag_dir;
  const auto rows = runSweep(configs, all, sp);

  const auto t1 = std::chrono::steady_clock::now();

  CsvProvenance prov;
  prov.run_id = run_id;
  prov.started_at_utc = nowUtcIso8601();
  prov.git_sha = buildGitSha();  // W6.3.2: from CMake at configure time
  prov.build_type = "Release";
  prov.compiler = buildCompiler();
  prov.host = buildHost();
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
