// LOS/shadow-guard delta on the HAXR occupancy arm (ticket proof obligation #4).
//
// The guard ships ON in imm_cv_ct_pmbm_occupancy_detector_coverage. HAXR is a
// FIXED shore radar station (kattwyk_08), so the ticket's expectation is that the
// guard is near-inert there and does NOT regress the increment-8 occupancy A/B —
// "say what you find". This runs that config on the decimated kattwyk_08 window
// with the guard ON vs OFF (an occ-params override — the same no-global-toggle
// A/B pattern as test_occupancy_ab.cpp) and reports the delta, asserting no
// material regression. Skip-guarded on the local-only HAXR fixtures.
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {
namespace {

double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  return n ? sum / n : 0.0;
}

bool fileExists(const std::string& p) {
  std::ifstream f(p);
  return static_cast<bool>(f);
}

std::string srcAbs(const std::string& rel) {
  return std::string(NAVTRACKER_SOURCE_DIR) + "/" + rel;
}

const Config* byLabel(const std::vector<Config>& all, const std::string& l) {
  for (const auto& c : all)
    if (c.label == l) return &c;
  return nullptr;
}

// RAII save/restore of an env var so this test cannot perturb the other HAXR
// tests in the same binary (they key on the default kattwyk_08_t40 fixture).
struct ScopedEnv {
  std::string key;
  bool had;
  std::string old;
  ScopedEnv(const char* k, const std::string& v) : key(k) {
    const char* cur = std::getenv(k);
    had = (cur != nullptr);
    if (had) old = cur;
    setenv(k, v.c_str(), 1);
  }
  ~ScopedEnv() {
    if (had)
      setenv(key.c_str(), old.c_str(), 1);
    else
      unsetenv(key.c_str());
  }
};

}  // namespace

TEST(LosGuardHaxrAB, GuardIsNearInertOnFixedShoreStation) {
  const std::string plots =
      srcAbs("tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv");
  const std::string ais = srcAbs("data/dlr/kattwyk_08-UTC.csv");
  const std::string stations = srcAbs("data/dlr/stations.csv");
  if (!fileExists(plots) || !fileExists(ais) || !fileExists(stations))
    GTEST_SKIP() << "HAXR fixtures (local-only) absent";

  // Point HaxrScenarioRun at absolute decimated paths (ctest cwd is build/, where
  // the compiled-in relative defaults do not resolve). Restored on scope exit.
  ScopedEnv e_plots("HAXR_PLOTS_CSV", plots);
  ScopedEnv e_ais("HAXR_AIS_CSV", ais);
  ScopedEnv e_stations("HAXR_STATIONS_CSV", stations);

  const auto all = defaultConfigs();
  const Config* cov =
      byLabel(all, "imm_cv_ct_pmbm_occupancy_detector_coverage");
  ASSERT_NE(cov, nullptr);
  ASSERT_TRUE(cov->live_occupancy_params.has_value());
  ASSERT_TRUE(cov->live_occupancy_params->shadow_guard.enabled)
      << "the coverage-decay config must ship with the guard ON";

  Config guard_on = *cov;
  guard_on.label = "haxr_guard_on";
  Config guard_off = *cov;
  guard_off.label = "haxr_guard_off";
  LiveOccupancyParams off_params = *guard_off.live_occupancy_params;
  off_params.shadow_guard.enabled = false;
  guard_off.live_occupancy_params = off_params;

  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultReplayScenarios())
    if (s->descriptor().label == "haxr" &&
        !s->generate(0).measurements.empty())
      scen.push_back(std::move(s));
  if (scen.empty()) GTEST_SKIP() << "haxr scenario did not generate";

  SweepParams params;
  params.run_id = "los_guard_haxr_ab";
  params.synthetic_seeds = 1;  // replay is single-seed
  std::vector<Config> configs = {guard_off, guard_on};
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  const char* OFF = "haxr_guard_off";
  const char* ON = "haxr_guard_on";
  const double ce_off = meanMetric(rows, OFF, "card_err_mean");
  const double ce_on = meanMetric(rows, ON, "card_err_mean");
  const double g_off = meanMetric(rows, OFF, "gospa_mean");
  const double g_on = meanMetric(rows, ON, "gospa_mean");

  std::cout << "\n=== LOS-guard HAXR delta (kattwyk_08 decimated, fixed shore) ==="
            << "\n  metric                guard_OFF     guard_ON\n";
  for (const char* m : {"card_err_mean", "gospa_mean", "gospa_false",
                        "gospa_missed", "lifetime_ratio", "occ_peak_structures",
                        "occ_peak_persistence", "occ_suppress_hits"}) {
    std::cout << "  " << m;
    for (std::size_t k = std::string(m).size(); k < 20; ++k) std::cout << ' ';
    std::cout << "  " << meanMetric(rows, OFF, m) << "\t" << meanMetric(rows, ON, m)
              << "\n";
  }
  std::cout << std::flush;

  // No material regression from the guard (the increment-8 A/B must hold). On a
  // fixed shore station occlusions are rare, so the guard should be near-inert;
  // we allow a small band and assert the guard does not WORSEN accuracy.
  EXPECT_LE(ce_on, ce_off + 0.5)
      << "guard regressed HAXR card_err: " << ce_off << " -> " << ce_on;
  EXPECT_LE(g_on, g_off * 1.02 + 1.0)
      << "guard regressed HAXR gospa: " << g_off << " -> " << g_on;
}

}  // namespace benchmark
}  // namespace navtracker
