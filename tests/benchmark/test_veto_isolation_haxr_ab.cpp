// Corroboration-veto isolation A/B on the HAXR AIS arm (2026-07-09 ticket).
//
// Increment-8 validated the veto's WIRING on HAXR but could not measure its
// ISOLATED benefit: adding AIS turned the veto on AND changed the fusion input
// at once, so the effects entangled. The fix (this test): hold the AIS arm ON in
// BOTH runs and toggle ONLY `corroboration_veto_enabled` — a per-instance
// LiveOccupancyParams override, the same no-global-toggle A/B pattern as
// test_los_guard_haxr_ab.cpp. Three fixed shore stations (kattwyk / parkhafen /
// seemannshoeft, hour 08, decimated eps=50, common 285 s window), the increment-8
// metric set per site so the rows are comparable to that eval-log entry.
//
// This is a MEASUREMENT test: it prints the per-site ON/OFF table (the verdict
// numbers live in docs/baselines/2026-07-09_veto_isolation.md + the eval-log, not
// frozen as thresholds — #24). The veto's conservation INVARIANT (default ON == a
// fix lifts suppression to 0; OFF only raises suppression back, never orphans a
// birth) is proven fixed-input in
// LiveOccupancyModel.CorroborationVetoToggleDefaultOnReproducesVetoOffFallsThrough.
// The only asserts here are arm well-formedness (both arms produced the full
// metric set) — banded, no cross-config pin on the marginal occupancy metrics.
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

// RAII save/restore of an env var (so this test cannot perturb the other HAXR
// tests in the same binary, which key on the default kattwyk_08_t40 fixture).
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

struct Site {
  const char* station;
  std::string plots;  // decimated, common 285 s window
};

}  // namespace

TEST(VetoIsolationHaxrAB, VetoIsolatedOnAisArmThreeSites) {
  const std::vector<Site> sites = {
      {"kattwyk", srcAbs("tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv")},
      {"parkhafen",
       srcAbs("tests/fixtures/haxr_cfar/out/parkhafen_08_dec50_w285.csv")},
      {"seemannshoeft",
       srcAbs("tests/fixtures/haxr_cfar/out/seemannshoeft_08_dec50_w285.csv")}};
  const std::string stations = srcAbs("data/dlr/stations.csv");
  if (!fileExists(stations) || !fileExists(sites[0].plots))
    GTEST_SKIP() << "HAXR fixtures (local-only) absent";

  const auto all = defaultConfigs();
  const Config* cov =
      byLabel(all, "imm_cv_ct_pmbm_occupancy_detector_coverage");
  ASSERT_NE(cov, nullptr);
  ASSERT_TRUE(cov->live_occupancy_params.has_value());
  ASSERT_TRUE(cov->live_occupancy_params->corroboration_veto_enabled)
      << "the shipped config must have the veto ON (the default under test)";

  const char* METRICS[] = {"card_err_mean",      "gospa_mean",
                           "gospa_false",        "gospa_missed",
                           "lifetime_ratio",     "occ_peak_structures",
                           "occ_peak_persistence", "occ_suppress_hits"};

  std::cout << "\n=== veto-isolation HAXR A/B (AIS arm ON in BOTH; veto toggled) "
               "===\n    (kattwyk/parkhafen/seemannshoeft_08, dec50, 285 s window)\n";

  int sites_run = 0;
  for (const Site& s : sites) {
    const std::string ais =
        srcAbs(std::string("data/dlr/") + s.station + "_08-UTC.csv");
    if (!fileExists(s.plots) || !fileExists(ais)) {
      std::cout << "  [" << s.station << "] fixtures absent — skipped\n";
      continue;
    }
    // Point HaxrScenarioRun at this site's decimated plots + AIS, feed the AIS
    // arm (HAXR_FEED_AIS) so observeVesselFix fires and the veto engages. All
    // restored on scope exit; AIS is held CONSTANT across the two arms.
    ScopedEnv e_plots("HAXR_PLOTS_CSV", s.plots);
    ScopedEnv e_ais("HAXR_AIS_CSV", ais);
    ScopedEnv e_stations("HAXR_STATIONS_CSV", stations);
    ScopedEnv e_station("HAXR_STATION", s.station);
    ScopedEnv e_feed("HAXR_FEED_AIS", "1");

    std::vector<std::unique_ptr<ScenarioRun>> scen;
    for (auto& sc : defaultReplayScenarios())
      if (sc->descriptor().label == "haxr" &&
          !sc->generate(0).measurements.empty())
        scen.push_back(std::move(sc));
    if (scen.empty()) {
      std::cout << "  [" << s.station << "] haxr scenario did not generate — skipped\n";
      continue;
    }

    Config veto_on = *cov;
    veto_on.label = "veto_on";
    Config veto_off = *cov;
    veto_off.label = "veto_off";
    LiveOccupancyParams off_params = *veto_off.live_occupancy_params;
    off_params.corroboration_veto_enabled = false;  // the ONLY difference
    veto_off.live_occupancy_params = off_params;

    SweepParams params;
    params.run_id = std::string("veto_iso_haxr_") + s.station;
    params.synthetic_seeds = 1;  // replay is single-seed
    std::vector<Config> configs = {veto_off, veto_on};
    const auto rows = runSweep(configs, scen, params);
    ASSERT_FALSE(rows.empty()) << s.station;

    std::cout << "\n  --- " << s.station << " ---\n"
              << "  metric                 veto_OFF        veto_ON       delta(ON-OFF)\n";
    for (const char* m : METRICS) {
      const double off = meanMetric(rows, "veto_off", m);
      const double on = meanMetric(rows, "veto_on", m);
      std::cout << "  " << m;
      for (std::size_t k = std::string(m).size(); k < 21; ++k) std::cout << ' ';
      std::cout << off << "\t" << on << "\t" << (on - off) << "\n";
    }
    std::cout << std::flush;

    // Well-formedness only (both arms ran the tracker and scored a real scene) —
    // the comparative numbers are the deliverable, recorded in the baseline doc,
    // not frozen here (#24: no cross-config pin on the marginal occupancy metrics).
    EXPECT_GT(meanMetric(rows, "veto_off", "gospa_mean"), 0.0) << s.station;
    EXPECT_GT(meanMetric(rows, "veto_on", "gospa_mean"), 0.0) << s.station;
    ++sites_run;
  }
  std::cout << "\n  sites run: " << sites_run << " / " << sites.size() << "\n"
            << std::flush;
  EXPECT_GE(sites_run, 1) << "no HAXR site produced an A/B — fixtures/harness?";
}

}  // namespace benchmark
}  // namespace navtracker
