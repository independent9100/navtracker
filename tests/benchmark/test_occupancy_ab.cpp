// Stage 1b-i decider measurement: does the SAFE birth-only live-occupancy layer
// actually suppress phantom structure — and in which regime?
//
//   A = imm_cv_ct_pmbm_land        (coastal baseline, no occupancy layer)
//   B = imm_cv_ct_pmbm_occupancy   (A + LiveOccupancyModel, birth-channel only,
//                                    NO lambda_C coupling)
//
// Scenarios span the regime axis the 2026-07-03 eval-log finding turns on:
//   - harbor_complete_truth        pier P_D 0.9 → cohort confirms once, never
//                                  dies → birth suppression structurally inert
//                                  (reproduces the "inert" finding);
//   - harbor_complete_truth_churn  pier P_D 0.4 → phantoms decay + must re-birth
//                                  → the ONLY regime a birth suppressor can act;
//                                  complete truth, so card_err/lifetime are
//                                  honestly scored → this is the DECIDER;
//   - philos                       real radar churn (per-scan P_D ~0.07).
//
// This is a MEASUREMENT, not a pass/fail gate (mirrors PhilosClutterMapAB). It
// prints every key metric for both arms plus the occupancy layer's
// truth-INDEPENDENT introspection (occ_peak_structures / occ_peak_persistence /
// occ_suppress_hits) — "did the mechanism fire at all?", which needs no ground
// truth and is therefore immune to the philos AIS-only-truth gaming risk.
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/static/LiveOccupancyModel.hpp"

using namespace navtracker::benchmark;

namespace {

double meanMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
                  const std::string& scen, const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == scen && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

bool hasMetric(const std::vector<MetricRow>& rows, const std::string& cfg,
               const std::string& scen, const std::string& metric) {
  for (const auto& r : rows)
    if (r.config == cfg && r.scenario == scen && r.metric == metric) return true;
  return false;
}

const Config* configByLabel(const std::vector<Config>& all,
                            const std::string& label) {
  for (const auto& c : all)
    if (c.label == label) return &c;
  return nullptr;
}

}  // namespace

TEST(OccupancyAB, BirthOnlySuppressionAcrossRegimes) {
  const auto all = defaultConfigs();
  const Config* land = configByLabel(all, "imm_cv_ct_pmbm_land");
  const Config* occ = configByLabel(all, "imm_cv_ct_pmbm_occupancy");
  const Config* occ_sens =
      configByLabel(all, "imm_cv_ct_pmbm_occupancy_sensitive");
  ASSERT_NE(land, nullptr);
  ASSERT_NE(occ, nullptr);
  ASSERT_NE(occ_sens, nullptr);

  // Third occupancy tuning built locally (not a shipped config): very coarse
  // 100 m cells + a 0.2 bar with the default alpha. Brackets the tuning space so
  // "no tuning classifies real philos structure" rests on three points spanning
  // 25 m/0.5 → 50 m/0.25 → 100 m/0.2, not one.
  Config occ_coarse = *occ;
  occ_coarse.label = "imm_cv_ct_pmbm_occupancy_coarse";
  navtracker::LiveOccupancyParams coarse;
  coarse.cell_size_m = 100.0;
  coarse.ewma_alpha = 0.3;
  coarse.persistence_bar = 0.2;
  coarse.extended_cells_min = 3;
  occ_coarse.live_occupancy_params = coarse;

  std::vector<Config> configs = {*land, *occ, *occ_sens, occ_coarse};

  // Sim scenarios by label (yardstick + churn decider + dense_clutter safety).
  const std::set<std::string> sim_labels = {"harbor_complete_truth",
                                             "harbor_complete_truth_churn",
                                             "dense_clutter"};
  std::vector<std::unique_ptr<ScenarioRun>> scen;
  for (auto& s : defaultSimScenarios())
    if (sim_labels.count(s->descriptor().label)) scen.push_back(std::move(s));
  ASSERT_EQ(scen.size(), sim_labels.size());

  // philos (real churn) — skip cleanly when the fixture is not reachable from
  // cwd (ctest runs from build/; run from the repo root to include it).
  bool philos_included = false;
  for (auto& s : defaultReplayScenarios()) {
    if (s->descriptor().label != "philos") continue;
    if (!s->generate(0).measurements.empty()) {
      scen.push_back(std::move(s));
      philos_included = true;
    }
  }
  if (!philos_included)
    std::cout << "[OccupancyAB] philos fixture not reachable from cwd — "
                 "sim-only measurement.\n";

  SweepParams params;
  params.run_id = "occupancy_ab";
  params.synthetic_seeds = 8;
  const auto rows = runSweep(configs, scen, params);
  ASSERT_FALSE(rows.empty());

  std::set<std::string> scenarios;
  for (const auto& r : rows) scenarios.insert(r.scenario);
  const char* A = "imm_cv_ct_pmbm_land";
  const char* B = "imm_cv_ct_pmbm_occupancy";
  const char* C = "imm_cv_ct_pmbm_occupancy_sensitive";
  const char* D = "imm_cv_ct_pmbm_occupancy_coarse";
  const std::vector<std::string> keys = {"card_err_mean", "gospa_mean",
                                         "gospa_false",   "gospa_missed",
                                         "lifetime_ratio"};

  std::cout << "\n=== occupancy A/B  (land | +occ birth-only | +occ_sensitive) ===\n";
  for (const auto& s : scenarios) {
    std::cout << "\n-- " << s << " --\n";
    for (const auto& m : keys) {
      const double a = meanMetric(rows, A, s, m);
      const double b = meanMetric(rows, B, s, m);
      const double c = meanMetric(rows, C, s, m);
      std::cout << "  " << m;
      for (std::size_t k = m.size(); k < 18; ++k) std::cout << ' ';
      std::cout << "  land=" << a << "\t+occ=" << b << "\t+occ_sens=" << c
                << "\n";
    }
    // Truth-independent mechanism observation: did the classifier fire? (default
    // occupancy | sensitive occupancy). occ_suppress_hits > 0 means a birth was
    // actually gated at classified structure.
    for (const char* cfg : {B, C, D}) {
      const std::string tag =
          (cfg == B) ? "occ" : (cfg == C) ? "occ_sens" : "occ_coarse";
      std::cout << "  [" << tag << "] structures="
                << meanMetric(rows, cfg, s, "occ_peak_structures")
                << "  persistence="
                << meanMetric(rows, cfg, s, "occ_peak_persistence")
                << "  suppress_hits="
                << meanMetric(rows, cfg, s, "occ_suppress_hits") << "\n";
    }
  }
  std::cout << std::flush;

  // Sanity (not the finding): the occupancy layer is wired for B on the harbor
  // scenarios (they carry a datum + a pier) and is NOT present for the land
  // baseline A. This guards the instrument itself.
  EXPECT_TRUE(hasMetric(rows, B, "harbor_complete_truth", "occ_peak_structures"));
  EXPECT_FALSE(hasMetric(rows, A, "harbor_complete_truth", "occ_peak_structures"));
  // At the P_D-0.9 yardstick the pier classifies as structure in a majority of
  // seeds (marginal on the 4-cell extent gate under return noise). This guards
  // the instrument only — the FINDING is that this classification vanishes on
  // the churn variant and on philos (see printed occ_peak_structures = 0 there).
  EXPECT_GE(meanMetric(rows, B, "harbor_complete_truth", "occ_peak_structures"),
            0.5);
}
