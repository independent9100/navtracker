#include <gtest/gtest.h>

#include <set>
#include <string>

#include "core/benchmark/Config.hpp"

using navtracker::benchmark::Config;
using navtracker::benchmark::defaultConfigs;

TEST(Config, DefaultConfigsHaveUniqueLabels) {
  const auto configs = defaultConfigs();
  // 28: 27 standing + Task 6 (Step 6) imm_cv_ct_pmbm_coverage_land
  // (CoastlineModel land-prior wiring: suppresses adaptive-birth intensity
  // at land positions via Boston Harbor GeoJSON, philos only).
  ASSERT_EQ(configs.size(), 29u);
  // Canonical config is listed first.
  EXPECT_EQ(configs.front().label, "imm_cv_ct_mht");
  // Canonical wires the bias estimator unconditionally; the
  // null-anchor path stays bit-identical to legacy by construction.
  EXPECT_NE(configs.front().build_sensor_bias_estimator, nullptr);
  std::set<std::string> labels;
  for (const auto& c : configs) {
    labels.insert(c.label);
    EXPECT_NE(c.build_estimator, nullptr);
    EXPECT_NE(c.build_associator, nullptr);
  }
  EXPECT_EQ(labels.size(), 29u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_adapt"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_adapt_k3"), 1u);
  // Phase 9 probe siblings dropped 2026-06-23 (S4 fold-in):
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_adapt_k3_altgate"), 0u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_adapt_k3_xparent"), 0u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_ekf"), 1u);
  // imm_cv_ct_mht_ukf retired 2026-06-20: UKF promoted to canonical,
  // imm_cv_ct_mht IS the UKF stack now; _ekf preserves the prior canonical.
  EXPECT_EQ(labels.count("imm_cv_ct_mht_ukf"), 0u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_nobias"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_novis"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_robust"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_ipda"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_mofn"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_cmap"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_bearguard"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_jpda"), 1u);
  EXPECT_EQ(labels.count("ukf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ukf_ct_gnn"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_jpda"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_jpda_persensor"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_jpda_persensor"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_noisy_jpda"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_mht"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_noisy_mht"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_mht_recapture"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_birthtarget"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_cmap"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_bundle"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_coverage"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_coverage_land"), 1u);
  // biascal label removed — its wiring is now the canonical.
  EXPECT_EQ(labels.count("imm_cv_ct_mht_biascal"), 0u);
}

// The clutter-map flag is set on exactly two configs: the canonical MHT
// ablation (imm_cv_ct_mht_cmap) and the PMBM ablation (imm_cv_ct_pmbm_cmap).
// All other configs must keep the fixed-table path.
TEST(Config, ClutterMapAblationFlagsExactlyTwoConfigs) {
  const std::set<std::string> expected_cmap_labels = {
      "imm_cv_ct_mht_cmap", "imm_cv_ct_pmbm_cmap"};
  std::set<std::string> flagged_labels;
  for (const auto& c : defaultConfigs()) {
    if (c.use_clutter_map) {
      flagged_labels.insert(c.label);
      EXPECT_EQ(expected_cmap_labels.count(c.label), 1u)
          << "Unexpected clutter-map config: " << c.label;
    }
  }
  EXPECT_EQ(flagged_labels.size(), 2u);
  EXPECT_EQ(flagged_labels.count("imm_cv_ct_mht_cmap"), 1u);
  EXPECT_EQ(flagged_labels.count("imm_cv_ct_pmbm_cmap"), 1u);
}

TEST(Config, FactoriesProduceUsableObjects) {
  for (const auto& c : defaultConfigs()) {
    auto est = c.build_estimator();
    auto asc = c.build_associator();
    EXPECT_NE(est, nullptr) << "label=" << c.label;
    EXPECT_NE(asc, nullptr) << "label=" << c.label;
  }
}

TEST(Config, FactoriesReturnDistinctInstancesPerCall) {
  // Each call must construct a fresh instance — Sweep relies on this so
  // sweeps over scenarios × seeds never share mutable estimator state.
  for (const auto& c : defaultConfigs()) {
    auto e1 = c.build_estimator();
    auto e2 = c.build_estimator();
    auto a1 = c.build_associator();
    auto a2 = c.build_associator();
    EXPECT_NE(e1.get(), e2.get()) << "label=" << c.label;
    EXPECT_NE(a1.get(), a2.get()) << "label=" << c.label;
  }
}
