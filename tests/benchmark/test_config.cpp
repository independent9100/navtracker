#include <gtest/gtest.h>

#include <set>
#include <string>

#include "core/benchmark/Config.hpp"

using navtracker::benchmark::Config;
using navtracker::benchmark::defaultConfigs;

TEST(Config, DefaultConfigsHaveUniqueLabels) {
  const auto configs = defaultConfigs();
  // 34: 31 standing + imm_cv_ct_pmbm_land_pda (PDA soft detected-branch update,
  // opt-in; A/B vs imm_cv_ct_pmbm_land for the open-sea K=1 gap) +
  // imm_cv_ct_pmbm_land_pda_wateronly (land-aware pool; A/B vs _land_pda) +
  // imm_cv_ct_pmbm_occupancy (Stage 1b live occupancy layer; A/B vs _land) +
  // imm_cv_ct_pmbm_occupancy_sensitive (diagnostic: coarser/lower-bar occupancy
  // classifier to separate mis-tuning from architectural sparsity limit) +
  // imm_cv_ct_pmbm_occupancy_detector (Stage 1b-ii: coarse grid, adaptive bar) +
  // imm_cv_ct_pmbm_occupancy_detector_coverage (6c: coverage-aware decay arm,
  // differs from _detector in estimate_coverage_sector only).
  ASSERT_EQ(configs.size(), 37u);
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
  EXPECT_EQ(labels.size(), 37u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_adapt"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_land"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_land_pda"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_land_pda_wateronly"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_static"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_occupancy"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_occupancy_sensitive"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_occupancy_detector"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_occupancy_detector_coverage"), 1u);
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

// 6c A/B validity: the coverage-aware detector arm must differ from the
// universal-decay detector in the estimate_coverage_sector flag ALONE, so any
// measured delta (structure-hazard stability, KEEP_MIXED departure recovery) is
// attributable to the coverage gate and nothing else. This guards against silent
// drift that would invalidate the A/B comparison.
TEST(Config, OccupancyDetectorArmsDifferOnlyInCoverageFlag) {
  const auto configs = defaultConfigs();
  const Config* base = nullptr;
  const Config* cov = nullptr;
  for (const auto& c : configs) {
    if (c.label == "imm_cv_ct_pmbm_occupancy_detector") base = &c;
    if (c.label == "imm_cv_ct_pmbm_occupancy_detector_coverage") cov = &c;
  }
  ASSERT_NE(base, nullptr);
  ASSERT_NE(cov, nullptr);
  ASSERT_TRUE(base->pmbm_config && cov->pmbm_config);
  const auto b = base->pmbm_config();
  const auto v = cov->pmbm_config();
  // The one variable under test.
  EXPECT_FALSE(b.estimate_coverage_sector);
  EXPECT_TRUE(v.estimate_coverage_sector);
  // Every other knob that defines the detector must match.
  EXPECT_EQ(b.lambda_birth, v.lambda_birth);
  EXPECT_EQ(b.min_new_bernoulli_existence, v.min_new_bernoulli_existence);
  EXPECT_EQ(b.adaptive_birth, v.adaptive_birth);
  EXPECT_EQ(b.use_land_model, v.use_land_model);
  EXPECT_EQ(b.use_static_obstacle_model, v.use_static_obstacle_model);
  // ...and the occupancy-layer wiring + grid/classifier params.
  EXPECT_EQ(base->use_live_occupancy_model, cov->use_live_occupancy_model);
  EXPECT_EQ(base->occupancy_adaptive_clutter_bar,
            cov->occupancy_adaptive_clutter_bar);
  ASSERT_TRUE(base->live_occupancy_params && cov->live_occupancy_params);
  EXPECT_EQ(base->live_occupancy_params->cell_size_m,
            cov->live_occupancy_params->cell_size_m);
  EXPECT_EQ(base->live_occupancy_params->ewma_alpha,
            cov->live_occupancy_params->ewma_alpha);
  EXPECT_EQ(base->live_occupancy_params->persistence_bar,
            cov->live_occupancy_params->persistence_bar);
  EXPECT_EQ(base->live_occupancy_params->extended_cells_min,
            cov->live_occupancy_params->extended_cells_min);
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
