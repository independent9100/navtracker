#include <gtest/gtest.h>

#include <set>
#include <string>

#include "core/benchmark/Config.hpp"

using navtracker::benchmark::Config;
using navtracker::benchmark::defaultConfigs;

TEST(Config, DefaultConfigsHaveUniqueLabels) {
  const auto configs = defaultConfigs();
  ASSERT_EQ(configs.size(), 6u);
  std::set<std::string> labels;
  for (const auto& c : configs) {
    labels.insert(c.label);
    EXPECT_NE(c.build_estimator, nullptr);
    EXPECT_NE(c.build_associator, nullptr);
  }
  EXPECT_EQ(labels.size(), 6u);
  EXPECT_EQ(labels.count("ekf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_jpda"), 1u);
  EXPECT_EQ(labels.count("ukf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ukf_ct_gnn"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_jpda"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_noisy_jpda"), 1u);
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
