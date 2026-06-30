#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
// Mean of a metric over seeds for (config, scenario).
double meanMetric(const std::vector<MetricRow>& rows, const std::string& config,
                  const std::string& scenario, const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows) {
    if (r.config == config && r.scenario == scenario && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  }
  return n == 0 ? 0.0 : sum / n;
}

std::vector<std::unique_ptr<ScenarioRun>> shoreScenariosOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultSimScenarios()) {
    const std::string label = s->descriptor().label;
    if (label == "shore_clutter_open" || label == "shore_clutter_nearshore")
      out.push_back(std::move(s));
  }
  return out;
}
}  // namespace

TEST(SyntheticClutterAB, LandModelRemovesShoreOverCountKeepsRealTargets) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs()) {
    if (c.label == "imm_cv_ct_pmbm_coverage" ||
        c.label == "imm_cv_ct_pmbm_coverage_land")
      configs.push_back(c);
  }
  ASSERT_EQ(configs.size(), 2u);

  SweepParams params;
  params.run_id = "synthetic_clutter_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, shoreScenariosOnly(), params);

  for (const std::string scenario :
       {"shore_clutter_open", "shore_clutter_nearshore"}) {
    const double card_off = meanMetric(rows, "imm_cv_ct_pmbm_coverage",
                                       scenario, "card_err_mean");
    const double card_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "card_err_mean");
    // Land ON must reduce the (positive) over-count toward 0.
    EXPECT_LT(card_on, card_off) << scenario;
    EXPECT_LE(card_on, 1.0) << scenario;
    // Real targets are not dropped: lifetime_ratio stays healthy with land ON.
    const double life_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "lifetime_ratio");
    EXPECT_GT(life_on, 0.5) << scenario;
  }
}
