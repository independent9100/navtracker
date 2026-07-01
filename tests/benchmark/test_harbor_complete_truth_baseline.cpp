// Milestone-1 baseline: run TODAY's PMBM on the harbor_complete_truth
// yardstick and print every metric. This is the "before" number the honest
// static-occupancy layer must improve — NOT a tuning target. Light sanity
// asserts only (the movers are trackable; the scene produces tracks).
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
double meanMetric(const std::vector<MetricRow>& rows, const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.scenario == "harbor_complete_truth" && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

std::vector<std::unique_ptr<ScenarioRun>> harborOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_complete_truth")
      out.push_back(std::move(s));
  return out;
}
}  // namespace

TEST(HarborCompleteTruthBaseline, TodaysPmbmMetrics) {
  const Config* base = nullptr;
  const auto all = defaultConfigs();
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm") base = &c;
  ASSERT_NE(base, nullptr);

  SweepParams params;
  params.run_id = "harbor_complete_truth_baseline";
  params.synthetic_seeds = 5;
  const auto rows = runSweep({*base}, harborOnly(), params);
  ASSERT_FALSE(rows.empty());

  const char* keys[] = {"card_err_mean", "gospa_mean", "gospa_false",
                        "gospa_missed", "lifetime_ratio"};
  std::cout << "\n=== harbor_complete_truth  (baseline imm_cv_ct_pmbm) ===\n";
  for (const auto* m : keys)
    std::cout << "  " << m << " = " << meanMetric(rows, m) << "\n";
  std::cout << std::flush;

  // Sanity: the scene is trackable — some tracks live long enough to match
  // truth (movers + boats), so lifetime is clearly non-zero.
  EXPECT_GT(meanMetric(rows, "lifetime_ratio"), 0.3);
}
