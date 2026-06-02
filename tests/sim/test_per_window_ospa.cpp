#include "core/scenario/Metrics.hpp"

#include <gtest/gtest.h>

#include "core/scenario/Harness.hpp"

using namespace navtracker;

namespace {

ScenarioResult makeResult(const std::vector<std::pair<double, double>>& time_ospa) {
  ScenarioResult r;
  for (const auto& [t, o] : time_ospa) {
    ScenarioStep s;
    s.time = Timestamp::fromSeconds(t);
    r.steps.push_back(std::move(s));
    r.ospa_per_step.push_back(o);
  }
  return r;
}

}  // namespace

TEST(PerWindowOspa, GroupsByOneSecondWindowAndMeansWithinEach) {
  // Steps at t = 0.1, 0.2, 1.1, 1.2, 1.3, 2.5 with OSPA [10, 12, 5, 5, 8, 20].
  const ScenarioResult r = makeResult({
      {0.1, 10.0}, {0.2, 12.0},
      {1.1,  5.0}, {1.2,  5.0}, {1.3, 8.0},
      {2.5, 20.0},
  });
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), /*window_dt_s=*/1.0);
  ASSERT_EQ(w.per_window.size(), 3u);
  EXPECT_NEAR(w.per_window[0], 11.0, 1e-9);
  EXPECT_NEAR(w.per_window[1],  6.0, 1e-9);
  EXPECT_NEAR(w.per_window[2], 20.0, 1e-9);
  // Mean of [11, 6, 20] = 37/3.
  EXPECT_NEAR(w.mean, 37.0 / 3.0, 1e-9);
  // sample stddev of [11, 6, 20] is sqrt(((11-37/3)^2 + (6-37/3)^2 + (20-37/3)^2)/2)
  // ~= 7.094. Range check rather than exact value.
  EXPECT_GT(w.stddev, 5.0);
  EXPECT_LT(w.stddev, 10.0);
}

TEST(PerWindowOspa, EmptyResultReturnsZeroes) {
  ScenarioResult r;
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), 1.0);
  EXPECT_EQ(w.per_window.size(), 0u);
  EXPECT_EQ(w.mean, 0.0);
  EXPECT_EQ(w.stddev, 0.0);
}

TEST(PerWindowOspa, SingleWindowSingleStepReportsItVerbatim) {
  const ScenarioResult r = makeResult({{0.5, 17.0}});
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), 1.0);
  ASSERT_EQ(w.per_window.size(), 1u);
  EXPECT_DOUBLE_EQ(w.per_window[0], 17.0);
  EXPECT_DOUBLE_EQ(w.mean, 17.0);
  EXPECT_DOUBLE_EQ(w.stddev, 0.0);  // single sample => stddev defined as 0
}
