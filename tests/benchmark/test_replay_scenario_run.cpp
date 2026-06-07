#include <gtest/gtest.h>

#include <set>

#include "adapters/benchmark/ReplayScenarioRun.hpp"

using namespace navtracker::benchmark;

TEST(ReplayScenarioRun, TwoReplaysWithExpectedLabels) {
  const auto scenarios = defaultReplayScenarios();
  ASSERT_EQ(scenarios.size(), 2u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    labels.insert(s->descriptor().label);
    EXPECT_FALSE(s->descriptor().is_multi_seed);
    EXPECT_EQ(s->descriptor().seed_count, 1u);
  }
  EXPECT_EQ(labels.count("philos"), 1u);
  EXPECT_EQ(labels.count("haxr"), 1u);
}

TEST(ReplayScenarioRun, GenerateReturnsNonEmpty) {
  for (auto& s : defaultReplayScenarios()) {
    const auto data = s->generate(0);
    EXPECT_FALSE(data.measurements.empty())
        << "replay " << s->descriptor().label << " produced no measurements";
  }
}
