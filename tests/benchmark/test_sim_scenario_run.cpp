#include <gtest/gtest.h>

#include <set>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

TEST(SimScenarioRun, ProducesExpectedDefaultScenarios) {
  const auto scenarios = defaultSimScenarios();
  ASSERT_EQ(scenarios.size(), 10u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) labels.insert(s->descriptor().label);
  EXPECT_EQ(labels.count("crossing"), 1u);
  EXPECT_EQ(labels.count("overtaking"), 1u);
  EXPECT_EQ(labels.count("head_on"), 1u);
  EXPECT_EQ(labels.count("parallel_targets"), 1u);
  EXPECT_EQ(labels.count("ais_dropout"), 1u);
  EXPECT_EQ(labels.count("clock_skew"), 1u);
  EXPECT_EQ(labels.count("speed_change"), 1u);
  EXPECT_EQ(labels.count("non_cooperative"), 1u);
  EXPECT_EQ(labels.count("dense_clutter"), 1u);
  EXPECT_EQ(labels.count("crossing_dropout"), 1u);
}

TEST(SimScenarioRun, GenerateIsDeterministicForSameSeed) {
  const auto scenarios = defaultSimScenarios();
  const auto a = scenarios[0]->generate(0);
  const auto b = scenarios[0]->generate(0);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].time, b.measurements[i].time);
  }
}
