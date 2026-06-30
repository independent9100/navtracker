#include <gtest/gtest.h>

#include <set>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

TEST(SimScenarioRun, ProducesExpectedDefaultScenarios) {
  const auto scenarios = defaultSimScenarios();
  ASSERT_EQ(scenarios.size(), 17u);
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
  EXPECT_EQ(labels.count("parallel_lanes_dense"), 1u);
  EXPECT_EQ(labels.count("crossing_30"), 1u);
  EXPECT_EQ(labels.count("crossing_60"), 1u);
  EXPECT_EQ(labels.count("crossing_90"), 1u);
  EXPECT_EQ(labels.count("convoy_overtake"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_open"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_nearshore"), 1u);
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

// Every synthetic scenario declares an honest per-sensor detection
// table — a *scenario property*, like the calibrated autoferry table.
// The generators emit exactly one detection per target per scan
// (P_D ≈ 1; declared 0.95) and a known clutter rate: zero for the
// clutter-free scenarios (declared as the 1e-6 m^-2 floor — lambda = 0
// degenerates the log-likelihood ratio) and 4 FA per 600x200 m box for
// dense_clutter (4 / 120000 = 3.33e-5 m^-2). Scoring the clutter-free
// scenarios with the legacy global 1e-4 made a gated hit on a young
// (unconverged) track score as evidence AGAINST existence — the
// measured 6-scan IPDA confirmation latency on crossing.
TEST(SimScenarioRun, AllScenariosDeclareDetectionTables) {
  const auto scenarios = navtracker::benchmark::defaultSimScenarios();
  for (const auto& s : scenarios) {
    const auto d = s->descriptor();
    EXPECT_FALSE(d.detection_table.empty())
        << d.label << " must declare a detection table";
    for (const auto& e : d.detection_table) {
      EXPECT_GT(e.params.probability_of_detection, 0.0) << d.label;
      EXPECT_LT(e.params.probability_of_detection, 1.0) << d.label;
      EXPECT_GT(e.params.clutter_intensity, 0.0) << d.label;
    }
  }
}

TEST(SimScenarioRun, ShoreClutterScenariosExposeCoastlineAndDatum) {
  const auto scenarios = navtracker::benchmark::defaultSimScenarios();
  int checked = 0;
  for (const auto& s : scenarios) {
    const std::string label = s->descriptor().label;
    if (label != "shore_clutter_open" && label != "shore_clutter_nearshore")
      continue;
    ++checked;
    EXPECT_TRUE(s->syntheticCoastline().has_value()) << label;
    const auto scen = s->generate(0);
    EXPECT_TRUE(scen.datum.has_value()) << label;
    bool has_shore = false;
    for (const auto& m : scen.measurements)
      if (m.source_id == "sim_shore") has_shore = true;
    EXPECT_TRUE(has_shore) << label;
  }
  EXPECT_EQ(checked, 2);
}

TEST(SimScenarioRun, ClutterFreeScenariosDeclareFloorDensity) {
  const auto scenarios = navtracker::benchmark::defaultSimScenarios();
  for (const auto& s : scenarios) {
    const auto d = s->descriptor();
    if (d.label == "dense_clutter") {
      ASSERT_EQ(d.detection_table.size(), 1u);
      EXPECT_NEAR(d.detection_table[0].params.clutter_intensity, 3.33e-5,
                  1e-6);
    } else if (d.label == "non_cooperative") {
      ASSERT_EQ(d.detection_table.size(), 1u);
      EXPECT_EQ(d.detection_table[0].model,
                navtracker::MeasurementModel::Bearing2D);
    } else if (d.label == "shore_clutter_open" ||
               d.label == "shore_clutter_nearshore") {
      ASSERT_EQ(d.detection_table.size(), 2u) << d.label;
    } else {
      ASSERT_EQ(d.detection_table.size(), 1u) << d.label;
      EXPECT_DOUBLE_EQ(d.detection_table[0].params.clutter_intensity, 1e-6)
          << d.label;
    }
  }
}
