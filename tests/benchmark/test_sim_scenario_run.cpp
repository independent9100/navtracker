#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

TEST(SimScenarioRun, ProducesExpectedDefaultScenarios) {
  const auto scenarios = defaultSimScenarios();
  ASSERT_EQ(scenarios.size(), 26u);
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
  EXPECT_EQ(labels.count("dense_clutter_datum"), 1u);
  EXPECT_EQ(labels.count("crossing_dropout"), 1u);
  EXPECT_EQ(labels.count("parallel_lanes_dense"), 1u);
  EXPECT_EQ(labels.count("crossing_30"), 1u);
  EXPECT_EQ(labels.count("crossing_60"), 1u);
  EXPECT_EQ(labels.count("crossing_90"), 1u);
  EXPECT_EQ(labels.count("convoy_overtake"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_open"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_nearshore"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_transit"), 1u);
  EXPECT_EQ(labels.count("harbor_complete_truth"), 1u);
  EXPECT_EQ(labels.count("harbor_charted_pier"), 1u);
  EXPECT_EQ(labels.count("harbor_boat_near_pier"), 1u);
  EXPECT_EQ(labels.count("harbor_large_anchored_ship"), 1u);
  EXPECT_EQ(labels.count("harbor_compact_dolphin"), 1u);
  EXPECT_EQ(labels.count("harbor_complete_truth_churn"), 1u);
  EXPECT_EQ(labels.count("harbor_anchored_gets_underway"), 1u);
}

// The churn variant is harbor_complete_truth with the pier detected at a low
// per-scan probability (~0.4 vs 0.9), so its phantom cohort decays and must
// re-birth — the only regime where a BIRTH-channel occupancy suppressor can
// act. Complete truth (boats scored) is unchanged: only the no-truth pier's
// detection rate drops. Observable: strictly fewer returns on the same seed,
// since only the pier's detection draw changes (movers/boats/clutter use
// disjoint seed streams).
TEST(SimScenarioRun, ChurnVariantEmitsFewerPierReturnsThanBaseline) {
  std::unique_ptr<ScenarioRun> baseline, churn;
  for (auto& s : defaultSimScenarios()) {
    const auto label = s->descriptor().label;
    if (label == "harbor_complete_truth") baseline = std::move(s);
    else if (label == "harbor_complete_truth_churn") churn = std::move(s);
  }
  ASSERT_TRUE(baseline) << "harbor_complete_truth missing";
  ASSERT_TRUE(churn) << "harbor_complete_truth_churn missing";
  const auto b = baseline->generate(0);
  const auto c = churn->generate(0);
  EXPECT_LT(c.measurements.size(), b.measurements.size())
      << "churn pier P_D should be lower than baseline";
}

// dense_clutter_datum is dense_clutter (two crossing targets + uniform
// transient clutter) with a datum attached, so the live-occupancy layer wires
// (Sweep gates on scen.datum). It is the end-to-end death-spiral guard: the
// detector runs on dense uniform clutter and must NOT classify it as structure
// (adaptive bar) nor drop the real targets. Contract: same closed truth as
// dense_clutter (ids 1-2), but a datum is present.
TEST(SimScenarioRun, DenseClutterDatumCarriesDatumWithSameTruthAsDenseClutter) {
  std::unique_ptr<ScenarioRun> plain, with_datum;
  for (auto& s : defaultSimScenarios()) {
    const auto label = s->descriptor().label;
    if (label == "dense_clutter") plain = std::move(s);
    else if (label == "dense_clutter_datum") with_datum = std::move(s);
  }
  ASSERT_TRUE(plain) << "dense_clutter missing";
  ASSERT_TRUE(with_datum) << "dense_clutter_datum missing";
  const auto p = plain->generate(0);
  const auto d = with_datum->generate(0);
  EXPECT_FALSE(p.datum.has_value()) << "dense_clutter must stay datum-free";
  EXPECT_TRUE(d.datum.has_value()) << "dense_clutter_datum must carry a datum";
  // Same measurements/truth (only the datum differs) — the guard measures the
  // layer's effect on the identical clutter environment.
  ASSERT_EQ(p.measurements.size(), d.measurements.size());
  EXPECT_EQ(p.truth.size(), d.truth.size());
  std::set<std::uint64_t> ids;
  for (const auto& ts : d.truth) ids.insert(ts.truth_id);
  EXPECT_EQ(ids, (std::set<std::uint64_t>{1u, 2u}));
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
    if (d.label == "dense_clutter" || d.label == "dense_clutter_datum") {
      ASSERT_EQ(d.detection_table.size(), 1u);
      EXPECT_NEAR(d.detection_table[0].params.clutter_intensity, 3.33e-5,
                  1e-6);
    } else if (d.label == "non_cooperative") {
      ASSERT_EQ(d.detection_table.size(), 1u);
      EXPECT_EQ(d.detection_table[0].model,
                navtracker::MeasurementModel::Bearing2D);
    } else if (d.label == "shore_clutter_open" ||
               d.label == "shore_clutter_nearshore" ||
               d.label == "shore_clutter_transit" ||
               d.label == "harbor_complete_truth" ||
               d.label == "harbor_charted_pier" ||
               d.label == "harbor_boat_near_pier" ||
               d.label == "harbor_large_anchored_ship" ||
               d.label == "harbor_compact_dolphin" ||
               d.label == "harbor_complete_truth_churn" ||
               d.label == "harbor_anchored_gets_underway") {
      ASSERT_EQ(d.detection_table.size(), 2u) << d.label;
    } else {
      ASSERT_EQ(d.detection_table.size(), 1u) << d.label;
      EXPECT_DOUBLE_EQ(d.detection_table[0].params.clutter_intensity, 1e-6)
          << d.label;
    }
  }
}
