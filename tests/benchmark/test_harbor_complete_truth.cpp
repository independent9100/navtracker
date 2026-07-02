// Ground-truth contract for the harbor_complete_truth yardstick: exactly 5
// truth targets (2 movers, 3 anchored boats with zero velocity), the pier and
// uniform clutter add NO truth, and the scenario is chart-free (no coastline).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
// unique_ptr::get() yields a non-const ScenarioRun* even from a const
// unique_ptr, so generate() (non-const) is directly callable.
ScenarioRun* findHarbor(const std::vector<std::unique_ptr<ScenarioRun>>& v) {
  for (const auto& s : v)
    if (s->descriptor().label == "harbor_complete_truth") return s.get();
  return nullptr;
}
}  // namespace

TEST(HarborCompleteTruth, TruthIsClosedFiveTargets) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);

  std::set<std::uint64_t> ids;
  std::map<std::uint64_t, double> max_speed;
  for (const auto& ts : scen.truth) {
    ids.insert(ts.truth_id);
    max_speed[ts.truth_id] =
        std::max(max_speed[ts.truth_id], ts.velocity.norm());
  }
  EXPECT_EQ(ids, (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u}));
  // Movers move; anchored boats are stationary.
  EXPECT_GT(max_speed[1u], 1.0);
  EXPECT_GT(max_speed[2u], 1.0);
  EXPECT_EQ(max_speed[3u], 0.0);
  EXPECT_EQ(max_speed[4u], 0.0);
  EXPECT_EQ(max_speed[5u], 0.0);
}

TEST(HarborCompleteTruth, PierAndClutterAddNoTruth) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);
  // 5 targets x 40 scans = 200 truth samples exactly.
  EXPECT_EQ(scen.truth.size(), 5u * 40u);
  bool has_pier = false, has_clutter = false, has_ais = false, has_anch = false;
  for (const auto& m : scen.measurements) {
    if (m.source_id == "sim_pier") has_pier = true;
    if (m.source_id == "sim_clutter") has_clutter = true;
    if (m.source_id == "sim_anchored") has_anch = true;
    if (m.sensor == SensorKind::Ais) has_ais = true;
  }
  EXPECT_TRUE(has_pier);
  EXPECT_TRUE(has_clutter);
  EXPECT_TRUE(has_anch);
  EXPECT_TRUE(has_ais);
  EXPECT_TRUE(scen.datum.has_value());
}

TEST(HarborCompleteTruth, TruthIsTimeSortedIntoFortyCompleteGroups) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);

  // (a) Truth must be sorted by non-decreasing time. BenchRunner::groupTruth
  //     opens a new bucket only when the timestamp changes, so out-of-order
  //     truth silently fragments into duplicate groups and corrupts every
  //     metric. size()==200 does NOT catch this — the count is right, the
  //     grouping is wrong.
  for (std::size_t i = 1; i < scen.truth.size(); ++i)
    ASSERT_LE(scen.truth[i - 1].time.seconds(), scen.truth[i].time.seconds())
        << "truth not time-sorted at index " << i;

  // (b) Grouping truth exactly as BenchRunner does must yield 40 complete
  //     ticks, each holding all five targets {1..5} — not 80 fragmented ones
  //     (40 mover-only + 40 boat-only).
  std::vector<double> group_times;
  std::vector<std::set<std::uint64_t>> group_ids;
  for (const auto& ts : scen.truth) {
    const double t = ts.time.seconds();
    if (group_times.empty() || group_times.back() != t) {
      group_times.push_back(t);
      group_ids.emplace_back();
    }
    group_ids.back().insert(ts.truth_id);
  }
  EXPECT_EQ(group_times.size(), 40u);
  for (const auto& ids : group_ids)
    EXPECT_EQ(ids, (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u}));
}

TEST(HarborCompleteTruth, ChartFreeNoCoastline) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  EXPECT_FALSE(h->syntheticCoastline().has_value());
}

TEST(HarborCompleteTruth, DeterministicForSameSeed) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  auto a = h->generate(0);
  auto b = h->generate(0);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].value, b.measurements[i].value);
    EXPECT_EQ(a.measurements[i].source_id, b.measurements[i].source_id);
  }
}
