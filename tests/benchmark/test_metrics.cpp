#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/benchmark/Metrics.hpp"

using namespace navtracker;
using benchmark::BenchResult;
using benchmark::BenchStep;
using benchmark::TrackStateSnapshot;
using benchmark::TruthStateSnapshot;

namespace {
BenchStep makeStep(double t,
                   std::vector<Eigen::Vector2d> truth_pos,
                   std::vector<Eigen::Vector2d> track_pos) {
  BenchStep s;
  s.time = Timestamp::fromSeconds(t);
  for (std::size_t i = 0; i < truth_pos.size(); ++i) {
    s.truth.push_back({static_cast<std::uint64_t>(i + 1),
                       truth_pos[i],
                       Eigen::Vector2d::Zero()});
  }
  for (std::size_t i = 0; i < track_pos.size(); ++i) {
    s.tracks.push_back({TrackId{static_cast<std::uint64_t>(i + 1)},
                        track_pos[i],
                        Eigen::Vector2d::Zero()});
  }
  return s;
}
}  // namespace

TEST(Metrics, OspaPerStepMatchesHandComputed) {
  BenchResult r;
  // Step 0: truth (0,0) and track (3,4) -> distance 5
  r.steps.push_back(makeStep(0.0, {{0, 0}}, {{3, 4}}));
  // Step 1: truth (10,0) and track (10,0) -> 0
  r.steps.push_back(makeStep(1.0, {{10, 0}}, {{10, 0}}));

  const auto per_step = benchmark::computeOspaPerStep(r, /*cutoff=*/500.0);
  ASSERT_EQ(per_step.size(), 2u);
  EXPECT_NEAR(per_step[0], 5.0, 1e-9);
  EXPECT_NEAR(per_step[1], 0.0, 1e-9);
}

TEST(Metrics, MeanAndPercentile) {
  std::vector<double> v{1.0, 2.0, 3.0, 4.0, 5.0};
  EXPECT_NEAR(benchmark::mean(v), 3.0, 1e-9);
  // Linear-interpolated percentile, NumPy-style: q=0.95 -> idx = 0.95*(5-1) =
  // 3.8 -> v[3]*0.2 + v[4]*0.8 = 4.8.
  EXPECT_NEAR(benchmark::percentile(v, 0.95), 4.8, 1e-9);
  EXPECT_NEAR(benchmark::percentile(v, 1.0), 5.0, 1e-9);
  EXPECT_NEAR(benchmark::percentile(v, 0.5), 3.0, 1e-9);
}

TEST(Metrics, AssignPerStepGreedyWithinGate) {
  BenchResult r;
  // 2 truths, 2 tracks: track 1 next to truth 1, track 2 next to truth 2.
  r.steps.push_back(makeStep(0.0,
                             {{0, 0}, {100, 0}},
                             {{1, 0}, {101, 0}}));
  // Step 2: track 2 disappears (out of gate).
  r.steps.push_back(makeStep(1.0,
                             {{10, 0}, {110, 0}},
                             {{11, 0}, {999, 0}}));

  const auto assigns = benchmark::assignPerStep(r, /*gate=*/100.0);
  ASSERT_EQ(assigns.size(), 2u);
  ASSERT_EQ(assigns[0].size(), 2u);
  ASSERT_TRUE(assigns[0][0].has_value());
  ASSERT_TRUE(assigns[0][1].has_value());
  EXPECT_EQ(assigns[0][0]->value, 1u);
  EXPECT_EQ(assigns[0][1]->value, 2u);

  ASSERT_TRUE(assigns[1][0].has_value());
  EXPECT_FALSE(assigns[1][1].has_value());  // track 2 out of gate
}

TEST(Metrics, RmseLinearMotion) {
  BenchResult r;
  // 3 steps, truth at constant velocity (10, 0) m/s, no error in tracks.
  for (int k = 0; k < 3; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {10.0 * k, 0}, {10, 0}});
    s.tracks.push_back({TrackId{1}, {10.0 * k, 0}, {10, 0}});
    r.steps.push_back(s);
  }
  const auto a = benchmark::assignPerStep(r, 100.0);
  const auto rmse = benchmark::computeRmse(r, a);
  EXPECT_NEAR(rmse.pos_rmse_m, 0.0, 1e-9);
  EXPECT_NEAR(rmse.sog_rmse_mps, 0.0, 1e-9);
  EXPECT_NEAR(rmse.cog_rmse_deg, 0.0, 1e-9);
}

TEST(Metrics, RmseConstantOffsets) {
  BenchResult r;
  for (int k = 0; k < 4; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {0, 0}, {10, 0}});
    // 3m offset; SOG off by +1; COG rotated by 90 degrees (track velocity (0,11)).
    s.tracks.push_back({TrackId{1}, {3, 0}, {0, 11}});
    r.steps.push_back(s);
  }
  const auto a = benchmark::assignPerStep(r, 100.0);
  const auto rmse = benchmark::computeRmse(r, a);
  EXPECT_NEAR(rmse.pos_rmse_m, 3.0, 1e-6);
  EXPECT_NEAR(rmse.sog_rmse_mps, 1.0, 1e-6);
  EXPECT_NEAR(rmse.cog_rmse_deg, 90.0, 1e-6);
}

TEST(Metrics, ContinuityKnownPatterns) {
  // 1 truth (id 1) present in all 6 steps, assignments: [1,1,_, _,1,2]
  BenchResult r;
  for (int k = 0; k < 6; ++k)
    r.steps.push_back(makeStep(k, {{0, 0}}, {}));

  std::vector<benchmark::StepAssignment> a;
  a.push_back({TrackId{1}});
  a.push_back({TrackId{1}});
  a.push_back({std::nullopt});
  a.push_back({std::nullopt});
  a.push_back({TrackId{1}});
  a.push_back({TrackId{2}});  // <- 1 id switch

  const auto c = benchmark::computeContinuity(r, a);
  EXPECT_NEAR(c.lifetime_ratio, 4.0 / 6.0, 1e-9);
  EXPECT_NEAR(c.track_breaks, 1.0, 1e-9);  // one nullopt run
  EXPECT_NEAR(c.id_switches, 1.0, 1e-9);
}

namespace {
// Step builder with explicit truth ids — the slot order in step.truth is
// part of what these tests exercise, so ids can't just be slot+1.
BenchStep makeStepIds(
    double t,
    std::vector<std::pair<std::uint64_t, Eigen::Vector2d>> truth,
    std::vector<std::pair<std::uint64_t, Eigen::Vector2d>> tracks) {
  BenchStep s;
  s.time = Timestamp::fromSeconds(t);
  for (const auto& [id, p] : truth)
    s.truth.push_back({id, p, Eigen::Vector2d(10, 0)});
  for (const auto& [id, p] : tracks)
    s.tracks.push_back({TrackId{id}, p, Eigen::Vector2d(10, 0)});
  return s;
}
}  // namespace

TEST(Metrics, ContinuityKeyedByTruthIdNotSlot) {
  // Two targets, two steps; the truth vector's SLOT ORDER swaps between
  // steps (as happens with real per-target data). Each physical target is
  // followed by the same track throughout — there is no identity churn,
  // so switches and breaks must be zero.
  BenchResult r;
  r.steps.push_back(makeStepIds(0.0,
                                {{7, {0, 0}}, {9, {100, 0}}},
                                {{1, {1, 0}}, {2, {101, 0}}}));
  r.steps.push_back(makeStepIds(1.0,
                                {{9, {100, 0}}, {7, {0, 0}}},
                                {{1, {1, 0}}, {2, {101, 0}}}));

  const auto a = benchmark::assignPerStep(r, 50.0);
  const auto c = benchmark::computeContinuity(r, a);
  EXPECT_NEAR(c.lifetime_ratio, 1.0, 1e-9);
  EXPECT_NEAR(c.track_breaks, 0.0, 1e-9);
  EXPECT_NEAR(c.id_switches, 0.0, 1e-9);
}

TEST(Metrics, ContinuityTimeVaryingCardinality) {
  // Target 7 exists for steps 0-1, target 9 for steps 1-2. Each is
  // perfectly tracked while present. Lifetime is measured against
  // presence, and appearance/disappearance is not a break or a switch.
  BenchResult r;
  r.steps.push_back(makeStepIds(0.0, {{7, {0, 0}}}, {{1, {1, 0}}}));
  r.steps.push_back(makeStepIds(1.0,
                                {{7, {0, 0}}, {9, {100, 0}}},
                                {{1, {1, 0}}, {2, {101, 0}}}));
  r.steps.push_back(makeStepIds(2.0, {{9, {100, 0}}}, {{2, {101, 0}}}));

  const auto a = benchmark::assignPerStep(r, 50.0);
  const auto c = benchmark::computeContinuity(r, a);
  EXPECT_NEAR(c.lifetime_ratio, 1.0, 1e-9);
  EXPECT_NEAR(c.track_breaks, 0.0, 1e-9);
  EXPECT_NEAR(c.id_switches, 0.0, 1e-9);
}

TEST(Metrics, RmseKeyedByTruthIdAcrossReorderedSlots) {
  // Target 7 is tracked with a constant 3 m offset, target 9 exactly.
  // Slot order swaps between steps. Per-truth RMSE must follow the id:
  // mean(rmse_7, rmse_9) = mean(3, 0) = 1.5 — not the slot-mixed 2.12.
  BenchResult r;
  r.steps.push_back(makeStepIds(0.0,
                                {{7, {0, 0}}, {9, {100, 0}}},
                                {{1, {3, 0}}, {2, {100, 0}}}));
  r.steps.push_back(makeStepIds(1.0,
                                {{9, {100, 0}}, {7, {0, 0}}},
                                {{1, {3, 0}}, {2, {100, 0}}}));

  const auto a = benchmark::assignPerStep(r, 50.0);
  const auto rmse = benchmark::computeRmse(r, a);
  EXPECT_NEAR(rmse.pos_rmse_m, 1.5, 1e-9);
}

TEST(Metrics, ComputeMetricsBundlesAll) {
  BenchResult r;
  for (int k = 0; k < 3; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {10.0 * k, 0}, {10, 0}});
    s.tracks.push_back({TrackId{1}, {10.0 * k + 1, 0}, {10, 0}});
    r.steps.push_back(s);
  }
  const auto m = benchmark::computeMetrics(r, {});
  EXPECT_NEAR(m.ospa_mean, 1.0, 1e-6);
  EXPECT_NEAR(m.pos_rmse_m, 1.0, 1e-6);
  EXPECT_NEAR(m.lifetime_ratio, 1.0, 1e-9);
  EXPECT_NEAR(m.track_breaks, 0.0, 1e-9);
  EXPECT_NEAR(m.id_switches, 0.0, 1e-9);
}

TEST(Metrics, PerTruthBreakdownReflectsIndividualTargets) {
  // Two targets across three steps. Target 1 is tracked perfectly
  // (pos_rmse = 0). Target 2 is offset by 4 m every step. Aggregate
  // pos_rmse = mean(0, 4) = 2; per-truth pos_rmse must split them.
  BenchResult r;
  for (int k = 0; k < 3; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {0.0, 0.0}, {0, 0}});
    s.truth.push_back({2, {100.0, 0.0}, {0, 0}});
    s.tracks.push_back({TrackId{10}, {0.0, 0.0}, {0, 0}});
    s.tracks.push_back({TrackId{20}, {104.0, 0.0}, {0, 0}});
    r.steps.push_back(s);
  }
  const auto m = benchmark::computeMetrics(r, {});
  EXPECT_NEAR(m.pos_rmse_m, 2.0, 1e-6);
  ASSERT_EQ(m.per_truth.size(), 2u);
  EXPECT_NEAR(m.per_truth.at(1).pos_rmse_m, 0.0, 1e-6);
  EXPECT_NEAR(m.per_truth.at(2).pos_rmse_m, 4.0, 1e-6);
  EXPECT_NEAR(m.per_truth.at(1).lifetime_ratio, 1.0, 1e-9);
  EXPECT_NEAR(m.per_truth.at(2).lifetime_ratio, 1.0, 1e-9);
  EXPECT_EQ(m.per_truth.at(1).rmse_n, 3u);
  EXPECT_EQ(m.per_truth.at(2).rmse_n, 3u);
}
