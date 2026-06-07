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
