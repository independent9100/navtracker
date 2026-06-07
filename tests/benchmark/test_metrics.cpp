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
