#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/scenario/Builders.hpp"

using namespace navtracker;
using benchmark::BenchSink;

TEST(BenchRunner, ProducesPerTruthSnapshotsAndLifecycleEvents) {
  std::vector<double> times;
  for (int i = 1; i <= 10; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      times, 1.0, /*seed=*/7, /*truth_id=*/1);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  BenchSink sink;
  const auto result = benchmark::runBench(s, tracker, mgr, sink);

  EXPECT_EQ(result.steps.size(), times.size());
  for (const auto& step : result.steps) {
    EXPECT_EQ(step.truth.size(), 1u);
  }
  EXPECT_FALSE(result.sink_events.empty());
  const auto& last = result.steps.back();
  ASSERT_FALSE(last.tracks.empty());
  EXPECT_LT((last.tracks[0].position - last.truth[0].position).norm(), 25.0);
}

// Regression test for the JPDA-zero-tracks bug: BenchRunner used to drive
// the tracker one measurement at a time via Tracker::process(), which only
// consults the hard-match path. JpdaAssociator only populates the soft
// (betas) path, so every JPDA call fell through to "initiate new track"
// and no track ever confirmed. Switching BenchRunner to processBatch(scan)
// allows the soft path to run.
TEST(BenchRunner, JpdaConfigProducesAtLeastOneConfirmedTrack) {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0), Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0), Eigen::Vector2d(-25.0, 0.0),
      times, /*pos_noise_std_m=*/8.0, /*seed=*/11);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const JpdaAssociator assoc(/*gate=*/20.0, /*P_D=*/0.9, /*lambda_C=*/1e-4);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  BenchSink sink;
  const auto result = benchmark::runBench(s, tracker, mgr, sink);

  // At least one BenchStep must observe a Confirmed track — without the
  // processBatch switch every step has tracks.empty() == true.
  std::size_t total_step_tracks = 0;
  for (const auto& step : result.steps) total_step_tracks += step.tracks.size();
  // 2 truths × 40 ticks = 80 possible cells; allow some startup lag.
  EXPECT_GT(total_step_tracks, 50u)
      << "JPDA produced too few confirmed tracks — BenchRunner regressed.";
}
