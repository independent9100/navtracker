#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
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
