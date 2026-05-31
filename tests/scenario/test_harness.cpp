#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"

using navtracker::buildStraightLineScenario;
using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::runScenario;
using navtracker::Scenario;
using navtracker::ScenarioResult;
using navtracker::Tracker;
using navtracker::TrackManager;

TEST(Harness, SingleStraightTargetGetsLowOspa) {
  std::vector<double> times;
  for (int i = 1; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(100.0, 0.0), Eigen::Vector2d(5.0, 0.0),
      times, 5.0, 13);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 3);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  EXPECT_EQ(r.ospa_per_step.size(), 20u);
  EXPECT_LT(r.mean_ospa, 15.0);
}

TEST(Harness, RecordsPerStepSnapshots) {
  std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 0.0),
      times, 0.0, 7);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 3);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  ASSERT_EQ(r.steps.size(), 3u);
  EXPECT_DOUBLE_EQ(r.steps.front().time.seconds(), 1.0);
  EXPECT_EQ(r.steps.front().truth.size(), 1u);
  EXPECT_GE(r.steps.back().tracks.size(), 1u);
}
