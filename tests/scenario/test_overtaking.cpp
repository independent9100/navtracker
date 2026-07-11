#include <iostream>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

TEST(Stress, OvertakingKeepsBothTracksDistinct) {
  std::vector<double> times;
  for (int i = 1; i <= 60; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildOvertakingScenario(
      Eigen::Vector2d(0.0, 30.0),
      Eigen::Vector2d(10.0, 0.0),
      Eigen::Vector2d(-500.0, 0.0),
      Eigen::Vector2d(20.0, 0.0),
      times, 5.0, 23);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  std::cerr << "[overtaking] mean_ospa=" << r.mean_ospa
            << " id_switches=" << countIdSwitches(r.steps, 30.0)
            << " tracks=" << mgr.size() << "\n";
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_EQ(countIdSwitches(r.steps, 30.0), 0);  // swap-proof (strict) — kept
  // #24 (W3 assertion-quality#1): the ==0 switch bound is swap-proof but the test
  // asserted NO accuracy vs truth (mean_ospa was discarded, and countIdSwitches is
  // drift-blind — a track >cutoff off truth scores 0). Add a fixed-bound accuracy
  // gate: mean_ospa clipped at 50; a correctly-tracked overtaking pass is small,
  // so < 25 catches an off-truth drift/divergence (which saturates OSPA → 50).
  EXPECT_LT(r.mean_ospa, 25.0)
      << "overtaking accuracy collapsed (tracks drifted off truth): mean_ospa="
      << r.mean_ospa;
}
