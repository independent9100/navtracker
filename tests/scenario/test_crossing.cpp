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

TEST(Stress, CrossingTargetsStayCountedAndIdsMostlyStable) {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, 11);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  const int id_switches = countIdSwitches(r.steps, 30.0);
  std::cerr << "[crossing] mean_ospa=" << r.mean_ospa
            << " id_switches=" << id_switches << " tracks=" << mgr.size() << "\n";
  EXPECT_EQ(mgr.size(), 2u);
  // #24 (W3 assertion-quality#1 + required-scenarios#2): the id-switch count was
  // the ONLY substantive assertion and it is (a) drift-blind — countIdSwitches
  // scores 0 when a track sits > cutoff off truth (best_id=0, no switch counted) —
  // and (b) a clean full 2-target swap counts exactly 2, which passed the old
  // EXPECT_LE(...,2). Add a fixed-bound accuracy gate that catches off-truth drift
  // (mean_ospa is clipped at 50 here; measured ~6, so < 25 keeps > 3x headroom and
  // a drift/divergence saturates OSPA toward 50 → red), and tighten the swap bound
  // so a full swap (=2) fails.
  EXPECT_LT(r.mean_ospa, 25.0)
      << "crossing accuracy collapsed (tracks drifted off truth): mean_ospa="
      << r.mean_ospa;
  EXPECT_LE(id_switches, 1);
}
