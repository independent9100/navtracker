#include <gtest/gtest.h>

#include <memory>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/tracking/TrackManager.hpp"

TEST(HarnessBatched, MatchesRunScenarioOnSingleMeasurementPerTimestamp) {
  std::vector<double> times;
  for (int i = 1; i <= 10; ++i) times.push_back(static_cast<double>(i));
  const navtracker::Scenario s = navtracker::buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 0.0), times, 1.0, 7);

  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);
  navtracker::GnnAssociator gnn(50.0);
  navtracker::TrackManager mgr(1, 5);
  navtracker::Tracker tr(ekf, gnn, mgr, 10.0);

  const navtracker::ScenarioResult r =
      navtracker::runScenarioBatched(s, tr, mgr, 50.0);
  EXPECT_EQ(mgr.size(), 1u);
  EXPECT_LT(r.mean_ospa, 10.0);
}
