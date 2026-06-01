#include "sim/TruthTrajectory.hpp"

#include <gtest/gtest.h>

using namespace navtracker;
using sim::ConstantVelocityTrajectory;
using sim::TruthState;

TEST(ConstantVelocityTrajectory, EvaluatesPositionAndVelocity) {
  ConstantVelocityTrajectory traj(
      Eigen::Vector2d(100.0, -50.0),
      Eigen::Vector2d(5.0, 2.5),
      Timestamp::fromSeconds(0.0));

  const TruthState s0 = traj.eval(Timestamp::fromSeconds(0.0));
  EXPECT_DOUBLE_EQ(s0.position.x(), 100.0);
  EXPECT_DOUBLE_EQ(s0.position.y(), -50.0);
  EXPECT_DOUBLE_EQ(s0.velocity.x(), 5.0);
  EXPECT_DOUBLE_EQ(s0.velocity.y(), 2.5);

  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  EXPECT_DOUBLE_EQ(s10.position.x(), 150.0);
  EXPECT_DOUBLE_EQ(s10.position.y(), -25.0);
  EXPECT_DOUBLE_EQ(s10.velocity.x(), 5.0);
  EXPECT_DOUBLE_EQ(s10.velocity.y(), 2.5);
}

TEST(ConstantVelocityTrajectory, ConstVelocityIndependentOfT0) {
  ConstantVelocityTrajectory traj(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d(1.0, 0.0),
      Timestamp::fromSeconds(5.0));

  const TruthState s = traj.eval(Timestamp::fromSeconds(7.0));
  EXPECT_DOUBLE_EQ(s.position.x(), 2.0);   // (7 - 5) * 1.0
  EXPECT_DOUBLE_EQ(s.position.y(), 0.0);
}
