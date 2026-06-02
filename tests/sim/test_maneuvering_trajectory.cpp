#include "sim/TruthTrajectory.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace navtracker;
using sim::ManeuveringTrajectory;
using sim::TruthState;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(ManeuveringTrajectory, StraightLegMatchesConstantVelocity) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      /*straight_duration_s=*/5.0,
      /*turn_duration_s=*/5.0,
      /*omega_rad_s=*/0.2,
      Timestamp::fromSeconds(0.0));

  const TruthState s0 = traj.eval(Timestamp::fromSeconds(0.0));
  EXPECT_DOUBLE_EQ(s0.position.x(), 0.0);
  EXPECT_DOUBLE_EQ(s0.position.y(), 0.0);
  EXPECT_DOUBLE_EQ(s0.velocity.x(), 10.0);
  EXPECT_DOUBLE_EQ(s0.velocity.y(),  0.0);

  // End of straight leg: t = 5 s, no turn yet → linear.
  const TruthState s5 = traj.eval(Timestamp::fromSeconds(5.0));
  EXPECT_NEAR(s5.position.x(), 50.0, 1e-9);
  EXPECT_NEAR(s5.position.y(),  0.0, 1e-9);
  EXPECT_NEAR(s5.velocity.x(), 10.0, 1e-9);
  EXPECT_NEAR(s5.velocity.y(),  0.0, 1e-9);
}

TEST(ManeuveringTrajectory, TurnLegRotatesVelocityVector) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, 0.2, Timestamp::fromSeconds(0.0));

  // Mid-turn: t = 7.5 s → τ' = 2.5 s into the turn; heading rotated 0.5 rad.
  const TruthState s75 = traj.eval(Timestamp::fromSeconds(7.5));
  const double speed = std::hypot(s75.velocity.x(), s75.velocity.y());
  EXPECT_NEAR(speed, 10.0, 1e-9);
  const double heading = std::atan2(s75.velocity.y(), s75.velocity.x());
  EXPECT_NEAR(heading, 0.5, 1e-9);

  // End of turn: t = 10 s → heading rotated by ω·Δt = 1.0 rad.
  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  const double end_heading = std::atan2(s10.velocity.y(), s10.velocity.x());
  EXPECT_NEAR(end_heading, 1.0, 1e-9);
}

TEST(ManeuveringTrajectory, PostTurnLegIsLinearInPostTurnVelocity) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, 0.2, Timestamp::fromSeconds(0.0));

  // After-turn velocity = 10 m/s at heading 1.0 rad.
  const TruthState end_turn = traj.eval(Timestamp::fromSeconds(10.0));
  const Eigen::Vector2d v_post = end_turn.velocity;

  // At t = 12 s (2 s into post-turn leg) the position should advance by 2*v_post.
  const TruthState s12 = traj.eval(Timestamp::fromSeconds(12.0));
  EXPECT_NEAR(s12.position.x(), end_turn.position.x() + 2.0 * v_post.x(), 1e-9);
  EXPECT_NEAR(s12.position.y(), end_turn.position.y() + 2.0 * v_post.y(), 1e-9);
  EXPECT_DOUBLE_EQ(s12.velocity.x(), v_post.x());
  EXPECT_DOUBLE_EQ(s12.velocity.y(), v_post.y());
}

TEST(ManeuveringTrajectory, NegativeOmegaTurnsTheOtherWay) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, -0.2, Timestamp::fromSeconds(0.0));
  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  const double heading = std::atan2(s10.velocity.y(), s10.velocity.x());
  EXPECT_NEAR(heading, -1.0, 1e-9);
}
