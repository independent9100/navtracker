#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"

using navtracker::ConstantVelocity2D;

TEST(ConstantVelocity2D, TransitionPropagatesPosition) {
  ConstantVelocity2D model(1.0);
  const Eigen::MatrixXd f = model.transitionMatrix(2.0);
  const Eigen::Vector4d x(0.0, 0.0, 3.0, -1.0);
  const Eigen::Vector4d xp = f * x;
  EXPECT_DOUBLE_EQ(xp(0), 6.0);
  EXPECT_DOUBLE_EQ(xp(1), -2.0);
  EXPECT_DOUBLE_EQ(xp(2), 3.0);
  EXPECT_DOUBLE_EQ(xp(3), -1.0);
}

TEST(ConstantVelocity2D, ProcessNoiseMatchesWhiteAccelModel) {
  ConstantVelocity2D model(1.0);
  const Eigen::MatrixXd q = model.processNoise(2.0);
  EXPECT_NEAR(q(0, 0), 8.0 / 3.0, 1e-12);  // dt^3/3
  EXPECT_NEAR(q(0, 2), 2.0, 1e-12);        // dt^2/2
  EXPECT_NEAR(q(2, 2), 2.0, 1e-12);        // dt
  EXPECT_TRUE(q.isApprox(q.transpose()));
}

TEST(ConstantVelocity2D, StateDimIsFour) {
  ConstantVelocity2D model(1.0);
  EXPECT_EQ(model.stateDim(), 4);
}
