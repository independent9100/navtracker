#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity5State.hpp"

using navtracker::ConstantVelocity5State;

TEST(ConstantVelocity5State, TransitionMatrixIsCVWithPassiveOmega) {
  ConstantVelocity5State m(0.1, 0.01);
  const Eigen::MatrixXd F = m.transitionMatrix(2.0);
  ASSERT_EQ(F.rows(), 5);
  ASSERT_EQ(F.cols(), 5);
  EXPECT_DOUBLE_EQ(F(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(F(0, 2), 2.0);
  EXPECT_DOUBLE_EQ(F(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(F(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(F(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(F(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(F(4, 4), 1.0);
  EXPECT_DOUBLE_EQ(F(0, 4), 0.0);
  EXPECT_DOUBLE_EQ(F(2, 4), 0.0);
}

TEST(ConstantVelocity5State, ProcessNoiseIsPSDAndOmegaIsAdditive) {
  ConstantVelocity5State m(0.5, 0.01);
  const Eigen::MatrixXd Q = m.processNoise(1.0);
  ASSERT_EQ(Q.rows(), 5);
  ASSERT_EQ(Q.cols(), 5);
  EXPECT_NEAR(Q(4, 4), 0.01 * 1.0, 1e-12);
  EXPECT_NEAR(Q(0, 0), 0.5 * 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(Q(2, 2), 0.5 * 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(Q(0, 4), 0.0);
  EXPECT_DOUBLE_EQ(Q(2, 4), 0.0);
}
