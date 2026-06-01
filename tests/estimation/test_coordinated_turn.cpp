#include <gtest/gtest.h>
#include <cmath>
#include "core/estimation/CoordinatedTurn.hpp"

using navtracker::CoordinatedTurn;

TEST(CoordinatedTurn, ReducesToCVAtZeroOmega) {
  CoordinatedTurn m(0.1, 0.01);
  m.setOmega(0.0);
  const Eigen::MatrixXd F = m.transitionMatrix(2.0);
  ASSERT_EQ(F.rows(), 5);
  ASSERT_EQ(F.cols(), 5);
  EXPECT_NEAR(F(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(F(0, 2), 2.0, 1e-12);
  EXPECT_NEAR(F(1, 3), 2.0, 1e-12);
  EXPECT_NEAR(F(2, 2), 1.0, 1e-12);
  EXPECT_NEAR(F(3, 3), 1.0, 1e-12);
  EXPECT_NEAR(F(4, 4), 1.0, 1e-12);
  EXPECT_NEAR(F(0, 3), 0.0, 1e-12);
}

TEST(CoordinatedTurn, FiniteOmegaRotatesVelocityBlock) {
  CoordinatedTurn m(0.1, 0.01);
  const double w = 0.5;
  const double dt = 1.0;
  m.setOmega(w);
  const Eigen::MatrixXd F = m.transitionMatrix(dt);
  EXPECT_NEAR(F(2, 2),  std::cos(w * dt), 1e-12);
  EXPECT_NEAR(F(2, 3), -std::sin(w * dt), 1e-12);
  EXPECT_NEAR(F(3, 2),  std::sin(w * dt), 1e-12);
  EXPECT_NEAR(F(3, 3),  std::cos(w * dt), 1e-12);
  EXPECT_NEAR(F(0, 2),  std::sin(w * dt) / w, 1e-12);
  EXPECT_NEAR(F(0, 3), -(1.0 - std::cos(w * dt)) / w, 1e-12);
  EXPECT_NEAR(F(1, 2),  (1.0 - std::cos(w * dt)) / w, 1e-12);
  EXPECT_NEAR(F(1, 3),  std::sin(w * dt) / w, 1e-12);
  EXPECT_NEAR(F(4, 4),  1.0, 1e-12);
}

TEST(CoordinatedTurn, ProcessNoiseOmegaDiagonal) {
  CoordinatedTurn m(0.5, 0.02);
  const Eigen::MatrixXd Q = m.processNoise(1.0);
  ASSERT_EQ(Q.rows(), 5);
  ASSERT_EQ(Q.cols(), 5);
  EXPECT_NEAR(Q(4, 4), 0.02 * 1.0, 1e-12);
  EXPECT_NEAR(Q(0, 0), 0.5 * 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(Q(2, 2), 0.5 * 1.0, 1e-12);
}
