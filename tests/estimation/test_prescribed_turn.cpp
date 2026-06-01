#include <gtest/gtest.h>
#include <cmath>
#include "core/estimation/PrescribedTurn.hpp"

using navtracker::PrescribedTurn;

TEST(PrescribedTurn, TransitionMatrixUsesPrescribedOmegaNotState) {
  const double w_pre = 0.3;
  PrescribedTurn m(w_pre, 0.1, 0.001);
  const Eigen::MatrixXd F = m.transitionMatrix(2.0);
  ASSERT_EQ(F.rows(), 5);
  ASSERT_EQ(F.cols(), 5);
  // Velocity-rotation block is R(w_pre · dt)
  const double wdt = w_pre * 2.0;
  EXPECT_NEAR(F(2, 2),  std::cos(wdt), 1e-12);
  EXPECT_NEAR(F(2, 3), -std::sin(wdt), 1e-12);
  EXPECT_NEAR(F(3, 2),  std::sin(wdt), 1e-12);
  EXPECT_NEAR(F(3, 3),  std::cos(wdt), 1e-12);
  // Omega row is identity (state omega is passive)
  EXPECT_DOUBLE_EQ(F(4, 4), 1.0);
}

TEST(PrescribedTurn, ZeroPrescribedOmegaReducesToCV) {
  PrescribedTurn m(0.0, 0.1, 0.001);
  const Eigen::MatrixXd F = m.transitionMatrix(2.0);
  EXPECT_NEAR(F(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(F(0, 2), 2.0, 1e-12);
  EXPECT_NEAR(F(1, 3), 2.0, 1e-12);
  EXPECT_NEAR(F(2, 2), 1.0, 1e-12);
  EXPECT_NEAR(F(3, 3), 1.0, 1e-12);
  EXPECT_NEAR(F(2, 3), 0.0, 1e-12);
}

TEST(PrescribedTurn, ProcessNoiseHasOmegaDiagonal) {
  PrescribedTurn m(0.2, 0.5, 0.02);
  const Eigen::MatrixXd Q = m.processNoise(1.0);
  EXPECT_NEAR(Q(4, 4), 0.02 * 1.0, 1e-12);
  EXPECT_NEAR(Q(2, 2), 0.5 * 1.0, 1e-12);
}
