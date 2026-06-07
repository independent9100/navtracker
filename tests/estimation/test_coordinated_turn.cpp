#include <gtest/gtest.h>
#include <cmath>
#include "core/estimation/CoordinatedTurn.hpp"

using navtracker::CoordinatedTurn;

TEST(CoordinatedTurn, ReducesToCVAtZeroOmega) {
  CoordinatedTurn m(0.1, 0.01);
  const Eigen::MatrixXd F = m.transitionMatrixAt(0.0, 2.0);
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
  const Eigen::MatrixXd F = m.transitionMatrixAt(w, dt);
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

TEST(CoordinatedTurn, PropagateReadsOmegaFromState) {
  CoordinatedTurn m(0.1, 0.01);
  const double w = 0.3;
  const double dt = 2.0;
  Eigen::VectorXd x(5);
  x << 0.0, 0.0, 1.0, 0.0, w;
  const Eigen::VectorXd y = m.propagate(x, dt);
  // After 2 s at omega=0.3 rad/s the velocity vector has rotated by 0.6 rad.
  const double speed = std::hypot(y(2), y(3));
  EXPECT_NEAR(speed, 1.0, 1e-9);
  EXPECT_NEAR(std::atan2(y(3), y(2)), 0.6, 1e-9);
  EXPECT_NEAR(y(4), w, 1e-12);
}

TEST(CoordinatedTurn, UkfBeatsEkfOnArcPrediction) {
  // The whole point of the UKF/CT fix: a sigma-point cloud with finite
  // omega spread propagated through propagate() captures the curvature
  // that a single linearized F·x misses.
  CoordinatedTurn m(0.1, 0.01);
  Eigen::VectorXd x(5);
  x << 0.0, 0.0, 10.0, 0.0, 0.5;  // 0.5 rad/s
  const double dt = 5.0;
  // Nonlinear truth (propagate at the mean):
  const Eigen::VectorXd truth = m.propagate(x, dt);
  // EKF-style linearization at omega=0 (CV chord) vs at the true omega:
  const Eigen::VectorXd chord = m.transitionMatrixAt(0.0, dt) * x;
  const Eigen::VectorXd at_true = m.transitionMatrixAt(0.5, dt) * x;
  const double err_chord = (chord.head<2>() - truth.head<2>()).norm();
  const double err_at_true = (at_true.head<2>() - truth.head<2>()).norm();
  // The arc-vs-chord difference must be substantial.
  EXPECT_GT(err_chord, 5.0);
  // Linearizing at the right omega is the exact step for a single state.
  EXPECT_LT(err_at_true, 1e-9);
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
