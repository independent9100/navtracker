#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/ImmEstimator.hpp"

using navtracker::ConstantVelocity5State;
using navtracker::CoordinatedTurn;
using navtracker::ImmEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;

namespace {

Measurement positionMeas(double x, double y, double std_m, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m);
  m.source_id = "test";
  return m;
}

}  // namespace

TEST(ImmEstimator, InitiateSeedsModesUniformly) {
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(0.1, 0.01),
      std::make_shared<CoordinatedTurn>(0.1, 0.05)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  const navtracker::Track t = imm.initiate(positionMeas(100.0, -50.0, 3.0, 0.0));
  ASSERT_EQ(t.imm_means.rows(), 5);
  ASSERT_EQ(t.imm_means.cols(), 2);
  ASSERT_EQ(t.imm_covariances.size(), 2u);
  ASSERT_EQ(t.imm_mode_probabilities.size(), 2);
  EXPECT_NEAR(t.imm_mode_probabilities(0), 0.5, 1e-12);
  EXPECT_NEAR(t.imm_mode_probabilities(1), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(t.imm_means(0, 0), 100.0);
  EXPECT_DOUBLE_EQ(t.imm_means(0, 1), 100.0);
  EXPECT_DOUBLE_EQ(t.imm_means(1, 0), -50.0);
  EXPECT_DOUBLE_EQ(t.imm_means(1, 1), -50.0);
  EXPECT_DOUBLE_EQ(t.imm_means(4, 0), 0.0);
  EXPECT_DOUBLE_EQ(t.imm_means(4, 1), 0.0);
  EXPECT_EQ(t.state.size(), 5);
  EXPECT_DOUBLE_EQ(t.state(0), 100.0);
  EXPECT_DOUBLE_EQ(t.state(1), -50.0);
  EXPECT_NEAR(t.covariance(2, 2), 100.0, 1e-9);
  EXPECT_NEAR(t.covariance(4, 4), 0.01, 1e-9);
}

TEST(ImmEstimator, PredictAdvancesModesByTheirOwnDynamics) {
  auto cv  = std::make_shared<ConstantVelocity5State>(0.0, 0.0);
  auto ct  = std::make_shared<CoordinatedTurn>(0.0, 0.0);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 1.0, 0.0;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);
  navtracker::Track t = imm.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  for (int j = 0; j < 2; ++j) {
    t.imm_means(2, j) = 5.0;
    t.imm_means(3, j) = 0.0;
    t.imm_means(4, j) = 0.5;
  }
  imm.predict(t, Timestamp::fromSeconds(1.0));
  EXPECT_NEAR(t.imm_means(0, 0), 5.0, 1e-9);
  EXPECT_NEAR(t.imm_means(1, 0), 0.0, 1e-9);
  EXPECT_NEAR(t.imm_means(2, 1), 5.0 * std::cos(0.5), 1e-9);
  EXPECT_NEAR(t.imm_means(3, 1), 5.0 * std::sin(0.5), 1e-9);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 1.0);
}

TEST(ImmEstimator, PredictOnEmptyImmIsNoOp) {
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(0.1, 0.01)};
  Eigen::MatrixXd pi(1, 1);
  pi << 1.0;
  Eigen::VectorXd mu0(1);
  mu0 << 1.0;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);
  navtracker::Track t;
  const navtracker::Timestamp before = t.last_update;
  imm.predict(t, Timestamp::fromSeconds(5.0));
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), before.seconds());
}

TEST(ImmEstimator, UpdateShrinksPositionCovariance) {
  auto cv  = std::make_shared<ConstantVelocity5State>(0.1, 0.01);
  auto ct  = std::make_shared<CoordinatedTurn>(0.1, 0.05);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);
  navtracker::Track t = imm.initiate(positionMeas(0.0, 0.0, 20.0, 0.0));
  const double var_before = t.covariance(0, 0);
  imm.update(t, positionMeas(0.0, 0.0, 1.0, 0.0));
  EXPECT_LT(t.covariance(0, 0), var_before * 0.2);
  EXPECT_LT(t.covariance(1, 1), var_before * 0.2);
  EXPECT_NEAR(t.imm_mode_probabilities.sum(), 1.0, 1e-9);
  EXPECT_GE(t.imm_mode_probabilities(0), 0.0);
  EXPECT_GE(t.imm_mode_probabilities(1), 0.0);
}

TEST(ImmEstimator, UpdateOnEmptyImmIsNoOp) {
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(0.1, 0.01)};
  Eigen::MatrixXd pi(1, 1);
  pi << 1.0;
  Eigen::VectorXd mu0(1);
  mu0 << 1.0;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);
  navtracker::Track t;
  imm.update(t, positionMeas(0.0, 0.0, 1.0, 1.0));
  EXPECT_EQ(t.imm_means.cols(), 0);
}
