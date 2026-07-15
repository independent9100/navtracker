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

// W4.1 anti-proliferation teeth (see EkfEstimator counterpart): a RangeBearing2D
// birth converts polar→ENU (about the sensor) into every mode; next scan gates.
TEST(ImmEstimator, InitiateFromRangeBearingConvertsToEnuAndNextScanGates) {
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(0.1, 0.01),
      std::make_shared<CoordinatedTurn>(0.1, 0.05)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05, 0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  const Eigen::Vector2d sensor(500.0, -300.0);
  const double range = 800.0, bearing = 0.6;
  const Eigen::Vector2d truth(sensor.x() + range * std::cos(bearing),
                              sensor.y() + range * std::sin(bearing));
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.source_id = "radar";
  z.value = Eigen::Vector2d(range, bearing);
  z.sensor_position_enu = sensor;
  Eigen::Matrix2d polar = Eigen::Matrix2d::Zero();
  polar(0, 0) = 25.0;
  polar(1, 1) = 0.02 * 0.02;
  z.covariance = polar;

  const navtracker::Track t = imm.initiate(z);
  // Every mode seeded at the true ENU position (moment-matched projection too).
  EXPECT_NEAR(t.state(0), truth.x(), 1.0);
  EXPECT_NEAR(t.state(1), truth.y(), 1.0);
  EXPECT_NEAR(t.imm_means(0, 0), truth.x(), 1.0);
  EXPECT_NEAR(t.imm_means(0, 1), truth.x(), 1.0);
  Measurement z2 = z;
  z2.time = Timestamp::fromSeconds(1.0);
  EXPECT_TRUE(imm.gate(t, z2, 30.0));
}

// W4.2 on the DEPLOYED path: the canonical IMM configs run use_ukf=true, so the
// circular-mean fix must hold in ImmEstimator::update's inner-UKF branch, not
// just standalone UkfEstimator. Same due-west straddle scenario as the UKF test.
TEST(ImmEstimator, DueWestBearingCircularMeanInUkfInnerFilter) {
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(0.1, 0.01),
      std::make_shared<CoordinatedTurn>(0.1, 0.05)};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05, 0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  // use_ukf=true, wide ukf_alpha=1 so sigma points genuinely straddle ±π.
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1, nullptr, false,
                   /*use_ukf=*/true, /*ukf_alpha=*/1.0, /*ukf_beta=*/2.0,
                   /*ukf_kappa=*/0.0);

  // Build an IMM track at the true due-west target with a wide cross-range prior.
  navtracker::Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  const int K = 2;
  t.imm_means = Eigen::MatrixXd::Zero(5, K);
  t.imm_covariances.clear();
  for (int j = 0; j < K; ++j) {
    t.imm_means.col(j) << -1000.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, 5);
    P(0, 0) = 100.0;
    P(1, 1) = 640000.0;  // σ=800 m cross-range → strong ±π straddle
    P(2, 2) = 25.0; P(3, 3) = 25.0; P(4, 4) = 0.01;
    t.imm_covariances.push_back(P);
  }
  t.imm_mode_probabilities = mu0;
  t.state = t.imm_means.col(0);
  t.covariance = t.imm_covariances[0];

  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(1000.0, 3.14159265358979323846);  // due west
  z.covariance = Eigen::Matrix2d::Zero();
  z.covariance(0, 0) = 25.0;
  z.covariance(1, 1) = 0.02 * 0.02;
  imm.update(t, z);

  // Consistent westward measurement ⇒ ≈0 innovation with the circular mean; the
  // linear-mean bug injects a spurious cross-range innovation into every mode.
  EXPECT_NEAR(t.state(1), 0.0, 30.0)
      << "IMM inner-UKF must use the circular bearing mean on the deployed path";
  EXPECT_NEAR(t.state(0), -1000.0, 30.0);
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

TEST(ImmEstimator, UkfInnerFilterTracksAndShrinksLikeEkfOnLinearMeasurement) {
  // Cl-2 #3 sanity check. Position2D is a linear measurement
  // (H = [I 0]); for a linear h(x) the UKF sigma-point reconstruction
  // produces (z_pred, S, Pxz) that exactly equal the EKF's
  // (H x, H P H', P H'). UKF and EKF inner filters must therefore land
  // within numerical noise of each other on this case.
  auto cv = std::make_shared<ConstantVelocity5State>(0.1, 0.01);
  auto ct = std::make_shared<CoordinatedTurn>(0.1, 0.05);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;

  ImmEstimator imm_ekf(motions, pi, mu0, 10.0, 0.1);
  ImmEstimator imm_ukf(motions, pi, mu0, 10.0, 0.1,
                       /*noise=*/nullptr,
                       /*bearing_range_guard=*/false,
                       /*use_ukf=*/true);

  navtracker::Track t_ekf = imm_ekf.initiate(positionMeas(0.0, 0.0, 20.0, 0.0));
  navtracker::Track t_ukf = imm_ukf.initiate(positionMeas(0.0, 0.0, 20.0, 0.0));
  imm_ekf.predict(t_ekf, Timestamp::fromSeconds(1.0));
  imm_ukf.predict(t_ukf, Timestamp::fromSeconds(1.0));
  imm_ekf.update(t_ekf, positionMeas(5.0, 0.0, 1.0, 1.0));
  imm_ukf.update(t_ukf, positionMeas(5.0, 0.0, 1.0, 1.0));

  // Position mean and covariance match within tight numerical tolerance.
  EXPECT_NEAR((t_ukf.state.head<2>() - t_ekf.state.head<2>()).norm(),
              0.0, 1e-6);
  EXPECT_NEAR((t_ukf.covariance.topLeftCorner<2, 2>() -
               t_ekf.covariance.topLeftCorner<2, 2>())
                  .norm(),
              0.0, 1e-6);
  // Mode probabilities also match (likelihood reconstruction is path-
  // equivalent for linear h).
  EXPECT_NEAR((t_ukf.imm_mode_probabilities -
               t_ekf.imm_mode_probabilities).norm(),
              0.0, 1e-6);
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

TEST(ImmEstimator, SoftUpdateWithSingleConfidentMeasurementMatchesHardUpdate) {
  // β = [1], β_0 = 0 should be exactly equivalent to a hard update().
  auto cv = std::make_shared<ConstantVelocity5State>(0.5, 0.01);
  auto ct = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  navtracker::Track hard = imm.initiate(positionMeas(0.0, 0.0, 5.0, 0.0));
  navtracker::Track soft = hard;
  const Measurement z = positionMeas(1.0, 0.5, 1.0, 0.0);

  imm.update(hard, z);

  Eigen::VectorXd betas(1);
  betas << 1.0;
  imm.softUpdate(soft, {z}, betas, /*beta_0=*/0.0);

  for (int i = 0; i < hard.state.size(); ++i)
    EXPECT_NEAR(soft.state(i), hard.state(i), 1e-9);
  EXPECT_NEAR((soft.covariance - hard.covariance).norm(), 0.0, 1e-6);
}

TEST(ImmEstimator, SoftUpdateActuallyMovesStateTowardMeasurement) {
  // The bug being regression-tested: ImmEstimator inherited the no-op
  // softUpdate default, so JPDA-driven IMM tracks never folded in any
  // measurement and their velocity stayed at the (0,0) initial seed.
  auto cv = std::make_shared<ConstantVelocity5State>(0.5, 0.01);
  auto ct = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  navtracker::Track t = imm.initiate(positionMeas(0.0, 0.0, 5.0, 0.0));
  const auto before = t.state;
  Eigen::VectorXd betas(1);
  betas << 0.8;
  imm.softUpdate(t, {positionMeas(10.0, 0.0, 1.0, 1.0)}, betas, /*beta_0=*/0.2);

  // State must move toward the (10, 0) measurement.
  EXPECT_GT(t.state(0), before(0) + 1.0)
      << "softUpdate did not move x toward the measurement — IMM's "
         "softUpdate is the no-op default again.";
  EXPECT_NEAR(t.imm_mode_probabilities.sum(), 1.0, 1e-9);
}

TEST(ImmEstimator, LogLikelihoodIsModeWeightedMixtureNotMomentMatched) {
  // The bug task #1 was supposed to fix: gating/scoring through the
  // moment-matched track.covariance double-counts inter-mode mean
  // separation when modes disagree, making the surrogate covariance
  // strictly larger than any individual mode's S. The proper
  // logLikelihood is the mode-weighted mixture log Σⱼ μⱼ · N(z; ẑⱼ, Sⱼ).
  // Construct a track whose two modes have visibly different means;
  // compare the IMM logLikelihood against the moment-matched
  // single-Gaussian baseline. The mixture must beat the baseline at a
  // point that lies near one mode (not at the mixture mean).
  auto cv = std::make_shared<ConstantVelocity5State>(0.5, 0.01);
  auto ct = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  navtracker::Track t = imm.initiate(positionMeas(0.0, 0.0, 5.0, 0.0));
  // Force the two modes' means apart by hand. Mode 0 sits at (10, 0),
  // mode 1 sits at (-10, 0); both with unit covariance. Equal mixing
  // weights.
  t.imm_means.resize(5, 2);
  t.imm_means.col(0) << 10.0, 0.0, 0.0, 0.0, 0.0;
  t.imm_means.col(1) << -10.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(5, 5);
  t.imm_covariances = {P, P};
  t.imm_mode_probabilities = Eigen::Vector2d(0.5, 0.5);

  // A measurement at (10, 0) — exactly on mode 0's mean.
  const auto z = positionMeas(10.0, 0.0, 1.0, 1.0);

  // For comparison: project the mixture to a single Gaussian
  // (what the OLD code did) and score that.
  Eigen::VectorXd x_proj = 0.5 * t.imm_means.col(0) + 0.5 * t.imm_means.col(1);
  Eigen::MatrixXd P_proj = Eigen::MatrixXd::Zero(5, 5);
  for (int j = 0; j < 2; ++j) {
    const Eigen::VectorXd d = t.imm_means.col(j) - x_proj;
    P_proj += 0.5 * (t.imm_covariances[j] + d * d.transpose());
  }
  navtracker::Track t_proj = t;
  t_proj.imm_means.resize(0, 0);
  t_proj.imm_covariances.clear();
  t_proj.state = x_proj;
  t_proj.covariance = P_proj;

  const double mixture_ll = imm.logLikelihood(t, z);
  const double moment_matched_ll = imm.logLikelihood(t_proj, z);

  // The mixture likelihood at (10, 0) is dominated by mode 0
  // (unit covariance, μ=0.5), so it should significantly beat the
  // moment-matched score (wide projected covariance that includes the
  // 20m spread between modes).
  EXPECT_GT(mixture_ll, moment_matched_ll)
      << "Mixture log-likelihood should beat moment-matched at a "
         "near-mode point; mixture=" << mixture_ll
      << " moment_matched=" << moment_matched_ll;
}

TEST(ImmEstimator, AnyModeGateAcceptsNearAnyMode) {
  // Mazor 1998 §V: IMM gate passes iff any mode's per-mode gate
  // passes. Construct the same two-mode track as above, then verify
  // a measurement near mode 0 passes the gate AND a measurement near
  // mode 1 passes the gate, even though both lie *outside* a
  // moment-matched single-Gaussian gate centered on the projected
  // mean (0, 0).
  auto cv = std::make_shared<ConstantVelocity5State>(0.5, 0.01);
  auto ct = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05, 0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  navtracker::Track t = imm.initiate(positionMeas(0.0, 0.0, 5.0, 0.0));
  t.imm_means.resize(5, 2);
  t.imm_means.col(0) << 10.0, 0.0, 0.0, 0.0, 0.0;
  t.imm_means.col(1) << -10.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(5, 5);
  t.imm_covariances = {P, P};
  t.imm_mode_probabilities = Eigen::Vector2d(0.5, 0.5);

  const auto z_near_mode0 = positionMeas(10.5, 0.0, 1.0, 1.0);
  const auto z_near_mode1 = positionMeas(-10.5, 0.0, 1.0, 1.0);
  const auto z_in_no_mode = positionMeas(0.0, 0.0, 1.0, 1.0);

  EXPECT_TRUE(imm.gate(t, z_near_mode0, 9.0));
  EXPECT_TRUE(imm.gate(t, z_near_mode1, 9.0));
  // (0, 0) is at the moment-matched centre but 10σ away from each
  // individual mode mean — must be REJECTED by any-mode gating.
  EXPECT_FALSE(imm.gate(t, z_in_no_mode, 9.0));
}

// --- dt-scaled mode transitions --------------------------------------------
//
// The configured transition matrix is the 1-second TPM. predict() must
// apply π^dt (continuous-time semantics), not π once per call: per-call
// application at 16 Hz mixes modes 16× faster than the same parameters
// at 1 Hz, washing the mode probabilities toward stationarity between
// every measurement (observed on AutoFerry: IMM ≈ blurred single model).
// predict() advances the mode probabilities to the predicted prior
// c = π(dt)ᵀ μ; update() consumes that prior directly.

TEST(ImmEstimator, ModeTransitionIsDtScaled) {
  auto cv = std::make_shared<ConstantVelocity5State>(0.0, 0.0);
  auto ct = std::make_shared<CoordinatedTurn>(0.0, 0.0);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.9, 0.1,
        0.1, 0.9;  // symmetric: eigenvalues 1 and 0.8
  Eigen::VectorXd mu0(2);
  mu0 << 1.0, 0.0;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  // dt = 1 s: the classical one-step prior [0.9, 0.1].
  navtracker::Track a = imm.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  imm.predict(a, Timestamp::fromSeconds(1.0));
  EXPECT_NEAR(a.imm_mode_probabilities(0), 0.9, 1e-12);
  EXPECT_NEAR(a.imm_mode_probabilities(1), 0.1, 1e-12);

  // dt = 0.1 s: π^0.1 has diagonal (1 + 0.8^0.1)/2 — mode 0 keeps
  // almost all of its probability instead of leaking 0.1 per call.
  const double lam = std::pow(0.8, 0.1);
  navtracker::Track b = imm.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  imm.predict(b, Timestamp::fromSeconds(0.1));
  EXPECT_NEAR(b.imm_mode_probabilities(0), (1.0 + lam) / 2.0, 1e-9);
  EXPECT_NEAR(b.imm_mode_probabilities(1), (1.0 - lam) / 2.0, 1e-9);
}

TEST(ImmEstimator, ModeTransitionComposesAcrossPredicts) {
  // Semigroup property: predicting 0 → 0.5 s → 1.0 s must land on the
  // same mode prior as one 0 → 1.0 s predict. Per-call TPM application
  // fails this badly (two applications vs one).
  auto cv = std::make_shared<ConstantVelocity5State>(0.0, 0.0);
  auto ct = std::make_shared<CoordinatedTurn>(0.0, 0.0);
  std::vector<std::shared_ptr<navtracker::IMotionModel>> motions = {cv, ct};
  Eigen::MatrixXd pi(2, 2);
  pi << 0.95, 0.05,
        0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 1.0, 0.0;
  ImmEstimator imm(motions, pi, mu0, 10.0, 0.1);

  navtracker::Track two_steps = imm.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  imm.predict(two_steps, Timestamp::fromSeconds(0.5));
  imm.predict(two_steps, Timestamp::fromSeconds(1.0));

  navtracker::Track one_step = imm.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  imm.predict(one_step, Timestamp::fromSeconds(1.0));

  ASSERT_EQ(two_steps.imm_mode_probabilities.size(),
            one_step.imm_mode_probabilities.size());
  for (int j = 0; j < one_step.imm_mode_probabilities.size(); ++j) {
    EXPECT_NEAR(two_steps.imm_mode_probabilities(j),
                one_step.imm_mode_probabilities(j), 1e-9);
  }
}
