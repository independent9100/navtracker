#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/UkfEstimator.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::CoordinatedTurn;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::UkfEstimator;

TEST(UkfEstimator, PredictAdvancesPositionAndGrowsCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const UkfEstimator ukf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(0.0, 0.0, 2.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  ukf.predict(t, Timestamp::fromSeconds(3.0));
  EXPECT_NEAR(t.state(0), 6.0, 1e-9);
  EXPECT_NEAR(t.state(1), 0.0, 1e-9);
  EXPECT_NEAR(t.state(2), 2.0, 1e-9);
  EXPECT_GT(t.covariance(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 3.0);
}

TEST(UkfEstimator, PositionUpdatePullsStateAndShrinksCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const UkfEstimator ukf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(10.0);
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity() * 100.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(10.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  ukf.update(t, z);
  EXPECT_GT(t.state(0), 9.0);
  EXPECT_LT(t.state(0), 10.0);
  EXPECT_NEAR(t.state(1), 0.0, 1e-6);
  EXPECT_LT(t.covariance(0, 0), 100.0);
}

TEST(UkfEstimator, RangeBearingUpdateOnConsistentMeasurementIsStable) {
  // Realistic maritime geometry: 5 km range, 10 m position uncertainty.
  // Far enough that the second-moment range bias is negligible (<< 1 m), so
  // a measurement matching h(state) doesn't perturb the state appreciably.
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const UkfEstimator ukf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(3000.0, 4000.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 100.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(5000.0, std::atan2(4000.0, 3000.0));
  z.covariance = Eigen::Matrix2d::Identity() * 1.0;
  ukf.update(t, z);
  EXPECT_NEAR(t.state(0), 3000.0, 1.0);
  EXPECT_NEAR(t.state(1), 4000.0, 1.0);
  EXPECT_LT(t.covariance(0, 0), 100.0);
}

TEST(UkfEstimator, InitiateSeedsStateFromPositionMeasurement) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const UkfEstimator ukf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "ais";
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * 9.0;
  z.hints.mmsi = 211000000u;
  const Track t = ukf.initiate(z);
  EXPECT_EQ(t.status, navtracker::TrackStatus::Tentative);
  EXPECT_DOUBLE_EQ(t.state(0), 100.0);
  EXPECT_DOUBLE_EQ(t.state(1), -50.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 9.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 64.0);
  ASSERT_TRUE(t.attributes.mmsi.has_value());
  EXPECT_EQ(*t.attributes.mmsi, 211000000u);
}

// W4.1 anti-proliferation teeth (see EkfEstimator counterpart): a RangeBearing2D
// birth must convert polar→ENU (about the sensor) and the next scan must gate.
TEST(UkfEstimator, InitiateFromRangeBearingConvertsToEnuAndNextScanGates) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const UkfEstimator ukf(model);
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

  const Track t = ukf.initiate(z);
  EXPECT_NEAR(t.state(0), truth.x(), 1.0);
  EXPECT_NEAR(t.state(1), truth.y(), 1.0);
  Measurement z2 = z;
  z2.time = Timestamp::fromSeconds(1.0);
  EXPECT_TRUE(ukf.gate(t, z2, 30.0));
}

// W4.2: a target ~due WEST of the sensor has a well-determined bearing (≈π),
// but its sigma-point bearings straddle the ±π branch cut. With the buggy linear
// mean the predicted bearing collapses to ≈0 and the innovation covariance
// FALSELY inflates → the (consistent) measurement is treated as uninformative →
// the cross-range (north) variance does NOT shrink. The circular mean fixes the
// predicted bearing to ≈π so the measurement constrains cross-range as it should.
TEST(UkfEstimator, DueWestBearingCircularMeanKeepsUpdateConsistent) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  // Wide sigma spread (alpha=1) so the sigma points genuinely straddle ±π for a
  // due-west target; the production-default tiny alpha keeps points hugging the
  // mean, where the bug is dormant. beta=2, kappa=0.
  const UkfEstimator ukf(model, /*init_speed_std=*/8.0, /*init_omega_std=*/0.1,
                         /*alpha=*/1.0, /*beta=*/2.0, /*kappa=*/0.0);
  // Track AT the true target, due west of the sensor, with a wide cross-range
  // (north) prior so the sigma-point bearings straddle the ±π branch cut.
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(-1000.0, 0.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Zero();
  t.covariance(0, 0) = 100.0;
  t.covariance(1, 1) = 640000.0;  // σ=800 m → strong straddle across ±π
  t.covariance(2, 2) = 25.0;
  t.covariance(3, 3) = 25.0;

  // A CONSISTENT measurement of the true target: range 1000, bearing = π.
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(1000.0, 3.14159265358979323846);
  z.covariance = Eigen::Matrix2d::Zero();
  z.covariance(0, 0) = 25.0;
  z.covariance(1, 1) = 0.02 * 0.02;
  ukf.update(t, z);

  // With the circular mean the predicted bearing is ≈π, so a consistent
  // measurement produces a ≈0 innovation: the state stays put and the
  // cross-range variance collapses. The linear-mean bug drags the predicted
  // bearing off π, injecting a spurious cross-range innovation that shoves the
  // north estimate far from the (correct) 0 and corrupts the covariance.
  EXPECT_NEAR(t.state(1), 0.0, 30.0)
      << "consistent westward measurement must not shove the cross-range "
         "estimate; the ±π linear-mean bug injects a spurious innovation";
  EXPECT_NEAR(t.state(0), -1000.0, 30.0);
  EXPECT_LT(t.covariance(1, 1), 0.05 * 640000.0);  // cross-range gets constrained
}

TEST(UkfEstimator, AgreesWithEkfOnLinearPositionUpdate) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const navtracker::EkfEstimator ekf(model, 5.0);
  const UkfEstimator ukf(model, 5.0);

  Track t0;
  t0.last_update = Timestamp::fromSeconds(0.0);
  t0.state = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
  t0.covariance = Eigen::Matrix4d::Identity() * 10.0;

  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(2.0, 1.0);
  z.covariance = Eigen::Matrix2d::Identity() * 0.5;

  Track t_ekf = t0;
  Track t_ukf = t0;
  ekf.update(t_ekf, z);
  ukf.update(t_ukf, z);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(t_ekf.state(i), t_ukf.state(i), 1e-9);
  }
  EXPECT_TRUE(t_ekf.covariance.isApprox(t_ukf.covariance, 1e-9));
}

// Standalone UKF + CoordinatedTurn must initiate a 5-state track and
// then predict/update without reading past the state vector. Before the
// fix, initiate() hardcoded Vector4d while CT::propagate reads x(4) —
// undefined behaviour that produced ~1e5 m/s velocity errors in the
// benchmark `ukf_ct_gnn` column.
TEST(UkfEstimator, InitiateWithCtMotionProducesFiveStateTrack) {
  auto model = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  const UkfEstimator ukf(model, /*init_speed_std=*/10.0,
                         /*init_omega_std=*/0.1);
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * 9.0;
  const Track t = ukf.initiate(z);
  ASSERT_EQ(t.state.size(), 5);
  ASSERT_EQ(t.covariance.rows(), 5);
  ASSERT_EQ(t.covariance.cols(), 5);
  EXPECT_DOUBLE_EQ(t.state(0), 100.0);
  EXPECT_DOUBLE_EQ(t.state(1), -50.0);
  EXPECT_DOUBLE_EQ(t.state(2), 0.0);
  EXPECT_DOUBLE_EQ(t.state(3), 0.0);
  EXPECT_DOUBLE_EQ(t.state(4), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 100.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3), 100.0);
  EXPECT_DOUBLE_EQ(t.covariance(4, 4), 0.01);
}

TEST(UkfEstimator, PredictAndUpdateWithCtMotionAreStable) {
  // Same plant the production `ukf_ct_gnn` config uses. A non-zero turn
  // rate forces propagate() to use x(4) — if initiate() ever regresses to
  // a 4-state vector this test should crash or produce NaNs.
  auto model = std::make_shared<CoordinatedTurn>(0.5, 0.1);
  const UkfEstimator ukf(model, /*init_speed_std=*/10.0,
                         /*init_omega_std=*/0.1);
  Measurement z0;
  z0.time = Timestamp::fromSeconds(0.0);
  z0.model = MeasurementModel::Position2D;
  z0.value = Eigen::Vector2d(0.0, 0.0);
  z0.covariance = Eigen::Matrix2d::Identity() * 4.0;
  Track t = ukf.initiate(z0);
  t.state(2) = 5.0;   // 5 m/s eastward.
  t.state(4) = 0.05;  // gentle turn ~2.9 deg/s.

  for (int k = 1; k <= 10; ++k) {
    ukf.predict(t, Timestamp::fromSeconds(k * 1.0));
    Measurement zk;
    zk.time = Timestamp::fromSeconds(k * 1.0);
    zk.model = MeasurementModel::Position2D;
    zk.value = t.state.head<2>();  // self-consistent measurement
    zk.covariance = Eigen::Matrix2d::Identity() * 1.0;
    ukf.update(t, zk);
    ASSERT_TRUE(t.state.allFinite());
    ASSERT_TRUE(t.covariance.allFinite());
    ASSERT_LT(std::abs(t.state(2)), 50.0)  // speed stays sane
        << "step " << k << " vx=" << t.state(2);
    ASSERT_LT(std::abs(t.state(3)), 50.0)
        << "step " << k << " vy=" << t.state(3);
  }
}

// UKF tracks the closed-form CT step; EKF doesn't. Quarter-arc (90°) at
// ω = π/2 / 60 s, speed 5 m/s. With a near-deterministic prior the UKF
// sigma points collapse onto the mean and propagate through the
// nonlinear CT step, so UKF's predicted mean equals the analytic
// arc-end. EKF::predict uses motion_->transitionMatrix(dt), which for
// CoordinatedTurn is the CV-limit F (ω = 0 — see CoordinatedTurn.cpp:34
// "exists only to satisfy IMotionModel for callers that ignore state"),
// so EKF predicts a straight line and ends the arc several hundred
// metres away from truth.
//
// This test fails before the fix because UkfEstimator::initiate was a
// hardcoded Vector4d — predicting a 5-state CT couldn't even run.
TEST(UkfEstimator, BeatsEkfOnCoordinatedTurnQuarterArc) {
  const double speed = 5.0;
  const double dt = 60.0;
  const double omega = (M_PI / 2.0) / dt;  // 90° over 60 s
  auto model = std::make_shared<CoordinatedTurn>(/*accel_psd=*/0.0,
                                                  /*omega_psd=*/0.0);
  const UkfEstimator ukf(model, 10.0, 0.1);
  const navtracker::EkfEstimator ekf(model, 10.0);

  Eigen::VectorXd x0(5);
  x0 << 0.0, 0.0, speed, 0.0, omega;
  // Near-deterministic prior so the unscented mean collapses onto
  // propagate(x0, dt). Not literally zero so Cholesky in SigmaPoints
  // succeeds.
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(5, 5) * 1e-12;

  Track tu;
  tu.state = x0; tu.covariance = P0;
  tu.last_update = Timestamp::fromSeconds(0.0);
  Track te = tu;

  ukf.predict(tu, Timestamp::fromSeconds(dt));
  ekf.predict(te, Timestamp::fromSeconds(dt));

  const Eigen::VectorXd truth = model->propagate(x0, dt);
  const double err_ukf = (tu.state.head<2>() - truth.head<2>()).norm();
  const double err_ekf = (te.state.head<2>() - truth.head<2>()).norm();

  // Expected: UKF ≈ 0 (closed-form), EKF ≈ chord-vs-arc gap ~ 88 m
  // (chord across a quarter circle of radius v/ω ≈ 191 m).
  EXPECT_LT(err_ukf, 1e-6) << "UKF arc-end error " << err_ukf << " m";
  EXPECT_GT(err_ekf, 50.0) << "EKF arc-end error " << err_ekf << " m";
  EXPECT_GT(err_ekf, err_ukf * 1e6)
      << "EKF should dominate UKF by many orders of magnitude on CT predict";
}

TEST(UkfEstimator, AgreesWithEkfOnLinearPredict) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const navtracker::EkfEstimator ekf(model, 5.0);
  const UkfEstimator ukf(model, 5.0);

  Track t0;
  t0.last_update = Timestamp::fromSeconds(0.0);
  t0.state = Eigen::Vector4d(1.0, 2.0, 3.0, -1.0);
  t0.covariance = Eigen::Matrix4d::Identity() * 4.0;

  Track t_ekf = t0;
  Track t_ukf = t0;
  ekf.predict(t_ekf, Timestamp::fromSeconds(2.0));
  ukf.predict(t_ukf, Timestamp::fromSeconds(2.0));

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(t_ekf.state(i), t_ukf.state(i), 1e-9);
  }
  EXPECT_TRUE(t_ekf.covariance.isApprox(t_ukf.covariance, 1e-9));
}
