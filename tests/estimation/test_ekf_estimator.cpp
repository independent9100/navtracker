#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;

TEST(EkfEstimator, PredictAdvancesPositionAndGrowsCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(0.0, 0.0, 2.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  ekf.predict(t, Timestamp::fromSeconds(3.0));
  EXPECT_DOUBLE_EQ(t.state(0), 6.0);
  EXPECT_DOUBLE_EQ(t.state(1), 0.0);
  EXPECT_DOUBLE_EQ(t.state(2), 2.0);
  EXPECT_GT(t.covariance(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 3.0);
}

TEST(EkfEstimator, PositionUpdatePullsStateAndShrinksCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(10.0);
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity() * 100.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(10.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  ekf.update(t, z);
  EXPECT_GT(t.state(0), 9.0);
  EXPECT_LT(t.state(0), 10.0);
  EXPECT_NEAR(t.state(1), 0.0, 1e-9);
  EXPECT_LT(t.covariance(0, 0), 100.0);
}

TEST(EkfEstimator, WrongSizeMeasurementCovarianceIsRejectedNotCrash) {
  // #35 M1: a Position2D (2-D) measurement carrying a 3x3 covariance. The
  // dimension-blind PSD guard passed it, then H·P·Hᵀ (2x2) + z.cov (3x3)
  // aborted in Eigen (debug) / read OOB (NDEBUG). The dimension check now
  // rejects it — the update is a safe no-op, state/covariance untouched.
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(10.0);
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity() * 100.0;
  const Eigen::Vector4d state0 = t.state;
  const Eigen::Matrix4d cov0 = t.covariance;
  Measurement z;
  z.time = Timestamp::fromSeconds(10.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::MatrixXd::Identity(3, 3);  // wrong size for a 2-D measurement
  ekf.update(t, z);
  EXPECT_EQ(t.state, state0);
  EXPECT_EQ(t.covariance, cov0);
}

TEST(EkfEstimator, RangeBearingUpdateOnConsistentMeasurementIsStable) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(3.0, 4.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 10.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(5.0, std::atan2(4.0, 3.0));
  z.covariance = Eigen::Matrix2d::Identity() * 0.01;
  ekf.update(t, z);
  EXPECT_NEAR(t.state(0), 3.0, 1e-6);
  EXPECT_NEAR(t.state(1), 4.0, 1e-6);
  EXPECT_LT(t.covariance(0, 0), 10.0);
}

TEST(EkfEstimator, InitiateSeedsStateFromPositionMeasurement) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "ais";
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * 9.0;
  z.hints.mmsi = 211000000u;
  const Track t = ekf.initiate(z);
  EXPECT_EQ(t.status, navtracker::TrackStatus::Tentative);
  EXPECT_DOUBLE_EQ(t.state(0), 100.0);
  EXPECT_DOUBLE_EQ(t.state(1), -50.0);
  EXPECT_DOUBLE_EQ(t.state(2), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 9.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 64.0);  // 8^2
  ASSERT_TRUE(t.attributes.mmsi.has_value());
  EXPECT_EQ(*t.attributes.mmsi, 211000000u);
  EXPECT_EQ(t.contributing_sources.size(), 1u);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 5.0);
}

// #20: a birth-velocity prior hint (ARPA TTM speed/course) seeds the birth
// velocity mean, while the birth covariance keeps the wide init_speed_std
// variance (the prior nudges direction without over-committing). Used only at
// initiate — a prior consumed once cannot double-count (guide §3).
TEST(EkfEstimator, InitiateSeedsBirthVelocityFromPriorHint) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "arpa";
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * 9.0;
  z.hints.birth_velocity_enu = Eigen::Vector2d(6.0, -2.0);
  const Track t = ekf.initiate(z);
  EXPECT_DOUBLE_EQ(t.state(2), 6.0);   // v_east seeded from the prior
  EXPECT_DOUBLE_EQ(t.state(3), -2.0);  // v_north seeded from the prior
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 64.0);  // variance still 8^2 (not tightened)
}

// W4.1 anti-proliferation teeth: a RangeBearing2D birth must land at the TRUE
// ENU position (converted from polar about the sensor), and the next scan of the
// same target must GATE to it. Pre-fix, initiate() planted (range_m, bearing_rad)
// as (east, north) with mixed m²/rad² covariance, so the born track sat far from
// the target and nothing gated → per-scan phantom proliferation.
TEST(EkfEstimator, InitiateFromRangeBearingConvertsToEnuAndNextScanGates) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  // Sensor away from the ENU origin so the polar-as-ENU bug is unmistakable.
  const Eigen::Vector2d sensor(500.0, -300.0);
  const double range = 800.0;
  const double bearing = 0.6;  // rad, math convention (CCW from East)
  const Eigen::Vector2d truth(sensor.x() + range * std::cos(bearing),
                              sensor.y() + range * std::sin(bearing));
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.source_id = "radar";
  z.value = Eigen::Vector2d(range, bearing);
  z.sensor_position_enu = sensor;
  Eigen::Matrix2d polar = Eigen::Matrix2d::Zero();
  polar(0, 0) = 25.0;          // range σ = 5 m
  polar(1, 1) = 0.02 * 0.02;   // bearing σ = 0.02 rad
  z.covariance = polar;

  const Track t = ekf.initiate(z);
  ASSERT_EQ(t.state.size(), 4);
  // (1) Born at the true ENU position, NOT at (range_m, bearing_rad).
  EXPECT_NEAR(t.state(0), truth.x(), 1.0);
  EXPECT_NEAR(t.state(1), truth.y(), 1.0);
  // ENU birth covariance is finite, positive, and reasonably sized (not rad²).
  EXPECT_GT(t.covariance(0, 0), 0.0);
  EXPECT_LT(t.covariance(0, 0), 1e4);
  EXPECT_LT(t.covariance(1, 1), 1e4);
  // (2) The next scan's range/bearing of the same target GATES to the track.
  Measurement z2 = z;
  z2.time = Timestamp::fromSeconds(1.0);
  EXPECT_TRUE(ekf.gate(t, z2, 30.0))
      << "a RangeBearing2D-born track must gate to the next scan of the same "
         "target (pre-fix it sat at (range,bearing) and never gated)";
}

TEST(EkfEstimator, InitiateWithEmptyCovarianceDoesNotCreateTrack) {
  // The documented "no uncertainty" sentinel is a 0x0 covariance. initiate()
  // must not read it out of bounds nor birth a track from it — it returns the
  // "did not initiate" result (empty state, no contributing sources).
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "ais";
  z.value = Eigen::Vector2d(100.0, -50.0);
  // Measurement::covariance is a dynamic MatrixXd; the documented
  // "no uncertainty" sentinel the builders leave behind is an empty 0x0
  // matrix (a fixed Matrix2d can never be 0x0).
  z.covariance = Eigen::MatrixXd();
  ASSERT_EQ(z.covariance.size(), 0);
  Track t;
  ASSERT_NO_THROW(t = ekf.initiate(z));
  EXPECT_EQ(t.state.size(), 0);
  EXPECT_TRUE(t.contributing_sources.empty());
}

TEST(EkfEstimator, InitiateWithNonPsdCovarianceDoesNotCreateTrack) {
  // A non-PSD covariance (negative eigenvalue) is also rejected, mirroring
  // update()'s skip guard.
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "ais";
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * -1.0;
  const Track t = ekf.initiate(z);
  EXPECT_EQ(t.state.size(), 0);
  EXPECT_TRUE(t.contributing_sources.empty());
}

TEST(EkfEstimator, InitiateDispatchesViaIEstimatorBaseReference) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 5.0);
  const navtracker::IEstimator& base = ekf;
  Measurement z;
  z.time = Timestamp::fromSeconds(1.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "s";
  z.value = Eigen::Vector2d(7.0, -3.0);
  z.covariance = Eigen::Matrix2d::Identity();
  const Track t = base.initiate(z);
  EXPECT_DOUBLE_EQ(t.state(0), 7.0);
  EXPECT_DOUBLE_EQ(t.state(1), -3.0);
  EXPECT_EQ(t.status, navtracker::TrackStatus::Tentative);
}

TEST(EkfEstimator, SoftUpdateWithOneMeasurementEqualsHardUpdate) {
  // betas = [1.0], beta_0 = 0: softUpdate must equal hard update.
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 5.0);
  z.covariance = Eigen::Matrix2d::Identity() * 4.0;
  z.source_id = "t";

  navtracker::Track t_hard = ekf.initiate(z);
  navtracker::Measurement z2 = z;
  z2.time = navtracker::Timestamp::fromSeconds(2.0);
  z2.value = Eigen::Vector2d(12.0, 6.0);
  ekf.predict(t_hard, z2.time);
  navtracker::Track t_soft = t_hard;
  ekf.update(t_hard, z2);

  Eigen::VectorXd betas(1);
  betas << 1.0;
  ekf.softUpdate(t_soft, {z2}, betas, 0.0);

  ASSERT_EQ(t_hard.state.size(), t_soft.state.size());
  for (int i = 0; i < t_hard.state.size(); ++i)
    EXPECT_NEAR(t_hard.state(i), t_soft.state(i), 1e-9);
  for (int r = 0; r < t_hard.covariance.rows(); ++r)
    for (int c = 0; c < t_hard.covariance.cols(); ++c)
      EXPECT_NEAR(t_hard.covariance(r, c), t_soft.covariance(r, c), 1e-9);
}

TEST(EkfEstimator, SoftUpdateWithBetaZeroOneIsNoOpOnState) {
  // beta_0 = 1: no measurement assigned. State and covariance unchanged.
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);

  navtracker::Measurement z;
  z.time = navtracker::Timestamp::fromSeconds(1.0);
  z.model = navtracker::MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 5.0);
  z.covariance = Eigen::Matrix2d::Identity() * 4.0;
  z.source_id = "t";

  navtracker::Track t = ekf.initiate(z);
  const Eigen::VectorXd x_before = t.state;
  const Eigen::MatrixXd P_before = t.covariance;

  navtracker::Measurement z2 = z;
  z2.time = navtracker::Timestamp::fromSeconds(2.0);
  z2.value = Eigen::Vector2d(12.0, 6.0);

  Eigen::VectorXd betas(1);
  betas << 0.0;
  ekf.softUpdate(t, {z2}, betas, 1.0);

  for (int i = 0; i < t.state.size(); ++i)
    EXPECT_NEAR(t.state(i), x_before(i), 1e-12);
  for (int r = 0; r < t.covariance.rows(); ++r)
    for (int c = 0; c < t.covariance.cols(); ++c)
      EXPECT_NEAR(t.covariance(r, c), P_before(r, c), 1e-12);
}
