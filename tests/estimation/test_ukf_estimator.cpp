#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/UkfEstimator.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
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
