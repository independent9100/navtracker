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
