#include <gtest/gtest.h>
#include "core/association/Gating.hpp"

using navtracker::mahalanobisDistance;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Track;

TEST(Gating, ZeroWhenMeasurementMatchesPrediction) {
  Track t;
  t.state = Eigen::Vector4d(10.0, 0.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  EXPECT_NEAR(mahalanobisDistance(t, z), 0.0, 1e-12);
}

TEST(Gating, KnownDistanceForOffsetMeasurement) {
  Track t;
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity();  // position cov = I
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(3.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();  // R = I
  // S = H P H^T + R = 2*I2; d2 = [3,0] (0.5*I2) [3,0]^T = 4.5
  EXPECT_NEAR(mahalanobisDistance(t, z), 4.5, 1e-12);
}
