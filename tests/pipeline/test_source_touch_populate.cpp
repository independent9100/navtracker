#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>

#include "core/pipeline/SourceTouchPopulate.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(SourceTouchPopulate, Position2DFillsValueAndCovariance) {
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(123.0, -45.0);
  z.covariance = (Eigen::Matrix2d() << 4.0, 0.1, 0.1, 9.0).finished();
  Track::SourceTouch touch;
  fillSourceTouchEnu(touch, z);
  EXPECT_NEAR(touch.value_enu.x(), 123.0, 1e-9);
  EXPECT_NEAR(touch.value_enu.y(), -45.0, 1e-9);
  EXPECT_NEAR(touch.covariance(0, 0), 4.0, 1e-9);
  EXPECT_NEAR(touch.covariance(1, 1), 9.0, 1e-9);
}

TEST(SourceTouchPopulate, RangeBearing2DProjectsToEnuRelativeToSensor) {
  Measurement z;
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(100.0, kPi / 4.0);  // 100 m, 45 deg ENU
  z.sensor_position_enu = Eigen::Vector2d(50.0, 20.0);
  z.covariance = Eigen::Matrix2d::Zero();
  z.covariance(0, 0) = 4.0;            // σ_r² = 2 m
  z.covariance(1, 1) = 1e-4;           // σ_β² ≈ 0.01 rad
  Track::SourceTouch touch;
  fillSourceTouchEnu(touch, z);
  // Expected target: sensor + 100 * (cos45, sin45) = (50 + 70.71, 20 + 70.71)
  EXPECT_NEAR(touch.value_enu.x(), 50.0 + 100.0 * std::cos(kPi / 4.0), 1e-6);
  EXPECT_NEAR(touch.value_enu.y(), 20.0 + 100.0 * std::sin(kPi / 4.0), 1e-6);
  // Projected covariance trace = σ_r² + r² σ_β² = 4 + 1 = 5.
  EXPECT_NEAR(touch.covariance.trace(), 5.0, 1e-6);
}

TEST(SourceTouchPopulate, Bearing2DLeavesValueAtZero) {
  Measurement z;
  z.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = 0.3;
  z.value = v;
  z.sensor_position_enu = Eigen::Vector2d(10.0, 5.0);
  Eigen::MatrixXd r(1, 1);
  r(0, 0) = 1e-4;
  z.covariance = r;
  Track::SourceTouch touch;
  fillSourceTouchEnu(touch, z);
  // No range observation; touch carries no ENU position.
  EXPECT_NEAR(touch.value_enu.x(), 0.0, 1e-12);
  EXPECT_NEAR(touch.value_enu.y(), 0.0, 1e-12);
}

}  // namespace
}  // namespace navtracker
