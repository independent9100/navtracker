#include <cmath>

#include <gtest/gtest.h>
#include "core/estimation/MeasurementModels.hpp"

using navtracker::isMeasurementCovariancePsd;
using navtracker::MeasurementModel;
using navtracker::measurementResidual;
using navtracker::predictMeasurement;
using navtracker::wrapAngle;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// #35 M1: isMeasurementCovariancePsd never received the measurement dimension,
// so a square-but-wrong-size R passed the PSD check and then mismatched in
// H·P·Hᵀ + z.cov (Eigen abort in debug / OOB read under NDEBUG). The
// dimension-aware overload rejects a wrong-size R up front.
TEST(MeasurementModels, PsdGuardRejectsWrongSizeRForMeasurement) {
  const Eigen::MatrixXd r3 = Eigen::MatrixXd::Identity(3, 3);
  EXPECT_FALSE(isMeasurementCovariancePsd(r3, 2));  // 3x3 R for a 2-D measurement
  const Eigen::MatrixXd r2 = Eigen::MatrixXd::Identity(2, 2);
  EXPECT_TRUE(isMeasurementCovariancePsd(r2, 2));   // correctly sized

  // The dimension check composes with the finite/PSD checks, not replaces them.
  const Eigen::MatrixXd r2_neg = Eigen::MatrixXd::Identity(2, 2) * -1.0;
  EXPECT_FALSE(isMeasurementCovariancePsd(r2_neg, 2));  // right size, not PSD
  const Eigen::MatrixXd r1 = Eigen::MatrixXd::Identity(1, 1);
  EXPECT_TRUE(isMeasurementCovariancePsd(r1, 1));   // 1-D (bearing-only)

  // Single-arg overload unchanged: dimension-agnostic PSD check.
  EXPECT_TRUE(isMeasurementCovariancePsd(r2));
  EXPECT_FALSE(isMeasurementCovariancePsd(r2_neg));
}

TEST(MeasurementModels, Position2DLinear) {
  const Eigen::Vector4d x(3.0, 4.0, 1.0, -2.0);
  const auto p = predictMeasurement(MeasurementModel::Position2D, x);
  EXPECT_EQ(p.z_pred.size(), 2);
  EXPECT_DOUBLE_EQ(p.z_pred(0), 3.0);
  EXPECT_DOUBLE_EQ(p.z_pred(1), 4.0);
  EXPECT_EQ(p.H.rows(), 2);
  EXPECT_EQ(p.H.cols(), 4);
  EXPECT_DOUBLE_EQ(p.H(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(p.H(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(p.H(0, 2), 0.0);
}

TEST(MeasurementModels, RangeBearingValuesAndJacobian) {
  const Eigen::Vector4d x(3.0, 4.0, 0.0, 0.0);
  const auto p = predictMeasurement(MeasurementModel::RangeBearing2D, x);
  EXPECT_NEAR(p.z_pred(0), 5.0, 1e-12);
  EXPECT_NEAR(p.z_pred(1), std::atan2(4.0, 3.0), 1e-12);
  EXPECT_NEAR(p.H(0, 0), 0.6, 1e-12);
  EXPECT_NEAR(p.H(0, 1), 0.8, 1e-12);
  EXPECT_NEAR(p.H(1, 0), -0.16, 1e-12);
  EXPECT_NEAR(p.H(1, 1), 0.12, 1e-12);
}

TEST(MeasurementModels, JacobianMatchesFiniteDifference) {
  const Eigen::Vector4d x(120.0, -50.0, 2.0, 1.0);
  const auto p = predictMeasurement(MeasurementModel::RangeBearing2D, x);
  const double eps = 1e-6;
  for (int j = 0; j < 4; ++j) {
    Eigen::Vector4d xp = x;
    xp(j) += eps;
    const auto pp = predictMeasurement(MeasurementModel::RangeBearing2D, xp);
    const Eigen::Vector2d num = (pp.z_pred - p.z_pred) / eps;
    EXPECT_NEAR(p.H(0, j), num(0), 1e-3);
    EXPECT_NEAR(p.H(1, j), num(1), 1e-3);
  }
}

TEST(MeasurementModels, BearingResidualWrapsAcrossPi) {
  const Eigen::Vector2d z(10.0, -3.0);
  const Eigen::Vector2d zpred(10.0, 3.0);
  const Eigen::VectorXd y =
      measurementResidual(MeasurementModel::RangeBearing2D, z, zpred);
  EXPECT_NEAR(y(0), 0.0, 1e-12);
  EXPECT_NEAR(y(1), -6.0 + 2.0 * kPi, 1e-9);
}

TEST(MeasurementModels, WrapAngleRange) {
  EXPECT_NEAR(wrapAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapAngle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(wrapAngle(-3.0 * kPi), kPi, 1e-9);
}

TEST(MeasurementModels, Bearing2DPredictAndJacobian) {
  Eigen::Vector4d x;
  x << 100.0, 200.0, 5.0, -3.0;
  const navtracker::MeasurementPrediction p =
      navtracker::predictMeasurement(navtracker::MeasurementModel::Bearing2D, x);
  EXPECT_EQ(p.z_pred.size(), 1);
  EXPECT_NEAR(p.z_pred(0), std::atan2(200.0, 100.0), 1e-12);
  ASSERT_EQ(p.H.rows(), 1);
  ASSERT_EQ(p.H.cols(), 4);
  const double r2 = 100.0 * 100.0 + 200.0 * 200.0;
  EXPECT_NEAR(p.H(0, 0), -200.0 / r2, 1e-12);
  EXPECT_NEAR(p.H(0, 1),  100.0 / r2, 1e-12);
  EXPECT_DOUBLE_EQ(p.H(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(p.H(0, 3), 0.0);
}

TEST(MeasurementModels, Bearing2DResidualWrapped) {
  Eigen::VectorXd z(1), zp(1);
  z(0) = 3.0;      // near +π
  zp(0) = -3.0;    // near −π
  const Eigen::VectorXd y =
      navtracker::measurementResidual(navtracker::MeasurementModel::Bearing2D, z, zp);
  // Raw diff is 6.0 rad; wrapped to (−π,π] should be about 6.0 − 2π ≈ −0.283.
  EXPECT_NEAR(y(0), 6.0 - 2.0 * 3.14159265358979323846, 1e-9);
}

TEST(MeasurementModels, Position2DJacobianAdaptsToStateSize) {
  Eigen::VectorXd x5(5);
  x5 << 10.0, 20.0, 5.0, -3.0, 0.1;
  const navtracker::MeasurementPrediction p =
      navtracker::predictMeasurement(navtracker::MeasurementModel::Position2D, x5);
  ASSERT_EQ(p.H.rows(), 2);
  ASSERT_EQ(p.H.cols(), 5);
  EXPECT_DOUBLE_EQ(p.H(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(p.H(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(p.H(0, 4), 0.0);
  EXPECT_DOUBLE_EQ(p.H(1, 4), 0.0);
}

TEST(MeasurementModels, RangeBearingJacobianAdaptsToStateSize) {
  Eigen::VectorXd x5(5);
  x5 << 100.0, 0.0, 5.0, -3.0, 0.1;
  const navtracker::MeasurementPrediction p =
      navtracker::predictMeasurement(navtracker::MeasurementModel::RangeBearing2D, x5);
  ASSERT_EQ(p.H.rows(), 2);
  ASSERT_EQ(p.H.cols(), 5);
}

TEST(MeasurementModels, RangeBearingFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 100.0, 0.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(50.0, 0.0);
  const Eigen::VectorXd z =
      navtracker::predictMeasurementValue(
          navtracker::MeasurementModel::RangeBearing2D, state, sensor);
  EXPECT_NEAR(z(0), 50.0, 1e-9);
  EXPECT_NEAR(z(1), 0.0, 1e-9);
}

TEST(MeasurementModels, BearingOnlyFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 0.0, 100.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(0.0, 50.0);
  const Eigen::VectorXd z =
      navtracker::predictMeasurementValue(
          navtracker::MeasurementModel::Bearing2D, state, sensor);
  EXPECT_NEAR(z(0), 3.14159265358979323846 / 2.0, 1e-9);
}

TEST(MeasurementModels, RangeBearingJacobianFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 60.0, 80.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(10.0, 20.0);
  const navtracker::MeasurementPrediction p =
      navtracker::predictMeasurement(
          navtracker::MeasurementModel::RangeBearing2D, state, sensor);
  const double r = std::sqrt(6100.0);
  EXPECT_NEAR(p.H(0, 0), 50.0 / r, 1e-9);
  EXPECT_NEAR(p.H(0, 1), 60.0 / r, 1e-9);
  EXPECT_NEAR(p.H(1, 0), -60.0 / (r * r), 1e-9);
  EXPECT_NEAR(p.H(1, 1),  50.0 / (r * r), 1e-9);
}
