#include "core/types/SensorDefaults.hpp"

#include <Eigen/Core>
#include <cmath>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

TEST(SensorDefaultsTest, PessimisticFactoryMatchesSpecValues) {
  const SensorDefaults d = pessimisticSensorDefaults();
  EXPECT_DOUBLE_EQ(d.ais_position.sigma_pos_m, 30.0);
  EXPECT_DOUBLE_EQ(d.arpa_tll_position.sigma_pos_m, 50.0);
  EXPECT_DOUBLE_EQ(d.arpa_ttm_range_bearing.sigma_range_m, 75.0);
  EXPECT_DOUBLE_EQ(d.arpa_ttm_range_bearing.sigma_bearing_rad,
                   1.5 * kDeg2Rad);
  EXPECT_DOUBLE_EQ(d.eoir_range_bearing.sigma_range_m, 50.0);
  EXPECT_DOUBLE_EQ(d.eoir_range_bearing.sigma_bearing_rad,
                   1.0 * kDeg2Rad);
  EXPECT_DOUBLE_EQ(d.eoir_bearing_only.sigma_bearing_rad,
                   1.5 * kDeg2Rad);
}

TEST(SensorDefaultsTest, CovarianceForReturnsCorrectShape) {
  const SensorDefaults d = pessimisticSensorDefaults();
  const auto ais_cov = d.covarianceFor(SensorKind::Ais,
                                       MeasurementModel::Position2D);
  ASSERT_EQ(ais_cov.rows(), 2);
  ASSERT_EQ(ais_cov.cols(), 2);
  EXPECT_DOUBLE_EQ(ais_cov(0, 0), 900.0);  // 30^2
  EXPECT_DOUBLE_EQ(ais_cov(1, 1), 900.0);

  const auto ttm_cov = d.covarianceFor(SensorKind::ArpaTtm,
                                       MeasurementModel::RangeBearing2D);
  ASSERT_EQ(ttm_cov.rows(), 2);
  EXPECT_DOUBLE_EQ(ttm_cov(0, 0), 75.0 * 75.0);
  EXPECT_NEAR(ttm_cov(1, 1),
              (1.5 * kDeg2Rad) * (1.5 * kDeg2Rad), 1e-12);
}

TEST(SensorDefaultsTest, ApplyDefaultsFillsEmptyAndFlags) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::Position2D;
  // covariance left empty.
  applyDefaultsIfEmpty(m, d);
  EXPECT_TRUE(m.covariance_is_default);
  EXPECT_EQ(m.covariance.rows(), 2);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 900.0);
}

TEST(SensorDefaultsTest, ApplyDefaultsNoOpWhenCovSet) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::Position2D;
  m.covariance = Eigen::Matrix2d::Identity() * 4.0;
  applyDefaultsIfEmpty(m, d);
  EXPECT_FALSE(m.covariance_is_default);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 4.0);
}

TEST(SensorDefaultsTest, ApplyDefaultsUnknownComboLeavesEmpty) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::RangeBearing2D;  // unknown combo
  applyDefaultsIfEmpty(m, d);
  EXPECT_FALSE(m.covariance_is_default);
  EXPECT_EQ(m.covariance.size(), 0);
}

}  // namespace navtracker
