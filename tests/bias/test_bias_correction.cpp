#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/pipeline/BiasCorrection.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorBiasProvider.hpp"

using namespace navtracker;

namespace {

Measurement makePos(const Eigen::Vector2d& enu, double sigma_pos) {
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.sensor = SensorKind::Lidar;
  z.source_id = "lidar0";
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(enu.x(), enu.y());
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = sigma_pos * sigma_pos;
  R(1, 1) = sigma_pos * sigma_pos;
  z.covariance = R;
  return z;
}

Measurement makeBearing(double beta_rad, double sigma_rad) {
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.sensor = SensorKind::EoIr;
  z.source_id = "eo0";
  z.model = MeasurementModel::Bearing2D;
  z.value = Eigen::Matrix<double, 1, 1>(beta_rad);
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = sigma_rad * sigma_rad;
  z.covariance = R;
  return z;
}

Measurement makeRangeBearing(double r_m, double beta_rad,
                             double sigma_r, double sigma_b) {
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.sensor = SensorKind::ArpaTtm;
  z.source_id = "radar0";
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(r_m, beta_rad);
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = sigma_r * sigma_r;
  R(1, 1) = sigma_b * sigma_b;
  z.covariance = R;
  return z;
}

}  // namespace

TEST(BiasCorrection, NullProviderReturnsZUnchanged) {
  const auto z = makePos({100.0, 50.0}, 2.0);
  const auto out = applyBiasCorrection(z, nullptr);
  EXPECT_DOUBLE_EQ(out.value(0), z.value(0));
  EXPECT_DOUBLE_EQ(out.value(1), z.value(1));
  EXPECT_DOUBLE_EQ(out.covariance(0, 0), z.covariance(0, 0));
}

TEST(BiasCorrection, UnpublishedBiasIsNoop) {
  NullBiasProvider prov;
  const auto z = makePos({100.0, 50.0}, 2.0);
  const auto out = applyBiasCorrection(z, &prov);
  EXPECT_DOUBLE_EQ(out.value(0), z.value(0));
  EXPECT_DOUBLE_EQ(out.value(1), z.value(1));
  EXPECT_DOUBLE_EQ(out.covariance(0, 0), z.covariance(0, 0));
  EXPECT_DOUBLE_EQ(out.covariance(1, 1), z.covariance(1, 1));
}

TEST(BiasCorrection, Position2DShiftsValueAndInflatesR) {
  FixedSensorBiasProvider prov;
  Eigen::Matrix2d Pb = Eigen::Matrix2d::Zero();
  Pb(0, 0) = 0.5;  // σ_b² = 0.5 m² per axis
  Pb(1, 1) = 0.5;
  prov.setPositionBias({SensorKind::Lidar, "lidar0"},
                       Eigen::Vector2d(3.0, -2.0), Pb);

  const auto z = makePos({100.0, 50.0}, /*sigma_pos=*/2.0);
  const auto out = applyBiasCorrection(z, &prov);
  // Mean shift: z - b.
  EXPECT_NEAR(out.value(0), 97.0, 1e-12);
  EXPECT_NEAR(out.value(1), 52.0, 1e-12);
  // R inflation: R_eff = R + P_b (H_b = I for additive position bias).
  // R was diag(4, 4); add diag(0.5, 0.5) → diag(4.5, 4.5).
  EXPECT_NEAR(out.covariance(0, 0), 4.5, 1e-12);
  EXPECT_NEAR(out.covariance(1, 1), 4.5, 1e-12);
}

TEST(BiasCorrection, Bearing2DShiftsValueAndInflatesR) {
  FixedSensorBiasProvider prov;
  const SensorBiasKey eo_key{SensorKind::EoIr, "eo0"};
  prov.setBearingBias(eo_key,
                      /*bias_rad=*/0.03, /*variance_rad2=*/1e-4);

  const auto z = makeBearing(/*beta=*/1.0, /*sigma=*/0.02);
  const auto out = applyBiasCorrection(z, &prov);
  EXPECT_NEAR(out.value(0), 1.0 - 0.03, 1e-12);
  EXPECT_NEAR(out.covariance(0, 0), 0.0004 + 1e-4, 1e-12);
}

TEST(BiasCorrection, RangeBearing2DTouchesOnlyBearingComponent) {
  FixedSensorBiasProvider prov;
  const SensorBiasKey radar_key{SensorKind::ArpaTtm, "radar0"};
  prov.setBearingBias(radar_key,
                      /*bias_rad=*/0.01, /*variance_rad2=*/2e-4);

  const auto z = makeRangeBearing(/*r=*/500.0, /*beta=*/0.5,
                                  /*sigma_r=*/5.0, /*sigma_b=*/0.02);
  const auto out = applyBiasCorrection(z, &prov);
  // Range component must be untouched.
  EXPECT_NEAR(out.value(0), 500.0, 1e-12);
  EXPECT_NEAR(out.covariance(0, 0), 25.0, 1e-12);
  EXPECT_NEAR(out.covariance(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(out.covariance(1, 0), 0.0, 1e-12);
  // Bearing component shifted + variance inflated.
  EXPECT_NEAR(out.value(1), 0.5 - 0.01, 1e-12);
  EXPECT_NEAR(out.covariance(1, 1), 0.0004 + 2e-4, 1e-12);
}
