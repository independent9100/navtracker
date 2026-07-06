#include "core/types/MeasurementBuilders.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/projection/Projection.hpp"
#include "core/types/Ids.hpp"
#include "core/types/SensorDefaults.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

geo::Datum makeTestDatum() {
  return geo::Datum(geo::Geodetic{59.0, 10.0, 0.0});
}

OwnShipPose makePose(Timestamp t,
                     double lat_deg,
                     double lon_deg,
                     double heading_true_deg,
                     double position_std_m = 0.0) {
  OwnShipPose p;
  p.time = t;
  p.lat_deg = lat_deg;
  p.lon_deg = lon_deg;
  p.alt_m = 0.0;
  p.heading_true_deg = heading_true_deg;
  p.position_std_m = position_std_m;
  return p;
}

}  // namespace

TEST(MeasurementBuildersTest, RelativeBearingProducesEnuConsistentWithDirectProjection) {
  OwnShipProvider provider;
  const Timestamp t = Timestamp::fromSeconds(10.0);
  const double heading_deg = 45.0;
  const double pos_std = 3.0;
  // Slightly offset from the datum origin so own_xy is non-zero.
  const OwnShipPose pose = makePose(t, 59.01, 10.01, heading_deg, pos_std);
  provider.update(pose);

  const double range_m = 1500.0;
  const double rel_bearing_rad = 30.0 * kDeg2Rad;
  const double range_std_m = 20.0;
  const double bearing_std_rad = 1.0 * kDeg2Rad;

  const Measurement m = makeMeasurementFromRelativeBearing(
      SensorKind::ArpaTtm, "TTM-A", t, range_m, rel_bearing_rad,
      range_std_m, bearing_std_rad, provider);

  // Reference: compute exactly what the builder should produce.
  const geo::Datum& datum = provider.datum();
  const Eigen::Vector3d own_3 =
      datum.toEnu(geo::Geodetic{pose.lat_deg, pose.lon_deg, 0.0});
  const Eigen::Vector2d own_xy(own_3.x(), own_3.y());
  const double true_bearing_rad = rel_bearing_rad + heading_deg * kDeg2Rad;
  const PointAndCov2D expected = projectRangeBearingToEnu(
      range_m, true_bearing_rad, range_std_m, bearing_std_rad,
      /*sigma_heading_rad=*/0.0, pos_std, own_xy);

  ASSERT_EQ(m.value.size(), 2);
  ASSERT_EQ(m.covariance.rows(), 2);
  ASSERT_EQ(m.covariance.cols(), 2);
  EXPECT_NEAR(m.value.x(), expected.pos_enu.x(), 1e-9);
  EXPECT_NEAR(m.value.y(), expected.pos_enu.y(), 1e-9);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      EXPECT_NEAR(m.covariance(i, j), expected.cov(i, j), 1e-9);
    }
  }

  EXPECT_EQ(m.sensor, SensorKind::ArpaTtm);
  EXPECT_EQ(m.source_id, "TTM-A");
  EXPECT_EQ(m.model, MeasurementModel::Position2D);
  EXPECT_FALSE(m.covariance_is_default);
  EXPECT_NEAR(m.sensor_position_enu.x(), own_xy.x(), 1e-9);
  EXPECT_NEAR(m.sensor_position_enu.y(), own_xy.y(), 1e-9);
  EXPECT_DOUBLE_EQ(m.sensor_position_std_m, pos_std);
  EXPECT_EQ(m.time.nanos(), t.nanos());
}

TEST(MeasurementBuildersTest, RelativeBearingUsesPoseFromHistoryAtTimestamp) {
  OwnShipProvider provider;

  const Timestamp t1 = Timestamp::fromSeconds(1.0);
  const Timestamp t2 = Timestamp::fromSeconds(2.0);
  const Timestamp t3 = Timestamp::fromSeconds(3.0);

  const double heading_a = 10.0;
  const double heading_b = 200.0;
  // Same lat/lon for both poses so the only thing that changes is heading.
  provider.update(makePose(t1, 59.0, 10.0, heading_a));
  provider.update(makePose(t3, 59.0, 10.0, heading_b));

  const double range_m = 1000.0;
  const double rel_bearing_rad = 0.0;  // dead ahead

  const Measurement m = makeMeasurementFromRelativeBearing(
      SensorKind::ArpaTtm, "TTM", t2, range_m, rel_bearing_rad,
      /*range_std_m=*/0.0, /*bearing_std_rad=*/0.0, provider);

  ASSERT_EQ(m.value.size(), 2);
  // Bearing-true used by builder should equal heading_a (since rel = 0).
  // Bearing convention from Projection: x = own.x + r*sin(b), y = own.y + r*cos(b).
  // Own ship is at origin (datum origin). So m.value should match a
  // projection from heading_a, NOT heading_b.
  const double expected_x = range_m * std::sin(heading_a * kDeg2Rad);
  const double expected_y = range_m * std::cos(heading_a * kDeg2Rad);
  EXPECT_NEAR(m.value.x(), expected_x, 1e-6);
  EXPECT_NEAR(m.value.y(), expected_y, 1e-6);

  // Sanity: the heading_b projection would differ noticeably.
  const double bad_x = range_m * std::sin(heading_b * kDeg2Rad);
  const double bad_y = range_m * std::cos(heading_b * kDeg2Rad);
  EXPECT_GT(std::hypot(m.value.x() - bad_x, m.value.y() - bad_y), 1.0);
}

TEST(MeasurementBuildersTest, TrueBearingSkipsHeadingCombo) {
  OwnShipProvider provider;

  const Timestamp t1 = Timestamp::fromSeconds(1.0);
  const Timestamp t2 = Timestamp::fromSeconds(2.0);
  const Timestamp t3 = Timestamp::fromSeconds(3.0);

  const double heading_a = 75.0;
  const double heading_b = 250.0;
  provider.update(makePose(t1, 59.0, 10.0, heading_a));
  provider.update(makePose(t3, 59.0, 10.0, heading_b));

  const double range_m = 800.0;
  const double true_bearing_rad = 90.0 * kDeg2Rad;  // due east

  const Measurement m = makeMeasurementFromTrueBearing(
      SensorKind::EoIr, "EOIR", t2, range_m, true_bearing_rad,
      /*range_std_m=*/0.0, /*bearing_std_rad=*/0.0, provider);

  ASSERT_EQ(m.value.size(), 2);
  // Own ship is at datum origin (lat/lon match), so own_xy = (0,0).
  // True bearing 90deg => x = r*sin(90)=r, y = r*cos(90)=0.
  EXPECT_NEAR(m.value.x(), range_m, 1e-6);
  EXPECT_NEAR(m.value.y(), 0.0, 1e-6);
}

TEST(MeasurementBuildersTest, EnuPositionPassesThroughCovariance) {
  const Timestamp t = Timestamp::fromSeconds(42.0);
  const Eigen::Vector2d enu(100.0, -50.0);
  Eigen::Matrix2d cov;
  cov << 25.0, 2.0,
          2.0, 16.0;

  AssociationHints hints;
  hints.mmsi = 1234567u;

  const Measurement m = makeMeasurementFromEnuPosition(
      SensorKind::Ais, "AIS", t, enu, cov, hints);

  EXPECT_EQ(m.sensor, SensorKind::Ais);
  EXPECT_EQ(m.source_id, "AIS");
  EXPECT_EQ(m.model, MeasurementModel::Position2D);
  ASSERT_EQ(m.value.size(), 2);
  EXPECT_DOUBLE_EQ(m.value.x(), 100.0);
  EXPECT_DOUBLE_EQ(m.value.y(), -50.0);
  ASSERT_EQ(m.covariance.rows(), 2);
  ASSERT_EQ(m.covariance.cols(), 2);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 25.0);
  EXPECT_DOUBLE_EQ(m.covariance(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(m.covariance(1, 0), 2.0);
  EXPECT_DOUBLE_EQ(m.covariance(1, 1), 16.0);
  EXPECT_FALSE(m.covariance_is_default);
  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 1234567u);
  EXPECT_EQ(m.time.nanos(), t.nanos());
}

TEST(MeasurementBuildersTest, EnuPositionEmptyCovariancePlaysWithDefaults) {
  const Timestamp t = Timestamp::fromSeconds(7.0);
  const Eigen::Vector2d enu(10.0, 20.0);
  // Empty/sentinel covariance — all zeros.
  Eigen::Matrix2d empty_cov = Eigen::Matrix2d::Zero();

  Measurement m = makeMeasurementFromEnuPosition(
      SensorKind::Ais, "AIS", t, enu, empty_cov);

  // Builder should NOT populate covariance from a zero sentinel.
  EXPECT_EQ(m.covariance.size(), 0);
  EXPECT_FALSE(m.covariance_is_default);

  applyDefaultsIfEmpty(m, pessimisticSensorDefaults());

  ASSERT_EQ(m.covariance.rows(), 2);
  ASSERT_EQ(m.covariance.cols(), 2);
  // Pessimistic AIS sigma_pos_m = 30 → variance 900 on each diagonal.
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 900.0);
  EXPECT_DOUBLE_EQ(m.covariance(1, 1), 900.0);
  EXPECT_TRUE(m.covariance_is_default);
}

TEST(MeasurementBuildersTest, EmptyWhenProviderHasNoDatum) {
  OwnShipProvider provider;  // no pose pushed -> no datum
  // No registerDatumSink / update calls.
  Measurement m = makeMeasurementFromRelativeBearing(
      SensorKind::ArpaTtm, "test", Timestamp::fromSeconds(0.0),
      1500.0, 0.5, 50.0, 1.0 * 3.14159265358979 / 180.0,
      provider);
  EXPECT_EQ(m.value.size(), 0);
  EXPECT_EQ(m.covariance.size(), 0);
}

// ---- #16 per-pose heading σ composition -------------------------------------

namespace {
// Build a relative-bearing measurement with a given per-pose heading σ and
// floor, over a fixed geometry, and return its covariance.
Eigen::Matrix2d covWithHeading(std::optional<double> pose_heading_std_deg,
                               double floor_deg) {
  OwnShipProvider provider;
  const Timestamp t = Timestamp::fromSeconds(10.0);
  OwnShipPose pose = makePose(t, 59.01, 10.01, 45.0, /*pos_std=*/3.0);
  pose.heading_std_deg = pose_heading_std_deg;
  provider.update(pose);
  const Measurement m = makeMeasurementFromRelativeBearing(
      SensorKind::ArpaTtm, "TTM", t, /*range=*/1500.0,
      /*rel_bearing=*/30.0 * kDeg2Rad, /*range_std=*/20.0,
      /*bearing_std=*/1.0 * kDeg2Rad, provider, /*hints=*/{}, floor_deg);
  return m.covariance;
}
}  // namespace

TEST(MeasurementBuildersHeadingSigma, AbsentPoseAndZeroFloorIsBitIdenticalToNoHeading) {
  // The default path (no per-pose σ, floor 0) must reproduce the sigma=0
  // heading contribution exactly — i.e. today's covariance.
  const Eigen::Matrix2d cov = covWithHeading(std::nullopt, 0.0);

  OwnShipProvider provider;
  const Timestamp t = Timestamp::fromSeconds(10.0);
  const OwnShipPose pose = makePose(t, 59.01, 10.01, 45.0, 3.0);
  provider.update(pose);
  const Eigen::Vector3d own_3 =
      provider.datum().toEnu(geo::Geodetic{pose.lat_deg, pose.lon_deg, 0.0});
  const Eigen::Vector2d own_xy(own_3.x(), own_3.y());
  const PointAndCov2D expected = projectRangeBearingToEnu(
      1500.0, 30.0 * kDeg2Rad + 45.0 * kDeg2Rad, 20.0, 1.0 * kDeg2Rad,
      /*sigma_heading_rad=*/0.0, 3.0, own_xy);
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      EXPECT_NEAR(cov(i, j), expected.cov(i, j), 1e-9);
}

TEST(MeasurementBuildersHeadingSigma, PerPoseSigmaWidensCovariance) {
  // A pose reporting a real per-fix heading σ must inflate the covariance
  // (larger trace) vs. the no-heading-σ baseline.
  const double base_trace = covWithHeading(std::nullopt, 0.0).trace();
  const double wide_trace = covWithHeading(/*pose σ=*/5.0, /*floor=*/0.0).trace();
  EXPECT_GT(wide_trace, base_trace);
}

TEST(MeasurementBuildersHeadingSigma, ConfigFloorClampsOverconfidentPoseSigma) {
  // A pose claiming an implausibly TIGHT heading σ (0.01°) must not make the
  // measurement more confident than the floor allows: the covariance with a
  // 3° floor must equal the covariance from a 3° per-pose σ (floor wins), and
  // exceed the covariance from the tight 0.01° value with no floor.
  const Eigen::Matrix2d floored = covWithHeading(/*pose σ=*/0.01, /*floor=*/3.0);
  const Eigen::Matrix2d as_if_3deg = covWithHeading(/*pose σ=*/3.0, /*floor=*/0.0);
  const Eigen::Matrix2d unfloored = covWithHeading(/*pose σ=*/0.01, /*floor=*/0.0);
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      EXPECT_NEAR(floored(i, j), as_if_3deg(i, j), 1e-9);
  EXPECT_GT(floored.trace(), unfloored.trace());
}

TEST(MeasurementBuildersHeadingSigma, PerPoseSigmaWidensBeyondFloorWhenLarger) {
  // When the per-fix σ exceeds the floor, the per-fix value wins (only widens).
  const double floor_only = covWithHeading(std::nullopt, /*floor=*/2.0).trace();
  const double wider = covWithHeading(/*pose σ=*/8.0, /*floor=*/2.0).trace();
  EXPECT_GT(wider, floor_only);
}

}  // namespace navtracker
