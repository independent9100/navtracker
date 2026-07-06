#include <cmath>
#include <limits>
#include <optional>

#include <gtest/gtest.h>
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "ports/IHeadingBiasProvider.hpp"

using navtracker::CameraDetection;
using navtracker::EoIrAdapter;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;

namespace {

class StubBiasProvider : public navtracker::IHeadingBiasProvider {
 public:
  navtracker::HeadingBiasEstimate value;
  navtracker::HeadingBiasEstimate current() const override { return value; }
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

}  // namespace

TEST(EoIrAdapter, IngestProducesPositionAheadOfOwnShip) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow north
  provider.update(pose);

  Datum datum({53.5, 8.0, 0.0});
  EoIrAdapter adapter(datum, provider);

  CameraDetection d;
  d.time = Timestamp::fromSeconds(2.0);
  d.bearing_relative_deg = 0.0;
  d.range_m = 500.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 20.0;
  d.sensor_track_id = 7;
  adapter.ingest(d);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::EoIr);
  EXPECT_NEAR(out[0].value(0), 0.0, 1.0);
  EXPECT_NEAR(out[0].value(1), 500.0, 1.0);
  ASSERT_TRUE(out[0].hints.sensor_track_id.has_value());
  EXPECT_EQ(*out[0].hints.sensor_track_id, 7);
}

TEST(EoIrAdapter, DropsNonPositiveRangeAndNaNBearing) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  Datum datum({53.5, 8.0, 0.0});
  EoIrAdapter adapter(datum, provider);

  CameraDetection zero_range;
  zero_range.time = Timestamp::fromSeconds(1.0);
  zero_range.bearing_relative_deg = 0.0;
  zero_range.range_m = 0.0;  // parse-failure / target-on-own-ship
  adapter.ingest(zero_range);

  CameraDetection nan_bearing;
  nan_bearing.time = Timestamp::fromSeconds(2.0);
  nan_bearing.bearing_relative_deg = std::numeric_limits<double>::quiet_NaN();
  nan_bearing.range_m = 500.0;
  adapter.ingest(nan_bearing);

  EXPECT_TRUE(adapter.poll().empty());

  CameraDetection good;
  good.time = Timestamp::fromSeconds(3.0);
  good.bearing_relative_deg = 0.0;
  good.range_m = 500.0;
  adapter.ingest(good);
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(EoIrAdapter, HeadingStdInflatesCrossTrackCovariance) {
  // Own-ship at datum origin, heading 0°. Camera detection at relative
  // bearing 90° (east), range 1 km, bearing σ 0.5°, range σ 10 m.
  navtracker::geo::Datum datum({53.5, 8.0, 0.0});
  navtracker::OwnShipProvider provider;
  navtracker::OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  pose.time = navtracker::Timestamp::fromSeconds(0.0);
  provider.update(pose);

  navtracker::EoIrAdapter a0(datum, provider, navtracker::EoIrAdapterConfig{});
  navtracker::EoIrAdapter a2(datum, provider,
                             navtracker::EoIrAdapterConfig{/*heading_std_deg=*/2.0});

  navtracker::CameraDetection d;
  d.time = navtracker::Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;

  a0.ingest(d);
  a2.ingest(d);
  const auto m0 = a0.poll();
  const auto m2 = a2.poll();
  ASSERT_EQ(m0.size(), 1u);
  ASSERT_EQ(m2.size(), 1u);

  // East-pointing measurement: north–north (cov(1,1)) is the cross-track
  // variance — should grow. East–east (cov(0,0)) along-range should not.
  EXPECT_GT(m2[0].covariance(1, 1), m0[0].covariance(1, 1));
  EXPECT_NEAR(m0[0].covariance(0, 0), m2[0].covariance(0, 0), 1e-3);
}

TEST(EoIrAdapter, PerPoseHeadingStdInflatesAndCfgFloorsIt) {
  // #16: OwnShipPose.heading_std_deg widens the cross-track covariance beyond
  // the config floor; a per-pose σ tighter than the floor is clamped.
  navtracker::geo::Datum datum({53.5, 8.0, 0.0});
  auto crossTrackVar = [&](std::optional<double> pose_sigma, double cfg_floor) {
    navtracker::OwnShipProvider provider;
    navtracker::OwnShipPose pose;
    pose.lat_deg = 53.5;
    pose.lon_deg = 8.0;
    pose.heading_true_deg = 0.0;
    pose.time = navtracker::Timestamp::fromSeconds(0.0);
    pose.heading_std_deg = pose_sigma;
    provider.update(pose);
    navtracker::EoIrAdapter a(
        datum, provider,
        navtracker::EoIrAdapterConfig{/*heading_std_deg=*/cfg_floor});
    navtracker::CameraDetection d;
    d.time = navtracker::Timestamp::fromSeconds(1.0);
    d.bearing_relative_deg = 90.0;
    d.range_m = 1000.0;
    d.bearing_std_deg = 0.5;
    d.range_std_m = 10.0;
    a.ingest(d);
    const auto m = a.poll();
    EXPECT_EQ(m.size(), 1u);
    return m[0].covariance(1, 1);
  };

  EXPECT_GT(crossTrackVar(6.0, 1.0), crossTrackVar(std::nullopt, 1.0));
  EXPECT_NEAR(crossTrackVar(0.1, 3.0), crossTrackVar(std::nullopt, 3.0), 1e-9);
}

TEST(EoIrAdapterTest, AppliesPublishedBiasToProjectedBearing) {
  // Own-ship at the datum origin heading 0 deg, camera detection at
  // relative bearing 90 deg (east), range 1000 m. With no bias the
  // projected ENU position is (1000, 0). With a published bias
  // b_hat = 5 deg, the adapter subtracts b_hat from the raw bearing ->
  // corrected bearing 85 deg. Projection uses x = R sin(b),
  // y = R cos(b), so the expected position is
  // (1000*sin(85 deg), 1000*cos(85 deg)).
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  StubBiasProvider bias_provider;
  bias_provider.value.bias_rad = 5.0 * kDeg2Rad;
  bias_provider.value.variance_rad2 = 0.0;
  bias_provider.value.is_published = true;

  EoIrAdapter adapter(datum, provider, navtracker::EoIrAdapterConfig{},
                      &bias_provider);

  CameraDetection d;
  d.time = Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;
  adapter.ingest(d);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);

  const double expected_x = 1000.0 * std::sin(85.0 * kDeg2Rad);
  const double expected_y = 1000.0 * std::cos(85.0 * kDeg2Rad);
  EXPECT_NEAR(out[0].value(0), expected_x, 0.5);
  EXPECT_NEAR(out[0].value(1), expected_y, 0.5);
}

TEST(EoIrAdapterTest, UnpublishedProviderActsIdentically) {
  // Same scenario but provider reports is_published=false. Behavior
  // must match the no-provider path: projected position == (1000, 0).
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  StubBiasProvider bias_provider;
  bias_provider.value.bias_rad = 5.0 * kDeg2Rad;  // should be ignored
  bias_provider.value.variance_rad2 = 1.0;        // should be ignored
  bias_provider.value.is_published = false;

  EoIrAdapter with_provider(datum, provider, navtracker::EoIrAdapterConfig{},
                            &bias_provider);
  EoIrAdapter without_provider(datum, provider, navtracker::EoIrAdapterConfig{});

  CameraDetection d;
  d.time = Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;

  with_provider.ingest(d);
  without_provider.ingest(d);

  const auto mw = with_provider.poll();
  const auto mn = without_provider.poll();
  ASSERT_EQ(mw.size(), 1u);
  ASSERT_EQ(mn.size(), 1u);

  EXPECT_NEAR(mw[0].value(0), mn[0].value(0), 0.5);
  EXPECT_NEAR(mw[0].value(1), mn[0].value(1), 0.5);
  EXPECT_NEAR(mw[0].covariance(0, 0), mn[0].covariance(0, 0), 1.0);
  EXPECT_NEAR(mw[0].covariance(0, 1), mn[0].covariance(0, 1), 1.0);
  EXPECT_NEAR(mw[0].covariance(1, 0), mn[0].covariance(1, 0), 1.0);
  EXPECT_NEAR(mw[0].covariance(1, 1), mn[0].covariance(1, 1), 1.0);
}

TEST(EoIrAdapterTest, ComposesVarianceWithConfiguredHeadingStd) {
  // cfg.heading_std_deg = 1.0; provider publishes var = (0.5 deg)^2.
  // The adapter must compose sigma_heading_eff = sqrt(1^2 + 0.5^2) deg.
  // Compare against a second adapter without a provider, configured
  // with heading_std_deg = sqrt(1^2 + 0.5^2) directly.
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  StubBiasProvider bias_provider;
  bias_provider.value.bias_rad = 0.0;  // no bias offset, only variance
  const double provider_sigma_deg = 0.5;
  bias_provider.value.variance_rad2 =
      (provider_sigma_deg * kDeg2Rad) * (provider_sigma_deg * kDeg2Rad);
  bias_provider.value.is_published = true;

  EoIrAdapter with_provider(
      datum, provider,
      navtracker::EoIrAdapterConfig{/*heading_std_deg=*/1.0}, &bias_provider);

  const double composed_deg =
      std::sqrt(1.0 * 1.0 + provider_sigma_deg * provider_sigma_deg);
  EoIrAdapter equivalent(
      datum, provider,
      navtracker::EoIrAdapterConfig{/*heading_std_deg=*/composed_deg});

  CameraDetection d;
  d.time = Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;

  with_provider.ingest(d);
  equivalent.ingest(d);

  const auto mp = with_provider.poll();
  const auto me = equivalent.poll();
  ASSERT_EQ(mp.size(), 1u);
  ASSERT_EQ(me.size(), 1u);

  EXPECT_NEAR(mp[0].covariance(0, 0), me[0].covariance(0, 0), 1.0);
  EXPECT_NEAR(mp[0].covariance(0, 1), me[0].covariance(0, 1), 1.0);
  EXPECT_NEAR(mp[0].covariance(1, 0), me[0].covariance(1, 0), 1.0);
  EXPECT_NEAR(mp[0].covariance(1, 1), me[0].covariance(1, 1), 1.0);
}

TEST(EoIrAdapterTest, InflatesCovarianceFromOwnShipGpsStd) {
  // Same camera detection scenario as the baseline tests: own-ship at the
  // datum origin heading 0 deg, EO/IR camera target at range 1000 m on
  // relative bearing 90 deg (east). Test with pose.position_std_m = 5:
  // expect covariance to increase by 25 (5^2) on the diagonal vs baseline
  // where position_std_m = 0.

  // Baseline scenario: no GPS std.
  OwnShipProvider provider_base;
  OwnShipPose pose_base;
  pose_base.time = Timestamp::fromSeconds(0.0);
  pose_base.lat_deg = 53.5;
  pose_base.lon_deg = 8.0;
  pose_base.heading_true_deg = 0.0;
  pose_base.position_std_m = 0.0;
  provider_base.update(pose_base);

  EoIrAdapter adapter_base(Datum({53.5, 8.0, 0.0}), provider_base);
  CameraDetection d;
  d.time = Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;
  adapter_base.ingest(d);
  const auto m_base = adapter_base.poll();
  ASSERT_EQ(m_base.size(), 1u);

  // High GPS std scenario: position_std_m = 5.
  OwnShipProvider provider_gps;
  OwnShipPose pose_gps;
  pose_gps.time = Timestamp::fromSeconds(0.0);
  pose_gps.lat_deg = 53.5;
  pose_gps.lon_deg = 8.0;
  pose_gps.heading_true_deg = 0.0;
  pose_gps.position_std_m = 5.0;
  provider_gps.update(pose_gps);

  EoIrAdapter adapter_gps(Datum({53.5, 8.0, 0.0}), provider_gps);
  adapter_gps.ingest(d);
  const auto m_gps = adapter_gps.poll();
  ASSERT_EQ(m_gps.size(), 1u);

  // The difference on the diagonal should be sigma_gps^2 = 25.
  const double sigma_gps_sq = 5.0 * 5.0;
  EXPECT_NEAR(m_gps[0].covariance(0, 0) - m_base[0].covariance(0, 0),
              sigma_gps_sq, 1.0);
  EXPECT_NEAR(m_gps[0].covariance(1, 1) - m_base[0].covariance(1, 1),
              sigma_gps_sq, 1.0);
  // Off-diagonal should be unchanged.
  EXPECT_NEAR(m_gps[0].covariance(0, 1) - m_base[0].covariance(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(m_gps[0].covariance(1, 0) - m_base[0].covariance(1, 0), 0.0, 1e-9);
}
