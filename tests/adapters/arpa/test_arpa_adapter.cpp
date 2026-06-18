#include <cmath>

#include <gtest/gtest.h>
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using navtracker::ArpaAdapter;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker_test::makeNmea;

namespace {
Datum kDatum({53.5, 8.0, 0.0});

class StubBiasProvider : public navtracker::IHeadingBiasProvider {
 public:
  navtracker::HeadingBiasEstimate value;
  navtracker::HeadingBiasEstimate current() const override { return value; }
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}  // namespace

TEST(ArpaAdapter, TllProducesPosition2D) {
  OwnShipProvider provider;
  ArpaAdapter adapter(kDatum, provider);
  // ddmm.mmmm: 53 + 30.6/60 = 53.51; 008 + 00.0/60 = 8.0; ~1.1 km north.
  EXPECT_TRUE(adapter.ingest(
      makeNmea("RATLL,01,5330.6,N,00800.0,E,TARG1,123456,T,R"),
      Timestamp::fromSeconds(10.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::ArpaTll);
  EXPECT_NEAR(out[0].value(0), 0.0, 5.0);
  EXPECT_GT(out[0].value(1), 1000.0);
  EXPECT_LT(out[0].value(1), 1200.0);
  ASSERT_TRUE(out[0].hints.sensor_track_id.has_value());
  EXPECT_EQ(*out[0].hints.sensor_track_id, 1);
}

TEST(ArpaAdapter, TllRejectsOutOfRangeLatLon) {
  OwnShipProvider provider;
  ArpaAdapter adapter(kDatum, provider);
  // 9130.0 ddmm -> lat 91.5°, out of range -> rejected.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("RATLL,01,9130.0,N,00800.0,E,TARG1,123456,T,R"),
      Timestamp::fromSeconds(10.0)));
  EXPECT_TRUE(adapter.poll().empty());
}

TEST(ArpaAdapter, TtmRejectsNonPositiveRange) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  ArpaAdapter adapter(kDatum, provider);
  // distance 0.0 (e.g. strtod parse failure) -> target on own-ship -> rejected.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("RATTM,01,0.0,90.0,T,12.0,90.0,T,0.0,0.0,N,TARG1,T,R,123456.78,A"),
      Timestamp::fromSeconds(5.0)));
  EXPECT_TRUE(adapter.poll().empty());
}

TEST(ArpaAdapter, TtmProducesPositionUsingOwnShip) {
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  ArpaAdapter adapter(kDatum, provider);
  // distance 1.0 NM = 1852 m, bearing 90 true (east), units N (nautical miles).
  EXPECT_TRUE(adapter.ingest(
      makeNmea("RATTM,01,1.0,90.0,T,12.0,90.0,T,0.0,0.0,N,TARG1,T,R,123456.78,A"),
      Timestamp::fromSeconds(5.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor, SensorKind::ArpaTtm);
  EXPECT_NEAR(out[0].value(0), 1852.0, 1.0);
  EXPECT_NEAR(out[0].value(1), 0.0, 1.0);
}

TEST(ArpaAdapter, HeadingStdInflatesTtmCovariance) {
  // Same own-ship + target geometry as the existing TTM tests in this
  // file: own-ship at the datum origin heading 0 deg, ARPA target at
  // range 1 NM on relative bearing 90 deg (so east in the ENU frame).
  OwnShipProvider provider;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  provider.update(pose);

  // Build two adapters: one with sigma_h=0, one with sigma_h=2 deg.
  ArpaAdapter a0(kDatum, provider, navtracker::ArpaAdapterConfig{});
  ArpaAdapter a2(kDatum, provider,
                 navtracker::ArpaAdapterConfig{/*heading_std_deg=*/2.0});

  const std::string ttm =
      makeNmea("RATTM,01,1.0,90.0,R,0.0,0.0,T,0.0,0.0,N,TARG1,T,R,123456.78,A");
  EXPECT_TRUE(a0.ingest(ttm, Timestamp::fromSeconds(1.0)));
  EXPECT_TRUE(a2.ingest(ttm, Timestamp::fromSeconds(1.0)));

  const auto m0 = a0.poll();
  const auto m2 = a2.poll();
  ASSERT_EQ(m0.size(), 1u);
  ASSERT_EQ(m2.size(), 1u);

  // East-pointing measurement: north-north (cov(1,1)) is the cross-track
  // variance. Inflation should make it strictly larger; east-east
  // (cov(0,0), along-range) should be unchanged.
  EXPECT_GT(m2[0].covariance(1, 1), m0[0].covariance(1, 1));
  EXPECT_NEAR(m0[0].covariance(0, 0), m2[0].covariance(0, 0), 1e-3);
}

TEST(ArpaAdapterTest, AppliesPublishedBiasToProjectedBearing) {
  // Own-ship at the datum origin heading 0 deg, ARPA target at range
  // 1000 m on true bearing 90 deg (east). With no bias the projected
  // ENU position is (1000, 0). With a published bias b_hat = 5 deg,
  // the adapter subtracts b_hat from the raw bearing -> corrected
  // bearing 85 deg. Projection uses x = R sin(b), y = R cos(b), so the
  // expected position is (1000*sin(85 deg), 1000*cos(85 deg)).
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

  ArpaAdapter adapter(kDatum, provider, navtracker::ArpaAdapterConfig{},
                      &bias_provider);

  // Range expressed in km (1.0 km = 1000 m) via units "K".
  const std::string ttm =
      makeNmea("RATTM,01,1.0,90.0,T,0.0,0.0,T,0.0,0.0,K,TARG1,T,R,123456.78,A");
  EXPECT_TRUE(adapter.ingest(ttm, Timestamp::fromSeconds(1.0)));
  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);

  const double expected_x = 1000.0 * std::sin(85.0 * kDeg2Rad);
  const double expected_y = 1000.0 * std::cos(85.0 * kDeg2Rad);
  EXPECT_NEAR(out[0].value(0), expected_x, 0.5);
  EXPECT_NEAR(out[0].value(1), expected_y, 0.5);
}

TEST(ArpaAdapterTest, UnpublishedProviderActsIdentically) {
  // Same scenario but provider reports is_published=false. Behavior
  // must match the no-provider path: projected position == (1000, 0).
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

  ArpaAdapter with_provider(kDatum, provider,
                            navtracker::ArpaAdapterConfig{}, &bias_provider);
  ArpaAdapter without_provider(kDatum, provider,
                               navtracker::ArpaAdapterConfig{});

  const std::string ttm =
      makeNmea("RATTM,01,1.0,90.0,T,0.0,0.0,T,0.0,0.0,K,TARG1,T,R,123456.78,A");
  EXPECT_TRUE(with_provider.ingest(ttm, Timestamp::fromSeconds(1.0)));
  EXPECT_TRUE(without_provider.ingest(ttm, Timestamp::fromSeconds(1.0)));

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

TEST(ArpaAdapterTest, ComposesVarianceWithConfiguredHeadingStd) {
  // cfg.heading_std_deg = 1.0; provider publishes var = (0.5 deg)^2.
  // The adapter must compose sigma_heading_eff = sqrt(1^2 + 0.5^2) deg.
  // Compare against a second adapter without a provider, configured
  // with heading_std_deg = sqrt(1^2 + 0.5^2) directly.
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

  ArpaAdapter with_provider(
      kDatum, provider,
      navtracker::ArpaAdapterConfig{/*heading_std_deg=*/1.0}, &bias_provider);

  const double composed_deg =
      std::sqrt(1.0 * 1.0 + provider_sigma_deg * provider_sigma_deg);
  ArpaAdapter equivalent(
      kDatum, provider,
      navtracker::ArpaAdapterConfig{/*heading_std_deg=*/composed_deg});

  const std::string ttm =
      makeNmea("RATTM,01,1.0,90.0,T,0.0,0.0,T,0.0,0.0,K,TARG1,T,R,123456.78,A");
  EXPECT_TRUE(with_provider.ingest(ttm, Timestamp::fromSeconds(1.0)));
  EXPECT_TRUE(equivalent.ingest(ttm, Timestamp::fromSeconds(1.0)));

  const auto mp = with_provider.poll();
  const auto me = equivalent.poll();
  ASSERT_EQ(mp.size(), 1u);
  ASSERT_EQ(me.size(), 1u);

  EXPECT_NEAR(mp[0].covariance(0, 0), me[0].covariance(0, 0), 1.0);
  EXPECT_NEAR(mp[0].covariance(0, 1), me[0].covariance(0, 1), 1.0);
  EXPECT_NEAR(mp[0].covariance(1, 0), me[0].covariance(1, 0), 1.0);
  EXPECT_NEAR(mp[0].covariance(1, 1), me[0].covariance(1, 1), 1.0);
}

TEST(ArpaAdapterTest, InflatesCovarianceFromOwnShipGpsStd) {
  // Same TTM scenario as the baseline tests: own-ship at the datum origin
  // heading 0 deg, ARPA target at range 1 NM on relative bearing 90 deg.
  // Test with pose.position_std_m = 5: expect covariance to increase by 25
  // (5^2) on the diagonal vs baseline where position_std_m = 0.

  // Baseline scenario: no GPS std.
  OwnShipProvider provider_base;
  OwnShipPose pose_base;
  pose_base.time = Timestamp::fromSeconds(0.0);
  pose_base.lat_deg = 53.5;
  pose_base.lon_deg = 8.0;
  pose_base.heading_true_deg = 0.0;
  pose_base.position_std_m = 0.0;
  provider_base.update(pose_base);

  ArpaAdapter adapter_base(kDatum, provider_base);
  const std::string ttm =
      makeNmea("RATTM,01,1.0,90.0,T,0.0,0.0,T,0.0,0.0,K,TARG1,T,R,123456.78,A");
  EXPECT_TRUE(adapter_base.ingest(ttm, Timestamp::fromSeconds(1.0)));
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

  ArpaAdapter adapter_gps(kDatum, provider_gps);
  EXPECT_TRUE(adapter_gps.ingest(ttm, Timestamp::fromSeconds(1.0)));
  const auto m_gps = adapter_gps.poll();
  ASSERT_EQ(m_gps.size(), 1u);

  // The difference on the diagonal should be sigma_gps^2 = 25.
  const double sigma_gps_sq = 5.0 * 5.0;
  EXPECT_NEAR(m_gps[0].covariance(0, 0) - m_base[0].covariance(0, 0),
              sigma_gps_sq, 1e-6);
  EXPECT_NEAR(m_gps[0].covariance(1, 1) - m_base[0].covariance(1, 1),
              sigma_gps_sq, 1e-6);
  // Off-diagonal should be unchanged.
  EXPECT_NEAR(m_gps[0].covariance(0, 1) - m_base[0].covariance(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(m_gps[0].covariance(1, 0) - m_base[0].covariance(1, 0), 0.0, 1e-9);
}

TEST(ArpaAdapterTest, TllUnaffectedByGpsStd) {
  // TLL provides absolute lat/lon directly from the ARPA radar; it does
  // not project from own-ship position. Therefore, GPS std on the own-ship
  // pose should have no effect on TLL measurements.

  // Scenario with position_std_m = 0.
  OwnShipProvider provider0;
  OwnShipPose pose0;
  pose0.time = Timestamp::fromSeconds(0.0);
  pose0.lat_deg = 53.5;
  pose0.lon_deg = 8.0;
  pose0.heading_true_deg = 0.0;
  pose0.position_std_m = 0.0;
  provider0.update(pose0);

  ArpaAdapter adapter0(kDatum, provider0);
  EXPECT_TRUE(adapter0.ingest(
      makeNmea("RATLL,01,5330.6,N,00800.0,E,TARG1,123456,T,R"),
      Timestamp::fromSeconds(10.0)));
  const auto m0 = adapter0.poll();
  ASSERT_EQ(m0.size(), 1u);

  // Same scenario with position_std_m = 5.
  OwnShipProvider provider5;
  OwnShipPose pose5;
  pose5.time = Timestamp::fromSeconds(0.0);
  pose5.lat_deg = 53.5;
  pose5.lon_deg = 8.0;
  pose5.heading_true_deg = 0.0;
  pose5.position_std_m = 5.0;
  provider5.update(pose5);

  ArpaAdapter adapter5(kDatum, provider5);
  EXPECT_TRUE(adapter5.ingest(
      makeNmea("RATLL,01,5330.6,N,00800.0,E,TARG1,123456,T,R"),
      Timestamp::fromSeconds(10.0)));
  const auto m5 = adapter5.poll();
  ASSERT_EQ(m5.size(), 1u);

  // Covariances must be identical.
  EXPECT_NEAR(m5[0].covariance(0, 0), m0[0].covariance(0, 0), 1e-9);
  EXPECT_NEAR(m5[0].covariance(0, 1), m0[0].covariance(0, 1), 1e-9);
  EXPECT_NEAR(m5[0].covariance(1, 0), m0[0].covariance(1, 0), 1e-9);
  EXPECT_NEAR(m5[0].covariance(1, 1), m0[0].covariance(1, 1), 1e-9);
}
