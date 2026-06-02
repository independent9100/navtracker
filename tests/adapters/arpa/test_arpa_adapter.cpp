#include <gtest/gtest.h>
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
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
}

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
