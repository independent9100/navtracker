#include <gtest/gtest.h>
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using navtracker::OwnShipNmeaAdapter;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;
using navtracker_test::makeNmea;

TEST(OwnShipNmeaAdapter, GgaUpdatesPositionInProvider) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // 4807.038 N -> 48 + 7.038/60 = 48.1173 deg
  // 01131.000 E -> 11 + 31.000/60 = 11.5166666... deg
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->lat_deg, 48.1173, 1e-4);
  EXPECT_NEAR(provider.latest()->lon_deg, 11.5166666, 1e-4);
}

TEST(OwnShipNmeaAdapter, HdtUpdatesHeading) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0));
  EXPECT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"),
                              Timestamp::fromSeconds(1001.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_DOUBLE_EQ(provider.latest()->heading_true_deg, 123.5);
}

TEST(OwnShipNmeaAdapter, RejectsMalformedLines) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  EXPECT_FALSE(adapter.ingest("garbage", Timestamp::fromSeconds(0.0)));
}
