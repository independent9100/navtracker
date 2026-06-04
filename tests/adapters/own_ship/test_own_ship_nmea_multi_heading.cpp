#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

std::string makeNmea(const std::string& payload) {
  std::uint8_t cs = 0;
  for (char c : payload) cs ^= static_cast<std::uint8_t>(c);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + payload + buf;
}

Timestamp at(double s) { return Timestamp::fromSeconds(s); }

}  // namespace

TEST(NmeaMultiHeading, HdgWithDeviationAndVariationParsed) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("IIHDG,123.5,1.0,E,3.0,W"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->magnetic_heading_deg, 124.5, 1e-9);
  EXPECT_NEAR(pose->magnetic_variation_deg, -3.0, 1e-9);
}

TEST(NmeaMultiHeading, HdgWithEmptyVariationLeavesNan) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("IIHDG,90.0,,,,"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->magnetic_heading_deg, 90.0, 1e-9);
  EXPECT_TRUE(std::isnan(pose->magnetic_variation_deg));
}

TEST(NmeaMultiHeading, GpHdtRoutesAsGyroByDefault) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  ASSERT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->heading_true_deg, 123.5, 1e-9);
  EXPECT_TRUE(std::isnan(pose->gps_true_heading_deg));
}

TEST(NmeaMultiHeading, GpHdtRoutesAsGpsHeadingWhenConfigured) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  ASSERT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"), at(1.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->gps_true_heading_deg, 123.5, 1e-9);
  EXPECT_DOUBLE_EQ(pose->heading_true_deg, 0.0);
}
