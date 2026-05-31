#include <gtest/gtest.h>
#include "adapters/util/Nmea.hpp"

using navtracker::parseNmea;

TEST(Nmea, ParsesValidSentence) {
  const auto parsed = parseNmea("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->talker, "GP");
  EXPECT_EQ(parsed->formatter, "GGA");
  ASSERT_GE(parsed->fields.size(), 4u);
  EXPECT_EQ(parsed->fields[0], "123519");
  EXPECT_EQ(parsed->fields[1], "4807.038");
}

TEST(Nmea, RejectsBadChecksum) {
  EXPECT_FALSE(parseNmea("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00").has_value());
}

TEST(Nmea, RejectsMissingDelimiters) {
  EXPECT_FALSE(parseNmea("GPGGA,foo,bar").has_value());
  EXPECT_FALSE(parseNmea("$GPGGA,foo,bar").has_value());
}
