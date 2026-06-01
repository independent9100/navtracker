#include "sim/NmeaEncode.hpp"

#include <gtest/gtest.h>

#include "adapters/util/Nmea.hpp"

using namespace navtracker;

TEST(NmeaEncode, LatDdmmFormatsPositiveAndNegative) {
  // 53.5 degrees = 53 deg 30.00000 min => "5330.00000,N"
  EXPECT_EQ(sim::formatLatDdmm(53.5),  std::string("5330.00000"));
  EXPECT_EQ(sim::latHemisphere(53.5),  'N');
  EXPECT_EQ(sim::latHemisphere(-53.5), 'S');
  EXPECT_EQ(sim::formatLatDdmm(-53.5), std::string("5330.00000"));
}

TEST(NmeaEncode, LonDdmmZeroPaddedToThreeDegreeDigits) {
  // 8.0 degrees => "00800.00000,E"
  EXPECT_EQ(sim::formatLonDdmm(8.0),  std::string("00800.00000"));
  EXPECT_EQ(sim::lonHemisphere(8.0),  'E');
  EXPECT_EQ(sim::lonHemisphere(-8.0), 'W');
}

TEST(NmeaEncode, WrapWithChecksumRoundTripsParser) {
  // Build a sentence and feed it through the existing NMEA parser; it must
  // accept the checksum we generated.
  const std::string wrapped =
      sim::wrapWithChecksum("GPGGA,000000.00,5330.00000,N,00800.00000,E,1,08,1.0,0.0,M,0.0,M,,");
  ASSERT_FALSE(wrapped.empty());
  EXPECT_EQ(wrapped.front(), '$');
  EXPECT_NE(wrapped.find('*'), std::string::npos);

  const auto parsed = parseNmea(wrapped);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->talker, "GP");
  EXPECT_EQ(parsed->formatter, "GGA");
}

TEST(NmeaEncode, MinuteRoundingCarriesToNextDegree) {
  // 53 deg + 59.999999.../60 deg ~= 53.999999..., rounds to "5400.00000".
  const double almost_54 = 54.0 - 1e-9;
  EXPECT_EQ(sim::formatLatDdmm(almost_54), std::string("5400.00000"));
}
