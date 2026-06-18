#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "adapters/util/EdgeValidation.hpp"

namespace edge = navtracker::edge;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

TEST(EdgeValidation, LatLonAcceptsPlausible) {
  EXPECT_TRUE(edge::isPlausibleLatLon(53.5, 8.0));
  EXPECT_TRUE(edge::isPlausibleLatLon(-90.0, 180.0));
  EXPECT_TRUE(edge::isPlausibleLatLon(0.0, 0.0));
}

TEST(EdgeValidation, LatLonRejectsAisSentinels) {
  // AIS "position not available": lat 91°, lon 181°.
  EXPECT_FALSE(edge::isPlausibleLatLon(91.0, 181.0));
  EXPECT_FALSE(edge::isPlausibleLatLon(91.0, 8.0));
  EXPECT_FALSE(edge::isPlausibleLatLon(53.5, 181.0));
}

TEST(EdgeValidation, LatLonRejectsOutOfRangeAndNaN) {
  EXPECT_FALSE(edge::isPlausibleLatLon(90.001, 8.0));
  EXPECT_FALSE(edge::isPlausibleLatLon(53.5, -180.001));
  EXPECT_FALSE(edge::isPlausibleLatLon(kNaN, 8.0));
  EXPECT_FALSE(edge::isPlausibleLatLon(53.5, kInf));
}

TEST(EdgeValidation, RangeAcceptsPositiveFinite) {
  EXPECT_TRUE(edge::isPlausibleRange(1.0));
  EXPECT_TRUE(edge::isPlausibleRange(1e6));
}

TEST(EdgeValidation, RangeRejectsZeroNegativeNaN) {
  EXPECT_FALSE(edge::isPlausibleRange(0.0));   // strtod parse failure
  EXPECT_FALSE(edge::isPlausibleRange(-5.0));
  EXPECT_FALSE(edge::isPlausibleRange(kNaN));
  EXPECT_FALSE(edge::isPlausibleRange(kInf));
}

TEST(EdgeValidation, FiniteValueGuard) {
  EXPECT_TRUE(edge::isFiniteValue(0.0));
  EXPECT_TRUE(edge::isFiniteValue(-123.4));
  EXPECT_FALSE(edge::isFiniteValue(kNaN));
  EXPECT_FALSE(edge::isFiniteValue(kInf));
}
