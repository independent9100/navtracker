#include <gtest/gtest.h>
#include "core/types/Timestamp.hpp"

using navtracker::Timestamp;

TEST(Timestamp, RoundTripsSeconds) {
  const Timestamp t = Timestamp::fromSeconds(12.5);
  EXPECT_DOUBLE_EQ(t.seconds(), 12.5);
  EXPECT_EQ(t.nanos(), 12'500'000'000);
}

TEST(Timestamp, SecondsSinceIsSignedDelta) {
  const Timestamp a = Timestamp::fromSeconds(10.0);
  const Timestamp b = Timestamp::fromSeconds(13.0);
  EXPECT_DOUBLE_EQ(b.secondsSince(a), 3.0);
  EXPECT_DOUBLE_EQ(a.secondsSince(b), -3.0);
}

TEST(Timestamp, OrdersChronologically) {
  const Timestamp a = Timestamp::fromSeconds(1.0);
  const Timestamp b = Timestamp::fromSeconds(2.0);
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(b > a);
  EXPECT_TRUE(a <= a);
  EXPECT_EQ(a, Timestamp::fromSeconds(1.0));
}
