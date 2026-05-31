#include <gtest/gtest.h>
#include "adapters/own_ship/OwnShipProvider.hpp"

using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;

TEST(OwnShipProvider, StartsEmptyThenReturnsLatest) {
  OwnShipProvider p;
  EXPECT_FALSE(p.latest().has_value());
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(5.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 90.0;
  p.update(pose);
  ASSERT_TRUE(p.latest().has_value());
  EXPECT_DOUBLE_EQ(p.latest()->heading_true_deg, 90.0);
}
