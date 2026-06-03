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

TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsExactMatch) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(b); p.update(c);

  const auto pose_at_2 = p.poseAtOrBefore(Timestamp::fromSeconds(2.0));
  ASSERT_TRUE(pose_at_2.has_value());
  EXPECT_DOUBLE_EQ(pose_at_2->lat_deg, 2.0);
}

TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsMostRecentEarlier) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(c);

  // No pose at exactly t=2 -> falls back to the pose at t=1.
  const auto pose_at_2 = p.poseAtOrBefore(Timestamp::fromSeconds(2.0));
  ASSERT_TRUE(pose_at_2.has_value());
  EXPECT_DOUBLE_EQ(pose_at_2->lat_deg, 1.0);
}

TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsNulloptWhenAllPosesNewer) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(5.0); a.lat_deg = 5.0;
  p.update(a);

  const auto pose_at_3 = p.poseAtOrBefore(Timestamp::fromSeconds(3.0));
  EXPECT_FALSE(pose_at_3.has_value());
}

TEST(OwnShipProviderTest, HistoryDropsOldestWhenLimitReached) {
  OwnShipProvider p(2);  // tiny limit for the test
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(b); p.update(c);

  // Pose at t=1 has been dropped.
  EXPECT_FALSE(p.poseAtOrBefore(Timestamp::fromSeconds(1.0)).has_value());
  EXPECT_TRUE(p.poseAtOrBefore(Timestamp::fromSeconds(2.0)).has_value());
}

TEST(OwnShipProviderTest, LatestSemanticsPreserved) {
  OwnShipProvider p;  // default size
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  p.update(a); p.update(b);
  const auto latest = p.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_DOUBLE_EQ(latest->lat_deg, 2.0);
}
