#include <gtest/gtest.h>
#include <stdexcept>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

using navtracker::DatumRecenterPolicy;
using navtracker::IDatumChangeSink;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

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

TEST(OwnShipPoseTest, VelocityFieldsDefaultUnset) {
  OwnShipPose p;
  EXPECT_DOUBLE_EQ(p.velocity_enu.x(), 0.0);
  EXPECT_DOUBLE_EQ(p.velocity_enu.y(), 0.0);
  EXPECT_DOUBLE_EQ(p.velocity_std_m_per_s, 0.0);
  EXPECT_FALSE(p.velocity_is_valid);
}

namespace {
class CountingSink : public IDatumChangeSink {
 public:
  int call_count{0};
  Datum last_old{Geodetic{0.0, 0.0, 0.0}};
  Datum last_new{Geodetic{0.0, 0.0, 0.0}};
  void onDatumRecentered(const Datum& o, const Datum& n) override {
    ++call_count;
    last_old = o;
    last_new = n;
  }
};
}  // namespace

TEST(OwnShipProviderTest, NoDatumBeforeFirstUpdate) {
  OwnShipProvider p;
  EXPECT_FALSE(p.hasDatum());
  EXPECT_THROW((void)p.datum(), std::runtime_error);
}

TEST(OwnShipProviderTest, DatumInitialisesFromFirstUpdate) {
  OwnShipProvider p;
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  p.update(pose);
  ASSERT_TRUE(p.hasDatum());
  // Datum at the pose -> ENU origin is (0,0).
  const auto enu = p.datum().toEnu(Geodetic{53.5, 8.0, 0.0});
  EXPECT_NEAR(enu.x(), 0.0, 1e-3);
  EXPECT_NEAR(enu.y(), 0.0, 1e-3);
}

TEST(OwnShipProviderTest, DatumStaysFixedBelowThreshold) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a;
  a.lat_deg = 53.5;
  a.lon_deg = 8.0;
  p.update(a);
  // Move ~10 km east at 53.5°N: 1° lon ~= 66 km, so +0.15° ~= 10 km.
  OwnShipPose b = a;
  b.lon_deg += 0.15;
  p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, RecenterFiresAtThreshold) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a;
  a.lat_deg = 53.5;
  a.lon_deg = 8.0;
  p.update(a);
  // Move ~50 km east: well past 30 km threshold.
  OwnShipPose b = a;
  b.lon_deg += 0.75;
  p.update(b);
  EXPECT_EQ(sink.call_count, 1);
  const auto enu_b_in_new = p.datum().toEnu(Geodetic{b.lat_deg, b.lon_deg, 0.0});
  // After recenter, b is at the new origin.
  EXPECT_NEAR(enu_b_in_new.x(), 0.0, 1e-3);
}

TEST(OwnShipProviderTest, RecenterDisabledByPolicy) {
  DatumRecenterPolicy policy;
  policy.enable_auto_recenter = false;
  OwnShipProvider p(16, policy);
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a;
  a.lat_deg = 53.5;
  a.lon_deg = 8.0;
  OwnShipPose b = a;
  b.lon_deg += 1.5;  // ~100 km
  p.update(a);
  p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, MultipleSinksAllFire) {
  OwnShipProvider p;
  CountingSink s1, s2;
  p.registerDatumSink(&s1);
  p.registerDatumSink(&s2);
  OwnShipPose a;
  a.lat_deg = 53.5;
  a.lon_deg = 8.0;
  OwnShipPose b = a;
  b.lon_deg += 0.75;
  p.update(a);
  p.update(b);
  EXPECT_EQ(s1.call_count, 1);
  EXPECT_EQ(s2.call_count, 1);
}

TEST(OwnShipProviderTest, UnregisteredSinkDoesNotFire) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  p.unregisterDatumSink(&sink);
  OwnShipPose a;
  a.lat_deg = 53.5;
  a.lon_deg = 8.0;
  OwnShipPose b = a;
  b.lon_deg += 0.75;
  p.update(a);
  p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, ExplicitDatumConstructorHasDatumImmediately) {
  Datum d(Geodetic{53.5, 8.0, 0.0});
  OwnShipProvider p(d);
  EXPECT_TRUE(p.hasDatum());
}
