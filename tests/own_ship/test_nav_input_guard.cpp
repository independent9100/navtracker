// #18 fact-free nav-input guard. Unit tests for evaluateNavInput (pure) plus the
// OwnShipProvider edge integration (fires INavHealthSink, never rewrites poses).
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "core/own_ship/NavInputGuard.hpp"
#include "core/own_ship/OwnShipProvider.hpp"
#include "ports/INavHealthSink.hpp"

using navtracker::evaluateNavInput;
using navtracker::INavHealthSink;
using navtracker::NavHealth;
using navtracker::NavInputGuardConfig;
using navtracker::OwnShipPose;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;

namespace {
OwnShipPose pose(double t, double lat, double lon, double hdg_deg, double sog,
                 bool sog_valid = true) {
  OwnShipPose p;
  p.time = Timestamp::fromSeconds(t);
  p.lat_deg = lat;
  p.lon_deg = lon;
  p.heading_true_deg = hdg_deg;
  p.velocity_enu = Eigen::Vector2d(sog, 0.0);  // magnitude = sog
  p.velocity_is_valid = sog_valid;
  return p;
}
const NavInputGuardConfig kCfg{};  // defaults
}  // namespace

TEST(NavInputGuard, CleanPoseHasNoFlags) {
  const auto prev = pose(0.0, 53.5, 8.0, 90.0, 5.0);
  // ~5 m east in 1 s: dlon = 5 / (111320·cos 53.5°).
  const double dlon = 5.0 / (111320.0 * std::cos(53.5 * M_PI / 180.0));
  const auto curr = pose(1.0, 53.5, 8.0 + dlon, 90.0, 5.0);
  const NavHealth h = evaluateNavInput(prev, curr, kCfg);
  EXPECT_FALSE(h.any());
}

TEST(NavInputGuard, LowSogFlagsHeadingUnreliable) {
  const auto curr = pose(1.0, 53.5, 8.0, 90.0, /*sog=*/0.2);  // below steerage way
  const NavHealth h = evaluateNavInput(std::nullopt, curr, kCfg);
  EXPECT_TRUE(h.heading_unreliable_low_sog);
  EXPECT_FALSE(h.stale_gap);
  EXPECT_FALSE(h.position_jump);
  EXPECT_FALSE(h.heading_jump);
  EXPECT_NEAR(h.sog_mps, 0.2, 1e-9);
}

TEST(NavInputGuard, StaleGapFlagged) {
  const auto prev = pose(0.0, 53.5, 8.0, 90.0, 5.0);
  const auto curr = pose(10.0, 53.5, 8.0, 90.0, 5.0);  // 10 s gap > 3 s
  const NavHealth h = evaluateNavInput(prev, curr, kCfg);
  EXPECT_TRUE(h.stale_gap);
  EXPECT_NEAR(h.gap_s, 10.0, 1e-9);
  EXPECT_FALSE(h.position_jump);  // stationary → no jump
  EXPECT_FALSE(h.heading_jump);
}

TEST(NavInputGuard, PositionJumpFlagged) {
  const auto prev = pose(0.0, 53.5, 8.0, 90.0, 5.0, /*sog_valid=*/false);
  const auto curr = pose(1.0, 53.6, 8.0, 90.0, 5.0, /*sog_valid=*/false);  // ~11 km/s
  const NavHealth h = evaluateNavInput(prev, curr, kCfg);
  EXPECT_TRUE(h.position_jump);
  EXPECT_GT(h.position_step_m, 10000.0);
}

TEST(NavInputGuard, HeadingJumpFlagged) {
  const auto prev = pose(0.0, 53.5, 8.0, 0.0, 5.0);
  const auto curr = pose(1.0, 53.5, 8.0, 90.0, 5.0);  // 90°/s > 60°/s
  const NavHealth h = evaluateNavInput(prev, curr, kCfg);
  EXPECT_TRUE(h.heading_jump);
  EXPECT_NEAR(h.heading_step_deg, 90.0, 1e-6);
  EXPECT_FALSE(h.position_jump);
}

TEST(NavInputGuard, FirstPoseChecksOnlyLowSog) {
  // No previous pose → jump/stale cannot be evaluated; low-SOG still can.
  const auto curr = pose(1.0, 53.5, 8.0, 90.0, /*sog=*/0.1);
  const NavHealth h = evaluateNavInput(std::nullopt, curr, kCfg);
  EXPECT_TRUE(h.heading_unreliable_low_sog);
  EXPECT_FALSE(h.stale_gap);
  EXPECT_FALSE(h.position_jump);
  EXPECT_FALSE(h.heading_jump);
}

TEST(NavInputGuard, OutOfOrderPoseSkipsTwoPoseChecks) {
  const auto prev = pose(5.0, 53.5, 8.0, 0.0, 5.0);
  const auto curr = pose(0.0, 53.6, 8.0, 90.0, 5.0);  // earlier than prev, far, turned
  const NavHealth h = evaluateNavInput(prev, curr, kCfg);
  EXPECT_FALSE(h.stale_gap) << "negative dt must not be read as a stale gap";
  EXPECT_FALSE(h.position_jump) << "non-positive dt must skip the jump check";
  EXPECT_FALSE(h.heading_jump);
}

// ---- OwnShipProvider edge integration ------------------------------------

struct SpyNavSink : INavHealthSink {
  std::vector<NavHealth> events;
  void onNavHealth(const NavHealth& h) override { events.push_back(h); }
};

TEST(OwnShipProviderNavGuard, FiresSinkOnTrippingPoseButNotOnCleanOnes) {
  OwnShipProvider provider;
  SpyNavSink spy;
  provider.setNavHealthSink(&spy);

  provider.update(pose(0.0, 53.5, 8.0, 90.0, 5.0));  // clean first pose → no event
  EXPECT_TRUE(spy.events.empty());

  provider.update(pose(10.0, 53.5, 8.0, 90.0, 5.0));  // 10 s gap → stale event
  ASSERT_EQ(spy.events.size(), 1u);
  EXPECT_TRUE(spy.events.back().stale_gap);
}

TEST(OwnShipProviderNavGuard, InertWithoutASink) {
  OwnShipProvider provider;
  // No sink wired: a wildly bad stream must still just be stored (no crash, and
  // the poses are trusted as-is — the guard never rewrites them).
  provider.update(pose(0.0, 53.5, 8.0, 0.0, 0.1));
  provider.update(pose(10.0, 60.0, 8.0, 200.0, 0.1));  // stale + jump + low-sog
  EXPECT_EQ(provider.historySize(), 2u);
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_DOUBLE_EQ(provider.latest()->lat_deg, 60.0);  // pose passed through intact
}

TEST(OwnShipProviderNavGuard, CleanStreamFiresNothing) {
  OwnShipProvider provider;
  SpyNavSink spy;
  provider.setNavHealthSink(&spy);
  const double dlon = 5.0 / (111320.0 * std::cos(53.5 * M_PI / 180.0));
  for (int k = 0; k < 5; ++k)
    provider.update(pose(k, 53.5, 8.0 + k * dlon, 90.0, 5.0));
  EXPECT_TRUE(spy.events.empty());
}
