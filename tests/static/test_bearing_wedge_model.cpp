// #17 option 1 — bearing-wedge hazard (camera-only "never invisible" safety net).
// Covers the four review steers: (1) claim is per-drain SUPPRESSION, not latched
// retirement; (2) IDatumChangeSink keeps apexes geographically stable across a
// recenter; (3) contact_id reuse / suspect mints a fresh wedge_id; (4) half-width
// has a floor.
#include <cmath>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/static/BearingWedgeModel.hpp"

using navtracker::BearingWedgeModel;
using navtracker::BearingWedgeParams;
using navtracker::geo::Datum;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
Datum anchor() { return Datum(navtracker::geo::Geodetic{53.5, 8.0, 0.0}); }

// A bearing detection due EAST (math angle atan2(dN,dE)=0 → true bearing 90°).
constexpr double kEastMath = 0.0;
constexpr double kSigma = 0.5 * kDeg2Rad;  // 0.5° composed σ
}  // namespace

TEST(BearingWedgeModel, ObserveCreatesWedgeAndOutputConvertsToTrueBearing) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d(0.0, 0.0), kEastMath, kSigma, "cam", 7);
  ASSERT_EQ(m.size(), 1u);
  const auto live = m.liveWedges();
  ASSERT_EQ(live.size(), 1u);
  EXPECT_GT(live[0].wedge_id, 0u);

  const auto out = m.hazardOutputs();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].bearing_true_deg, 90.0, 1e-6);  // math-east → true-east
  EXPECT_FALSE(out[0].is_charted);
  EXPECT_EQ(out[0].source_id, "cam");
  // Apex ≈ anchor origin (own-ship at the datum origin).
  EXPECT_NEAR(out[0].apex_lat_deg, 53.5, 1e-6);
  EXPECT_NEAR(out[0].apex_lon_deg, 8.0, 1e-6);
  EXPECT_FALSE(out[0].max_range_m.has_value());  // unbounded by default
}

TEST(BearingWedgeModel, HalfWidthUsesFloorWhenSigmaTiny) {
  BearingWedgeParams p;
  p.half_width_sigma_mult = 2.0;
  p.min_half_width_rad = 1.5 * kDeg2Rad;
  BearingWedgeModel m(anchor(), p);
  // 2 × 0.1° = 0.2° < floor → clamped to 1.5°.
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, 0.1 * kDeg2Rad,
                   "cam", 1);
  EXPECT_NEAR(m.hazardOutputs()[0].half_width_deg, 1.5, 1e-6);
  // 2 × 2.0° = 4.0° > floor → used as-is.
  m.observeBearing(2.0, Eigen::Vector2d::Zero(), kEastMath, 2.0 * kDeg2Rad,
                   "cam", 2);
  double hw2 = 0.0;
  for (const auto& o : m.hazardOutputs())
    if (o.hazard_id == 2u || o.half_width_deg > 3.0) hw2 = o.half_width_deg;
  EXPECT_NEAR(hw2, 4.0, 1e-6);
}

TEST(BearingWedgeModel, RefreshKeepsStableId) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  const std::uint64_t id0 = m.liveWedges()[0].wedge_id;
  m.observeBearing(2.0, Eigen::Vector2d::Zero(), 0.1, kSigma, "cam", 7);  // same key
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m.liveWedges()[0].wedge_id, id0) << "refresh must keep the wedge id";
}

// Steer 1: claim is a per-drain SUPPRESSION, not latched retirement. A near
// vessel crossing the bearing of a far, still-refreshed camera contact must NOT
// permanently erase it (the ADR-0002 forbidden failure).
TEST(BearingWedgeModel, ClaimSuppressesButDoesNotDeleteAndReappears) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);

  // A confirmed track sits on the bearing (due east) → wedge suppressed.
  m.observeConfirmedTracks({Eigen::Vector2d(500.0, 0.0)});
  EXPECT_EQ(m.size(), 1u) << "the wedge is still LIVE, only suppressed";
  EXPECT_EQ(m.liveWedges().size(), 1u);
  EXPECT_TRUE(m.activeWedges().empty()) << "claimed → suppressed from the drain";
  EXPECT_TRUE(m.hazardOutputs().empty());

  // The track moves off the bearing (to the north) while the camera keeps
  // seeing the contact → the wedge reappears, no state machine, no re-observe.
  m.observeConfirmedTracks({Eigen::Vector2d(0.0, 500.0)});
  EXPECT_EQ(m.activeWedges().size(), 1u) << "unclaimed → reappears on next drain";

  // Clearing the track set also un-suppresses.
  m.observeConfirmedTracks({Eigen::Vector2d(500.0, 0.0)});
  ASSERT_TRUE(m.activeWedges().empty());
  m.observeConfirmedTracks({});
  EXPECT_EQ(m.activeWedges().size(), 1u);
}

TEST(BearingWedgeModel, OffBearingTrackDoesNotClaim) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  m.observeConfirmedTracks({Eigen::Vector2d(0.0, 500.0)});  // due north
  EXPECT_EQ(m.activeWedges().size(), 1u)
      << "a track off the wedge bearing must not claim it";
}

TEST(BearingWedgeModel, BoundedRangeTrackBeyondMaxDoesNotClaim) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7,
                   /*max_range=*/std::optional<double>{300.0});
  // On-bearing but beyond the 300 m wedge → not inside → no claim.
  m.observeConfirmedTracks({Eigen::Vector2d(1000.0, 0.0)});
  EXPECT_EQ(m.activeWedges().size(), 1u);
  // On-bearing and within 300 m → claim.
  m.observeConfirmedTracks({Eigen::Vector2d(200.0, 0.0)});
  EXPECT_TRUE(m.activeWedges().empty());
}

// Steer 3: a pruned (dead) contact number that reappears must mint a NEW id.
TEST(BearingWedgeModel, ReusedContactAfterPruneMintsNewId) {
  BearingWedgeParams p;
  p.stale_window_s = 5.0;
  BearingWedgeModel m(anchor(), p);
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  const std::uint64_t id0 = m.liveWedges()[0].wedge_id;

  m.pruneStale(100.0);  // way past the window → contact 7 dropped
  EXPECT_EQ(m.size(), 0u);

  // The sensor reuses target number 7 for a different physical contact.
  m.observeBearing(101.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  EXPECT_NE(m.liveWedges()[0].wedge_id, id0)
      << "a reused contact number must not resurrect the dead wedge's identity";
}

// Steer 3 bonus: a suspect flag mints a new id even without a stale gap.
TEST(BearingWedgeModel, SuspectFlagForcesFreshIdWithoutGap) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  const std::uint64_t id0 = m.liveWedges()[0].wedge_id;
  m.observeBearing(2.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7,
                   std::nullopt, /*suspect=*/true);
  EXPECT_NE(m.liveWedges()[0].wedge_id, id0)
      << "a suspected number-reuse must mint a fresh wedge id";
}

// Steer 2: after an own-ship datum recenter, an existing wedge's apex must stay
// geographically fixed (anchor-frame storage; not drift to a new place).
TEST(BearingWedgeModel, DatumRecenterKeepsApexGeographicallyStable) {
  BearingWedgeModel m(anchor());
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 7);
  const auto before = m.hazardOutputs()[0];

  // Own-ship moved > 30 km; the provider recenters to a datum ~5 km north.
  Datum shifted(navtracker::geo::Geodetic{53.545, 8.0, 0.0});
  m.onDatumRecentered(anchor(), shifted);

  const auto after = m.hazardOutputs()[0];
  EXPECT_NEAR(after.apex_lat_deg, before.apex_lat_deg, 1e-6)
      << "the wedge apex must not move when the datum recenters";
  EXPECT_NEAR(after.apex_lon_deg, before.apex_lon_deg, 1e-6);

  // A NEW detection fed in the shifted frame (own-ship at the new origin) lands
  // at the new geographic position, not the old one.
  m.observeBearing(2.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 8);
  bool found_new = false;
  for (const auto& o : m.hazardOutputs())
    if (o.hazard_id != before.hazard_id) {
      found_new = true;
      EXPECT_NEAR(o.apex_lat_deg, 53.545, 1e-4);
    }
  EXPECT_TRUE(found_new);
}

TEST(BearingWedgeModel, PruneStaleRemovesQuietContactsOnly) {
  BearingWedgeParams p;
  p.stale_window_s = 5.0;
  BearingWedgeModel m(anchor(), p);
  m.observeBearing(1.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 1);
  m.observeBearing(10.0, Eigen::Vector2d::Zero(), kEastMath, kSigma, "cam", 2);
  m.pruneStale(11.0);  // contact 1 is 10 s stale (>5), contact 2 is 1 s fresh
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m.liveWedges()[0].source_id, "cam");
  EXPECT_EQ(m.hazardOutputs()[0].hazard_id, m.liveWedges()[0].wedge_id);
}
