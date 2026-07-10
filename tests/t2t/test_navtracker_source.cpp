// Unit tests for the NavtrackerSource self-adapter's pure conversion
// (ticket §5): Track -> ExternalTrack with pedigree auto-filled from
// contributing_sources.

#include "adapters/t2t/NavtrackerSource.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/t2t/Pedigree.hpp"

namespace navtracker::t2t {
namespace {

Track makeTrack() {
  Track t;
  t.id = TrackId{7};
  t.last_update = Timestamp::fromSeconds(5.0);
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::VectorXd(4);
  t.state << 100.0, 200.0, 3.0, 4.0;
  t.covariance = Eigen::MatrixXd::Identity(4, 4) * 10.0;
  t.velocity_observed = true;
  t.attributes.mmsi = 12345u;
  t.contributing_sources = {"ais", "radar"};
  return t;
}

TEST(NavtrackerSource, ConvertsTrackWithExactPedigree) {
  const Track t = makeTrack();
  const auto e = NavtrackerSource::toExternalTrack("navtracker", t,
                                                   Timestamp::fromSeconds(5.0));
  ASSERT_TRUE(e.has_value());
  EXPECT_EQ(e->source_tracker_id, "navtracker");
  EXPECT_EQ(e->source_track_id, "7");
  EXPECT_TRUE(e->position_enu.isApprox(Eigen::Vector2d(100.0, 200.0)));
  ASSERT_TRUE(e->velocity_valid);
  EXPECT_TRUE(e->velocity_enu.isApprox(Eigen::Vector2d(3.0, 4.0)));
  ASSERT_TRUE(e->source_status.has_value());
  EXPECT_EQ(*e->source_status, TrackStatus::Confirmed);
  ASSERT_TRUE(e->attributes.mmsi.has_value());
  EXPECT_EQ(*e->attributes.mmsi, 12345u);

  // Pedigree is known EXACTLY: used streams from contributing_sources, else NotUsed.
  ASSERT_TRUE(e->pedigree.has_value());
  EXPECT_EQ(e->pedigree->default_usage, SensorUsage::NotUsed);
  EXPECT_EQ(e->pedigree->usageOf("ais"), SensorUsage::Used);
  EXPECT_EQ(e->pedigree->usageOf("radar"), SensorUsage::Used);
  EXPECT_EQ(e->pedigree->usageOf("eoir"), SensorUsage::NotUsed);
}

TEST(NavtrackerSource, PositionOnlyTrackHasNoVelocity) {
  Track t = makeTrack();
  t.velocity_observed = false;
  const auto e = NavtrackerSource::toExternalTrack("navtracker", t,
                                                   Timestamp::fromSeconds(5.0));
  ASSERT_TRUE(e.has_value());
  EXPECT_FALSE(e->velocity_valid);
}

TEST(NavtrackerSource, EmptyStateYieldsNullopt) {
  Track t;  // default: empty state
  const auto e = NavtrackerSource::toExternalTrack("navtracker", t,
                                                   Timestamp::fromSeconds(1.0));
  EXPECT_FALSE(e.has_value());
}

TEST(NavtrackerSource, PedigreeComposesWithAnotherSharedStreamSource) {
  // A navtracker track that used ais+radar, vs another source that used ais:
  // they share "ais" -> possibly correlated (demonstrates the pedigree wiring).
  const Track t = makeTrack();
  const auto e = NavtrackerSource::toExternalTrack("navtracker", t,
                                                   Timestamp::fromSeconds(5.0));
  ASSERT_TRUE(e.has_value() && e->pedigree.has_value());
  SourcePedigree other;
  other.default_usage = SensorUsage::NotUsed;
  other.sensors["ais"] = SensorUsage::Used;
  EXPECT_EQ(independenceOfPair(*e->pedigree, other),
            IndependenceClass::PossiblyCorrelated);
}

}  // namespace
}  // namespace navtracker::t2t
