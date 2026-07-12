// Unit tests for the NavtrackerSource self-adapter's pure conversion
// (ticket §5): Track -> ExternalTrack with pedigree auto-filled from
// contributing_sources.

#include "adapters/t2t/NavtrackerSource.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/output/TrackOutput.hpp"
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

// --- Rider A (ticket §10): covariance-axis convention guard --------------
//
// The pre-release review CONFIRMED (HIGH) that TrackOutput's position-covariance
// axis ordering is NOT trustworthy from the header: the header claims the frame
// is "local NED (north-east)", but toGeodeticWithCov emits R·cov_enu·Rᵀ where R
// is only a small convergence-angle *rotation* between two ENU frames — it never
// relabels the axes to NED. So the emitted slot order is ENU (east, north),
// contradicting the header. NEES (Checkpoint 2) is exactly the metric a silent
// axis swap corrupts while staying plausible-looking, so we (1) pin the true
// ordering empirically and (2) prove our Checkpoint-2 pipeline never touches the
// affected field.

// (1) Empirical ordering probe + swap-detector. Target sits AT the datum origin,
// so R = I exactly and the emitted covariance is the ENU covariance verbatim —
// any reordering would be a relabel, not a rotation. This test asserts the
// ordering toTrackOutputENU emits by contract (F3 resolution 2026-07-12: dual
// API toTrackOutputENU / toTrackOutputNED; the ambiguous toTrackOutput was
// removed). This was the Rider-A empirical probe; it is now the permanent
// ENU-side contract pin. The NED-side pin lives in
// tests/output/test_track_output.cpp.
TEST(TrackOutputCovarianceAxis, ToTrackOutputEnuEmitsEastNorthByContract) {
  const geo::Datum datum(geo::Geodetic{59.0, 10.0, 0.0});
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.last_update = Timestamp::fromSeconds(1.0);
  t.state = Eigen::VectorXd(2);
  t.state << 0.0, 0.0;                 // at the datum origin -> R = I (no rotation)
  t.covariance = Eigen::MatrixXd::Zero(2, 2);
  t.covariance(0, 0) = 1.0;            // ENU east variance
  t.covariance(1, 1) = 100.0;          // ENU north variance (deliberately asymmetric)

  const TrackOutput out = toTrackOutputENU(t, datum);
  const Eigen::Matrix2d& P = out.position.position_covariance_m2;

  // toTrackOutputENU emits ENU order by contract: slot (0,0) = EAST, (1,1) =
  // NORTH, and the frame tag says so. (Operator-facing north-first is the
  // separate toTrackOutputNED entry point.)
  EXPECT_EQ(out.covariance_frame, navtracker::CovarianceFrame::Enu);
  EXPECT_NEAR(P(0, 0), 1.0, 1e-6) << "slot (0,0) must carry the EAST variance";
  EXPECT_NEAR(P(1, 1), 100.0, 1e-6) << "slot (1,1) must carry the NORTH variance";
  EXPECT_NEAR(P(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(P(1, 0), 0.0, 1e-9);
}

// (2) Immunity guard: the Checkpoint-2 NEES pipeline reads the RAW ENU track
// covariance through this adapter and scores via fusedTracksEnu()/computeNees in
// ENU — it NEVER routes through toTrackOutput/toGeodeticWithCov. So the Rider-A
// mismatch above cannot reach the NEES number. Pin that: toExternalTrack copies
// the top-left 2×2 of track.covariance verbatim, in ENU (east,north) order.
TEST(NavtrackerSource, CopiesRawEnuCovarianceBypassingTrackOutput) {
  Track t = makeTrack();
  t.covariance = Eigen::MatrixXd::Zero(4, 4);
  t.covariance(0, 0) = 1.0;            // east
  t.covariance(1, 1) = 100.0;          // north (asymmetric — a swap would be visible)
  t.covariance(2, 2) = 5.0;
  t.covariance(3, 3) = 5.0;
  const auto e = NavtrackerSource::toExternalTrack("navtracker", t,
                                                   Timestamp::fromSeconds(5.0));
  ASSERT_TRUE(e.has_value());
  Eigen::Matrix2d expect;
  expect << 1.0, 0.0, 0.0, 100.0;
  EXPECT_TRUE(e->position_cov.isApprox(expect))
      << "adapter must copy the RAW ENU covariance, not the NED-rotated TrackOutput";
}

}  // namespace
}  // namespace navtracker::t2t
