// Unit tests for ExternalTrack edge validation, defaults, the per-source stale
// guard, the absent==all-Unknown pedigree invariant, and the builders
// (ticket §6.3, third bullet + §2.1/§2.2 invariants).

#include "core/t2t/ExternalTrack.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <optional>

#include "core/own_ship/OwnShipProvider.hpp"
#include "core/t2t/Pedigree.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::t2t {
namespace {

ExternalTrack mk(const std::string& tracker, const std::string& track, double tsec) {
  return makeExternalTrackFromEnu(tracker, track, Timestamp::fromSeconds(tsec),
                                  Eigen::Vector2d(0.0, 0.0),
                                  Eigen::Matrix2d::Identity() * 100.0);
}

TEST(ExternalTrackValidation, AcceptsWellFormed) {
  EXPECT_TRUE(validateExternalTrack(mk("A", "1", 1.0)));
}

TEST(ExternalTrackValidation, RejectsEmptyRequiredIds) {
  ExternalTrack a = mk("A", "1", 1.0);
  a.source_tracker_id.clear();
  std::string why;
  EXPECT_FALSE(validateExternalTrack(a, &why));
  EXPECT_FALSE(why.empty());

  ExternalTrack b = mk("A", "1", 1.0);
  b.source_track_id.clear();
  EXPECT_FALSE(validateExternalTrack(b));
}

TEST(ExternalTrackValidation, RejectsNonFinitePosition) {
  ExternalTrack nan = mk("A", "1", 1.0);
  nan.position_enu(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateExternalTrack(nan));

  ExternalTrack inf = mk("A", "1", 1.0);
  inf.position_enu(1) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateExternalTrack(inf));
}

TEST(ExternalTrackValidation, RejectsNonPsdCovariance) {
  ExternalTrack a = mk("A", "1", 1.0);
  a.position_cov << 1.0, 2.0, 2.0, 1.0;  // eigenvalues 3, -1 -> not PD
  EXPECT_FALSE(validateExternalTrack(a));

  ExternalTrack neg = mk("A", "1", 1.0);
  neg.position_cov = Eigen::Matrix2d::Identity() * -1.0;
  EXPECT_FALSE(validateExternalTrack(neg));
}

TEST(ExternalTrackValidation, RejectsNonPsdVelocityWhenValid) {
  ExternalTrack a = mk("A", "1", 1.0);
  a.velocity_valid = true;
  a.velocity_enu = Eigen::Vector2d(1.0, 1.0);
  a.velocity_cov << 1.0, 2.0, 2.0, 1.0;
  EXPECT_FALSE(validateExternalTrack(a));
}

TEST(ExternalTrackDefaults, ZeroCovarianceIsUnsetSentinelThenFilledPessimistically) {
  ExternalTrack unset = makeExternalTrackFromEnu(
      "A", "1", Timestamp::fromSeconds(1.0), Eigen::Vector2d(0.0, 0.0),
      Eigen::Matrix2d::Zero());
  // Zero (unset) passes validation; it is the accepted sentinel.
  EXPECT_TRUE(validateExternalTrack(unset));
  EXPECT_FALSE(unset.covariance_is_pessimistic_default);

  applyExternalDefaultsIfEmpty(unset);
  EXPECT_TRUE(unset.covariance_is_pessimistic_default);
  EXPECT_NEAR(unset.position_cov(0, 0), 2500.0, 1e-9);  // 50 m, 1-sigma
  EXPECT_NEAR(unset.position_cov(1, 1), 2500.0, 1e-9);
  EXPECT_TRUE(validateExternalTrack(unset));  // now positive-definite

  // A second application is a no-op (covariance already set).
  const Eigen::Matrix2d before = unset.position_cov;
  applyExternalDefaultsIfEmpty(unset);
  EXPECT_TRUE(unset.position_cov.isApprox(before, 1e-12));
}

TEST(ExternalTrackDefaults, VelocityDefaultDoesNotSetPositionFlag) {
  ExternalTrack t = makeExternalTrackFromEnu(
      "A", "1", Timestamp::fromSeconds(1.0), Eigen::Vector2d(0.0, 0.0),
      Eigen::Matrix2d::Identity() * 9.0);  // position covariance already set
  t.velocity_valid = true;
  t.velocity_enu = Eigen::Vector2d(1.0, 0.0);  // velocity_cov left zero

  applyExternalDefaultsIfEmpty(t);
  EXPECT_NEAR(t.velocity_cov(0, 0), 9.0, 1e-9);  // 3 m/s, 1-sigma
  EXPECT_FALSE(t.covariance_is_pessimistic_default);  // position was real
}

TEST(PerSourceStaleGuard, TracksHighWaterPerSourceAndCountsDrops) {
  PerSourceStaleGuard g(/*reject_stale=*/true);
  EXPECT_TRUE(g.accept(mk("A", "1", 10.0)));   // first from A
  EXPECT_FALSE(g.accept(mk("A", "1", 5.0)));   // older than A's high-water
  EXPECT_EQ(g.staleDropped(), 1u);
  EXPECT_TRUE(g.accept(mk("A", "1", 12.0)));   // advances A
  // A slow source B with an old clock is judged against ITS OWN high-water,
  // not A's — so its first (old) report is accepted.
  EXPECT_TRUE(g.accept(mk("B", "9", 1.0)));
  EXPECT_EQ(g.staleDropped(), 1u);
}

TEST(PerSourceStaleGuard, RejectionDisabledAcceptsStaleAndCountsNothing) {
  PerSourceStaleGuard g(/*reject_stale=*/false);
  EXPECT_TRUE(g.accept(mk("A", "1", 10.0)));
  EXPECT_TRUE(g.accept(mk("A", "1", 5.0)));  // stale but accepted
  EXPECT_EQ(g.staleDropped(), 0u);
}

TEST(ExternalTrackPedigree, AbsentBehavesIdenticallyToAllUnknown) {
  ExternalTrack absent = mk("A", "1", 1.0);
  ASSERT_EQ(absent.pedigree, std::nullopt);

  // The invariant: an ABSENT per-track pedigree resolves to all-Unknown. Assert
  // that DIRECTLY on the resolved value — not by comparing effectivePedigree()
  // against a value trivially identical to it, which would pass even if the
  // resolution were broken (combined-review, m1-external-track lens: the prior
  // form compared a pure function to itself).
  const SourcePedigree effective = effectivePedigree(absent);
  EXPECT_TRUE(effective.sensors.empty());
  EXPECT_EQ(effective.default_usage, SensorUsage::Unknown);
  EXPECT_EQ(effective.usageOf("anything"), SensorUsage::Unknown);

  // And it must CLASSIFY as Unknown (not NotUsed) against a discriminating peer:
  // a peer listing a Used stream is PossiblyCorrelated with all-Unknown (overlap
  // is possible) but would be ProvablyIndependent against an all-NotUsed
  // pedigree — so this fails if absent ever resolved to NotUsed instead.
  const SourcePedigree other{{{"ais:feed", SensorUsage::Used}}, SensorUsage::NotUsed};
  EXPECT_EQ(independenceOfPair(effective, other),
            IndependenceClass::PossiblyCorrelated);
}

TEST(ExternalTrackBuilders, GeodeticRequiresDatumThenProjectsToEnu) {
  OwnShipProvider provider;  // no datum yet
  const auto none = makeExternalTrackFromGeodetic(
      "A", "1", Timestamp::fromSeconds(1.0), 60.0, 10.0,
      Eigen::Matrix2d::Identity() * 100.0, provider);
  EXPECT_FALSE(none.has_value());  // no fix yet -> nullopt

  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 60.0;
  pose.lon_deg = 10.0;
  provider.update(pose);
  ASSERT_TRUE(provider.hasDatum());

  const auto at_origin = makeExternalTrackFromGeodetic(
      "A", "1", Timestamp::fromSeconds(1.0), 60.0, 10.0,
      Eigen::Matrix2d::Identity() * 100.0, provider);
  ASSERT_TRUE(at_origin.has_value());
  // Same lat/lon as the datum origin -> ENU near (0,0).
  EXPECT_NEAR(at_origin->position_enu(0), 0.0, 1.0);
  EXPECT_NEAR(at_origin->position_enu(1), 0.0, 1.0);
  EXPECT_EQ(at_origin->source_tracker_id, "A");

  // Out-of-range latitude is rejected regardless of datum.
  const auto bad = makeExternalTrackFromGeodetic(
      "A", "1", Timestamp::fromSeconds(1.0), 999.0, 10.0,
      Eigen::Matrix2d::Identity() * 100.0, provider);
  EXPECT_FALSE(bad.has_value());
}

}  // namespace
}  // namespace navtracker::t2t
