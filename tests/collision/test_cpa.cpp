#include <cmath>
#include <gtest/gtest.h>

#include "core/collision/Cpa.hpp"

using navtracker::CpaResult;
using navtracker::computeCpa;
using navtracker::Timestamp;
using navtracker::Track;

namespace {

Track makeCvTrack(double px, double py, double vx, double vy, double t) {
  Track tr;
  tr.last_update = Timestamp::fromSeconds(t);
  tr.state = Eigen::VectorXd(4);
  tr.state << px, py, vx, vy;
  tr.covariance = Eigen::MatrixXd::Identity(4, 4);
  return tr;
}

}  // namespace

TEST(Cpa, HeadOnCollisionGivesZeroDistanceAndPositiveTcpa) {
  // A at (-100, 0) moving +x at 10 m/s; B at (+100, 0) moving -x at 10 m/s.
  // They meet at the origin in 10 s with distance 0.
  const Track a = makeCvTrack(-100.0, 0.0,  10.0, 0.0, 0.0);
  const Track b = makeCvTrack( 100.0, 0.0, -10.0, 0.0, 0.0);
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(r.tcpa_seconds,  10.0, 1e-9);
  EXPECT_NEAR(r.cpa_distance_m, 0.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}

TEST(Cpa, ParallelCoursesWithLateralOffsetClampsToCurrentDistance) {
  // Both moving +x at 10 m/s, 20 m apart on y. Parallel motion -> tcpa = 0,
  // cpa = current distance = 20 m, not diverging.
  const Track a = makeCvTrack(0.0,   0.0, 10.0, 0.0, 0.0);
  const Track b = makeCvTrack(0.0,  20.0, 10.0, 0.0, 0.0);
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(r.tcpa_seconds,  0.0, 1e-12);
  EXPECT_NEAR(r.cpa_distance_m, 20.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}

TEST(Cpa, AlreadyPastClosestApproachReportsCurrentDistanceAndDiverging) {
  // A at origin, B at (50, 0) — both moving in same direction along +x with
  // B faster. They are already separating; CPA was in the past.
  const Track a = makeCvTrack(0.0,  0.0, 5.0, 0.0, 0.0);
  const Track b = makeCvTrack(50.0, 0.0, 10.0, 0.0, 0.0);
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(r.tcpa_seconds, 0.0, 1e-12);
  EXPECT_NEAR(r.cpa_distance_m, 50.0, 1e-9);  // current distance
  EXPECT_TRUE(r.is_diverging);
}

TEST(Cpa, PerpendicularPassGivesCorrectMinimum) {
  // A at (-100, 0) moving +x at 10 m/s; B stationary at (0, 5).
  // CPA: A passes through origin at t=10s; minimum distance is 5 m.
  const Track a = makeCvTrack(-100.0, 0.0, 10.0, 0.0, 0.0);
  const Track b = makeCvTrack(   0.0, 5.0,  0.0, 0.0, 0.0);
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(r.tcpa_seconds,  10.0, 1e-9);
  EXPECT_NEAR(r.cpa_distance_m, 5.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}

TEST(Cpa, DifferentLastUpdateTimesExtrapolatedCorrectly) {
  // A's state is at t=0: (-100, 0) with vx=10. t_ref is t=5, so A should be
  // extrapolated to (-50, 0). B's state is at t=3: (100, 0) with vx=-10, so
  // at t_ref=5 B is at (80, 0). Collision course in 6.5s from t_ref=5.
  const Track a = makeCvTrack(-100.0, 0.0,  10.0, 0.0, 0.0);
  const Track b = makeCvTrack( 100.0, 0.0, -10.0, 0.0, 3.0);
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(5.0));
  // dp at t_ref = (-50, 0) - (80, 0) = (-130, 0); dv = (20, 0).
  // t_cpa = -dp.dv / dv.dv = -(-130*20)/(400) = 6.5
  EXPECT_NEAR(r.tcpa_seconds, 6.5, 1e-9);
  EXPECT_NEAR(r.cpa_distance_m, 0.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}

TEST(Cpa, BothStationaryReportsConstantSeparation) {
  // Edge case: both tracks have zero velocity. dv = 0, parallel branch.
  const Track a = makeCvTrack(0.0,  0.0, 0.0, 0.0, 0.0);
  const Track b = makeCvTrack(30.0, 40.0, 0.0, 0.0, 0.0);  // 50 m away
  const CpaResult r = computeCpa(a, b, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(r.tcpa_seconds, 0.0, 1e-12);
  EXPECT_NEAR(r.cpa_distance_m, 50.0, 1e-9);
  EXPECT_FALSE(r.is_diverging);
}
