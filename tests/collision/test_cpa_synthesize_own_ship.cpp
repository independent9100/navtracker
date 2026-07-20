#include "core/collision/CpaOwnShip.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/collision/Cpa.hpp"
#include "core/geo/Datum.hpp"

namespace navtracker {

TEST(SynthesizeOwnShipTrack, PlacesPoseAtCorrectEnu) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_is_valid = false;

  const Track t = synthesizeOwnShipTrack(pose, provider);
  EXPECT_NEAR(t.state(0), 0.0, 1e-3);
  EXPECT_NEAR(t.state(1), 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(t.state(2), 5.0);
  EXPECT_DOUBLE_EQ(t.state(3), 3.0);
}

TEST(SynthesizeOwnShipTrack, CovarianceMatchesSigmaPos) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 5.0;
  pose.velocity_enu = Eigen::Vector2d::Zero();
  pose.velocity_is_valid = false;
  const Track t = synthesizeOwnShipTrack(pose, provider);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 1), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 1),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 0),  0.0);
}

TEST(SynthesizeOwnShipTrack, SynthesizedTrackCarriesVelocityCovariance) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_std_m_per_s = 1.0;
  pose.velocity_is_valid = true;
  const Track t = synthesizeOwnShipTrack(pose, provider);
  EXPECT_DOUBLE_EQ(t.state(2), 5.0);
  EXPECT_DOUBLE_EQ(t.state(3), 3.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 2), 0.0);
}

TEST(SynthesizeOwnShipTrack, InvalidVelocityProducesZeroVelocityCovariance) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_std_m_per_s = 1.0;
  pose.velocity_is_valid = false;
  const Track t = synthesizeOwnShipTrack(pose, provider);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 2), 0.0);
}

// #29: the own-ship track was stamped last_update = QUERY time t, while its
// state is the raw latest fix (at pose.time). computeCpaWithUncertainty then
// extrapolates every track to t_ref, so own-ship's dt_own = t_ref - t = 0 and
// it is never advanced from pose.time despite a valid velocity — injecting a
// |v_own|·(t_ref - pose.time) error into cpa_distance / tcpa / P(below). Stamp
// the FIX time so own-ship extrapolates symmetrically with the targets (and its
// velocity uncertainty grows over the interval via the CPA Jacobian).
TEST(SynthesizeOwnShipTrack, StampsFixTimeSoCpaExtrapolatesOwnShip) {
  geo::Datum datum({0.0, 0.0, 0.0});
  OwnShipProvider provider(datum);
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);   // fix at t=0
  pose.lat_deg = 0.0;
  pose.lon_deg = 0.0;                         // ENU origin (0,0)
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(0.0, 10.0);  // +10 m/s north
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;

  // Query 10 s after the fix (stale GPS).
  const Track own = synthesizeOwnShipTrack(pose, provider);
  // The synthesized track must be stamped at the FIX time, not the query time.
  EXPECT_DOUBLE_EQ(own.last_update.seconds(), 0.0);

  // Stationary target at ENU (100, 0), current at the query time.
  Track tgt;
  tgt.status = TrackStatus::Confirmed;
  tgt.state = Eigen::Vector4d(100.0, 0.0, 0.0, 0.0);
  tgt.covariance = Eigen::Matrix4d::Identity();
  tgt.covariance(0, 0) = tgt.covariance(1, 1) = 4.0;
  tgt.covariance(2, 2) = tgt.covariance(3, 3) = 0.04;
  tgt.last_update = Timestamp::fromSeconds(10.0);

  const CpaPrediction pred =
      computeCpaWithUncertainty(own, tgt, Timestamp::fromSeconds(10.0), 50.0);
  // Own-ship advances to (0,100) by t=10; distance to the target (100,0) is
  // sqrt(100^2 + 100^2) ~= 141.4 m and they are diverging, so cpa_distance is
  // the current separation. Before the fix own-ship stayed at (0,0) (dt=0), so
  // cpa_distance was ~100 m — the injected |v|·dt error.
  EXPECT_NEAR(pred.cpa_distance_m, 141.421356, 1e-3);
}

}  // namespace navtracker
