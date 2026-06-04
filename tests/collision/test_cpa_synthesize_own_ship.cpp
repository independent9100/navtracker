#include "core/collision/CpaOwnShip.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

namespace navtracker {

TEST(SynthesizeOwnShipTrack, PlacesPoseAtCorrectEnu) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_is_valid = false;

  const Track t = synthesizeOwnShipTrack(pose,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_NEAR(t.state(0), 0.0, 1e-3);
  EXPECT_NEAR(t.state(1), 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(t.state(2), 5.0);
  EXPECT_DOUBLE_EQ(t.state(3), 3.0);
}

TEST(SynthesizeOwnShipTrack, CovarianceMatchesSigmaPos) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 5.0;
  pose.velocity_enu = Eigen::Vector2d::Zero();
  pose.velocity_is_valid = false;
  const Track t = synthesizeOwnShipTrack(pose,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 1), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 1),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 0),  0.0);
}

TEST(SynthesizeOwnShipTrack, SynthesizedTrackCarriesVelocityCovariance) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_std_m_per_s = 1.0;
  pose.velocity_is_valid = true;
  const Track t = synthesizeOwnShipTrack(pose,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_DOUBLE_EQ(t.state(2), 5.0);
  EXPECT_DOUBLE_EQ(t.state(3), 3.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 2), 0.0);
}

TEST(SynthesizeOwnShipTrack, InvalidVelocityProducesZeroVelocityCovariance) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 3.0);
  pose.velocity_std_m_per_s = 1.0;
  pose.velocity_is_valid = false;
  const Track t = synthesizeOwnShipTrack(pose,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 2), 0.0);
}

}  // namespace navtracker
