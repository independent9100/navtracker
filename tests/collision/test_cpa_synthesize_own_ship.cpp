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
  const Eigen::Vector2d v(5.0, 3.0);

  const Track t = synthesizeOwnShipTrack(pose, v, 1.0,
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
  const Track t = synthesizeOwnShipTrack(pose, Eigen::Vector2d::Zero(),
                                         5.0,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 1), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 1),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 0),  0.0);
}

}  // namespace navtracker
