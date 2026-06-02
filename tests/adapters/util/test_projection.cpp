#include <cmath>

#include <gtest/gtest.h>
#include "adapters/util/Projection.hpp"

using navtracker::projectRangeBearingToEnu;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST(Projection, EastBearingPutsTargetEastOfOwnShip) {
  const Eigen::Vector2d own(100.0, 200.0);
  const auto out = projectRangeBearingToEnu(1000.0, kPi / 2.0, 5.0, 0.01, 0.0, own);
  EXPECT_NEAR(out.pos_enu.x(), 1100.0, 1e-9);
  EXPECT_NEAR(out.pos_enu.y(), 200.0, 1e-6);
}

TEST(Projection, CovarianceAnisotropyMatchesPolarJacobian) {
  const Eigen::Vector2d own(0.0, 0.0);
  const auto out = projectRangeBearingToEnu(1000.0, kPi / 2.0, 5.0, 0.01, 0.0, own);
  EXPECT_NEAR(out.cov(0, 0), 25.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 1), 100.0, 1e-6);
  EXPECT_NEAR(out.cov(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 0), 0.0, 1e-6);
}

TEST(Projection, SigmaHeadingInflatesCrossTrackVarianceInQuadrature) {
  // Same configuration as the existing CovarianceAnisotropyMatchesPolarJacobian
  // test (1 km east of own-ship, 5 m range σ, 0.01 rad bearing σ), but
  // adds 0.02 rad of heading uncertainty. The expected effect is that the
  // total angular variance becomes 0.01² + 0.02² rad², and the ENU
  // covariance's "cross-track" eigenvalue grows proportionally.
  const Eigen::Vector2d own(0.0, 0.0);
  const double range = 1000.0;
  const double range_std = 5.0;
  const double bearing_std = 0.01;
  const double sigma_heading = 0.02;

  const auto zero_h = projectRangeBearingToEnu(
      range, kPi / 2.0, range_std, bearing_std, 0.0, own);
  const auto with_h = projectRangeBearingToEnu(
      range, kPi / 2.0, range_std, bearing_std, sigma_heading, own);

  // East-pointing target: along-range axis is east, cross-track axis is
  // north. So cov(1,1) (north–north) is the cross-track variance.
  const double expected_xtrack_no_h =
      (range * bearing_std) * (range * bearing_std);
  const double expected_xtrack_with_h =
      (range * range) * (bearing_std * bearing_std + sigma_heading * sigma_heading);

  EXPECT_NEAR(zero_h.cov(1, 1), expected_xtrack_no_h, 1e-6);
  EXPECT_NEAR(with_h.cov(1, 1), expected_xtrack_with_h, 1e-6);

  // Along-range variance (east–east) is unchanged.
  EXPECT_NEAR(zero_h.cov(0, 0), with_h.cov(0, 0), 1e-9);
}
