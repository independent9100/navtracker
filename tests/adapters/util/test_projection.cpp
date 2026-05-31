#include <cmath>

#include <gtest/gtest.h>
#include "adapters/util/Projection.hpp"

using navtracker::projectRangeBearingToEnu;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST(Projection, EastBearingPutsTargetEastOfOwnShip) {
  const Eigen::Vector2d own(100.0, 200.0);
  const auto out = projectRangeBearingToEnu(1000.0, kPi / 2.0, 5.0, 0.01, own);
  EXPECT_NEAR(out.pos_enu.x(), 1100.0, 1e-9);
  EXPECT_NEAR(out.pos_enu.y(), 200.0, 1e-6);
}

TEST(Projection, CovarianceAnisotropyMatchesPolarJacobian) {
  const Eigen::Vector2d own(0.0, 0.0);
  const auto out = projectRangeBearingToEnu(1000.0, kPi / 2.0, 5.0, 0.01, own);
  EXPECT_NEAR(out.cov(0, 0), 25.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 1), 100.0, 1e-6);
  EXPECT_NEAR(out.cov(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(out.cov(1, 0), 0.0, 1e-6);
}
