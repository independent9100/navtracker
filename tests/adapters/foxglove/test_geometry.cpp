#include <gtest/gtest.h>
#include <cmath>
#include "adapters/foxglove/Geometry.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

TEST(Geometry, EllipseAxesMatchEigenvaluesForDiagonalCov) {
  Eigen::Matrix2d cov;
  cov << 9.0, 0.0, 0.0, 4.0;            // sigma_x=3, sigma_y=2
  auto pts = covarianceEllipse(Eigen::Vector2d(10.0, -5.0), cov, /*k=*/2.0, /*n=*/4);
  // n=4 -> points at angles 0, 90, 180, 270 deg in eigenbasis.
  // Max |x-cx| should be k*sigma_x = 6, max |y-cy| = k*sigma_y = 4.
  double max_dx = 0, max_dy = 0;
  for (auto& p : pts) { max_dx = std::max(max_dx, std::abs(p.x - 10.0));
                        max_dy = std::max(max_dy, std::abs(p.y + 5.0)); }
  EXPECT_NEAR(max_dx, 6.0, 1e-9);
  EXPECT_NEAR(max_dy, 4.0, 1e-9);
  for (auto& p : pts) EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST(Geometry, EllipseIsClosedLoop) {
  auto pts = covarianceEllipse(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity(), 1.0, 16);
  ASSERT_GE(pts.size(), 2u);
  EXPECT_NEAR(pts.front().x, pts.back().x, 1e-9);
  EXPECT_NEAR(pts.front().y, pts.back().y, 1e-9);
}

TEST(Geometry, BearingWedgeApexAtSensorAndSpread) {
  auto w = bearingWedge(Eigen::Vector2d(1.0, 2.0), /*alpha=*/0.0, /*sigma=*/0.1,
                        /*length=*/100.0, /*k=*/2.0);
  ASSERT_EQ(w.size(), 3u);
  EXPECT_NEAR(w[1].x, 1.0, 1e-9);     // apex == sensor
  EXPECT_NEAR(w[1].y, 2.0, 1e-9);
  // alpha=0 is +east; edges at +/-0.2 rad. Edge1 y > sensor y, edge2 y < sensor y.
  EXPECT_GT(w[0].y, 2.0);
  EXPECT_LT(w[2].y, 2.0);
}

TEST(Geometry, CircleRadiusAndClosed) {
  auto pts = circle(Eigen::Vector2d(5.0, -3.0), /*radius=*/10.0, /*n=*/32);
  ASSERT_GE(pts.size(), 2u);
  EXPECT_NEAR(pts.front().x, pts.back().x, 1e-9);   // closed loop
  EXPECT_NEAR(pts.front().y, pts.back().y, 1e-9);
  for (auto& p : pts) {
    const double r = std::hypot(p.x - 5.0, p.y + 3.0);
    EXPECT_NEAR(r, 10.0, 1e-9);                      // every vertex on the radius
    EXPECT_DOUBLE_EQ(p.z, 0.0);
  }
}

TEST(Geometry, SectorArcApexAndSpanClosed) {
  const double cx = 2.0, cy = 1.0, range = 50.0, half = 0.3;
  auto s = sectorArc(Eigen::Vector2d(cx, cy), /*center_rad=*/0.0, half, range, /*n=*/8);
  ASSERT_GE(s.size(), 3u);
  EXPECT_NEAR(s.front().x, cx, 1e-9);               // starts at apex
  EXPECT_NEAR(s.front().y, cy, 1e-9);
  EXPECT_NEAR(s.back().x, cx, 1e-9);                // closes at apex
  EXPECT_NEAR(s.back().y, cy, 1e-9);
  // Arc points sit at `range` from the apex.
  for (std::size_t i = 1; i + 1 < s.size(); ++i)
    EXPECT_NEAR(std::hypot(s[i].x - cx, s[i].y - cy), range, 1e-9);
}

TEST(Geometry, SectorArcOmniIsFullCircle) {
  auto s = sectorArc(Eigen::Vector2d::Zero(), 0.0, /*half_width=*/M_PI, 20.0, 16);
  // Omni -> full circle: no apex vertex, every point on the radius.
  for (auto& p : s) EXPECT_NEAR(std::hypot(p.x, p.y), 20.0, 1e-9);
}

TEST(Geometry, ColorIsStablePerSource) {
  auto a = colorForSensor(SensorKind::EoIr, "cam-1");
  auto b = colorForSensor(SensorKind::EoIr, "cam-1");
  auto c = colorForSensor(SensorKind::EoIr, "cam-2");
  EXPECT_EQ(a.r, b.r); EXPECT_EQ(a.g, b.g); EXPECT_EQ(a.b, b.b);
  EXPECT_FALSE(a.r == c.r && a.g == c.g && a.b == c.b);
}
