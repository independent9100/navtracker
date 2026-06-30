#include <gtest/gtest.h>
#include "core/land/CoastlineGeometry.hpp"

using navtracker::CoastlineGeometry;
using navtracker::CoastlinePriorParams;
using navtracker::LandPolygon;

namespace {
CoastlineGeometry squareIsland() {
  LandPolygon p;
  p.outer = { {-71.06,42.36}, {-71.04,42.36}, {-71.04,42.38}, {-71.06,42.38}, {-71.06,42.36} };
  CoastlinePriorParams pr; pr.inland_halfwidth_m = 50.0; pr.offshore_halfwidth_m = 50.0;
  return CoastlineGeometry({p}, pr);
}
}  // namespace

TEST(CoastlineGeometry, WellInsideLandIsOne) {
  auto g = squareIsland();
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-9);  // center, deep inland
}
TEST(CoastlineGeometry, OpenWaterFarOutsideIsZero) {
  auto g = squareIsland();
  EXPECT_NEAR(g.priorAtGeodetic(42.300, -71.200), 0.0, 1e-9);  // km away
}
TEST(CoastlineGeometry, WaterlineIsAboutHalf) {
  auto g = squareIsland();
  // A point essentially ON the west edge (lon -71.06) at mid-latitude.
  const double c = g.priorAtGeodetic(42.370, -71.06);
  EXPECT_GT(c, 0.35);
  EXPECT_LT(c, 0.65);  // ~0.5 at the boundary with equal in/off margins
}
TEST(CoastlineGeometry, MonotonicAcrossShore) {
  auto g = squareIsland();
  // Walk west→east across the west edge: deep water, just-offshore, on-edge,
  // just-inland, deep-inland → prior must be non-decreasing.
  const double far_w  = g.priorAtGeodetic(42.370, -71.0630); // ~250 m offshore
  const double near_w = g.priorAtGeodetic(42.370, -71.0605); // ~40 m offshore
  const double inland = g.priorAtGeodetic(42.370, -71.0590); // ~80 m inland
  EXPECT_LE(far_w, near_w);
  EXPECT_LE(near_w, inland);
  EXPECT_NEAR(far_w, 0.0, 1e-6);
}
TEST(CoastlineGeometry, EmptyGeometryIsZero) {
  CoastlineGeometry g;
  EXPECT_TRUE(g.empty());
  EXPECT_NEAR(g.priorAtGeodetic(42.37, -71.05), 0.0, 1e-9);
}
