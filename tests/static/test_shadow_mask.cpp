// Unit tests for the LOS/shadow-guard geometry (core/static/ShadowMask.hpp).
//
// The guard's job: identify cells whose line of sight is blocked by a strong
// closer occluder on (about) the same bearing this scan, so the coverage-aware
// decay does NOT count them as observed-empty. This tests the pure geometry:
// a set of ENU returns about a sensor -> shadow wedges -> is a point shadowed.
#include "core/static/ShadowMask.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace {

using navtracker::computeShadowWedges;
using navtracker::isShadowed;
using navtracker::ShadowGuardParams;
using navtracker::ShadowWedge;

constexpr double kPi = 3.14159265358979323846;

// A return at (bearing, range) about `sensor` (ENU, atan2(dy,dx) convention).
Eigen::Vector2d ret(const Eigen::Vector2d& sensor, double bearing_rad,
                    double range_m) {
  return sensor + range_m * Eigen::Vector2d(std::cos(bearing_rad),
                                            std::sin(bearing_rad));
}

// A dense occluder cluster: `n` returns spread evenly over [c-half, c+half] at
// `range` about the sensor.
std::vector<Eigen::Vector2d> cluster(const Eigen::Vector2d& sensor, double center,
                                     double half_width, double range, int n) {
  std::vector<Eigen::Vector2d> out;
  for (int i = 0; i < n; ++i) {
    const double f = (n == 1) ? 0.0 : (2.0 * i / (n - 1) - 1.0);  // -1..1
    out.push_back(ret(sensor, center + f * half_width, range));
  }
  return out;
}

ShadowGuardParams params() {
  ShadowGuardParams p;
  p.enabled = true;
  p.min_occluder_returns = 6;
  p.cluster_gap_rad = 0.105;   // ~6 deg
  p.wedge_pad_rad = 0.035;     // ~2 deg
  p.range_margin_m = 50.0;
  return p;
}

}  // namespace

// A strong occluder cluster shadows a cell directly behind it on the same
// bearing; a cell closer than the occluder, or on a different bearing, is not.
TEST(ShadowMask, StrongOccluderShadowsCellBehindOnSameBearing) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  const auto occ = cluster(sensor, /*center=*/0.0, /*half=*/0.05, /*range=*/200.0,
                           /*n=*/10);
  const auto wedges = computeShadowWedges(sensor, occ, params());
  ASSERT_EQ(wedges.size(), 1u);

  // Behind the occluder, same bearing, well beyond it -> shadowed.
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, 0.0, 1000.0), wedges,
                         params().range_margin_m));
  // Closer than the occluder -> NOT shadowed (LOS not blocked).
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, 0.0, 100.0), wedges,
                          params().range_margin_m));
  // Same range as occluder is not "beyond" it -> NOT shadowed (edge).
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, 0.0, 200.0), wedges,
                          params().range_margin_m));
  // Different bearing (90 deg away) at long range -> NOT shadowed.
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, kPi / 2, 1000.0), wedges,
                          params().range_margin_m));
}

// A weak return set (below the strength floor) casts no shadow — a lone echo or
// a small craft must not shield the region behind it.
TEST(ShadowMask, WeakClusterCastsNoShadow) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  const auto weak = cluster(sensor, 0.0, 0.05, 200.0, /*n=*/3);  // < min 6
  const auto wedges = computeShadowWedges(sensor, weak, params());
  EXPECT_TRUE(wedges.empty());
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, 0.0, 1000.0), wedges,
                          params().range_margin_m));
}

// The wedge's angular half-width derives from the occluder's own extent (plus a
// small pad), not a fixed constant: a wide occluder shadows a wider arc.
TEST(ShadowMask, WedgeWidthDerivesFromOccluderExtent) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  const double half = 0.20;  // ~11.5 deg half-extent
  const auto occ = cluster(sensor, 0.0, half, 200.0, /*n=*/12);
  const auto wedges = computeShadowWedges(sensor, occ, params());
  ASSERT_EQ(wedges.size(), 1u);
  EXPECT_NEAR(wedges[0].half_width_rad, half + params().wedge_pad_rad, 1e-6);
  // A cell just inside the occluder's angular edge, behind it -> shadowed.
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, half * 0.9, 1000.0), wedges,
                         params().range_margin_m));
  // A cell well outside the arc -> not shadowed.
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, half + 0.2, 1000.0), wedges,
                          params().range_margin_m));
}

// Two separate strong occluders (bearing-disjoint) each cast their own shadow;
// a cell between them, beyond neither, is not shadowed.
TEST(ShadowMask, MultipleOccludersEachCastShadow) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  std::vector<Eigen::Vector2d> pts;
  const auto a = cluster(sensor, /*center=*/0.0, 0.05, 200.0, 8);
  const auto b = cluster(sensor, /*center=*/kPi / 2, 0.05, 300.0, 8);
  pts.insert(pts.end(), a.begin(), a.end());
  pts.insert(pts.end(), b.begin(), b.end());
  const auto wedges = computeShadowWedges(sensor, pts, params());
  ASSERT_EQ(wedges.size(), 2u);
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, 0.0, 1000.0), wedges,
                         params().range_margin_m));
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, kPi / 2, 1000.0), wedges,
                         params().range_margin_m));
  // Between the two occluders (45 deg) -> not shadowed.
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, kPi / 4, 1000.0), wedges,
                          params().range_margin_m));
}

// An occluder straddling the +/-pi bearing wrap is a single cluster, and shadows
// a cell behind it at ~pi (not split into two spurious wedges).
TEST(ShadowMask, BearingWraparoundIsOneOccluder) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  std::vector<Eigen::Vector2d> pts;
  // returns near +pi and -pi (same physical direction).
  for (int i = 0; i < 5; ++i) pts.push_back(ret(sensor, kPi - 0.04 + 0.02 * i, 200.0));
  for (int i = 0; i < 5; ++i) pts.push_back(ret(sensor, -kPi + 0.02 * i, 200.0));
  const auto wedges = computeShadowWedges(sensor, pts, params());
  ASSERT_EQ(wedges.size(), 1u);
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, kPi, 1000.0), wedges,
                         params().range_margin_m));
}

// Disabled params: no wedges, nothing shadowed (the guard is per-instance off).
TEST(ShadowMask, DisabledProducesNoWedges) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  ShadowGuardParams off = params();
  off.enabled = false;
  const auto occ = cluster(sensor, 0.0, 0.05, 200.0, 10);
  EXPECT_TRUE(computeShadowWedges(sensor, occ, off).empty());
}

// The range margin: a cell only just beyond the occluder (within the margin) is
// not yet shadowed; past the margin it is. Guards against clipping the occluder's
// own far side.
TEST(ShadowMask, RangeMarginDelaysShadowOnset) {
  const Eigen::Vector2d sensor(0.0, 0.0);
  const auto occ = cluster(sensor, 0.0, 0.05, 200.0, 10);
  const auto wedges = computeShadowWedges(sensor, occ, params());
  ASSERT_EQ(wedges.size(), 1u);
  // occluder nearest surface ~200 m; margin 50 m.
  EXPECT_FALSE(isShadowed(sensor, ret(sensor, 0.0, 230.0), wedges, 50.0));
  EXPECT_TRUE(isShadowed(sensor, ret(sensor, 0.0, 270.0), wedges, 50.0));
}
