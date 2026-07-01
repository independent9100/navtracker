#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::ObstacleCategory;
using navtracker::StaticObstacle;
using navtracker::StaticObstacleModel;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {

// A datum near Boston Harbor; the exact origin is irrelevant to the ramp math.
Datum datum() { return Datum(Geodetic{42.35, -71.05, 0.0}); }

// One obstacle placed at the datum origin (ENU ~ (0,0)), 15 m hard footprint,
// 100 m soft keep-clear buffer.
StaticObstacle originObstacle() {
  StaticObstacle o;
  o.position = Geodetic{42.35, -71.05, 0.0};
  o.footprint_radius_m = 15.0;
  o.keep_clear_radius_m = 100.0;
  o.category = ObstacleCategory::Pile;
  return o;
}

}  // namespace

// Inside the hard footprint → suppression 1.0 (hard-gate region).
TEST(StaticObstacleModel, InsideFootprintFullySuppressed) {
  StaticObstacleModel m({originObstacle()}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(5.0, 0.0)), 1.0, 1e-9);
}

// Just outside the footprint → soft, and STRICTLY below the 0.95 hard gate
// (soft_max default 0.9), so the keep-clear buffer never hard-drops a vessel.
TEST(StaticObstacleModel, JustOutsideFootprintIsSoftBelowGate) {
  StaticObstacleModel m({originObstacle()}, datum());
  const double c = m.birthSuppression(Eigen::Vector2d(16.0, 0.0));
  EXPECT_GT(c, 0.0);
  EXPECT_LT(c, 0.95);
}

// Mid-buffer ramps down; farther is less suppressed than nearer.
TEST(StaticObstacleModel, BufferRampsDownWithDistance) {
  StaticObstacleModel m({originObstacle()}, datum());
  const double near = m.birthSuppression(Eigen::Vector2d(40.0, 0.0));
  const double far = m.birthSuppression(Eigen::Vector2d(80.0, 0.0));
  EXPECT_GT(near, far);
  EXPECT_GT(far, 0.0);
}

// Beyond the keep-clear radius → clear water, no suppression.
TEST(StaticObstacleModel, BeyondKeepClearNoSuppression) {
  StaticObstacleModel m({originObstacle()}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(150.0, 0.0)), 0.0, 1e-9);
}

// No obstacles → always 0.0.
TEST(StaticObstacleModel, EmptyModelNeverSuppresses) {
  StaticObstacleModel m({}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_TRUE(m.obstacles().empty());
}

// obstacles() returns the charted list verbatim.
TEST(StaticObstacleModel, ObstaclesAccessor) {
  StaticObstacleModel m({originObstacle()}, datum());
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_EQ(m.obstacles()[0].category, ObstacleCategory::Pile);
  EXPECT_DOUBLE_EQ(m.obstacles()[0].keep_clear_radius_m, 100.0);
}

// After a datum recenter, the obstacle's ENU cache is rebuilt so a query at
// the obstacle's true position is still fully suppressed. Use a datum shifted
// by a large offset; the obstacle's geodetic position is unchanged, so its ENU
// coordinate changes and the model must follow.
TEST(StaticObstacleModel, DatumRecenterRebuildsEnu) {
  StaticObstacleModel m({originObstacle()}, datum());
  // Query at the obstacle's ENU under the OLD datum is inside footprint.
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 1.0, 1e-9);

  Datum shifted(Geodetic{42.40, -71.05, 0.0});  // ~5.5 km north
  m.onDatumRecentered(datum(), shifted);
  // Under the NEW datum, the obstacle sits at ~(0, -5560) m; a query at (0,0)
  // is now far away (no suppression), while a query at the obstacle's new ENU
  // is fully suppressed.
  const Eigen::Vector3d obs_enu =
      shifted.toEnu(Geodetic{42.35, -71.05, 0.0});
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(obs_enu.x(), obs_enu.y())),
              1.0, 1e-9);
}
