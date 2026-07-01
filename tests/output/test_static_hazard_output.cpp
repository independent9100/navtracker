#include <gtest/gtest.h>

#include "core/geo/Wgs84.hpp"
#include "core/output/StaticHazardOutput.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::AtoNRealism;
using navtracker::ObstacleCategory;
using navtracker::StaticHazardOutput;
using navtracker::StaticObstacle;
using navtracker::WaterLevel;
using navtracker::staticHazardId;
using navtracker::toStaticHazardOutput;
using navtracker::geo::Geodetic;

namespace {

StaticObstacle wreck() {
  StaticObstacle o;
  o.position = Geodetic{42.351, -71.052, 0.0};
  o.footprint_radius_m = 20.0;
  o.keep_clear_radius_m = 120.0;
  o.position_uncertainty_m = 5.0;
  o.category = ObstacleCategory::Wreck;
  o.water_level = WaterLevel::AlwaysSubmerged;
  o.depth_m = 3.5;
  o.lit = true;
  o.aton = AtoNRealism::Real;
  o.source_id = "ENC:US5MA...";
  return o;
}

}  // namespace

// Conversion copies charted attributes verbatim; is_charted true.
TEST(StaticHazardOutput, ConversionCopiesAttributes) {
  const StaticHazardOutput o = toStaticHazardOutput(wreck());
  EXPECT_DOUBLE_EQ(o.position.lat_deg, 42.351);
  EXPECT_DOUBLE_EQ(o.position.lon_deg, -71.052);
  EXPECT_DOUBLE_EQ(o.keep_clear_radius_m, 120.0);
  EXPECT_DOUBLE_EQ(o.footprint_radius_m, 20.0);
  EXPECT_DOUBLE_EQ(o.position_uncertainty_m, 5.0);
  EXPECT_EQ(o.category, ObstacleCategory::Wreck);
  EXPECT_EQ(o.water_level, WaterLevel::AlwaysSubmerged);
  EXPECT_DOUBLE_EQ(o.depth_m, 3.5);
  EXPECT_TRUE(o.lit);
  EXPECT_EQ(o.aton, AtoNRealism::Real);
  EXPECT_TRUE(o.is_charted);
  EXPECT_EQ(o.source_id, "ENC:US5MA...");
}

// Id is deterministic and order-independent (function of position + category).
TEST(StaticHazardOutput, IdDeterministicAndStable) {
  EXPECT_EQ(staticHazardId(wreck()), staticHazardId(wreck()));
  EXPECT_EQ(toStaticHazardOutput(wreck()).hazard_id, staticHazardId(wreck()));
}

// Distinct positions → distinct ids.
TEST(StaticHazardOutput, DistinctObstaclesDistinctIds) {
  StaticObstacle a = wreck();
  StaticObstacle b = wreck();
  b.position.lat_deg += 0.01;  // ~1.1 km away
  EXPECT_NE(staticHazardId(a), staticHazardId(b));
}
