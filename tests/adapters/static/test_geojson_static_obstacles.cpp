#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "adapters/static/GeoJsonStaticObstacles.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::AtoNRealism;
using navtracker::ObstacleCategory;
using navtracker::StaticObstacle;
using navtracker::WaterLevel;
using navtracker::loadStaticObstaclesGeoJson;
using navtracker::parseStaticObstaclesGeoJson;

// Full-attribute feature parses into all fields.
TEST(GeoJsonStaticObstacles, ParsesFullAttributes) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.052,42.351]},
      "properties":{"category":"wreck","watlev":"submerged","depth_m":3.5,
      "lit":true,"aton":"real","footprint_radius_m":20.0,
      "keep_clear_radius_m":120.0,"position_uncertainty_m":5.0,
      "source_id":"ENC:wreck-1"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_DOUBLE_EQ(obs[0].position.lat_deg, 42.351);
  EXPECT_DOUBLE_EQ(obs[0].position.lon_deg, -71.052);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Wreck);
  EXPECT_EQ(obs[0].water_level, WaterLevel::AlwaysSubmerged);
  EXPECT_DOUBLE_EQ(obs[0].depth_m, 3.5);
  EXPECT_TRUE(obs[0].lit);
  EXPECT_EQ(obs[0].aton, AtoNRealism::Real);
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 20.0);
  EXPECT_DOUBLE_EQ(obs[0].keep_clear_radius_m, 120.0);
  EXPECT_DOUBLE_EQ(obs[0].position_uncertainty_m, 5.0);
  EXPECT_EQ(obs[0].source_id, "ENC:wreck-1");
}

// Sparse feature uses defaults (NaN depth, zero radii, Unknown category).
TEST(GeoJsonStaticObstacles, MissingPropertiesUseDefaults) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"pile"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Pile);
  EXPECT_TRUE(std::isnan(obs[0].depth_m));
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 0.0);
  EXPECT_FALSE(obs[0].lit);
  EXPECT_EQ(obs[0].aton, AtoNRealism::NotAtoN);
}

// Non-Point / null geometry features are skipped.
TEST(GeoJsonStaticObstacles, SkipsInvalidGeometry) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":null,"properties":{}},
     {"type":"Feature","geometry":{"type":"LineString",
      "coordinates":[[-71.05,42.35],[-71.06,42.36]]},"properties":{}},
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"rock"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Rock);
}

// The on-disk fixture loads two obstacles. Use NAVTRACKER_SOURCE_DIR (a
// compile-time absolute path) so the test is working-directory-independent and
// passes under ctest — matching tests/land/test_geojson_coastline.cpp.
TEST(GeoJsonStaticObstacles, LoadsFixtureFromDisk) {
  const std::vector<StaticObstacle> obs = loadStaticObstaclesGeoJson(
      std::string(NAVTRACKER_SOURCE_DIR) +
      "/tests/fixtures/static/harbor_obstacles.geojson");
  ASSERT_EQ(obs.size(), 2u);
}

// Missing file throws.
TEST(GeoJsonStaticObstacles, MissingFileThrows) {
  EXPECT_THROW(loadStaticObstaclesGeoJson("tests/fixtures/static/nope.geojson"),
               std::runtime_error);
}
