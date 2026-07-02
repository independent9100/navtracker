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

// R7.2: a non-numeric coordinate is skipped, not thrown (header contract says
// invalid-geometry features are skipped).
TEST(GeoJsonStaticObstacles, NonNumericCoordinateSkipped) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point",
      "coordinates":["not_a_number",42.35]},"properties":{"category":"rock"}}]})";
  std::vector<StaticObstacle> obs;
  EXPECT_NO_THROW(obs = parseStaticObstaclesGeoJson(json));
  EXPECT_EQ(obs.size(), 0u);
}

// R7.2: latitude outside [-90, 90] is skipped.
TEST(GeoJsonStaticObstacles, OutOfRangeLatitudeSkipped) {
  const std::string hi = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,95.0]},
      "properties":{}}]})";
  const std::string lo = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,-91.0]},
      "properties":{}}]})";
  EXPECT_EQ(parseStaticObstaclesGeoJson(hi).size(), 0u);
  EXPECT_EQ(parseStaticObstaclesGeoJson(lo).size(), 0u);
}

// R7.2: longitude outside [-180, 180] is skipped.
TEST(GeoJsonStaticObstacles, OutOfRangeLongitudeSkipped) {
  const std::string hi = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[181.0,42.35]},
      "properties":{}}]})";
  const std::string lo = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-181.0,42.35]},
      "properties":{}}]})";
  EXPECT_EQ(parseStaticObstaclesGeoJson(hi).size(), 0u);
  EXPECT_EQ(parseStaticObstaclesGeoJson(lo).size(), 0u);
}

// R7.2 finding #6: a negative radius (footprint / keep-clear / position-
// uncertainty) is CLAMPED to 0, not used to drop the whole obstacle. A charted
// hazard is safety-critical data — losing it entirely over one malformed
// optional field is worse than zeroing that field; the position + category (and
// any valid radii) still map the hazard. Only the offending magnitude is reset.
TEST(GeoJsonStaticObstacles, NegativeRadiusClampedObstacleKept) {
  // Negative footprint clamps to 0; the valid keep-clear alarm ring survives.
  const std::string json = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"wreck","footprint_radius_m":-5.0,
      "keep_clear_radius_m":120.0,"position_uncertainty_m":-2.0}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);  // hazard preserved, not dropped
  EXPECT_EQ(obs[0].category, ObstacleCategory::Wreck);
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 0.0);      // clamped
  EXPECT_DOUBLE_EQ(obs[0].position_uncertainty_m, 0.0);  // clamped
  EXPECT_DOUBLE_EQ(obs[0].keep_clear_radius_m, 120.0);   // valid field intact
}

// Review finding #2: a negative keep_clear_radius_m (e.g. a sign-flipped export)
// clamps to 0 — the obstacle is KEPT but becomes footprint-only with NO
// proximity-alarm ring (keep_clear==0 == "no operational keep-clear margin", a
// valid chart state). This is the DOCUMENTED behaviour (header contract): a
// consumer that relies on the keep-clear alarm must treat keep_clear==0 hazards
// accordingly rather than assume every charted obstacle alarms. Explicit
// coverage of exactly this field (the earlier rewrite dropped it).
TEST(GeoJsonStaticObstacles, NegativeKeepClearClampsToFootprintOnly) {
  const std::string json = R"({"type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"rock","footprint_radius_m":8.0,
      "keep_clear_radius_m":-120.0}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);                            // hazard kept
  EXPECT_EQ(obs[0].category, ObstacleCategory::Rock);
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 8.0);     // birth suppression intact
  EXPECT_DOUBLE_EQ(obs[0].keep_clear_radius_m, 0.0);    // alarm ring cleared
}

// R7.2: malformed JSON surfaces as std::runtime_error (documented), not a raw
// nlohmann::json::parse_error leaking to the caller.
TEST(GeoJsonStaticObstacles, MalformedJsonThrowsRuntimeError) {
  EXPECT_THROW(parseStaticObstaclesGeoJson("{invalid json"), std::runtime_error);
}

// R7.2: an invalid feature is skipped without discarding a following valid one.
TEST(GeoJsonStaticObstacles, ValidFeatureAfterInvalidIsKept) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point",
      "coordinates":["bad",42.35]},"properties":{"category":"rock"}},
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"pile"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Pile);
}
