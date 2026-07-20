#include <gtest/gtest.h>
#include <fstream>
#include "tests/support/FixtureGuard.hpp"
#include "adapters/land/GeoJsonCoastline.hpp"

using navtracker::CoastlinePriorParams;
using navtracker::parseCoastlineGeoJson;
using navtracker::loadCoastlineGeoJson;

namespace {
const char* kTiny = R"({
  "type":"FeatureCollection",
  "features":[
    {"type":"Feature","properties":{},
     "geometry":{"type":"Polygon","coordinates":[
       [[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]}}
  ]})";
}

TEST(GeoJsonCoastline, ParsesPolygonAndPriorIsOneInside) {
  auto g = parseCoastlineGeoJson(kTiny, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);  // inside
  EXPECT_NEAR(g.priorAtGeodetic(42.300, -71.200), 0.0, 1e-6);  // far water
}

TEST(GeoJsonCoastline, ParsesMultiPolygon) {
  const char* mp = R"({"type":"FeatureCollection","features":[
    {"type":"Feature","properties":{},"geometry":{"type":"MultiPolygon","coordinates":[
      [[[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]]}}]})";
  auto g = parseCoastlineGeoJson(mp, CoastlinePriorParams{});
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);
}

// #26 M20: the coastline parser leaked a raw nlohmann parse_error on malformed
// input and used const operator[] on a possibly-missing "coordinates" key
// (assert in debug / UB under NDEBUG). The sibling GeoJsonStaticObstacles was
// hardened (R7.2); mirror it: surface malformed JSON as std::runtime_error and
// skip a malformed feature rather than aborting the whole parse.
TEST(GeoJsonCoastline, MalformedJsonThrowsRuntimeError) {
  EXPECT_THROW(parseCoastlineGeoJson("{ this is not json", CoastlinePriorParams{}),
               std::runtime_error);
}

TEST(GeoJsonCoastline, PolygonMissingCoordinatesKeyIsSkippedNotCrash) {
  // First feature is a Polygon with NO "coordinates" key (const operator[] on
  // a missing key is UB today); the second is valid. The bad feature must be
  // skipped and the good one parsed.
  const char* mixed = R"({"type":"FeatureCollection","features":[
    {"type":"Feature","properties":{},"geometry":{"type":"Polygon"}},
    {"type":"Feature","properties":{},"geometry":{"type":"Polygon","coordinates":[
      [[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]}}]})";
  auto g = parseCoastlineGeoJson(mixed, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);  // valid feature parsed
}

TEST(GeoJsonCoastline, NonNumericCoordinatesSkippedNotThrow) {
  // A ring vertex whose entries are strings, not numbers: get<double>() throws
  // today and escapes the parse. Guard it — skip the bad feature.
  const char* bad = R"({"type":"FeatureCollection","features":[
    {"type":"Feature","properties":{},"geometry":{"type":"Polygon","coordinates":[
      [["x","y"],["a","b"],["c","d"],["x","y"]]]}},
    {"type":"Feature","properties":{},"geometry":{"type":"Polygon","coordinates":[
      [[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]}}]})";
  auto g = parseCoastlineGeoJson(bad, CoastlinePriorParams{});
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);
}

TEST(GeoJsonCoastline, BostonFixtureSmoke) {
  const std::string path =
      std::string(NAVTRACKER_SOURCE_DIR) + "/tests/fixtures/philos/boston.geojson";
  std::ifstream f(path);
  NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP(!f.good(),
                                     "boston.geojson fixture not present");
  auto g = loadCoastlineGeoJson(path, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  // Charlestown Navy Yard area = land; mid-harbour = water.
  EXPECT_GT(g.priorAtGeodetic(42.3730, -71.0535), 0.5);
  EXPECT_NEAR(g.priorAtGeodetic(42.340, -71.000), 0.0, 1e-6);
}
