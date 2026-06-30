#include <gtest/gtest.h>
#include <fstream>
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

TEST(GeoJsonCoastline, BostonFixtureSmoke) {
  const std::string path =
      std::string(NAVTRACKER_SOURCE_DIR) + "/tests/fixtures/philos/boston.geojson";
  std::ifstream f(path);
  if (!f.good()) GTEST_SKIP() << "boston.geojson fixture not present";
  auto g = loadCoastlineGeoJson(path, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  // Charlestown Navy Yard area = land; mid-harbour = water.
  EXPECT_GT(g.priorAtGeodetic(42.3730, -71.0535), 0.5);
  EXPECT_NEAR(g.priorAtGeodetic(42.340, -71.000), 0.0, 1e-6);
}
