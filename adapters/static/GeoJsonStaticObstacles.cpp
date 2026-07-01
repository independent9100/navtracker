#include "adapters/static/GeoJsonStaticObstacles.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace navtracker {
namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

ObstacleCategory toCategory(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "rock") return ObstacleCategory::Rock;
  if (s == "wreck") return ObstacleCategory::Wreck;
  if (s == "obstruction") return ObstacleCategory::Obstruction;
  if (s == "pile") return ObstacleCategory::Pile;
  if (s == "platform") return ObstacleCategory::Platform;
  if (s == "buoy") return ObstacleCategory::Buoy;
  if (s == "beacon") return ObstacleCategory::Beacon;
  if (s == "other") return ObstacleCategory::Other;
  return ObstacleCategory::Unknown;
}

WaterLevel toWaterLevel(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "awash" || s == "covers") return WaterLevel::AwashCoversUncovers;
  if (s == "submerged") return WaterLevel::AlwaysSubmerged;
  if (s == "above") return WaterLevel::AlwaysAboveWater;
  if (s == "floating") return WaterLevel::Floating;
  return WaterLevel::Unknown;
}

AtoNRealism toAton(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "real") return AtoNRealism::Real;
  if (s == "synthetic") return AtoNRealism::Synthetic;
  if (s == "virtual") return AtoNRealism::Virtual;
  return AtoNRealism::NotAtoN;
}

double numberOr(const nlohmann::json& props, const char* key, double dflt) {
  if (props.contains(key) && props[key].is_number())
    return props[key].get<double>();
  return dflt;
}

std::string stringOr(const nlohmann::json& props, const char* key) {
  if (props.contains(key) && props[key].is_string())
    return props[key].get<std::string>();
  return std::string{};
}

}  // namespace

std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text) {
  std::vector<StaticObstacle> out;
  const nlohmann::json root = nlohmann::json::parse(json_text);
  if (!root.contains("features") || !root["features"].is_array()) return out;

  for (const auto& feat : root["features"]) {
    if (!feat.contains("geometry") || feat["geometry"].is_null()) continue;
    const auto& geom = feat["geometry"];
    if (!geom.contains("type") || geom["type"] != "Point") continue;
    if (!geom.contains("coordinates") || !geom["coordinates"].is_array() ||
        geom["coordinates"].size() < 2)
      continue;
    const double lon = geom["coordinates"][0].get<double>();
    const double lat = geom["coordinates"][1].get<double>();
    if (!std::isfinite(lon) || !std::isfinite(lat)) continue;

    StaticObstacle o;
    o.position = geo::Geodetic{lat, lon, 0.0};
    const nlohmann::json props =
        feat.contains("properties") && feat["properties"].is_object()
            ? feat["properties"]
            : nlohmann::json::object();
    o.category = toCategory(stringOr(props, "category"));
    o.water_level = toWaterLevel(stringOr(props, "watlev"));
    o.aton = toAton(stringOr(props, "aton"));
    o.depth_m = numberOr(props, "depth_m",
                         std::numeric_limits<double>::quiet_NaN());
    o.lit = props.contains("lit") && props["lit"].is_boolean()
                ? props["lit"].get<bool>()
                : false;
    o.footprint_radius_m = numberOr(props, "footprint_radius_m", 0.0);
    o.keep_clear_radius_m = numberOr(props, "keep_clear_radius_m", 0.0);
    o.position_uncertainty_m = numberOr(props, "position_uncertainty_m", 0.0);
    o.source_id = stringOr(props, "source_id");
    out.push_back(std::move(o));
  }
  return out;
}

std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path) {
  std::ifstream in(path);
  if (!in.good())
    throw std::runtime_error("cannot open static-obstacle GeoJSON: " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return parseStaticObstaclesGeoJson(ss.str());
}

}  // namespace navtracker
