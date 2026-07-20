#include "adapters/land/GeoJsonCoastline.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace navtracker {
namespace {

/// Parse one GeoJSON coordinate ring [lon, lat] → Eigen::Vector2d(lon, lat).
std::vector<Eigen::Vector2d> parseRing(const nlohmann::json& coords) {
  std::vector<Eigen::Vector2d> r;
  if (!coords.is_array()) return r;  // #26 M20: tolerate a non-array ring
  r.reserve(coords.size());
  for (const auto& pt : coords) {
    // #26 M20: a vertex must be an array of ≥2 NUMBERS. get<double>() on a
    // string / null threw a type_error that escaped the whole parse; skip it.
    if (pt.is_array() && pt.size() >= 2 && pt[0].is_number() &&
        pt[1].is_number())
      r.emplace_back(pt[0].get<double>(), pt[1].get<double>());  // [lon, lat]
  }
  return r;
}

/// Build a LandPolygon from a GeoJSON Polygon coordinates array
/// (array of rings; ring[0] = outer, rest = holes) and append to out.
void addPolygon(const nlohmann::json& poly_coords,
                std::vector<LandPolygon>& out) {
  if (!poly_coords.is_array() || poly_coords.empty()) return;
  LandPolygon p;
  p.outer = parseRing(poly_coords[0]);
  if (p.outer.size() < 3) return;
  for (std::size_t i = 1; i < poly_coords.size(); ++i) {
    auto hole = parseRing(poly_coords[i]);
    if (hole.size() >= 3)
      p.holes.push_back(std::move(hole));
  }
  out.push_back(std::move(p));
}

}  // namespace

CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text,
                                        CoastlinePriorParams params) {
  // #26 M20: surface malformed JSON as the documented std::runtime_error rather
  // than leaking a raw nlohmann::json::parse_error to the caller (mirrors the
  // R7.2 hardening of GeoJsonStaticObstacles).
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(json_text);
  } catch (const nlohmann::json::exception& e) {
    throw std::runtime_error(std::string("invalid coastline GeoJSON: ") +
                             e.what());
  }
  std::vector<LandPolygon> polys;

  if (!j.contains("features") || !j["features"].is_array())
    return CoastlineGeometry(std::move(polys), params);
  for (const auto& feat : j["features"]) {
    // #26 M20: any per-feature error skips that feature rather than aborting
    // the whole parse, and a missing "coordinates" key is guarded (const
    // operator[] on a missing key is UB / a debug assert).
    try {
      if (!feat.contains("geometry") || feat["geometry"].is_null()) continue;
      const auto& geom = feat["geometry"];
      const std::string type = geom.value("type", std::string{});
      if (!geom.contains("coordinates")) continue;
      const auto& coords = geom["coordinates"];
      if (type == "Polygon") {
        addPolygon(coords, polys);
      } else if (type == "MultiPolygon") {
        if (coords.is_array())
          for (const auto& poly : coords) addPolygon(poly, polys);
      }
      // Other geometry types (Point, LineString, etc.) are silently skipped.
    } catch (const nlohmann::json::exception&) {
      continue;
    }
  }
  return CoastlineGeometry(std::move(polys), params);
}

CoastlineGeometry loadCoastlineGeoJson(const std::string& path,
                                       CoastlinePriorParams params) {
  std::ifstream f(path);
  if (!f.good())
    throw std::runtime_error("coastline geojson not found: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return parseCoastlineGeoJson(ss.str(), params);
}

}  // namespace navtracker
