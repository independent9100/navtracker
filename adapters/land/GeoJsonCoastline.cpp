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
  r.reserve(coords.size());
  for (const auto& pt : coords) {
    if (pt.is_array() && pt.size() >= 2)
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
  for (std::size_t i = 1; i < poly_coords.size(); ++i)
    p.holes.push_back(parseRing(poly_coords[i]));
  out.push_back(std::move(p));
}

}  // namespace

CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text,
                                        CoastlinePriorParams params) {
  const auto j = nlohmann::json::parse(json_text);
  std::vector<LandPolygon> polys;

  const auto& feats =
      j.contains("features") ? j["features"] : nlohmann::json::array();
  for (const auto& feat : feats) {
    if (!feat.contains("geometry") || feat["geometry"].is_null()) continue;
    const auto& geom = feat["geometry"];
    const std::string type = geom.value("type", std::string{});
    if (type == "Polygon") {
      addPolygon(geom["coordinates"], polys);
    } else if (type == "MultiPolygon") {
      for (const auto& poly : geom["coordinates"]) addPolygon(poly, polys);
    }
    // Other geometry types (Point, LineString, etc.) are silently skipped.
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
