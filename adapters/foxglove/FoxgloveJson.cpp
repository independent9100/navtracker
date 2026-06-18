#include "adapters/foxglove/FoxgloveJson.hpp"
#include <cmath>
#include "adapters/foxglove/Schemas.hpp"

namespace navtracker::foxglove {
using nlohmann::json;

json timeJson(Timestamp t) {
  const long long ns = static_cast<long long>(t.nanos());
  return json{{"sec", ns / 1'000'000'000LL}, {"nsec", ns % 1'000'000'000LL}};
}

static json ptJson(const Pt& p) { return json{{"x", p.x}, {"y", p.y}, {"z", p.z}}; }
static json colorJson(const Rgba& c) {
  return json{{"r", c.r}, {"g", c.g}, {"b", c.b}, {"a", c.a}};
}
static json identityPose() {
  return json{{"position", {{"x",0},{"y",0},{"z",0}}},
              {"orientation", {{"x",0},{"y",0},{"z",0},{"w",1}}}};
}

json lineEntity(const std::string& id, const std::vector<Pt>& pts, const Rgba& color,
                double thickness) {
  json points = json::array();
  for (auto& p : pts) points.push_back(ptJson(p));
  json line{{"type", 0},                       // 0 = LINE_STRIP
            {"pose", identityPose()},
            {"thickness", thickness},
            {"scale_invariant", true},
            {"points", points},
            {"color", colorJson(color)},
            {"colors", json::array()},
            {"indices", json::array()}};
  return json{{"id", id}, {"frame_id", kRootFrame}, {"frame_locked", false},
              {"lifetime", {{"sec",0},{"nsec",0}}}, {"metadata", json::array()},
              {"arrows", json::array()}, {"cubes", json::array()},
              {"spheres", json::array()}, {"cylinders", json::array()},
              {"lines", json::array({line})}, {"triangles", json::array()},
              {"texts", json::array()}, {"models", json::array()}};
}

json textEntity(const std::string& id, const Pt& at, const std::string& text,
                const Rgba& color) {
  json txt{{"pose", {{"position", ptJson(at)}, {"orientation", {{"x",0},{"y",0},{"z",0},{"w",1}}}}},
           {"billboard", true}, {"font_size", 12.0}, {"scale_invariant", true},
           {"color", colorJson(color)}, {"text", text}};
  json e = lineEntity(id, {}, color);          // reuse the empty-arrays skeleton
  e["lines"] = json::array();
  e["texts"] = json::array({txt});
  return e;
}

json arrowEntity(const std::string& id, const Pt& a, const Pt& b, const Rgba& color) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len = std::sqrt(dx*dx + dy*dy);
  const double yaw = std::atan2(dy, dx);
  json arrow{{"pose", {{"position", ptJson(a)},
                       {"orientation", {{"x",0},{"y",0},{"z",std::sin(yaw/2)},{"w",std::cos(yaw/2)}}}}},
             {"shaft_length", len * 0.8}, {"shaft_diameter", 0.5},
             {"head_length", len * 0.2}, {"head_diameter", 1.0},
             {"color", colorJson(color)}};
  json e = lineEntity(id, {}, color);
  e["lines"] = json::array();
  e["arrows"] = json::array({arrow});
  return e;
}

json sceneUpdate(Timestamp t, const std::vector<json>& entities) {
  json ents = json::array();
  for (auto& e : entities) { json ec = e; ec["timestamp"] = timeJson(t); ents.push_back(ec); }
  return json{{"deletions", json::array()}, {"entities", ents}};
}

json locationFix(Timestamp t, double lat, double lon, const std::array<double,9>& cov) {
  return json{{"timestamp", timeJson(t)}, {"frame_id", ""},
              {"latitude", lat}, {"longitude", lon}, {"altitude", 0.0},
              {"position_covariance", cov}, {"position_covariance_type", 2}};  // 2 = DIAGONAL_KNOWN
}

json frameTransform(Timestamp t, const std::string& parent, const std::string& child,
                    double tx, double ty, double tz, double yaw) {
  return json{{"timestamp", timeJson(t)}, {"parent_frame_id", parent},
              {"child_frame_id", child},
              {"translation", {{"x",tx},{"y",ty},{"z",tz}}},
              {"rotation", {{"x",0},{"y",0},{"z",std::sin(yaw/2)},{"w",std::cos(yaw/2)}}}};
}

json logMsg(Timestamp t, int level, const std::string& name, const std::string& message) {
  return json{{"timestamp", timeJson(t)}, {"level", level}, {"message", message},
              {"name", name}, {"file", ""}, {"line", 0}};
}

}  // namespace navtracker::foxglove
