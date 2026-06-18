#pragma once
#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "adapters/foxglove/Geometry.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::foxglove {

nlohmann::json timeJson(Timestamp t);                       // {sec,nsec}

// A SceneUpdate "lines" primitive (type 0 = LINE_STRIP) of one polyline.
nlohmann::json lineEntity(const std::string& id, const std::vector<Pt>& pts,
                          const Rgba& color, double thickness = 1.0);
// A SceneUpdate "texts" label at a point.
nlohmann::json textEntity(const std::string& id, const Pt& at,
                          const std::string& text, const Rgba& color);
// A SceneUpdate "arrows" primitive from a->b.
nlohmann::json arrowEntity(const std::string& id, const Pt& a, const Pt& b,
                           const Rgba& color);

// Wrap a list of entity json objects into a SceneUpdate message at time t.
// Stamp `t` onto each entity and wrap into a SceneUpdate. When
// lifetime_sec > 0 each entity is given that lifetime (Foxglove auto-expires
// it that long after `t`), so the 3D panel shows recent state rather than the
// whole accumulated history. lifetime_sec == 0 means "persist" (default).
nlohmann::json sceneUpdate(Timestamp t, const std::vector<nlohmann::json>& entities,
                           double lifetime_sec = 0.0);

nlohmann::json locationFix(Timestamp t, double lat_deg, double lon_deg,
                           const std::array<double, 9>& cov_row_major);
nlohmann::json frameTransform(Timestamp t, const std::string& parent,
                              const std::string& child,
                              double tx, double ty, double tz, double yaw_rad);
nlohmann::json logMsg(Timestamp t, int level, const std::string& name,
                      const std::string& message);

}  // namespace navtracker::foxglove
