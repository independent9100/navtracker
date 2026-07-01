#pragma once

#include <string>
#include <vector>

#include "core/types/StaticObstacle.hpp"

namespace navtracker {

/// Parse a GeoJSON FeatureCollection of Point features into StaticObstacles.
/// See docs/superpowers/plans/2026-07-01-static-obstacle-stage1.md, Task 5 for
/// the property schema and defaults. Features without a valid Point geometry
/// (>= 2 finite coordinates) are skipped.
std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text);

/// Load a GeoJSON file from disk and parse it.
/// Throws std::runtime_error if the file cannot be opened.
std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path);

}  // namespace navtracker
