#pragma once

#include <string>
#include <vector>

#include "core/types/StaticObstacle.hpp"

namespace navtracker {

/// Parse a GeoJSON FeatureCollection of Point features into StaticObstacles.
/// See docs/superpowers/plans/2026-07-01-static-obstacle-stage1.md, Task 5 for
/// the property schema and defaults. A feature is skipped (not fatal) when it:
///   - has a null / non-Point geometry, or < 2 coordinates;
///   - has non-numeric or non-finite coordinates;
///   - has lat outside [-90, 90] or lon outside [-180, 180];
///   - has a negative footprint / keep-clear / position-uncertainty radius.
/// Throws std::runtime_error if the JSON itself is malformed.
std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text);

/// Load a GeoJSON file from disk and parse it.
/// Throws std::runtime_error if the file cannot be opened.
std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path);

}  // namespace navtracker
