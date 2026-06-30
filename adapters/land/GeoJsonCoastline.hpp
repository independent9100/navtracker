#pragma once

#include <string>
#include "core/land/CoastlineGeometry.hpp"

namespace navtracker {

/// Parse a GeoJSON FeatureCollection string into a CoastlineGeometry.
/// Supports Feature geometries of type Polygon and MultiPolygon.
/// Rings with fewer than 3 points and null geometry entries are skipped.
CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text,
                                        CoastlinePriorParams params);

/// Load a GeoJSON file from disk and parse it.
/// Throws std::runtime_error if the file cannot be opened.
CoastlineGeometry loadCoastlineGeoJson(const std::string& path,
                                       CoastlinePriorParams params);

}  // namespace navtracker
