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
///   - has lat outside [-90, 90] or lon outside [-180, 180].
/// A negative footprint / keep-clear / position-uncertainty radius is CLAMPED to
/// 0 with the obstacle kept (finding #6) — a charted hazard is never dropped over
/// one malformed optional field. NOTE: a negative (or zero) keep_clear_radius_m
/// therefore yields a FOOTPRINT-ONLY hazard with NO proximity-alarm ring
/// (keep_clear==0 == "no operational keep-clear margin", a valid chart state).
/// A consumer that relies on the keep-clear alarm must treat keep_clear==0
/// hazards accordingly rather than assume every charted obstacle alarms; a
/// pipeline that must distinguish "malformed" from "intentionally absent" should
/// validate radii before this parser (it has no per-field error channel).
/// Throws std::runtime_error if the JSON itself is malformed.
///
/// Caller contract (finding #5): individual bad features are skipped for
/// resilience, so a systematically corrupt file can parse to an EMPTY vector
/// without throwing. Because charted obstacles are safety-critical, a caller that
/// expects hazards should treat an empty result from a non-empty source as
/// suspicious (log / alert) rather than silently proceeding as "no hazards".
std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text);

/// Load a GeoJSON file from disk and parse it.
/// Throws std::runtime_error if the file cannot be opened.
std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path);

}  // namespace navtracker
