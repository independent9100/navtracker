#pragma once

#include <cstdint>
#include <string>

#include "core/geo/Wgs84.hpp"
#include "core/types/StaticObstacle.hpp"

namespace navtracker {

// Drainable output for one static hazard (pull-style, parallel to
// TrackOutput). Position is geodetic — obstacles are charted in WGS84, so no
// datum rotation is needed (unlike a kinematic track). is_charted separates
// Stage-1 chart hazards from Stage-2 live-detected occupancy.
struct StaticHazardOutput {
  std::uint64_t hazard_id{0};
  geo::Geodetic position{};
  double keep_clear_radius_m{0.0};
  double footprint_radius_m{0.0};
  ObstacleCategory category{ObstacleCategory::Unknown};
  WaterLevel water_level{WaterLevel::Unknown};
  double depth_m{0.0};
  bool lit{false};
  AtoNRealism aton{AtoNRealism::NotAtoN};
  bool is_charted{true};
  std::string source_id;
};

// Deterministic stable id from an obstacle's charted position + category.
// Order-independent (not a list index). Rounds lat/lon to ~1 m before hashing
// so numeric jitter does not change the id.
std::uint64_t staticHazardId(const StaticObstacle& obs);

// Build a StaticHazardOutput from a charted obstacle (attributes verbatim,
// id via staticHazardId, is_charted = true).
StaticHazardOutput toStaticHazardOutput(const StaticObstacle& obs);

}  // namespace navtracker
