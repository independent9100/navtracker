#pragma once

#include <limits>
#include <string>

#include "core/geo/Wgs84.hpp"  // geo::Geodetic

namespace navtracker {

// S-57/S-101-aligned category subset for charted hazards (CATOBS-ish).
enum class ObstacleCategory {
  Unknown, Rock, Wreck, Obstruction, Pile, Platform, Buoy, Beacon, Other
};

// S-57 WATLEV (water-level effect).
enum class WaterLevel {
  Unknown, AwashCoversUncovers, AlwaysSubmerged, AlwaysAboveWater, Floating
};

// AIS AtoN realism (Message 21): physical vs synthetic vs virtual.
enum class AtoNRealism { NotAtoN, Real, Synthetic, Virtual };

// A discrete charted static hazard: rock, wreck, pillar, dolphin, buoy, ...
// NOT coastline (ADR 0002 decision 4). Position is geodetic (chart frame).
// footprint_radius_m + position_uncertainty_m form the hard no-birth core;
// keep_clear_radius_m is the soft buffer and the operator keep-clear ring.
struct StaticObstacle {
  geo::Geodetic position;              // charted position (WGS84)
  double footprint_radius_m{0.0};      // physical extent (hard core)
  double keep_clear_radius_m{0.0};     // soft keep-clear buffer (>= footprint)
  double position_uncertainty_m{0.0};  // positional buffer, added to hard core
  ObstacleCategory category{ObstacleCategory::Unknown};
  WaterLevel water_level{WaterLevel::Unknown};
  double depth_m{std::numeric_limits<double>::quiet_NaN()};  // VALSOU (NaN=unknown)
  bool lit{false};
  AtoNRealism aton{AtoNRealism::NotAtoN};
  std::string source_id;               // provenance (chart id / AtoN MMSI)
};

}  // namespace navtracker
