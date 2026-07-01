#include "core/output/StaticHazardOutput.hpp"

#include <cmath>

namespace navtracker {

std::uint64_t staticHazardId(const StaticObstacle& obs) {
  // Round to ~1e-5 deg (~1 m) so tiny numeric jitter does not change the id.
  const long long lat = std::llround(obs.position.lat_deg * 1e5);
  const long long lon = std::llround(obs.position.lon_deg * 1e5);
  std::uint64_t h = 1469598103934665603ULL;  // FNV-1a offset basis
  auto mix = [&h](std::uint64_t v) {
    h ^= v;
    h *= 1099511628211ULL;  // FNV prime
  };
  mix(static_cast<std::uint64_t>(lat));
  mix(static_cast<std::uint64_t>(lon));
  mix(static_cast<std::uint64_t>(obs.category));
  return h;
}

StaticHazardOutput toStaticHazardOutput(const StaticObstacle& obs) {
  StaticHazardOutput o;
  o.hazard_id = staticHazardId(obs);
  o.position = obs.position;
  o.keep_clear_radius_m = obs.keep_clear_radius_m;
  o.footprint_radius_m = obs.footprint_radius_m;
  o.category = obs.category;
  o.water_level = obs.water_level;
  o.depth_m = obs.depth_m;
  o.lit = obs.lit;
  o.aton = obs.aton;
  o.is_charted = true;
  o.source_id = obs.source_id;
  return o;
}

}  // namespace navtracker
