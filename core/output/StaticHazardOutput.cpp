#include "core/output/StaticHazardOutput.hpp"

#include <cmath>

namespace navtracker {

std::uint64_t staticHazardId(const StaticObstacle& obs) {
  // Round to ~1e-5 deg (~1 m) so tiny numeric jitter does not change the id.
  const long long lat = std::llround(obs.position.lat_deg * 1e5);
  const long long lon = std::llround(obs.position.lon_deg * 1e5);
  std::uint64_t h = 14695981039346656037ULL;  // FNV-1a 64-bit offset basis
  auto mix = [&h](std::uint64_t v) {
    h ^= v;
    h *= 1099511628211ULL;  // FNV prime
  };
  mix(static_cast<std::uint64_t>(lat));
  mix(static_cast<std::uint64_t>(lon));
  mix(static_cast<std::uint64_t>(obs.category));
  // R7.3: when present, mix source_id so co-located ENC records (e.g. a wreck
  // and an obstruction at the same rounded position + category) get distinct
  // ids. Empty source_id leaves the id unchanged (backward-compatible).
  //
  // Finding #9 — stability contract: source_id MUST be a STABLE per-feature
  // identifier (e.g. the ENC LNAM / record RCID), NOT a volatile string that
  // changes format between exports. Given that, the id is both unique per
  // physical obstacle AND stable across chart re-exports — strictly better than
  // the position+category-only id it replaced, which collided for co-located
  // records. If a producer cannot supply a stable source_id it should leave it
  // empty (falls back to position+category, unchanged) rather than emit a
  // volatile one, which would churn hazard ids on every re-load.
  for (unsigned char c : obs.source_id) mix(static_cast<std::uint64_t>(c));
  return h;
}

StaticHazardOutput toStaticHazardOutput(const StaticObstacle& obs) {
  StaticHazardOutput o;
  o.hazard_id = staticHazardId(obs);
  o.position = obs.position;
  o.keep_clear_radius_m = obs.keep_clear_radius_m;
  o.footprint_radius_m = obs.footprint_radius_m;
  o.position_uncertainty_m = obs.position_uncertainty_m;
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
