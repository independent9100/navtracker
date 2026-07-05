#include "core/own_ship/NavInputGuard.hpp"

#include <cmath>

#include "core/own_ship/OwnShipProvider.hpp"  // OwnShipPose

namespace navtracker {
namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Fact-free equirectangular distance (m) between two geodetic points. Good
// enough for a jump sanity check; the guard does not need geodesic precision and
// must not depend on the (not-yet-inserted) working datum.
double approxMeters(double lat1, double lon1, double lat2, double lon2) {
  const double mean_lat = 0.5 * (lat1 + lat2) * kDeg2Rad;
  const double dnorth = (lat2 - lat1) * 111320.0;
  const double deast = (lon2 - lon1) * 111320.0 * std::cos(mean_lat);
  return std::sqrt(dnorth * dnorth + deast * deast);
}

// Smallest absolute difference between two headings (deg), wrapped to [0, 180].
double headingDeltaDeg(double a, double b) {
  double d = std::fmod(std::abs(a - b), 360.0);
  return d > 180.0 ? 360.0 - d : d;
}
}  // namespace

NavHealth evaluateNavInput(const std::optional<OwnShipPose>& prev,
                           const OwnShipPose& curr,
                           const NavInputGuardConfig& cfg) {
  NavHealth h;

  const bool have_prev = prev.has_value();
  const double dt =
      have_prev ? (curr.time.seconds() - prev->time.seconds()) : 0.0;

  bool have_step = false;
  double step_m = 0.0;
  if (have_prev) {
    step_m = approxMeters(prev->lat_deg, prev->lon_deg, curr.lat_deg,
                          curr.lon_deg);
    have_step = true;
    h.position_step_m = step_m;
    h.gap_s = dt;
  }

  // Speed over ground: the pose's own velocity when valid, else derived from the
  // position step. NaN ⇒ no SOG estimate ⇒ do not flag the heading.
  double sog = std::nan("");
  if (curr.velocity_is_valid) {
    sog = curr.velocity_enu.norm();
  } else if (have_step && dt > 0.0) {
    sog = step_m / dt;
  }
  if (!std::isnan(sog)) {
    h.sog_mps = sog;
    if (sog < cfg.heading_min_sog_mps) h.heading_unreliable_low_sog = true;
  }

  // Staleness: a long gap since the previous pose ⇒ the feed went quiet.
  if (have_prev && dt > cfg.stale_after_s) h.stale_gap = true;

  // Jumps need a positive timestep (skip out-of-order / duplicate timestamps).
  if (have_step && dt > 0.0) {
    if (step_m / dt > cfg.max_position_speed_mps) h.position_jump = true;
    const double dh = headingDeltaDeg(curr.heading_true_deg,
                                      prev->heading_true_deg);
    h.heading_step_deg = dh;
    if (dh / dt > cfg.max_heading_rate_dps) h.heading_jump = true;
  }

  return h;
}

}  // namespace navtracker
