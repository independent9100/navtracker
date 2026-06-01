#include "core/collision/Cpa.hpp"

namespace navtracker {

CpaResult computeCpa(const Track& a, const Track& b, Timestamp t_ref) {
  // Extrapolate each track from its own last_update to t_ref using
  // state(2..3) as the CV velocity.
  const double dt_a = t_ref.secondsSince(a.last_update);
  const double dt_b = t_ref.secondsSince(b.last_update);
  const Eigen::Vector2d pa(a.state(0) + a.state(2) * dt_a,
                           a.state(1) + a.state(3) * dt_a);
  const Eigen::Vector2d pb(b.state(0) + b.state(2) * dt_b,
                           b.state(1) + b.state(3) * dt_b);
  const Eigen::Vector2d va(a.state(2), a.state(3));
  const Eigen::Vector2d vb(b.state(2), b.state(3));
  const Eigen::Vector2d dp = pa - pb;
  const Eigen::Vector2d dv = va - vb;

  CpaResult r;
  const double dv2 = dv.dot(dv);
  if (dv2 < 1e-12) {
    // Parallel velocities -> constant separation. Not diverging.
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = dp.norm();
    r.is_diverging = false;
    return r;
  }
  const double t_cpa_raw = -dp.dot(dv) / dv2;
  if (t_cpa_raw <= 0.0) {
    // Closest approach is in the past (or exactly now). Report the
    // CURRENT distance, not the past minimum.
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = dp.norm();
    r.is_diverging = (t_cpa_raw < 0.0);
    return r;
  }
  r.tcpa_seconds = t_cpa_raw;
  const Eigen::Vector2d dp_at_cpa = dp + dv * t_cpa_raw;
  r.cpa_distance_m = dp_at_cpa.norm();
  r.is_diverging = false;
  return r;
}

}  // namespace navtracker
