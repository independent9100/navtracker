#pragma once

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Closest-point-of-approach between two constant-velocity tracks.
//
// `tcpa_seconds` is the time from `t_ref` until closest approach. It is
// **never negative**: if the closed-form solution places CPA in the past
// (tracks are now diverging), it is clamped to 0 and `cpa_distance_m`
// reports the *current* distance at t_ref (NOT the past minimum).
//
// `is_diverging` is true iff the unclamped closed-form t_cpa was strictly
// negative. Parallel motion (delta-v ~ 0) is reported as
// tcpa = 0, cpa_distance = current distance, is_diverging = false.
//
// Math
// ----
// Under CV assumption, positions at time t_ref + tau are:
//   p_a(tau) = pa + va * tau
//   p_b(tau) = pb + vb * tau
// where pa, pb are positions extrapolated to t_ref and va, vb are velocities.
//
// Squared separation: |dp + dv*tau|^2 where dp = pa - pb, dv = va - vb.
// Minimised by d/dtau = 0: tau* = -dp.dv / dv.dv
//
// Assumptions
// -----------
// - Both tracks are constant-velocity between last_update and t_cpa. The CV
//   velocity is taken directly from state(2..3); no model uncertainty is
//   propagated.
// - state.size() >= 4 for both tracks (px, py, vx, vy in ENU metres/m/s).
// - t_ref >= last_update for both tracks (forward extrapolation only at
//   the call site; the function accepts any sign but negative dt is unusual).
//
// Rationale
// ---------
// Closed-form CV CPA is O(1), exact under the CV assumption, and universally
// used in collision-avoidance standards (ARPA, AIS TCPA). The only branching
// needed is the parallel-velocity singularity and the past-CPA clamp.
//
// Ways to improve / what to test next
// ------------------------------------
// - Extend to 3-D (include vertical velocity state(4)) for aerial/submarine
//   targets if a 5- or 6-state model is adopted.
// - For IMM tracks, weight t_cpa contributions from each mode by its
//   probability to obtain a probabilistic CPA distribution.
// - Uncertainty ellipse on CPA: propagate covariance through the CPA formula
//   to report a sigma bound on cpa_distance_m.
struct CpaResult {
  double tcpa_seconds;
  double cpa_distance_m;
  bool is_diverging;
};

// Both tracks are extrapolated linearly from their own last_update to
// t_ref using their CV velocity components state(2..3). Requires
// track.state.size() >= 4 for both tracks (px, py, vx, vy).
CpaResult computeCpa(const Track& a, const Track& b, Timestamp t_ref);

}  // namespace navtracker
