#pragma once

#include <optional>

namespace navtracker {

struct OwnShipPose;  // core/own_ship/OwnShipProvider.hpp

/**
 * Fact-free own-ship nav-input sanity thresholds (backlog #18). These are
 * GENERIC plausibility bounds, not per-sensor-calibrated values — the tuned,
 * fact-dependent thresholds (exact heading source, GPS quality) stay on the
 * deployment shopping list. The guard FLAGS; it never rewrites a pose (validate
 * at the edge, invariant #6 — the core keeps trusting the poses it is given).
 */
struct NavInputGuardConfig {
  // Below this speed over ground (steerage way) a motion-derived heading is
  // unreliable — the own-ship twin of the COG-at-anchor pitfall. m/s.
  double heading_min_sog_mps{0.5};
  // A gap between consecutive poses longer than this means the nav feed went
  // quiet and projection was running on a stale fix. Seconds.
  double stale_after_s{3.0};
  // A position step faster than this is physically implausible for a vessel
  // (GPS glitch). m/s — default ≈ 100 kn.
  double max_position_speed_mps{50.0};
  // A heading step faster than this is an implausible yaw rate (gyro fault). °/s.
  double max_heading_rate_dps{60.0};
};

/**
 * Result of one nav-input check. Every bool is a "degrade visibly" flag; the
 * scalars carry the measured quantities for an operator/diagnostic message.
 */
struct NavHealth {
  bool heading_unreliable_low_sog{false};  // SOG below steerage way
  bool stale_gap{false};                    // gap since previous pose too long
  bool position_jump{false};                // implausible position step
  bool heading_jump{false};                 // implausible heading step
  double sog_mps{0.0};
  double gap_s{0.0};
  double position_step_m{0.0};
  double heading_step_deg{0.0};
  bool any() const {
    return heading_unreliable_low_sog || stale_gap || position_jump ||
           heading_jump;
  }
};

/**
 * Evaluate one incoming pose against the previous one (if any) and the generic
 * thresholds. Pure function — no state, no I/O; the OwnShipProvider drives it at
 * the edge. Two-pose checks (stale, jumps) are skipped when there is no previous
 * pose or the timestep is non-positive (out-of-order arrival). The low-SOG
 * heading flag needs an SOG estimate — the pose's own velocity when valid, else
 * derived from the position step; if neither is available it does not fire.
 */
NavHealth evaluateNavInput(const std::optional<OwnShipPose>& prev,
                           const OwnShipPose& curr,
                           const NavInputGuardConfig& cfg);

}  // namespace navtracker
