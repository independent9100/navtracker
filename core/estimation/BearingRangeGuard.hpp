#pragma once

#include <Eigen/Core>

namespace navtracker {

// Bearing-update range-variance non-decrease guard. Classical BOT (bearings-
// only tracking) pathology: a Bearing2D measurement carries no information
// about range, so the predicted-to-posterior change in range-direction
// position variance should be ≥ 0 (variance non-decreasing along the line
// of sight). The EKF Joseph update does not enforce this — through the
// kinematic cross-correlations in P, a stream of bearings can drive the
// along-LOS variance below its predicted value, leaving the filter
// overconfident in a dimension it never measured.
//
// === Math ===
// Let x̂⁻ be the predicted state with position p̂⁻, sensor at s, and
// θ = atan2(p̂⁻.y − s.y, p̂⁻.x − s.x). Define the LOS unit vector
// n_LOS = (cos θ, sin θ) and its tangent n_⊥ = (−sin θ, cos θ).
// In LOS-aligned coordinates (R = [n_LOS, n_⊥]):
//     P̃_xy = Rᵀ P_xy R
// has [0,0] = range-direction variance, [1,1] = cross-range variance,
// [0,1] = correlation. The guard:
//   1. Compute var_los_pre  = n_LOSᵀ P⁻_xy n_LOS.
//   2. Compute var_los_post = n_LOSᵀ P⁺_xy n_LOS.
//   3. If var_los_post < var_los_pre, raise the LOS-direction component
//      of P⁺_xy back to var_los_pre while preserving the cross-LOS
//      reduction (legitimate information from the bearing) and the
//      LOS↔cross-LOS correlation. In LOS coords:
//        P̃⁺_xy[0,0] ← var_los_pre
//   4. Rotate back: P⁺_xy ← R P̃⁺ Rᵀ. Replace the position block of
//      the full state covariance; leave velocity / cross blocks alone.
//
// === Assumptions ===
//   - State layout begins [px, py, vx, vy, ...] in ENU metres (CV2D
//     and CV/CT IMM both satisfy this; matches the codebase invariant).
//   - covariance P is at least 2×2 (skipped otherwise as a defensive
//     no-op).
//   - The predicted state and sensor position determine the LOS used
//     for the rotation. For an IMM mode-conditioned EKF, pass the
//     per-mode predicted x_j (not the moment-matched projection).
//   - Conservative bias: the guard never *increases* variance below
//     the predicted, only restores it. Cross-LOS variance and
//     position-velocity cross blocks are unchanged.
//
// === Rationale ===
//   - Post-update guard (not a state-representation switch to modified
//     polar coordinates): MPC would require rewriting the IMM mode
//     state and every measurement model; far beyond this slice. The
//     post-update LOS clamp catches the same pathology at the symptom
//     surface with one extra rotation per bearing update.
//   - Position-block-only modification: the cross-coupling that
//     leaked range info also touched velocity, but the dominant
//     symptom on AutoFerry sc5 is position cov collapse (β̂ = 39.5
//     posterior; α̂_radar = 2.17, α̂_lidar = 1.73 per-NIS). Velocity
//     guarding is a follow-up if NEES stays hot after the position
//     fix lands.
//   - Symmetry on output: the rotation + clamp can introduce tiny
//     asymmetry from rounding; we re-symmetrize via (M + Mᵀ)/2.
//
// === Ways to improve / what to test next ===
//   - Velocity-block analogue: the LOS-direction *velocity* variance
//     should arguably also be non-decreasing under a bearing update
//     (range-rate is unobservable). Add only if data shows it's
//     needed.
//   - Sigma-point (UKF) bearing update: a true unscented transform
//     for the bearing nonlinearity would not need the guard at all
//     for moderate uncertainty, but at high cross-range / low
//     distance the EKF linearization dominates anyway.
//   - State-representation: modified-polar coordinates (Aidala 1979)
//     factor the unobservable range from the state by construction.
//     Right thing for a single-observer bearings-only deployment; we
//     have multi-sensor data so the LOS clamp is enough.
//
// Returns the guarded full covariance (input covariance unchanged
// when the guard is a no-op).
Eigen::MatrixXd applyBearingRangeGuard(
    const Eigen::MatrixXd& covariance_pre,
    const Eigen::MatrixXd& covariance_post,
    const Eigen::VectorXd& predicted_state,
    const Eigen::Vector2d& sensor_position_enu);

}  // namespace navtracker
