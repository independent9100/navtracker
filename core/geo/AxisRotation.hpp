#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"

namespace navtracker::geo {

/**
 * 2x2 rotation matrix that re-expresses a vector given in `old_datum`'s ENU
 * axes into `new_datum`'s ENU axes:  v_new = R · v_old.
 *
 * Convention (fixed in W2.3): R is a rotation by −γ, where
 *   γ = wrap(Δlon) · sin(mean_lat)      (meridian convergence angle)
 * and Δlon = new.lon − old.lon wrapped into [−180°, 180°] (W2.2, antimeridian-
 * safe). The −γ sign is the E,N block of the exact frame map R_new · R_oldᵀ
 * (the Datum ecef→enu rotations); a +γ here rotates the WRONG way and is worse
 * than applying no rotation. At the equator γ = 0. At 60°N over 1° of longitude
 * |γ| ≈ 0.0151 rad, and an east-pointing (1,0) vector maps to (cos γ, −sin γ).
 *
 * Used by:
 *   - core/tracking/DatumShift: re-express track state across a datum recenter.
 *   - core/t2t/T2tFuser: re-express cached source-track ENU state on recenter.
 *   - core/output/TrackOutput: rotate position covariance into the
 *     target's local frame (Option A of the output-interface spec).
 */
Eigen::Matrix2d datumAxisRotation(const Datum& old_datum,
                                  const Datum& new_datum);

}  // namespace navtracker::geo
