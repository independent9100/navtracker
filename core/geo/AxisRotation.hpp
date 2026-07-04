#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"

namespace navtracker::geo {

/**
 * 2x2 rotation matrix between the ENU axes of two datums.
 * Implements the convergence angle
 *   gamma = delta_lon_rad * sin(mean_lat_rad)
 * At equator: gamma = 0. At 60°N over 1° of longitude: gamma ≈ 0.0151 rad.
 * Used by:
 *   - core/tracking/DatumShift: shift tracks across a datum recenter.
 *   - core/output/TrackOutput: rotate position covariance into the
 *     target's local NED frame (Option A of the output-interface spec).
 */
Eigen::Matrix2d datumAxisRotation(const Datum& old_datum,
                                  const Datum& new_datum);

}  // namespace navtracker::geo
