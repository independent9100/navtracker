#pragma once

#include <Eigen/Core>

#include "core/types/Measurement.hpp"

namespace navtracker {

/** A 2D ENU point with its 2×2 position covariance (m²). */
struct PointAndCov2D {
  Eigen::Vector2d pos_enu;
  Eigen::Matrix2d cov;
};

/**
 * Project a range/bearing sensor return to an absolute ENU point with a full
 * covariance. The point is own_ship_pos_enu offset by (range·sinθ, range·cosθ);
 * the covariance is the polar→Cartesian Jacobian applied to the sensor's range
 * and angular variances, where the angular variance sums the sensor's intrinsic
 * bearing variance and the own-ship heading variance in quadrature (both rotate
 * the line of sight), plus an isotropic sigma_gps_pos_m² translation term from
 * own-ship GPS position uncertainty.
 */
PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       double sigma_gps_pos_m,
                                       const Eigen::Vector2d& own_ship_pos_enu);

/**
 * Convert a `MeasurementModel::RangeBearing2D` measurement (range + bearing +
 * full 2×2 polar covariance) to an absolute ENU point with its ENU covariance,
 * for track INITIATION (W4.1). Uses the MATH bearing convention of
 * `core/estimation/MeasurementModels.cpp` — β is measured from the sensor as
 * `atan2(north_offset, east_offset)`: zero reference is due EAST of the sensor,
 * increasing counter-clockwise (+π/2 = due North), relative to the sensor's ENU
 * position. This is the SAME convention the RangeBearing2D update /
 * `predictMeasurement` path uses, so a track born here moves consistently under
 * subsequent range/bearing updates. (This deliberately differs from
 * `projectRangeBearingToEnu` above, which uses the marine north=0 / clockwise
 * convention and separate scalar σ's — that one serves the Position2D adapters.)
 *
 *   east  = sensor.x + range·cos β
 *   north = sensor.y + range·sin β
 *   J = ∂(e,n)/∂(range,β) = [[cos β, −range·sin β], [sin β, range·cos β]]
 *   cov_enu = J · polar_cov · Jᵀ           (polar_cov in (range, bearing) order)
 */
PointAndCov2D enuFromRangeBearing(double range_m, double bearing_rad,
                                  const Eigen::Vector2d& sensor_pos_enu,
                                  const Eigen::Matrix2d& polar_cov);

/**
 * The ENU birth position + 2×2 covariance for INITIATING a track from
 * measurement `z` (W4.1). ONE shared dispatch called by every estimator's
 * initiate():
 *   - RangeBearing2D → enuFromRangeBearing (polar → ENU + Jacobian covariance),
 *     centred on z.sensor_position_enu;
 *   - all other models already carry an ENU point in value[0..1] with an ENU
 *     position covariance in the top-left 2×2 → passed through unchanged.
 * Assumes z.value has ≥2 entries and z.covariance is ≥2×2. This is guaranteed
 * at every birth site by `canInitiateTrack(z.model)` — the ONLY <2-dim model,
 * Bearing2D, is not birth-eligible and is rejected there (the PSD check alone
 * does NOT enforce the dimension: a positive 1×1 covariance passes it). Callers
 * must keep the `canInitiateTrack` gate; do not call this for a Bearing2D `z`.
 */
PointAndCov2D initiationPosCov(const Measurement& z);

}  // namespace navtracker
