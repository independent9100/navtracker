#pragma once

#include <Eigen/Core>

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

}  // namespace navtracker
