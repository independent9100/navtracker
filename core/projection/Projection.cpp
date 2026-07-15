#include "core/projection/Projection.hpp"

#include <cmath>

namespace navtracker {

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       double sigma_gps_pos_m,
                                       const Eigen::Vector2d& own_ship_pos_enu) {
  const double sb = std::sin(bearing_true_rad);
  const double cb = std::cos(bearing_true_rad);

  PointAndCov2D out;
  out.pos_enu.x() = own_ship_pos_enu.x() + range_m * sb;
  out.pos_enu.y() = own_ship_pos_enu.y() + range_m * cb;

  Eigen::Matrix2d J;
  J << sb,  range_m * cb,
       cb, -range_m * sb;

  // Heading uncertainty adds to the angular component in quadrature: the
  // total angular variance is the sensor's intrinsic bearing variance plus
  // the own-ship heading variance, since both rotate the line of sight.
  const double angular_var =
      bearing_std_rad * bearing_std_rad +
      sigma_heading_rad * sigma_heading_rad;
  Eigen::Matrix2d R;
  R << range_std_m * range_std_m, 0.0,
       0.0, angular_var;
  out.cov = J * R * J.transpose();

  // Own-ship GPS position uncertainty translates the projected target by
  // the same (isotropic) shift, so it adds sigma_gps^2 * I to the output
  // covariance.
  out.cov(0, 0) += sigma_gps_pos_m * sigma_gps_pos_m;
  out.cov(1, 1) += sigma_gps_pos_m * sigma_gps_pos_m;
  return out;
}

PointAndCov2D enuFromRangeBearing(double range_m, double bearing_rad,
                                  const Eigen::Vector2d& sensor_pos_enu,
                                  const Eigen::Matrix2d& polar_cov) {
  // Math convention (matches MeasurementModels.cpp): β = atan2(north, east),
  // zero = East, CCW. So east = r·cos β, north = r·sin β.
  const double cb = std::cos(bearing_rad);
  const double sb = std::sin(bearing_rad);

  PointAndCov2D out;
  out.pos_enu.x() = sensor_pos_enu.x() + range_m * cb;
  out.pos_enu.y() = sensor_pos_enu.y() + range_m * sb;

  // J = ∂(east, north)/∂(range, bearing).
  Eigen::Matrix2d J;
  J << cb, -range_m * sb,
       sb,  range_m * cb;
  out.cov = J * polar_cov * J.transpose();
  return out;
}

PointAndCov2D initiationPosCov(const Measurement& z) {
  const Eigen::Matrix2d cov2 = z.covariance.topLeftCorner<2, 2>();
  if (z.model == MeasurementModel::RangeBearing2D) {
    return enuFromRangeBearing(z.value(0), z.value(1), z.sensor_position_enu,
                               cov2);
  }
  // Position2D / PositionVelocity2D: value[0..1] is already the ENU point and
  // the top-left 2×2 is its ENU position covariance.
  PointAndCov2D out;
  out.pos_enu = Eigen::Vector2d(z.value(0), z.value(1));
  out.cov = cov2;
  return out;
}

}  // namespace navtracker
