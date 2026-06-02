#include "adapters/util/Projection.hpp"

#include <cmath>

namespace navtracker {

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
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
  return out;
}

}  // namespace navtracker
