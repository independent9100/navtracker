#pragma once

#include <Eigen/Core>

namespace navtracker {

struct PointAndCov2D {
  Eigen::Vector2d pos_enu;
  Eigen::Matrix2d cov;
};

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       const Eigen::Vector2d& own_ship_pos_enu);

}  // namespace navtracker
