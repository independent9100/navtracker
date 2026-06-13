#include "core/estimation/BearingRangeGuard.hpp"

#include <cmath>

namespace navtracker {

Eigen::MatrixXd applyBearingRangeGuard(
    const Eigen::MatrixXd& covariance_pre,
    const Eigen::MatrixXd& covariance_post,
    const Eigen::VectorXd& predicted_state,
    const Eigen::Vector2d& sensor_position_enu) {
  if (covariance_post.rows() < 2 || covariance_post.cols() < 2) {
    return covariance_post;
  }
  if (predicted_state.size() < 2) return covariance_post;
  if (covariance_pre.rows() < 2 || covariance_pre.cols() < 2) {
    return covariance_post;
  }
  const double dx = predicted_state(0) - sensor_position_enu.x();
  const double dy = predicted_state(1) - sensor_position_enu.y();
  const double r2 = dx * dx + dy * dy;
  if (r2 < 1e-12) return covariance_post;
  const double inv_r = 1.0 / std::sqrt(r2);
  const Eigen::Vector2d n_los(dx * inv_r, dy * inv_r);

  const Eigen::Matrix2d P_xy_pre = covariance_pre.topLeftCorner<2, 2>();
  const Eigen::Matrix2d P_xy_post = covariance_post.topLeftCorner<2, 2>();
  const double var_los_pre = n_los.dot(P_xy_pre * n_los);
  const double var_los_post = n_los.dot(P_xy_post * n_los);
  if (var_los_post >= var_los_pre) return covariance_post;  // no-op

  const Eigen::Vector2d n_perp(-n_los.y(), n_los.x());
  Eigen::Matrix2d R;
  R.col(0) = n_los;
  R.col(1) = n_perp;
  Eigen::Matrix2d P_tilde = R.transpose() * P_xy_post * R;
  // Restore the predicted LOS-direction variance; leave cross-range
  // variance and LOS↔cross correlation as the bearing update set them.
  P_tilde(0, 0) = var_los_pre;
  Eigen::Matrix2d P_xy_guarded = R * P_tilde * R.transpose();
  // Re-symmetrize: rotation + clamp can introduce O(1e-15) asymmetry
  // from floating-point rounding that downstream Joseph/Cholesky
  // chains would amplify.
  P_xy_guarded = 0.5 * (P_xy_guarded + P_xy_guarded.transpose());

  Eigen::MatrixXd out = covariance_post;
  out.topLeftCorner<2, 2>() = P_xy_guarded;
  return out;
}

}  // namespace navtracker
