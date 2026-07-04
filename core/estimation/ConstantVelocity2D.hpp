#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

/**
 * 2D constant-velocity model. State = [px, py, vx, vy] in ENU (m, m/s).
 * Process noise from the continuous white-noise-acceleration model with
 * scalar acceleration PSD q, applied independently per axis.
 */
class ConstantVelocity2D : public IMotionModel {
 public:
  /** Construct with acceleration power spectral density `accel_psd` (q). */
  explicit ConstantVelocity2D(double accel_psd);

  /** State dimension: 4 ([px, py, vx, vy]). */
  int stateDim() const override { return 4; }
  /** Linear transition matrix F(dt) for the constant-velocity kinematics. */
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  /** Discrete process-noise covariance Q(dt) from the WNA model with PSD q. */
  Eigen::MatrixXd processNoise(double dt) const override;

 private:
  double q_;
};

}  // namespace navtracker
