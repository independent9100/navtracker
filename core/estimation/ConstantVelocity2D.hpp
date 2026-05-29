#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

// 2D constant-velocity model. State = [px, py, vx, vy] in ENU (m, m/s).
// Process noise from the continuous white-noise-acceleration model with
// scalar acceleration PSD q, applied independently per axis.
class ConstantVelocity2D : public IMotionModel {
 public:
  explicit ConstantVelocity2D(double accel_psd);

  int stateDim() const override { return 4; }
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  Eigen::MatrixXd processNoise(double dt) const override;

 private:
  double q_;
};

}  // namespace navtracker
