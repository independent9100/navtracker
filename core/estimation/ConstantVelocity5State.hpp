#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

/**
 * 5-state constant-velocity motion model with passive turn rate.
 * State: [px, py, vx, vy, omega]. Position advances with velocity; velocity
 * and omega are pure random walks (omega included only so this model lives
 * in the same state space as CoordinatedTurn for unified IMM).
 */
class ConstantVelocity5State : public IMotionModel {
 public:
  /** Construct with acceleration PSD `accel_psd` and turn-rate PSD `omega_psd`. */
  ConstantVelocity5State(double accel_psd, double omega_psd);
  /** State dimension: 5 ([px, py, vx, vy, omega]). */
  int stateDim() const override { return 5; }
  /** Linear transition matrix F(dt); omega propagates as a random walk. */
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  /** Discrete process-noise covariance Q(dt) over the 5-state layout. */
  Eigen::MatrixXd processNoise(double dt) const override;

 private:
  double q_a_;
  double q_omega_;
};

}  // namespace navtracker
