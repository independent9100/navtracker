#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

// 5-state coordinated-turn motion model with a turn rate fixed at
// construction. Unlike CoordinatedTurn (whose omega is read from the
// state each cycle), this model uses its constructor-prescribed omega
// for every transitionMatrix(dt) query, regardless of the state's omega
// component. State remains [px, py, vx, vy, omega] so this model lives
// in the same 5-d space as CV5State for unified IMM mixing.
//
// Use case: prescribed-rate three-mode IMM (CV + CT(+omega_hat) +
// CT(-omega_hat)). Each CT mode is committed to its own turn rate and
// doesn't have to "discover" omega from observations.
class PrescribedTurn : public IMotionModel {
 public:
  PrescribedTurn(double prescribed_omega, double accel_psd, double omega_psd);
  int stateDim() const override { return 5; }
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  Eigen::MatrixXd processNoise(double dt) const override;
  double omega() const { return omega_; }

 private:
  double omega_;
  double q_a_;
  double q_omega_;
};

}  // namespace navtracker
