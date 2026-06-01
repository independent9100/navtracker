#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

// 5-state coordinated-turn motion model. State: [px, py, vx, vy, omega].
// Velocity is rotated by omega · dt; position advances by the integral of
// the rotated velocity. Omega is treated as a random walk.
//
// IMPORTANT: F depends on the CURRENT omega estimate. The caller must hold
// the omega estimate in `omega_now` when querying F(dt); typically this is
// the mixed-prior omega from the IMM mixing step. CV limit (|omega|<1e-6)
// is handled via the Taylor expansion (i.e., reduces to the linear CV form).
class CoordinatedTurn : public IMotionModel {
 public:
  CoordinatedTurn(double accel_psd, double omega_psd);
  int stateDim() const override { return 5; }

  Eigen::MatrixXd transitionMatrix(double dt) const override;
  Eigen::MatrixXd processNoise(double dt) const override;

  // Mutable omega used to build F. Allows the IMM to evaluate F at the
  // mode's current omega without changing the IMotionModel interface.
  void setOmega(double omega) const { omega_ = omega; }
  double omega() const { return omega_; }

 private:
  double q_a_;
  double q_omega_;
  mutable double omega_{0.0};
};

}  // namespace navtracker
