#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

// 5-state coordinated-turn motion model. State: [px, py, vx, vy, omega].
// Velocity is rotated by omega · dt; position advances by the integral of
// the rotated velocity. Omega is treated as a random walk.
//
// Nonlinear in omega: F depends on the omega component of x. The
// canonical primitive is `transitionMatrixAt(omega, dt)`. The
// IMotionModel-mandated `transitionMatrix(dt)` evaluates F at omega=0
// (i.e., the CV limit) — it is intended only for callers that need a
// rough linear surrogate; estimators that care about CT accuracy use
// `propagate(x, dt)` (true nonlinear step) and the IMM uses
// `transitionMatrixAt(omega_mixed, dt)` for moment-matched mixing.
//
// CV limit (|omega|<1e-6) is handled via the Taylor expansion
// (reduces to the linear CV form).
class CoordinatedTurn : public IMotionModel {
 public:
  CoordinatedTurn(double accel_psd, double omega_psd);
  int stateDim() const override { return 5; }

  // F(dt) at omega=0 (CV limit). Provided only to satisfy IMotionModel;
  // the EKF predict path uses propagate(x, dt) which reads omega from x.
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  Eigen::MatrixXd processNoise(double dt) const override;

  // Closed-form F(omega, dt). Use this when an explicit linearization
  // around a specific omega is needed (e.g., IMM moment-matched mixing).
  Eigen::MatrixXd transitionMatrixAt(double omega, double dt) const;

  // True nonlinear one-step prediction. Reads omega from x(4).
  Eigen::VectorXd propagate(const Eigen::VectorXd& x,
                            double dt) const override;

 private:
  double q_a_;
  double q_omega_;
};

}  // namespace navtracker
