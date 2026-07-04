#pragma once

#include <Eigen/Core>

namespace navtracker {

/**
 * Motion model for a fixed state layout. Linear models supply F(dt) and
 * Q(dt); models that are nonlinear in the state (e.g. coordinated-turn
 * with ω read from the state) override `propagate(x, dt)` to apply the
 * true nonlinear transition per state — this is what the UKF needs in
 * order to extract any benefit over EKF.
 *
 * Default `propagate(x, dt)` is `F(dt) · x`, which is correct for any
 * linear model and a sane fallback elsewhere.
 */
class IMotionModel {
 public:
  virtual ~IMotionModel() = default;
  /** Dimension of the state vector this model propagates. */
  virtual int stateDim() const = 0;
  virtual Eigen::MatrixXd transitionMatrix(double dt) const = 0;  // F(dt)
  virtual Eigen::MatrixXd processNoise(double dt) const = 0;      // Q(dt)

  /**
   * Nonlinear one-step prediction x_{k+1} = f(x_k, dt). Default
   * implementation is the linear F(dt) · x. Override for genuinely
   * nonlinear models (CT with state-driven ω).
   */
  virtual Eigen::VectorXd propagate(const Eigen::VectorXd& x,
                                    double dt) const {
    return transitionMatrix(dt) * x;
  }
};

}  // namespace navtracker
