#pragma once

#include <Eigen/Core>

namespace navtracker {

/**
 * The deterministic sample set the unscented transform propagates through a
 * nonlinear function: the sigma points plus their mean (Wm) and covariance
 * (Wc) reconstruction weights. Produced by `computeSigmaPoints` and consumed
 * by UkfEstimator's predict/update.
 */
struct SigmaPoints {
  Eigen::MatrixXd points;  // n x (2n+1); column i is sigma point i
  Eigen::VectorXd Wm;      // size 2n+1 — mean weights
  Eigen::VectorXd Wc;      // size 2n+1 — covariance weights
};

/** Standard scaled UKF sigma points. lambda = alpha^2 (n+kappa) - n. */
SigmaPoints computeSigmaPoints(const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov,
                               double alpha,
                               double beta,
                               double kappa);

}  // namespace navtracker
