#pragma once

#include <Eigen/Core>

namespace navtracker {

/**
 * Strategy for robustifying a measurement update against outliers
 * (heavy-tailed measurement noise, sensor clutter that slipped the gate).
 *
 * Given the innovation y = z - h(x) and its nominal innovation covariance
 * S = H P Hᵀ + R, the model returns a multiplicative scale `s ≥ 1` applied
 * to the measurement-noise covariance R (equivalently inflating S). A
 * scale > 1 shrinks the Kalman gain, so a measurement that is improbable
 * under the nominal Gaussian model pulls the state less. The Gaussian
 * model returns 1 always (no change); a Student-t model inflates for
 * large normalised innovations and leaves inliers untouched.
 *
 * This is a port: the concrete model is injected into an estimator so the
 * robustification strategy is swappable without touching the filter math.
 */
class IMeasurementNoiseModel {
 public:
  virtual ~IMeasurementNoiseModel() = default;

  /**
   * Multiplicative scale on R for this measurement. Must be ≥ 1 and
   * exactly 1 for an inlier under the nominal model.
   */
  virtual double covarianceScale(const Eigen::VectorXd& innovation,
                                 const Eigen::MatrixXd& S) const = 0;
};

}  // namespace navtracker
