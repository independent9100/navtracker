#include "ports/IEstimator.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "core/estimation/GaussianScore.hpp"
#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

// Default `gate` and `logLikelihood` impls for single-Gaussian
// estimators (EKF, UKF, PF, and anything else that maintains a single
// kinematic Gaussian in track.state / track.covariance).
//
// For IMM the moment-matched projection over-states the spread (it
// folds inter-mode mean separation into the covariance), which makes
// the gate too loose and the score too low when modes disagree.
// ImmEstimator overrides both methods to compute per-mode quantities.

bool IEstimator::gate(const Track& track,
                      const Measurement& z,
                      double gate_threshold) const {
  // Defensive guard (Phase 8 iter 4 R3-strengthen): NaN/non-PSD R
  // would produce a NaN Mahalanobis and a non-deterministic gate
  // outcome. Reject the measurement (gate fails closed).
  if (!isMeasurementCovariancePsd(z.covariance)) return false;
  const MeasurementPrediction pred = predictMeasurement(
      z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y =
      measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd S =
      pred.H * track.covariance * pred.H.transpose() + z.covariance;
  const double d2 = y.transpose() * S.inverse() * y;
  return std::isfinite(d2) && d2 <= gate_threshold;
}

double IEstimator::logLikelihood(const Track& track,
                                 const Measurement& z) const {
  // Defensive guard (Phase 8 iter 4 R3-strengthen): non-finite R
  // would produce a NaN log-likelihood that propagates into the
  // associator cost matrix. Return −inf instead, which the
  // associator treats as "infeasible cell".
  if (!isMeasurementCovariancePsd(z.covariance)) {
    return -std::numeric_limits<double>::infinity();
  }
  const MeasurementPrediction pred = predictMeasurement(
      z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y =
      measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd S =
      pred.H * track.covariance * pred.H.transpose() + z.covariance;
  // One decomposition (not S.determinant() + S.inverse()): mahalanobis via a
  // solve, log|S| from the same factorization. Guard semantics unchanged.
  const GaussianScore s = gaussianScore(y, S);
  const int d = static_cast<int>(z.value.size());
  return -0.5 * static_cast<double>(d) * std::log(2.0 * M_PI) -
         0.5 * s.log_det_safe - 0.5 * s.mahalanobis;
}

}  // namespace navtracker
