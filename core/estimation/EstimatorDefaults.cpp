#include "ports/IEstimator.hpp"

#include <cmath>

#include <Eigen/Dense>

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
  const MeasurementPrediction pred = predictMeasurement(
      z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y =
      measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd S =
      pred.H * track.covariance * pred.H.transpose() + z.covariance;
  const double d2 = y.transpose() * S.inverse() * y;
  return d2 <= gate_threshold;
}

double IEstimator::logLikelihood(const Track& track,
                                 const Measurement& z) const {
  const MeasurementPrediction pred = predictMeasurement(
      z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y =
      measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd S =
      pred.H * track.covariance * pred.H.transpose() + z.covariance;
  const double det = S.determinant();
  const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
  const int d = static_cast<int>(z.value.size());
  const double mahal = y.transpose() * S.inverse() * y;
  return -0.5 * static_cast<double>(d) * std::log(2.0 * M_PI) -
         0.5 * std::log(safe_det) - 0.5 * mahal;
}

}  // namespace navtracker
