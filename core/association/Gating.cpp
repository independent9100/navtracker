#include "core/association/Gating.hpp"

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

double mahalanobisDistance(const Track& track, const Measurement& z) {
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd& h = pred.H;
  const Eigen::MatrixXd s = h * track.covariance * h.transpose() + z.covariance;
  return y.dot(s.inverse() * y);
}

}  // namespace navtracker
