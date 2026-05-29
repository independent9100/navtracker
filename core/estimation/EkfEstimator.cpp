#include "core/estimation/EkfEstimator.hpp"

#include <utility>

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

EkfEstimator::EkfEstimator(std::shared_ptr<const IMotionModel> motion)
    : motion_(std::move(motion)) {}

void EkfEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const Eigen::MatrixXd f = motion_->transitionMatrix(dt);
  const Eigen::MatrixXd q = motion_->processNoise(dt);
  track.state = f * track.state;
  track.covariance = f * track.covariance * f.transpose() + q;
  track.last_update = to;
}

void EkfEstimator::update(Track& track, const Measurement& z) const {
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state);
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd& h = pred.H;
  const Eigen::MatrixXd s = h * track.covariance * h.transpose() + z.covariance;
  const Eigen::MatrixXd k = track.covariance * h.transpose() * s.inverse();
  track.state += k * y;
  const auto n = track.state.size();
  const Eigen::MatrixXd id = Eigen::MatrixXd::Identity(n, n);
  track.covariance = (id - k * h) * track.covariance;
  track.last_update = z.time;
}

}  // namespace navtracker
