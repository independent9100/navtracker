#include "core/estimation/EkfEstimator.hpp"

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

EkfEstimator::EkfEstimator(std::shared_ptr<const IMotionModel> motion,
                           double init_speed_std)
    : motion_(std::move(motion)), init_speed_std_(init_speed_std) {}

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
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state, z.sensor_position_enu);
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

Track EkfEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = z.value(0);
  x(1) = z.value(1);
  t.state = x;

  Eigen::Matrix4d p = Eigen::Matrix4d::Zero();
  p(0, 0) = z.covariance(0, 0);
  p(0, 1) = z.covariance(0, 1);
  p(1, 0) = z.covariance(1, 0);
  p(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  p(2, 2) = vv;
  p(3, 3) = vv;
  t.covariance = p;

  if (z.hints.mmsi.has_value()) {
    t.attributes.mmsi = z.hints.mmsi;
  }
  t.contributing_sources.push_back(z.source_id);
  return t;
}

void EkfEstimator::softUpdate(Track& track,
                              const std::vector<Measurement>& gated_measurements,
                              const Eigen::VectorXd& betas,
                              double beta_0) const {
  const int M = static_cast<int>(gated_measurements.size());
  if (M == 0 || betas.size() != M) return;
  const Measurement& z0 = gated_measurements[0];
  const MeasurementPrediction pred = predictMeasurement(z0.model, track.state, z0.sensor_position_enu);
  const Eigen::MatrixXd& H = pred.H;
  const Eigen::MatrixXd S = H * track.covariance * H.transpose() + z0.covariance;
  const Eigen::MatrixXd S_inv = S.inverse();
  const Eigen::MatrixXd K = track.covariance * H.transpose() * S_inv;

  Eigen::VectorXd y_combined = Eigen::VectorXd::Zero(z0.value.size());
  Eigen::MatrixXd spread_sum =
      Eigen::MatrixXd::Zero(z0.value.size(), z0.value.size());
  for (int j = 0; j < M; ++j) {
    const Eigen::VectorXd y_j =
        measurementResidual(z0.model, gated_measurements[j].value, pred.z_pred);
    y_combined += betas(j) * y_j;
    spread_sum += betas(j) * y_j * y_j.transpose();
  }
  spread_sum -= y_combined * y_combined.transpose();

  const Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(track.state.size(), track.state.size());
  const Eigen::MatrixXd P_post_full = (I - K * H) * track.covariance;
  track.state += K * y_combined;
  track.covariance = beta_0 * track.covariance +
                     (1.0 - beta_0) * P_post_full +
                     K * spread_sum * K.transpose();
  track.last_update = z0.time;
}

}  // namespace navtracker
