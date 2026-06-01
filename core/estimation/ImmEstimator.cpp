#include "core/estimation/ImmEstimator.hpp"

#include <cmath>
#include <utility>

#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

ImmEstimator::ImmEstimator(std::vector<std::shared_ptr<IMotionModel>> motions,
                           Eigen::MatrixXd transition_matrix,
                           Eigen::VectorXd initial_mode_probabilities,
                           double init_speed_std,
                           double init_omega_std)
    : motions_(std::move(motions)),
      pi_(std::move(transition_matrix)),
      mu0_(std::move(initial_mode_probabilities)),
      init_speed_std_(init_speed_std),
      init_omega_std_(init_omega_std) {}

void ImmEstimator::projectMixtureToTrack(Track& track) const {
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < K; ++j)
    x += track.imm_mode_probabilities(j) * track.imm_means.col(j);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd d = track.imm_means.col(j) - x;
    P += track.imm_mode_probabilities(j) *
         (track.imm_covariances[j] + d * d.transpose());
  }
  track.state = x;
  track.covariance = P;
}

void ImmEstimator::predict(Track& track, Timestamp to) const {
  if (track.imm_means.cols() == 0) return;
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());

  // Mixing step.
  Eigen::VectorXd c(K);
  for (int j = 0; j < K; ++j) {
    double sum = 0.0;
    for (int i = 0; i < K; ++i)
      sum += pi_(i, j) * track.imm_mode_probabilities(i);
    c(j) = sum;
  }
  Eigen::MatrixXd mu_ij(K, K);
  for (int j = 0; j < K; ++j) {
    if (c(j) <= 0.0) {
      mu_ij.col(j).setZero();
      mu_ij(j, j) = 1.0;
    } else {
      for (int i = 0; i < K; ++i)
        mu_ij(i, j) = pi_(i, j) * track.imm_mode_probabilities(i) / c(j);
    }
  }

  // Mixed initial states per mode j.
  Eigen::MatrixXd x_mix(n, K);
  std::vector<Eigen::MatrixXd> P_mix(K, Eigen::MatrixXd::Zero(n, n));
  for (int j = 0; j < K; ++j) {
    Eigen::VectorXd xj = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < K; ++i)
      xj += mu_ij(i, j) * track.imm_means.col(i);
    x_mix.col(j) = xj;
    for (int i = 0; i < K; ++i) {
      const Eigen::VectorXd d = track.imm_means.col(i) - xj;
      P_mix[j] += mu_ij(i, j) *
                  (track.imm_covariances[i] + d * d.transpose());
    }
  }

  // Per-mode prediction.
  for (int j = 0; j < K; ++j) {
    if (auto* ct = dynamic_cast<CoordinatedTurn*>(motions_[j].get())) {
      ct->setOmega(x_mix(4, j));
    }
    const Eigen::MatrixXd F = motions_[j]->transitionMatrix(dt);
    const Eigen::MatrixXd Q = motions_[j]->processNoise(dt);
    track.imm_means.col(j) = F * x_mix.col(j);
    track.imm_covariances[j] = F * P_mix[j] * F.transpose() + Q;
  }

  projectMixtureToTrack(track);
  track.last_update = to;
}

void ImmEstimator::update(Track& /*track*/,
                          const Measurement& /*z*/) const {
  // Implemented in Task 7.
}

Track ImmEstimator::initiate(const Measurement& z) const {
  const int K = static_cast<int>(motions_.size());
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(5);
  x(0) = z.value(0);
  x(1) = z.value(1);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, 5);
  P(0, 0) = z.covariance(0, 0);
  P(0, 1) = z.covariance(0, 1);
  P(1, 0) = z.covariance(1, 0);
  P(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  P(2, 2) = vv;
  P(3, 3) = vv;
  P(4, 4) = init_omega_std_ * init_omega_std_;

  t.imm_means = Eigen::MatrixXd(5, K);
  t.imm_covariances.reserve(K);
  for (int j = 0; j < K; ++j) {
    t.imm_means.col(j) = x;
    t.imm_covariances.push_back(P);
  }
  t.imm_mode_probabilities = mu0_;

  projectMixtureToTrack(t);

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
