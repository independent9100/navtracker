#include "core/estimation/ImmEstimator.hpp"

#include <cmath>
#include <utility>

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

void ImmEstimator::predict(Track& /*track*/, Timestamp /*to*/) const {
  // Implemented in Task 6.
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
