#include "core/estimation/ParticleFilterEstimator.hpp"

#include <cmath>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"
#include "core/estimation/Resampling.hpp"

namespace navtracker {

ParticleFilterEstimator::ParticleFilterEstimator(
    std::shared_ptr<const IMotionModel> motion,
    int particle_count,
    double init_speed_std,
    double ess_fraction_threshold,
    std::uint64_t seed)
    : motion_(std::move(motion)),
      particle_count_(particle_count),
      init_speed_std_(init_speed_std),
      ess_threshold_(ess_fraction_threshold * particle_count),
      rng_(seed) {}

void ParticleFilterEstimator::projectToGaussian(Track& track) const {
  const int n = static_cast<int>(track.particles.rows());
  const int N = static_cast<int>(track.particles.cols());
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < N; ++i)
    mean += track.particle_weights(i) * track.particles.col(i);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < N; ++i) {
    const Eigen::VectorXd d = track.particles.col(i) - mean;
    cov += track.particle_weights(i) * d * d.transpose();
  }
  track.state = mean;
  track.covariance = cov;
}

void ParticleFilterEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const int n = static_cast<int>(track.particles.rows());
  const int N = static_cast<int>(track.particles.cols());
  const Eigen::MatrixXd F = motion_->transitionMatrix(dt);
  const Eigen::MatrixXd Q = motion_->processNoise(dt);

  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(n, N);
  const Eigen::LLT<Eigen::MatrixXd> llt(Q);
  if (llt.info() == Eigen::Success) {
    const Eigen::MatrixXd L = llt.matrixL();
    std::normal_distribution<double> n01(0.0, 1.0);
    Eigen::MatrixXd eta(n, N);
    for (int j = 0; j < N; ++j)
      for (int i = 0; i < n; ++i) eta(i, j) = n01(rng_);
    noise = L * eta;
  }
  // If Q is not PD (e.g. q == 0 → singular), skip the noise term entirely;
  // the predict becomes deterministic F·x for every particle.

  track.particles = (F * track.particles) + noise;
  projectToGaussian(track);
  track.last_update = to;
}

void ParticleFilterEstimator::update(Track& /*track*/,
                                     const Measurement& /*z*/) const {
  // Implemented in Task 5.
}

Track ParticleFilterEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = z.value(0);
  x(1) = z.value(1);

  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = z.covariance(0, 0);
  P(0, 1) = z.covariance(0, 1);
  P(1, 0) = z.covariance(1, 0);
  P(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  P(2, 2) = vv;
  P(3, 3) = vv;

  const Eigen::LLT<Eigen::Matrix4d> llt(P);
  if (llt.info() != Eigen::Success) {
    // Init covariance was not positive-definite — return a Tentative track
    // with the Gaussian carrier only (no particle ensemble). Predict/update
    // will see empty particles and skip ensemble work; the next update may
    // re-initiate. This guards against malformed measurement covariance.
    t.state = x;
    t.covariance = P;
    if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
    t.contributing_sources.push_back(z.source_id);
    return t;
  }
  const Eigen::Matrix4d L = llt.matrixL();

  std::normal_distribution<double> n01(0.0, 1.0);
  Eigen::MatrixXd particles(4, particle_count_);
  for (int i = 0; i < particle_count_; ++i) {
    Eigen::Vector4d eta;
    for (int j = 0; j < 4; ++j) eta(j) = n01(rng_);
    particles.col(i) = x + L * eta;
  }
  t.particles = particles;
  t.particle_weights =
      Eigen::VectorXd::Constant(particle_count_, 1.0 / particle_count_);

  projectToGaussian(t);

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
