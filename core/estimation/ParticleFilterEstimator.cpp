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
    std::uint64_t seed,
    double init_omega_std)
    : motion_(std::move(motion)),
      particle_count_(particle_count),
      init_speed_std_(init_speed_std),
      ess_threshold_(ess_fraction_threshold * particle_count),
      init_omega_std_(init_omega_std),
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
  if (track.particles.cols() == 0) return;
  const int n = static_cast<int>(track.particles.rows());
  const int N = static_cast<int>(track.particles.cols());
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
  // the predict becomes deterministic f(x) for every particle.

  // Apply the model's per-particle nonlinear transition. For a linear
  // model `propagate` reduces to F(dt)·x; for a coordinated-turn model it
  // turns each particle by its own state-driven ω, which the previous
  // linear F-matrix path silently dropped (collapsing the PF to CV).
  Eigen::MatrixXd propagated(n, N);
  for (int j = 0; j < N; ++j)
    propagated.col(j) = motion_->propagate(track.particles.col(j), dt);
  track.particles = propagated + noise;
  projectToGaussian(track);
  track.last_update = to;
}

void ParticleFilterEstimator::update(Track& track, const Measurement& z) const {
  if (track.particles.cols() == 0) return;
  const int N = static_cast<int>(track.particles.cols());
  const Eigen::MatrixXd Rinv = z.covariance.inverse();

  Eigen::VectorXd log_w(N);
  for (int i = 0; i < N; ++i) log_w(i) = std::log(track.particle_weights(i));

  for (int i = 0; i < N; ++i) {
    const Eigen::VectorXd z_pred =
        predictMeasurementValue(z.model, track.particles.col(i), z.sensor_position_enu);
    const Eigen::VectorXd y = measurementResidual(z.model, z.value, z_pred);
    log_w(i) += -0.5 * y.transpose() * Rinv * y;
  }

  const double max_lw = log_w.maxCoeff();
  Eigen::VectorXd w = (log_w.array() - max_lw).exp();
  const double sum = w.sum();
  if (!std::isfinite(sum) || sum <= 0.0) {
    // All particles deemed impossible: reset to uniform — degenerate case.
    w = Eigen::VectorXd::Constant(N, 1.0 / N);
  } else {
    w /= sum;
  }
  track.particle_weights = w;

  if (effectiveSampleSize(w) < ess_threshold_) {
    std::uniform_real_distribution<double> u01(
        0.0, 1.0 / static_cast<double>(N));
    const double u = u01(rng_);
    const std::vector<int> idx = systematicResample(w, u);
    Eigen::MatrixXd resampled(track.particles.rows(), N);
    for (int i = 0; i < N; ++i) resampled.col(i) = track.particles.col(idx[i]);
    track.particles = resampled;
    track.particle_weights = Eigen::VectorXd::Constant(N, 1.0 / N);
  }

  projectToGaussian(track);
  track.last_update = z.time;
}

Track ParticleFilterEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  // Size the ensemble from the motion model: CV2D → 4-state, CT / CV5State
  // → 5-state with ω as the trailing entry. Mirrors UkfEstimator::initiate
  // so a PF can be dropped into any IMM-eligible model slot without a
  // dimension-mismatch crash.
  const int n = motion_->stateDim();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  x(0) = z.value(0);
  x(1) = z.value(1);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
  P(0, 0) = z.covariance(0, 0);
  P(0, 1) = z.covariance(0, 1);
  P(1, 0) = z.covariance(1, 0);
  P(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  if (n >= 4) {
    P(2, 2) = vv;
    P(3, 3) = vv;
  }
  if (n >= 5) {
    P(4, 4) = init_omega_std_ * init_omega_std_;
  }

  const Eigen::LLT<Eigen::MatrixXd> llt(P);
  if (llt.info() != Eigen::Success) {
    // Init covariance was not positive-definite — return a Tentative track
    // with the Gaussian carrier only (no particle ensemble). Predict/update
    // will see empty particles and skip ensemble work; the next update may
    // re-initiate. This guards against malformed measurement covariance.
    t.state = x;
    t.covariance = P;
    if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
    if (z.hints.platform_id.has_value())
      t.attributes.platform_id = z.hints.platform_id;
    t.contributing_sources.push_back(z.source_id);
    return t;
  }
  const Eigen::MatrixXd L = llt.matrixL();

  std::normal_distribution<double> n01(0.0, 1.0);
  Eigen::MatrixXd particles(n, particle_count_);
  for (int i = 0; i < particle_count_; ++i) {
    Eigen::VectorXd eta(n);
    for (int j = 0; j < n; ++j) eta(j) = n01(rng_);
    particles.col(i) = x + L * eta;
  }
  t.particles = particles;
  t.particle_weights =
      Eigen::VectorXd::Constant(particle_count_, 1.0 / particle_count_);

  projectToGaussian(t);

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  if (z.hints.platform_id.has_value())
    t.attributes.platform_id = z.hints.platform_id;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
