#include "core/estimation/ImmEstimator.hpp"

#include <cassert>
#include <cmath>
#include <utility>

#include <Eigen/LU>

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

  // Per-mode prediction. For nonlinear motion models (CT with state-
  // driven omega) we linearize F at the mode's mixed-prior omega for the
  // covariance update, then apply the true nonlinear step to the mean.
  // Linear models hit the default propagate() == F·x; the two paths
  // coincide.
  for (int j = 0; j < K; ++j) {
    Eigen::MatrixXd F;
    if (auto* ct = dynamic_cast<CoordinatedTurn*>(motions_[j].get())) {
      F = ct->transitionMatrixAt(x_mix(4, j), dt);
    } else {
      F = motions_[j]->transitionMatrix(dt);
    }
    const Eigen::MatrixXd Q = motions_[j]->processNoise(dt);
    track.imm_means.col(j) = motions_[j]->propagate(x_mix.col(j), dt);
    track.imm_covariances[j] = F * P_mix[j] * F.transpose() + Q;
  }

  projectMixtureToTrack(track);
  track.last_update = to;
}

void ImmEstimator::update(Track& track, const Measurement& z) const {
  if (track.imm_means.cols() == 0) return;
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());

  // c_j: mode-prior at update time. μ here is the previous-cycle posterior
  // since predict does not modify it.
  Eigen::VectorXd c(K);
  for (int j = 0; j < K; ++j) {
    double sum = 0.0;
    for (int i = 0; i < K; ++i)
      sum += pi_(i, j) * track.imm_mode_probabilities(i);
    c(j) = sum;
  }

  // Per-mode EKF update + log-likelihood. Copies of (x_j, P_j) so we
  // read the prior while writing the posterior back to the same slot.
  Eigen::VectorXd log_lambda(K);
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd x_j = track.imm_means.col(j);
    const Eigen::MatrixXd P_j = track.imm_covariances[j];
    const MeasurementPrediction pred = predictMeasurement(z.model, x_j, z.sensor_position_enu);
    const Eigen::VectorXd y =
        measurementResidual(z.model, z.value, pred.z_pred);
    const Eigen::MatrixXd& H = pred.H;
    const Eigen::MatrixXd S = H * P_j * H.transpose() + z.covariance;
    const Eigen::MatrixXd S_inv = S.inverse();
    const Eigen::MatrixXd K_gain = P_j * H.transpose() * S_inv;
    const Eigen::VectorXd x_new = x_j + K_gain * y;
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    // Joseph form: P = (I-KH) P (I-KH)' + K R K'.
    const Eigen::MatrixXd IKH = I - K_gain * H;
    const Eigen::MatrixXd P_new =
        IKH * P_j * IKH.transpose() +
        K_gain * z.covariance * K_gain.transpose();
    track.imm_means.col(j) = x_new;
    track.imm_covariances[j] = P_new;

    const double det = S.determinant();
    const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
    log_lambda(j) = -0.5 * std::log(safe_det) -
                    0.5 * y.transpose() * S_inv * y;
  }

  // Mode-probability update (log-sum-exp).
  Eigen::VectorXd log_w(K);
  for (int j = 0; j < K; ++j)
    log_w(j) = std::log(std::max(c(j), 1e-300)) + log_lambda(j);
  const double max_lw = log_w.maxCoeff();
  Eigen::VectorXd w = (log_w.array() - max_lw).exp();
  const double sum = w.sum();
  if (!std::isfinite(sum) || sum <= 0.0) {
    w = Eigen::VectorXd::Constant(K, 1.0 / K);
  } else {
    w /= sum;
  }
  track.imm_mode_probabilities = w;

  projectMixtureToTrack(track);
  track.last_update = z.time;
}

// Math:
//   Standard IMM-PDA. For each mode j:
//     1. Compute the EKF gain K_j = P_j H_j^T S_j^{-1}, where
//        S_j = H_j P_j H_j^T + R, evaluated at the prior (x_j, P_j).
//     2. PDAF combined residual y_bar_j = sum_m β_m * y_{m,j}.
//        Spread term Y_j = sum_m β_m y_{m,j} y_{m,j}^T - y_bar_j y_bar_j^T.
//        Posterior state: x_j_new = x_j + K_j * y_bar_j.
//        Posterior covariance:
//          P_j_new = β_0 * P_j + (1-β_0) * (I - K_j H_j) P_j
//                    + K_j Y_j K_j^T.
//        (This is the canonical PDAF covariance update; the spread term
//        widens the posterior to reflect data-association uncertainty.)
//     3. Per-mode measurement-mixture likelihood:
//          Λ_j = β_0 + (1-β_0) / (V * P_D) * Σ_m β_m * f_{m,j},
//        where f_{m,j} = N(y_{m,j}; 0, S_j). Since V * P_D is not
//        plumbed through here, we use the unnormalised proxy
//          Λ_j ∝ β_0 + Σ_m β_m * f_{m,j}.
//        Mode prior: c_j = Σ_i π_{i,j} μ_i (matches update()).
//        Mode posterior: μ_j ∝ c_j * Λ_j (log-sum-exp normalised).
//     4. Project the mixture to (track.state, track.covariance).
//
// Assumptions:
//   - All gated measurements share the same measurement model and
//     covariance R (we use gated_measurements[0] for H, R, sensor pose).
//     True for any single-scan JPDA call from Tracker::processBatch.
//   - betas.size() == gated_measurements.size().
//   - 0 <= β_m, 0 <= β_0, sum β_m + β_0 == 1.
//   - If M == 0 we no-op (no information to fold in).
//
// Rationale:
//   Without softUpdate, the no-op default fired for every IMM+JPDA
//   call: tracks initiated with zero velocity and predicted forward
//   forever with no measurement correction. Per-mode PDAF + standard
//   IMM mode-probability update mirrors what update() does for a
//   single hard measurement, generalised to soft betas. Falls back
//   cleanly to update()-equivalent behaviour when M == 1 and
//   β_0 == 0 (single confident hard match).
//
// Improve next:
//   - Plumb clutter density (lambda_C from JpdaAssociator) and
//     P_D into the estimator so the Λ_j formula uses the proper
//     V * P_D normalisation rather than the unnormalised proxy. The
//     proxy still produces a valid relative ordering across modes;
//     absolute mode-likelihood magnitudes (mostly used for
//     diagnostics) will differ.
//   - Track-level PG (gating probability) for the no-detection arm
//     once gating becomes per-mode.
void ImmEstimator::softUpdate(Track& track,
                              const std::vector<Measurement>& gated_measurements,
                              const Eigen::VectorXd& betas,
                              double beta_0,
                              const PdaContext& ctx) const {
  const int M = static_cast<int>(gated_measurements.size());
  if (M == 0 || betas.size() != M || track.imm_means.cols() == 0) return;
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());
  const Measurement& z0 = gated_measurements[0];
  // PDAF requires H, R, and sensor pose to be common across the gated
  // batch (we linearize once at the predicted state). Loud assert
  // beats silent miscompute if a sensor mix-up creeps in.
  for (int m = 1; m < M; ++m) {
    assert(gated_measurements[m].model == z0.model &&
           "ImmEstimator::softUpdate: gated measurements must share "
           "MeasurementModel");
    assert(gated_measurements[m].sensor_position_enu == z0.sensor_position_enu &&
           "ImmEstimator::softUpdate: gated measurements must share "
           "sensor_position_enu");
  }

  // Mode prior c_j = sum_i pi(i,j) mu_i — same as update().
  Eigen::VectorXd c(K);
  for (int j = 0; j < K; ++j) {
    double sum = 0.0;
    for (int i = 0; i < K; ++i)
      sum += pi_(i, j) * track.imm_mode_probabilities(i);
    c(j) = sum;
  }

  Eigen::VectorXd log_lambda(K);
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd x_j = track.imm_means.col(j);
    const Eigen::MatrixXd P_j = track.imm_covariances[j];
    const MeasurementPrediction pred =
        predictMeasurement(z0.model, x_j, z0.sensor_position_enu);
    const Eigen::MatrixXd& H = pred.H;
    const Eigen::MatrixXd S = H * P_j * H.transpose() + z0.covariance;
    const Eigen::MatrixXd S_inv = S.inverse();
    const Eigen::MatrixXd K_gain = P_j * H.transpose() * S_inv;

    Eigen::VectorXd y_combined = Eigen::VectorXd::Zero(z0.value.size());
    Eigen::MatrixXd spread_sum =
        Eigen::MatrixXd::Zero(z0.value.size(), z0.value.size());
    double f_sum = 0.0;   // Σ_m β_m N(y_{m,j}; 0, S_j)
    const double det = S.determinant();
    const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
    const int d = static_cast<int>(z0.value.size());
    const double norm =
        1.0 / std::sqrt(std::pow(2.0 * M_PI, d) * safe_det);
    for (int m = 0; m < M; ++m) {
      const Eigen::VectorXd y_m = measurementResidual(
          z0.model, gated_measurements[m].value, pred.z_pred);
      y_combined += betas(m) * y_m;
      spread_sum += betas(m) * y_m * y_m.transpose();
      const double q = y_m.transpose() * S_inv * y_m;
      f_sum += betas(m) * norm * std::exp(-0.5 * q);
    }
    spread_sum -= y_combined * y_combined.transpose();

    // Per-mode gate volume V_j = c_d · γ^d · √|S_j| where γ² is the
    // chi-square gate threshold and c_d the unit-ball constant
    // (c_1=2, c_2=π). The full PDAF mixture likelihood
    //   Λ_j = β₀ + (1−β₀)/(V_j·P_D) · Σ_m β_m N(...)
    // restores absolute scale across modes. If `ctx.p_d` ≤ 0 we fall
    // back to the unnormalized proxy `Λ_j ∝ β₀ + Σ_m β_m N(...)`.
    double lambda_j;
    if (ctx.p_d > 0.0 && ctx.gate_threshold > 0.0) {
      const double gamma = std::sqrt(ctx.gate_threshold);
      const double c_d = (d == 1) ? 2.0 : ((d == 2) ? M_PI : 0.0);
      // c_d == 0 for unsupported dim → fall through to proxy below.
      if (c_d > 0.0) {
        const double V_j = c_d * std::pow(gamma, d) * std::sqrt(safe_det);
        const double denom = V_j * ctx.p_d;
        lambda_j = beta_0 + (1.0 - beta_0) / std::max(denom, 1e-300) * f_sum;
      } else {
        lambda_j = beta_0 + f_sum;
      }
    } else {
      lambda_j = beta_0 + f_sum;
    }

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    // Joseph form for the "all-correct" arm.
    const Eigen::MatrixXd IKH = I - K_gain * H;
    const Eigen::MatrixXd P_post_full =
        IKH * P_j * IKH.transpose() +
        K_gain * z0.covariance * K_gain.transpose();
    track.imm_means.col(j) = x_j + K_gain * y_combined;
    track.imm_covariances[j] = beta_0 * P_j +
                               (1.0 - beta_0) * P_post_full +
                               K_gain * spread_sum * K_gain.transpose();

    log_lambda(j) = std::log(std::max(lambda_j, 1e-300));
  }

  // Mode-probability update (log-sum-exp).
  Eigen::VectorXd log_w(K);
  for (int j = 0; j < K; ++j)
    log_w(j) = std::log(std::max(c(j), 1e-300)) + log_lambda(j);
  const double max_lw = log_w.maxCoeff();
  Eigen::VectorXd w = (log_w.array() - max_lw).exp();
  const double sum = w.sum();
  if (!std::isfinite(sum) || sum <= 0.0) {
    w = Eigen::VectorXd::Constant(K, 1.0 / K);
  } else {
    w /= sum;
  }
  track.imm_mode_probabilities = w;

  projectMixtureToTrack(track);
  track.last_update = z0.time;
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

// Any-mode gating (Mazor 1998 §V). For each mode j compute the
// per-mode innovation (yⱼ, Sⱼ); pass if any d²ⱼ = yⱼᵀ Sⱼ⁻¹ yⱼ ≤ γ.
// Falls back to the single-Gaussian gate (via the IEstimator default)
// for tracks that have not yet acquired imm_means (e.g., during the
// brief window before the first IMM predict populates them).
bool ImmEstimator::gate(const Track& track,
                        const Measurement& z,
                        double gate_threshold) const {
  if (track.imm_means.cols() == 0) {
    return IEstimator::gate(track, z, gate_threshold);
  }
  const int K = static_cast<int>(track.imm_means.cols());
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd x_j = track.imm_means.col(j);
    const Eigen::MatrixXd& P_j = track.imm_covariances[j];
    const MeasurementPrediction pred =
        predictMeasurement(z.model, x_j, z.sensor_position_enu);
    const Eigen::VectorXd y =
        measurementResidual(z.model, z.value, pred.z_pred);
    const Eigen::MatrixXd S =
        pred.H * P_j * pred.H.transpose() + z.covariance;
    const double d2 = y.transpose() * S.inverse() * y;
    if (d2 <= gate_threshold) return true;
  }
  return false;
}

// Mode-weighted mixture log-likelihood:
//   log Σⱼ μⱼ · N(z; ẑⱼ, Sⱼ)
//        = max_j logℓⱼ + log Σⱼ μⱼ · exp(logℓⱼ - max_j logℓⱼ)
// where logℓⱼ = -½ d log(2π) - ½ log|Sⱼ| - ½ yⱼᵀ Sⱼ⁻¹ yⱼ. Computed
// via log-sum-exp to avoid underflow when one mode dominates by
// many orders of magnitude (common during a maneuver).
double ImmEstimator::logLikelihood(const Track& track,
                                   const Measurement& z) const {
  if (track.imm_means.cols() == 0) {
    return IEstimator::logLikelihood(track, z);
  }
  const int K = static_cast<int>(track.imm_means.cols());
  const int d = static_cast<int>(z.value.size());
  const double log_2pi_term =
      -0.5 * static_cast<double>(d) * std::log(2.0 * M_PI);
  Eigen::VectorXd log_ell(K);
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd x_j = track.imm_means.col(j);
    const Eigen::MatrixXd& P_j = track.imm_covariances[j];
    const MeasurementPrediction pred =
        predictMeasurement(z.model, x_j, z.sensor_position_enu);
    const Eigen::VectorXd y =
        measurementResidual(z.model, z.value, pred.z_pred);
    const Eigen::MatrixXd S =
        pred.H * P_j * pred.H.transpose() + z.covariance;
    const double det = S.determinant();
    const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
    const double mahal = y.transpose() * S.inverse() * y;
    log_ell(j) = log_2pi_term - 0.5 * std::log(safe_det) - 0.5 * mahal;
  }
  // log Σⱼ μⱼ exp(log_ell_j) via log-sum-exp.
  const Eigen::VectorXd& mu = track.imm_mode_probabilities;
  const double max_log = log_ell.maxCoeff();
  double sum = 0.0;
  for (int j = 0; j < K; ++j) {
    sum += mu(j) * std::exp(log_ell(j) - max_log);
  }
  return max_log + std::log(std::max(sum, 1e-300));
}

}  // namespace navtracker
