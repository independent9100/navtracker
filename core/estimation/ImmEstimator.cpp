#include "core/estimation/ImmEstimator.hpp"

#include <cassert>
#include <cmath>
#include <utility>

#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

#include "core/estimation/BearingRangeGuard.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/estimation/SigmaPoints.hpp"

namespace navtracker {

ImmEstimator::ImmEstimator(std::vector<std::shared_ptr<IMotionModel>> motions,
                           Eigen::MatrixXd transition_matrix,
                           Eigen::VectorXd initial_mode_probabilities,
                           double init_speed_std,
                           double init_omega_std,
                           std::shared_ptr<const IMeasurementNoiseModel> noise,
                           bool bearing_range_guard,
                           bool use_ukf,
                           double ukf_alpha,
                           double ukf_beta,
                           double ukf_kappa)
    : motions_(std::move(motions)),
      pi_(std::move(transition_matrix)),
      mu0_(std::move(initial_mode_probabilities)),
      init_speed_std_(init_speed_std),
      init_omega_std_(init_omega_std),
      noise_(std::move(noise)),
      bearing_range_guard_(bearing_range_guard),
      use_ukf_(use_ukf),
      ukf_alpha_(ukf_alpha),
      ukf_beta_(ukf_beta),
      ukf_kappa_(ukf_kappa) {}

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

// π^dt with stochastic-matrix sanitation. dt == 1 short-circuits to the
// configured matrix (the common synthetic cadence — bit-identical to the
// legacy per-call behaviour there). The fractional power of a row-
// stochastic, diagonally-dominant TPM is again row-stochastic up to FP
// noise; tiny negative entries are clamped and rows re-normalised. A
// non-embeddable chain (NaNs in the power) falls back to the per-call
// matrix — wrong cadence scaling, but never garbage probabilities.
Eigen::MatrixXd ImmEstimator::transitionFor(double dt) const {
  if (dt == 1.0) return pi_;
  Eigen::MatrixXd p = pi_.pow(dt);
  if (!p.allFinite()) return pi_;
  p = p.cwiseMax(0.0);
  for (int i = 0; i < p.rows(); ++i) {
    const double row_sum = p.row(i).sum();
    if (row_sum > 0.0) p.row(i) /= row_sum;
  }
  return p;
}

void ImmEstimator::predict(Track& track, Timestamp to) const {
  if (track.imm_means.cols() == 0) return;
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());

  // Mixing step with the dt-scaled TPM (see class docs).
  const Eigen::MatrixXd pi_dt = transitionFor(dt);
  Eigen::VectorXd c(K);
  for (int j = 0; j < K; ++j) {
    double sum = 0.0;
    for (int i = 0; i < K; ++i)
      sum += pi_dt(i, j) * track.imm_mode_probabilities(i);
    c(j) = sum;
  }
  Eigen::MatrixXd mu_ij(K, K);
  for (int j = 0; j < K; ++j) {
    if (c(j) <= 0.0) {
      mu_ij.col(j).setZero();
      mu_ij(j, j) = 1.0;
    } else {
      for (int i = 0; i < K; ++i)
        mu_ij(i, j) = pi_dt(i, j) * track.imm_mode_probabilities(i) / c(j);
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

  // Per-mode prediction. EKF path (default): linearize F at the mode's
  // mixed-prior omega for the covariance update, apply the nonlinear
  // propagate() to the mean — for linear motion models F·x and
  // propagate() coincide. UKF path (use_ukf_): propagate sigma points
  // drawn from (x_mix, P_mix) through propagate() and reconstruct both
  // mean and covariance from the weighted sums, then add Q. The UKF
  // path captures nonlinearity exactly to second-order moments at the
  // cost of (2n+1) propagate() calls per mode.
  for (int j = 0; j < K; ++j) {
    const Eigen::MatrixXd Q = motions_[j]->processNoise(dt);
    if (use_ukf_) {
      const SigmaPoints sp = computeSigmaPoints(
          x_mix.col(j), P_mix[j], ukf_alpha_, ukf_beta_, ukf_kappa_);
      Eigen::MatrixXd prop(sp.points.rows(), sp.points.cols());
      for (int i = 0; i < sp.points.cols(); ++i) {
        prop.col(i) = motions_[j]->propagate(sp.points.col(i), dt);
      }
      Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
      for (int i = 0; i < prop.cols(); ++i) mean += sp.Wm(i) * prop.col(i);
      Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n);
      for (int i = 0; i < prop.cols(); ++i) {
        const Eigen::VectorXd d = prop.col(i) - mean;
        cov += sp.Wc(i) * d * d.transpose();
      }
      cov += Q;
      track.imm_means.col(j) = mean;
      track.imm_covariances[j] = cov;
    } else {
      Eigen::MatrixXd F;
      if (auto* ct = dynamic_cast<CoordinatedTurn*>(motions_[j].get())) {
        F = ct->transitionMatrixAt(x_mix(4, j), dt);
      } else {
        F = motions_[j]->transitionMatrix(dt);
      }
      track.imm_means.col(j) = motions_[j]->propagate(x_mix.col(j), dt);
      track.imm_covariances[j] = F * P_mix[j] * F.transpose() + Q;
    }
  }

  // Advance the mode probabilities to the predicted prior c = π(dt)ᵀ μ.
  // update()/softUpdate() consume this directly (μ⁺ ∝ c_j Λ_j), so the
  // TPM is applied exactly once per predict, scaled by its dt — never
  // once more per measurement.
  track.imm_mode_probabilities = c;

  projectMixtureToTrack(track);
  track.last_update = to;
}

void ImmEstimator::update(Track& track, const Measurement& z) const {
  // Defensive guard: NaN/non-PSD R poisons every mode's innovation
  // covariance and propagates through the IMM ensemble. (Phase 8 R3.)
  if (!isMeasurementCovariancePsd(z.covariance)) return;
  if (track.imm_means.cols() == 0) return;
  const int K = static_cast<int>(track.imm_means.cols());
  const int n = static_cast<int>(track.imm_means.rows());

  // c_j: mode-prior at update time. predict() already advanced μ to the
  // dt-scaled predicted prior π(dt)ᵀμ, so it is consumed as-is. (A
  // same-timestamp second update sees dt = 0 ⇒ π(0) = I — applying the
  // TPM again here would double-count the transition.)
  const Eigen::VectorXd c = track.imm_mode_probabilities;

  // Per-mode update + log-likelihood. EKF path (default): linearize h
  // at x_j (the Jacobian H from predictMeasurement), Joseph-form gain.
  // UKF path (use_ukf_): draw sigma points from (x_j, P_j), pass each
  // through h via predictMeasurement, reconstruct (z_pred, S, Pxz),
  // gain K = Pxz S^-1; covariance update P_new = P_j - K S K^T (the
  // standard unscented form; not Joseph). Both paths share R, the
  // robustness noise scale, and the bearing-range-guard hook.
  // Copies of (x_j, P_j) so we read the prior while writing back.
  Eigen::VectorXd log_lambda(K);
  for (int j = 0; j < K; ++j) {
    const Eigen::VectorXd x_j = track.imm_means.col(j);
    const Eigen::MatrixXd P_j = track.imm_covariances[j];

    Eigen::VectorXd y;
    Eigen::MatrixXd S;
    Eigen::MatrixXd K_gain;
    Eigen::VectorXd x_new;
    Eigen::MatrixXd P_new;

    if (use_ukf_) {
      const SigmaPoints sp = computeSigmaPoints(x_j, P_j, ukf_alpha_,
                                                ukf_beta_, ukf_kappa_);
      const int nz = static_cast<int>(z.value.size());
      Eigen::MatrixXd Zsp(nz, sp.points.cols());
      for (int i = 0; i < sp.points.cols(); ++i) {
        Zsp.col(i) = predictMeasurement(z.model, sp.points.col(i),
                                        z.sensor_position_enu).z_pred;
      }
      Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz);
      for (int i = 0; i < Zsp.cols(); ++i) z_pred += sp.Wm(i) * Zsp.col(i);
      Eigen::MatrixXd Sxx = Eigen::MatrixXd::Zero(nz, nz);
      Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, nz);
      for (int i = 0; i < Zsp.cols(); ++i) {
        Eigen::VectorXd zd = Zsp.col(i) - z_pred;
        if (z.model == MeasurementModel::RangeBearing2D) zd(1) = wrapAngle(zd(1));
        else if (z.model == MeasurementModel::Bearing2D)  zd(0) = wrapAngle(zd(0));
        const Eigen::VectorXd xd = sp.points.col(i) - x_j;
        Sxx += sp.Wc(i) * zd * zd.transpose();
        Pxz += sp.Wc(i) * xd * zd.transpose();
      }
      y = measurementResidual(z.model, z.value, z_pred);
      const Eigen::MatrixXd S_nom = Sxx + z.covariance;
      const double scale = noise_ ? noise_->covarianceScale(y, S_nom) : 1.0;
      const Eigen::MatrixXd R = z.covariance * scale;
      S = Sxx + R;
      K_gain = Pxz * S.inverse();
      x_new = x_j + K_gain * y;
      P_new = P_j - K_gain * S * K_gain.transpose();
    } else {
      const MeasurementPrediction pred = predictMeasurement(z.model, x_j, z.sensor_position_enu);
      y = measurementResidual(z.model, z.value, pred.z_pred);
      const Eigen::MatrixXd& H = pred.H;
      const Eigen::MatrixXd S_nom = H * P_j * H.transpose() + z.covariance;
      const double scale = noise_ ? noise_->covarianceScale(y, S_nom) : 1.0;
      const Eigen::MatrixXd R = z.covariance * scale;
      S = H * P_j * H.transpose() + R;
      const Eigen::MatrixXd S_inv = S.inverse();
      K_gain = P_j * H.transpose() * S_inv;
      x_new = x_j + K_gain * y;
      const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
      // Joseph form: P = (I-KH) P (I-KH)' + K R K'.
      const Eigen::MatrixXd IKH = I - K_gain * H;
      P_new = IKH * P_j * IKH.transpose() + K_gain * R * K_gain.transpose();
    }

    if (bearing_range_guard_ && z.model == MeasurementModel::Bearing2D) {
      // Guard per mode: each P_j has its own pre/post pair. The
      // per-mode predicted state x_j is the right LOS reference for
      // the mode-conditioned bearing.
      P_new = applyBearingRangeGuard(P_j, P_new, x_j, z.sensor_position_enu);
    }
    track.imm_means.col(j) = x_new;
    track.imm_covariances[j] = P_new;

    const Eigen::MatrixXd S_inv = S.inverse();
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

  // Mode prior: predict() already advanced μ to π(dt)ᵀμ — same as
  // update().
  const Eigen::VectorXd c = track.imm_mode_probabilities;

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
  if (z.hints.platform_id.has_value())
    t.attributes.platform_id = z.hints.platform_id;
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
