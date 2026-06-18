#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapToPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

double deltaSeconds(Timestamp a, Timestamp b) {
  return static_cast<double>(a.nanos() - b.nanos()) * 1e-9;
}

}  // namespace

HeadingBiasEstimator::HeadingBiasEstimator(HeadingBiasEstimatorConfig cfg)
    : cfg_(cfg),
      b_hat_(cfg.initial_bias_rad),
      p_b_(cfg.initial_variance_rad2) {}

void HeadingBiasEstimator::predictTo(Timestamp to) {
  if (last_predict_.nanos() == 0) {
    last_predict_ = to;
    return;
  }
  const double dt = deltaSeconds(to, last_predict_);
  if (dt <= 0.0) return;
  p_b_ += cfg_.process_noise_var_per_sec * dt;
  last_predict_ = to;
}

void HeadingBiasEstimator::observe(const AisArpaPairObservation& obs) {
  predictTo(obs.time);

  const Eigen::Vector2d ais_rel = obs.ais_target_position_enu - obs.own_position_enu;
  const Eigen::Vector2d arpa_rel = obs.arpa_target_position_enu - obs.own_position_enu;
  const double beta_truth = std::atan2(ais_rel.y(), ais_rel.x());
  const double beta_arpa = std::atan2(arpa_rel.y(), arpa_rel.x());

  const double z = wrapToPi(beta_arpa - beta_truth);
  const double r_ais = ais_rel.norm();
  // Guard tiny ranges; if AIS-reported target is essentially at own-ship
  // the bearing is undefined — skip.
  if (r_ais < 1.0) return;

  double sigma_v2 =
      obs.arpa_bearing_std_rad * obs.arpa_bearing_std_rad
      + (obs.ais_position_std_m * obs.ais_position_std_m) / (r_ais * r_ais);
  // GPS position-uncertainty floor on the measurement noise: a σ_GPS
  // error at the own-ship origin translates to a (σ_GPS / r_arpa) angular
  // floor on the ARPA-derived bearing.
  const double r_arpa = arpa_rel.norm();
  if (r_arpa > 1.0) {
    const double term = obs.own_position_std_m / r_arpa;
    sigma_v2 += term * term;
  }

  const double y = wrapToPi(z - b_hat_);
  const double s = p_b_ + sigma_v2;
  // Outlier gate (review #14): a single bad AIS↔ARPA pair (mis-association,
  // AIS position spoof/jump) must not corrupt a *converged* bias estimate.
  // The gate is cold-start exempt — it only applies once we have at least
  // one accepted observation (has_any_update_). At cold start the only
  // reference is the tight init prior (default σ=5°); gating against it
  // would reject any genuinely large uncalibrated bias forever, and this v1
  // AIS↔ARPA path is often the *primary* calibration source with no other
  // heading reference. Compare against the current data-backed innovation σ.
  if (has_any_update_ &&
      std::abs(y) > cfg_.bi_outlier_sigma * std::sqrt(s)) {
    ++rej_outlier_;
    return;
  }
  const double k = p_b_ / s;
  b_hat_ += k * y;
  p_b_ = (1.0 - k) * p_b_;
  last_update_ = obs.time;
  has_any_update_ = true;
}

void HeadingBiasEstimator::observe(const BearingInnovation& obs) {
  predictTo(obs.time);

  if (obs.range_m < cfg_.bi_min_range_m) {
    ++rej_range_;
    return;
  }

  // S = state_var + R, so R = variance_rad2 - predicted_state_var_rad2.
  const double R = obs.variance_rad2 - obs.predicted_state_var_rad2;
  if (R <= 0.0 ||
      obs.predicted_state_var_rad2 > cfg_.bi_state_var_ratio_max * R) {
    ++rej_state_var_;
    return;
  }

  const double s = obs.variance_rad2 + p_b_;
  const double sigma = std::sqrt(s);
  if (std::abs(obs.innovation_rad) > cfg_.bi_outlier_sigma * sigma) {
    ++rej_outlier_;
    return;
  }

  const double y = wrapToPi(obs.innovation_rad - b_hat_);
  const double k = p_b_ / s;
  b_hat_ += k * y;
  p_b_ = (1.0 - k) * p_b_;
  last_update_ = obs.time;
  has_any_update_ = true;
  ++accepted_bi_;
}

HeadingBiasEstimate HeadingBiasEstimator::current() const {
  HeadingBiasEstimate est;
  est.bias_rad = b_hat_;
  est.variance_rad2 = p_b_;
  if (!has_any_update_) {
    est.is_published = false;
    return est;
  }
  const bool tight = p_b_ <= cfg_.publish_variance_threshold_rad2;
  const double age_s = deltaSeconds(last_predict_, last_update_);
  const bool fresh = age_s <= cfg_.stale_seconds;
  est.is_published = tight && fresh;
  return est;
}

namespace {

bool applyScalarUpdate(double& b_hat, double& p_b,
                       double measurement, double R,
                       double outlier_sigma,
                       std::size_t& rej_outlier) {
  const double s = R + p_b;
  const double sigma = std::sqrt(s);
  const double y = wrapToPi(measurement - b_hat);
  if (std::abs(y) > outlier_sigma * sigma) {
    ++rej_outlier;
    return false;
  }
  const double k = p_b / s;
  b_hat += k * y;
  p_b = (1.0 - k) * p_b;
  return true;
}

}  // namespace

void HeadingBiasEstimator::observe(const GyroVsGpsHeadingObservation& obs) {
  predictTo(obs.time);
  const double measurement =
      wrapToPi(obs.gyro_rad - obs.gps_true_heading_rad);
  const double R = obs.gps_true_heading_std_rad * obs.gps_true_heading_std_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R,
                        cfg_.mhs_outlier_sigma, rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_gps_hdg_;
  }
}

void HeadingBiasEstimator::observe(const GyroVsGpsCogObservation& obs) {
  predictTo(obs.time);
  if (obs.sog_mps < cfg_.cog_min_sog_mps) {
    ++rej_cog_sog_;
    return;
  }
  if (std::abs(obs.gyro_rate_rad_per_s) > cfg_.cog_max_gyro_rate_rad_per_s) {
    ++rej_cog_rate_;
    return;
  }
  const double measurement = wrapToPi(obs.gyro_rad - obs.gps_cog_rad);
  const double R = obs.gps_cog_std_rad * obs.gps_cog_std_rad
                 + cfg_.cog_crab_budget_rad * cfg_.cog_crab_budget_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R,
                        cfg_.mhs_outlier_sigma, rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_cog_;
  }
}

void HeadingBiasEstimator::observe(const GyroVsMagneticObservation& obs) {
  predictTo(obs.time);
  if (!obs.magnetic_variation_rad.has_value()) return;
  const double mag_corrected =
      obs.magnetic_heading_rad + *obs.magnetic_variation_rad;
  const double measurement = wrapToPi(obs.gyro_rad - mag_corrected);
  const double R = obs.magnetic_heading_std_rad * obs.magnetic_heading_std_rad
                 + cfg_.mag_deviation_budget_rad * cfg_.mag_deviation_budget_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R,
                        cfg_.mhs_outlier_sigma, rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_mag_;
  }
}

}  // namespace navtracker
