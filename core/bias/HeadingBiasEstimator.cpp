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
  const double k = p_b_ / s;
  b_hat_ += k * y;
  p_b_ = (1.0 - k) * p_b_;
  last_update_ = obs.time;
  has_any_update_ = true;
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

}  // namespace navtracker
