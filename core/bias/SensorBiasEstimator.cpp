#include "core/bias/SensorBiasEstimator.hpp"

#include <cmath>
#include <cstdint>

#include <Eigen/LU>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAnglePi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double secondsBetween(Timestamp earlier, Timestamp later) {
  return (later.nanos() - earlier.nanos()) * 1e-9;
}

}  // namespace

SensorBiasEstimator::SensorBiasEstimator(SensorBiasEstimatorConfig cfg)
    : cfg_(cfg) {}

SensorBiasEstimator::PositionState&
SensorBiasEstimator::posStateFor(const SensorBiasKey& key) {
  auto it = pos_.find(key);
  if (it != pos_.end()) return it->second;
  PositionState s;
  const double v = cfg_.initial_pos_std_m * cfg_.initial_pos_std_m;
  s.P = (Eigen::Matrix2d() << v, 0.0, 0.0, v).finished();
  auto [ins, _] = pos_.emplace(key, std::move(s));
  return ins->second;
}

SensorBiasEstimator::BearingState&
SensorBiasEstimator::brgStateFor(const SensorBiasKey& key) {
  auto it = brg_.find(key);
  if (it != brg_.end()) return it->second;
  BearingState s;
  s.P = cfg_.initial_brg_std_rad * cfg_.initial_brg_std_rad;
  auto [ins, _] = brg_.emplace(key, std::move(s));
  return ins->second;
}

void SensorBiasEstimator::predictTo(Timestamp to) {
  for (auto& [key, s] : pos_) {
    if (s.last_predict.nanos() == 0) {
      s.last_predict = to;
      continue;
    }
    const double dt = secondsBetween(s.last_predict, to);
    if (dt <= 0.0) continue;
    const double q = cfg_.pos_process_noise_var_per_sec * dt;
    s.P(0, 0) += q;
    s.P(1, 1) += q;
    s.last_predict = to;
  }
  for (auto& [key, s] : brg_) {
    if (s.last_predict.nanos() == 0) {
      s.last_predict = to;
      continue;
    }
    const double dt = secondsBetween(s.last_predict, to);
    if (dt <= 0.0) continue;
    s.P += cfg_.brg_process_noise_var_per_sec * dt;
    s.last_predict = to;
  }
}

void SensorBiasEstimator::observe(const PositionBiasPairObservation& obs) {
  PositionState& s = posStateFor(obs.key);
  if (s.last_predict.nanos() == 0) {
    s.last_predict = obs.time;
  } else {
    const double dt = secondsBetween(s.last_predict, obs.time);
    if (dt > 0.0) {
      const double q = cfg_.pos_process_noise_var_per_sec * dt;
      s.P(0, 0) += q;
      s.P(1, 1) += q;
      s.last_predict = obs.time;
    }
  }

  // Range gate: pair-target range from own ship must exceed min_range_m.
  const Eigen::Vector2d midpoint =
      0.5 * (obs.z_sensor_enu + obs.z_anchor_enu);
  const double range = (midpoint - obs.own_position_enu).norm();
  if (range < cfg_.min_range_m) {
    ++rej_range_;
    return;
  }

  // Observation: r = z_sensor - z_anchor - b̂ ; R_obs = R_sensor + R_anchor.
  const Eigen::Vector2d r =
      obs.z_sensor_enu - obs.z_anchor_enu - s.b_hat;
  const Eigen::Matrix2d R_obs = obs.R_sensor + obs.R_anchor;
  const Eigen::Matrix2d S = s.P + R_obs;
  const Eigen::Matrix2d S_inv = S.inverse();

  // Outlier gate: Mahalanobis distance r^T S^-1 r > N^2 chi2 cutoff
  // (proxy for "5-sigma" in 2D).
  const double mahal2 = (r.transpose() * S_inv * r)(0, 0);
  const double cutoff = cfg_.outlier_sigma * cfg_.outlier_sigma;
  if (mahal2 > cutoff) {
    ++rej_outlier_;
    return;
  }

  // KF update with H = I.
  const Eigen::Matrix2d K = s.P * S_inv;
  s.b_hat += K * r;
  s.P = (Eigen::Matrix2d::Identity() - K) * s.P;
  s.has_update = true;
  ++acc_pos_;
}

void SensorBiasEstimator::observe(const BearingBiasPairObservation& obs) {
  BearingState& s = brgStateFor(obs.key);
  if (s.last_predict.nanos() == 0) {
    s.last_predict = obs.time;
  } else {
    const double dt = secondsBetween(s.last_predict, obs.time);
    if (dt > 0.0) {
      s.P += cfg_.brg_process_noise_var_per_sec * dt;
      s.last_predict = obs.time;
    }
  }

  const Eigen::Vector2d d =
      obs.anchor_target_position_enu - obs.sensor_position_enu;
  const double range = d.norm();
  if (range < cfg_.min_range_m) {
    ++rej_range_;
    return;
  }

  const double alpha_pred = std::atan2(d.y(), d.x()) + s.b_hat;
  const double r = wrapAnglePi(obs.alpha_observed_rad - alpha_pred);

  // Project anchor position noise into bearing noise:
  //   σ²_α_anchor = σ²_pos / r²  (1-D approximation, isotropic)
  // Adds the sensor's own bearing variance.
  const double r2 = range * range;
  const double anchor_var =
      (obs.anchor_position_std_m * obs.anchor_position_std_m) / r2;
  const double R_obs = obs.alpha_meas_var_rad2 + anchor_var;
  const double S = s.P + R_obs;

  if ((r * r) > cfg_.outlier_sigma * cfg_.outlier_sigma * S) {
    ++rej_outlier_;
    return;
  }

  const double K = s.P / S;
  s.b_hat += K * r;
  s.P = (1.0 - K) * s.P;
  s.has_update = true;
  ++acc_brg_;
}

void SensorBiasEstimator::setKnownPositionBias(
    const SensorBiasKey& key,
    const Eigen::Vector2d& bias_enu_m,
    const Eigen::Matrix2d& covariance_m2) {
  PositionState& s = posStateFor(key);
  s.b_hat = bias_enu_m;
  s.P = covariance_m2;
  s.has_update = true;
  // last_predict left at default (0); the next observe() call will
  // initialize it to obs.time, same as a fresh entry.
}

void SensorBiasEstimator::setKnownBearingBias(
    const SensorBiasKey& key, double bias_rad, double variance_rad2) {
  BearingState& s = brgStateFor(key);
  s.b_hat = bias_rad;
  s.P = variance_rad2;
  s.has_update = true;
}

PositionBiasEstimate
SensorBiasEstimator::positionBias(const SensorBiasKey& key) const {
  PositionBiasEstimate out;
  auto it = pos_.find(key);
  if (it == pos_.end()) return out;
  out.bias_enu_m = it->second.b_hat;
  out.covariance_m2 = it->second.P;
  // Publish only when *both* axes are sub-threshold AND the estimator
  // has been actually updated at least once.
  const bool below =
      it->second.P(0, 0) <= cfg_.publish_pos_var_threshold_m2 &&
      it->second.P(1, 1) <= cfg_.publish_pos_var_threshold_m2;
  out.is_published = it->second.has_update && below;
  return out;
}

BearingBiasEstimate
SensorBiasEstimator::bearingBias(const SensorBiasKey& key) const {
  BearingBiasEstimate out;
  auto it = brg_.find(key);
  if (it == brg_.end()) return out;
  out.bias_rad = it->second.b_hat;
  out.variance_rad2 = it->second.P;
  out.is_published =
      it->second.has_update &&
      it->second.P <= cfg_.publish_brg_var_threshold_rad2;
  return out;
}

}  // namespace navtracker
