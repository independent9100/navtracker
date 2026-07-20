#include "core/estimation/EkfEstimator.hpp"

#include <cassert>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "core/estimation/BearingRangeGuard.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/projection/Projection.hpp"

namespace navtracker {

EkfEstimator::EkfEstimator(std::shared_ptr<const IMotionModel> motion,
                           double init_speed_std,
                           std::shared_ptr<const IMeasurementNoiseModel> noise,
                           bool bearing_range_guard)
    : motion_(std::move(motion)),
      init_speed_std_(init_speed_std),
      noise_(std::move(noise)),
      bearing_range_guard_(bearing_range_guard) {}

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
  // Defensive guard: a non-finite or non-PSD R would propagate NaN into
  // K and permanently poison track.covariance. Skip the update — the
  // track stays at its predicted state for this scan. (Phase 8 R3 fix.)
  if (!isMeasurementCovariancePsd(z.covariance, z.dim())) return;  // #35 M1
  const Eigen::VectorXd x_pred = track.state;
  const Eigen::MatrixXd P_pred = track.covariance;
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state, z.sensor_position_enu);
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd& h = pred.H;
  // Robustify: inflate R by the noise model's scale (Gaussian → 1; a
  // Student-t down-weights outliers). The scale is computed from the
  // nominal innovation covariance, then folded into the effective R used
  // for both the gain and the Joseph covariance update.
  const Eigen::MatrixXd s_nom = h * track.covariance * h.transpose() + z.covariance;
  const double scale = noise_ ? noise_->covarianceScale(y, s_nom) : 1.0;
  const Eigen::MatrixXd R = z.covariance * scale;
  const Eigen::MatrixXd s = h * track.covariance * h.transpose() + R;
  const Eigen::MatrixXd k = track.covariance * h.transpose() * s.inverse();
  track.state += k * y;
  // Joseph form: symmetric and PD-preserving even when (I-KH)P leaks
  // negative eigenvalues through rounding. P = (I-KH) P (I-KH)' + K R K'.
  const auto n = track.state.size();
  const Eigen::MatrixXd id = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd ikh = id - k * h;
  track.covariance = ikh * track.covariance * ikh.transpose() +
                     k * R * k.transpose();
  if (bearing_range_guard_ && z.model == MeasurementModel::Bearing2D) {
    track.covariance = applyBearingRangeGuard(P_pred, track.covariance, x_pred,
                                              z.sensor_position_enu);
  }
  track.last_update = z.time;
}

Track EkfEstimator::initiate(const Measurement& z) const {
  Track t;
  // Empty (0x0 "no uncertainty" sentinel) or non-PSD measurement covariance
  // cannot seed a track: reading z.covariance(0,0)… would be out of bounds on
  // an empty matrix (assert in debug, UB in release), and a degenerate R would
  // yield a singular birth covariance. Mirror update()'s skip semantics —
  // return a default "did not initiate" track (empty state, no sources) so no
  // track is born from malformed input. Callers must supply R (or call
  // applyDefaultsIfEmpty) before initiating.
  if (!isMeasurementCovariancePsd(z.covariance, z.dim())) return t;  // #35 M1

  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  // W4.1: a RangeBearing2D measurement carries (range, bearing) + a polar 2×2
  // covariance, NOT an ENU point. CONVERT to ENU here (shared math-convention
  // helper) so the birth lands at the true position with a sane ENU covariance
  // and the next scan gates to it — instead of planting (range_m, bearing_rad)
  // as (east, north) with mixed m²/rad² covariance (nothing gates → phantom
  // proliferation). Other models already carry ENU (east, north) in value[0..1].
  const auto birth = initiationPosCov(z);
  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = birth.pos_enu(0);
  x(1) = birth.pos_enu(1);
  // #20: one-shot velocity prior at birth (ARPA TTM speed/course). Used ONCE
  // here to seed the birth velocity, then discarded — a prior cannot
  // double-count (guide §3). The birth covariance keeps the wide init_speed_std
  // variance, so the prior nudges the direction without over-committing.
  if (z.hints.birth_velocity_enu.has_value()) {
    x(2) = z.hints.birth_velocity_enu->x();
    x(3) = z.hints.birth_velocity_enu->y();
  }
  t.state = x;

  Eigen::Matrix4d p = Eigen::Matrix4d::Zero();
  p(0, 0) = birth.cov(0, 0);
  p(0, 1) = birth.cov(0, 1);
  p(1, 0) = birth.cov(1, 0);
  p(1, 1) = birth.cov(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  p(2, 2) = vv;
  p(3, 3) = vv;
  t.covariance = p;

  if (z.hints.mmsi.has_value()) {
    t.attributes.mmsi = z.hints.mmsi;
  }
  if (z.hints.platform_id.has_value()) {
    t.attributes.platform_id = z.hints.platform_id;
  }
  if (z.hints.heading_deg.has_value()) {
    t.attributes.heading_deg = z.hints.heading_deg;
  }
  if (z.hints.nav_status.has_value()) {
    t.attributes.nav_status = z.hints.nav_status;
  }
  t.contributing_sources.push_back(z.source_id);
  return t;
}

void EkfEstimator::softUpdate(Track& track,
                              const std::vector<Measurement>& gated_measurements,
                              const Eigen::VectorXd& betas,
                              double beta_0,
                              const PdaContext& /*ctx*/) const {
  // Single-mode EKF does not need PdaContext for its covariance update;
  // it is consumed by multi-mode estimators (ImmEstimator) to normalize
  // per-mode mixture likelihoods.
  const int M = static_cast<int>(gated_measurements.size());
  if (M == 0 || betas.size() != M) return;
  const Measurement& z0 = gated_measurements[0];
  // PDAF assumes H, R, and sensor pose are common across the gated
  // batch (linearization point is the predicted state, not the measured
  // value). Loud failure beats silent miscompute if an upstream change
  // ever mixes sensors into one JPDA scan.
  for (int m = 1; m < M; ++m) {
    assert(gated_measurements[m].model == z0.model &&
           "EkfEstimator::softUpdate: gated measurements must share "
           "MeasurementModel");
    assert(gated_measurements[m].sensor_position_enu == z0.sensor_position_enu &&
           "EkfEstimator::softUpdate: gated measurements must share "
           "sensor_position_enu");
  }
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
  // Joseph form for the "all-correct" arm: (I-KH) P (I-KH)' + K R K'.
  const Eigen::MatrixXd IKH = I - K * H;
  const Eigen::MatrixXd P_post_full =
      IKH * track.covariance * IKH.transpose() +
      K * z0.covariance * K.transpose();
  track.state += K * y_combined;
  track.covariance = beta_0 * track.covariance +
                     (1.0 - beta_0) * P_post_full +
                     K * spread_sum * K.transpose();
  track.last_update = z0.time;
}

}  // namespace navtracker
