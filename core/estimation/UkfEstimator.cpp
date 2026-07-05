#include "core/estimation/UkfEstimator.hpp"

#include <utility>

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"
#include "core/estimation/SigmaPoints.hpp"

namespace navtracker {

UkfEstimator::UkfEstimator(std::shared_ptr<const IMotionModel> motion,
                           double init_speed_std,
                           double init_omega_std,
                           double alpha,
                           double beta,
                           double kappa)
    : motion_(std::move(motion)),
      init_speed_std_(init_speed_std),
      init_omega_std_(init_omega_std),
      alpha_(alpha),
      beta_(beta),
      kappa_(kappa) {}

void UkfEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const Eigen::MatrixXd Q = motion_->processNoise(dt);

  // True UKF: propagate each sigma point through the nonlinear motion
  // model f(x, dt). For linear models the default IMotionModel::propagate
  // collapses to F(dt)·x; for CT the per-sigma-point omega is used to
  // build its own F. This is what makes UKF prediction more accurate
  // than EKF on the CT mode.
  const SigmaPoints sp =
      computeSigmaPoints(track.state, track.covariance, alpha_, beta_, kappa_);
  Eigen::MatrixXd prop(sp.points.rows(), sp.points.cols());
  for (int i = 0; i < sp.points.cols(); ++i) {
    prop.col(i) = motion_->propagate(sp.points.col(i), dt);
  }

  Eigen::VectorXd mean = Eigen::VectorXd::Zero(track.state.size());
  for (int i = 0; i < prop.cols(); ++i) mean += sp.Wm(i) * prop.col(i);

  Eigen::MatrixXd cov =
      Eigen::MatrixXd::Zero(track.state.size(), track.state.size());
  for (int i = 0; i < prop.cols(); ++i) {
    const Eigen::VectorXd diff = prop.col(i) - mean;
    cov += sp.Wc(i) * diff * diff.transpose();
  }
  cov += Q;

  track.state = mean;
  track.covariance = cov;
  track.last_update = to;
}

void UkfEstimator::update(Track& track, const Measurement& z) const {
  // Defensive guard: NaN/non-PSD R would propagate through K and
  // permanently corrupt track.covariance. (Phase 8 R3 fix.)
  if (!isMeasurementCovariancePsd(z.covariance)) return;
  // Standard-form UKF update: re-draw sigma points from the predicted
  // (state, covariance) — predict() left them there. That is why the
  // cross-covariance below uses xd = sp.points[i] - track.state: the
  // sigma points are *centered on* track.state, so this difference is
  // the per-sigma-point deviation. Stone Soup follows the same form
  // in its `unscented_transform`.
  const SigmaPoints sp =
      computeSigmaPoints(track.state, track.covariance, alpha_, beta_, kappa_);
  const int nz = static_cast<int>(z.value.size());

  Eigen::MatrixXd Zsp(nz, sp.points.cols());
  for (int i = 0; i < sp.points.cols(); ++i) {
    const MeasurementPrediction pred =
        predictMeasurement(z.model, sp.points.col(i), z.sensor_position_enu);
    Zsp.col(i) = pred.z_pred;
  }

  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz);
  for (int i = 0; i < Zsp.cols(); ++i) z_pred += sp.Wm(i) * Zsp.col(i);

  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nz, nz);
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(track.state.size(), nz);
  for (int i = 0; i < Zsp.cols(); ++i) {
    Eigen::VectorXd zd = Zsp.col(i) - z_pred;
    if (z.model == MeasurementModel::RangeBearing2D) zd(1) = wrapAngle(zd(1));
    else if (z.model == MeasurementModel::Bearing2D) zd(0) = wrapAngle(zd(0));
    const Eigen::VectorXd xd = sp.points.col(i) - track.state;
    S += sp.Wc(i) * zd * zd.transpose();
    Pxz += sp.Wc(i) * xd * zd.transpose();
  }
  S += z.covariance;

  // LDLT-based solve avoids the naive .inverse() failure mode on
  // ill-conditioned innovation covariance (Phase 8 R4 fix).
  Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
  Eigen::MatrixXd K;
  if (ldlt_S.info() == Eigen::Success) {
    K = ldlt_S.solve(Pxz.transpose()).transpose();
  } else {
    K = Pxz * S.inverse();  // fall back to legacy path
  }
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, z_pred);
  track.state += K * y;
  Eigen::MatrixXd P_new = track.covariance - K * S * K.transpose();
  // Resymmetrise: UKF lacks a Joseph form, and accumulated rounding
  // produces asymmetric covariance which then breaks the next predict's
  // sigma-point Cholesky factor (Phase 8 R4 fix).
  track.covariance = 0.5 * (P_new + P_new.transpose());
  track.last_update = z.time;
}

Track UkfEstimator::initiate(const Measurement& z) const {
  // Size from motion model. CV2D → 4-state, CT / CV5State → 5-state with
  // ω as the trailing entry. Position rows/cols come from the measurement,
  // velocity rows from init_speed_std, ω from init_omega_std.
  const int n = motion_->stateDim();
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  x(0) = z.value(0);
  x(1) = z.value(1);
  t.state = x;

  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(n, n);
  p(0, 0) = z.covariance(0, 0);
  p(0, 1) = z.covariance(0, 1);
  p(1, 0) = z.covariance(1, 0);
  p(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  if (n >= 4) {
    p(2, 2) = vv;
    p(3, 3) = vv;
  }
  if (n >= 5) {
    p(4, 4) = init_omega_std_ * init_omega_std_;
  }
  t.covariance = p;

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  if (z.hints.platform_id.has_value())
    t.attributes.platform_id = z.hints.platform_id;
  if (z.hints.heading_deg.has_value())
    t.attributes.heading_deg = z.hints.heading_deg;
  if (z.hints.nav_status.has_value())
    t.attributes.nav_status = z.hints.nav_status;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
