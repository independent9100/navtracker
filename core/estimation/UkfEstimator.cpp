#include "core/estimation/UkfEstimator.hpp"

#include <utility>

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"
#include "core/estimation/SigmaPoints.hpp"

namespace navtracker {

UkfEstimator::UkfEstimator(std::shared_ptr<const IMotionModel> motion,
                           double init_speed_std,
                           double alpha,
                           double beta,
                           double kappa)
    : motion_(std::move(motion)),
      init_speed_std_(init_speed_std),
      alpha_(alpha),
      beta_(beta),
      kappa_(kappa) {}

void UkfEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const Eigen::MatrixXd F = motion_->transitionMatrix(dt);
  const Eigen::MatrixXd Q = motion_->processNoise(dt);

  const SigmaPoints sp =
      computeSigmaPoints(track.state, track.covariance, alpha_, beta_, kappa_);
  Eigen::MatrixXd prop(sp.points.rows(), sp.points.cols());
  for (int i = 0; i < sp.points.cols(); ++i) {
    prop.col(i) = F * sp.points.col(i);
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
  const SigmaPoints sp =
      computeSigmaPoints(track.state, track.covariance, alpha_, beta_, kappa_);
  const int nz = static_cast<int>(z.value.size());

  Eigen::MatrixXd Zsp(nz, sp.points.cols());
  for (int i = 0; i < sp.points.cols(); ++i) {
    const MeasurementPrediction pred =
        predictMeasurement(z.model, sp.points.col(i));
    Zsp.col(i) = pred.z_pred;
  }

  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz);
  for (int i = 0; i < Zsp.cols(); ++i) z_pred += sp.Wm(i) * Zsp.col(i);

  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nz, nz);
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(track.state.size(), nz);
  for (int i = 0; i < Zsp.cols(); ++i) {
    Eigen::VectorXd zd = Zsp.col(i) - z_pred;
    if (z.model == MeasurementModel::RangeBearing2D) zd(1) = wrapAngle(zd(1));
    const Eigen::VectorXd xd = sp.points.col(i) - track.state;
    S += sp.Wc(i) * zd * zd.transpose();
    Pxz += sp.Wc(i) * xd * zd.transpose();
  }
  S += z.covariance;

  const Eigen::MatrixXd K = Pxz * S.inverse();
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, z_pred);
  track.state += K * y;
  track.covariance -= K * S * K.transpose();
  track.last_update = z.time;
}

Track UkfEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = z.value(0);
  x(1) = z.value(1);
  t.state = x;

  Eigen::Matrix4d p = Eigen::Matrix4d::Zero();
  p(0, 0) = z.covariance(0, 0);
  p(0, 1) = z.covariance(0, 1);
  p(1, 0) = z.covariance(1, 0);
  p(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  p(2, 2) = vv;
  p(3, 3) = vv;
  t.covariance = p;

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
