#include "core/association/JpdaAssociator.hpp"

#include <cmath>
#include <vector>

#include <Eigen/LU>

#include "core/association/Gating.hpp"
#include "core/association/JointEvents.hpp"
#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

JpdaAssociator::JpdaAssociator(double gate_threshold,
                               double probability_of_detection,
                               double clutter_density)
    : gate_threshold_(gate_threshold),
      p_d_(probability_of_detection),
      lambda_c_(clutter_density) {}

AssociationResult JpdaAssociator::associate(
    const std::vector<Track>& tracks,
    const std::vector<Measurement>& measurements) const {
  AssociationResult out;
  const int M = static_cast<int>(measurements.size());
  const int T = static_cast<int>(tracks.size());
  if (M == 0 || T == 0) {
    out.betas = Eigen::MatrixXd::Zero(M, T);
    out.beta_0 = Eigen::VectorXd::Ones(T);
    return out;
  }

  Eigen::MatrixXi V(M, T);
  Eigen::MatrixXd g(M, T);
  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < M; ++j) {
      const double d2 = mahalanobisDistance(tracks[t], measurements[j]);
      V(j, t) = (d2 <= gate_threshold_) ? 1 : 0;
      const MeasurementPrediction pred =
          predictMeasurement(measurements[j].model, tracks[t].state);
      const Eigen::MatrixXd S =
          pred.H * tracks[t].covariance * pred.H.transpose() +
          measurements[j].covariance;
      const int d = static_cast<int>(measurements[j].value.size());
      const double det = S.determinant();
      const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
      const double norm =
          1.0 / std::sqrt(std::pow(2.0 * M_PI, d) * safe_det);
      g(j, t) = norm * std::exp(-0.5 * d2);
    }
  }

  const std::vector<JointEvent> events = enumerateJointEvents(V);
  std::vector<double> log_w;
  log_w.reserve(events.size());
  for (const auto& e : events) {
    double log_weight = 0.0;
    std::vector<bool> track_detected(T, false);
    for (int j = 0; j < M; ++j) {
      const int code = e[j];
      if (code == 0) {
        log_weight += std::log(lambda_c_);
      } else {
        const int t = code - 1;
        log_weight += std::log(p_d_) + std::log(std::max(g(j, t), 1e-300));
        track_detected[t] = true;
      }
    }
    for (int t = 0; t < T; ++t) {
      if (!track_detected[t]) log_weight += std::log(1.0 - p_d_);
    }
    log_w.push_back(log_weight);
  }

  double max_lw = log_w[0];
  for (double v : log_w) if (v > max_lw) max_lw = v;
  std::vector<double> w(log_w.size());
  double sum = 0.0;
  for (std::size_t i = 0; i < log_w.size(); ++i) {
    w[i] = std::exp(log_w[i] - max_lw);
    sum += w[i];
  }
  if (!std::isfinite(sum) || sum <= 0.0) {
    out.betas = Eigen::MatrixXd::Zero(M, T);
    out.beta_0 = Eigen::VectorXd::Ones(T);
    return out;
  }
  for (double& v : w) v /= sum;

  out.betas = Eigen::MatrixXd::Zero(M, T);
  out.beta_0 = Eigen::VectorXd::Zero(T);
  for (std::size_t i = 0; i < events.size(); ++i) {
    const JointEvent& e = events[i];
    const double weight = w[i];
    std::vector<bool> detected(T, false);
    for (int j = 0; j < M; ++j) {
      if (e[j] >= 1) {
        const int t = e[j] - 1;
        out.betas(j, t) += weight;
        detected[t] = true;
      }
    }
    for (int t = 0; t < T; ++t) {
      if (!detected[t]) out.beta_0(t) += weight;
    }
  }
  return out;
}

}  // namespace navtracker
