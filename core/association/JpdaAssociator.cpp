#include "core/association/JpdaAssociator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/LU>

#include "core/association/Gating.hpp"
#include "core/association/JointEvents.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {

JpdaAssociator::JpdaAssociator(double gate_threshold,
                               double probability_of_detection,
                               double clutter_density)
    : gate_threshold_(gate_threshold),
      p_d_(probability_of_detection),
      lambda_c_(clutter_density) {}

AssociationResult JpdaAssociator::associate(
    const std::vector<Track>& tracks,
    const std::vector<Measurement>& measurements,
    const IEstimator* estimator) const {
  AssociationResult out;
  out.p_d = p_d_;
  out.gate_threshold = gate_threshold_;
  const int M = static_cast<int>(measurements.size());
  const int T = static_cast<int>(tracks.size());
  if (M == 0 || T == 0) {
    out.betas = Eigen::MatrixXd::Zero(M, T);
    out.beta_0 = Eigen::VectorXd::Ones(T);
    return out;
  }

  // Per-pair gate + density. With an estimator, route both through it
  // (any-mode gating + mode-weighted mixture likelihood for IMM).
  // Without, fall back to the single-Gaussian inline path.
  Eigen::MatrixXi V(M, T);
  Eigen::MatrixXd g(M, T);
  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < M; ++j) {
      bool in_gate;
      double density;
      if (estimator) {
        in_gate = estimator->gate(tracks[t], measurements[j], gate_threshold_);
        density = std::exp(estimator->logLikelihood(tracks[t], measurements[j]));
      } else {
        const double d2 = mahalanobisDistance(tracks[t], measurements[j]);
        in_gate = (d2 <= gate_threshold_);
        const MeasurementPrediction pred =
            predictMeasurement(measurements[j].model, tracks[t].state,
                               measurements[j].sensor_position_enu);
        const Eigen::MatrixXd S =
            pred.H * tracks[t].covariance * pred.H.transpose() +
            measurements[j].covariance;
        const int d = static_cast<int>(measurements[j].value.size());
        const double det = S.determinant();
        const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
        const double norm =
            1.0 / std::sqrt(std::pow(2.0 * M_PI, d) * safe_det);
        density = norm * std::exp(-0.5 * d2);
      }
      V(j, t) = in_gate ? 1 : 0;
      g(j, t) = density;
    }
  }

  // Full-enumeration JPDA is O(M^T) per gating cluster. On real radar with
  // clutter the per-scan track cluster can be large enough to exhaust
  // memory; cap the enumeration and degrade gracefully to greedy hard
  // (GNN-style) association when the cap trips. This is the documented
  // cluster-size safeguard for a JPDA without an EHM solver. The cap is
  // generous (1e6 events) so synthetic scenes — whose clusters are tiny —
  // are bit-identical; only pathological real-clutter clusters fall back.
  constexpr std::size_t kMaxJointEvents = 1'000'000;
  const std::vector<JointEvent> events =
      enumerateJointEvents(V, kMaxJointEvents);
  if (events.empty()) {
    // Overflow: greedy mutual-exclusion assignment by descending density.
    // Each measurement claims its best gating track if neither is taken;
    // the result is hard betas (0/1), i.e. this cluster is associated as
    // GNN would. Loud-ish but safe: tracking continues instead of OOM.
    out.betas = Eigen::MatrixXd::Zero(M, T);
    out.beta_0 = Eigen::VectorXd::Ones(T);
    struct Pair { int j; int t; double dens; };
    std::vector<Pair> pairs;
    for (int t = 0; t < T; ++t)
      for (int j = 0; j < M; ++j)
        if (V(j, t)) pairs.push_back({j, t, g(j, t)});
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.dens > b.dens; });
    std::vector<bool> m_used(M, false), t_used(T, false);
    for (const Pair& p : pairs) {
      if (m_used[p.j] || t_used[p.t]) continue;
      out.betas(p.j, p.t) = 1.0;
      out.beta_0(p.t) = 0.0;
      m_used[p.j] = true;
      t_used[p.t] = true;
    }
    out.overflow_fallback = true;
    return out;
  }
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
