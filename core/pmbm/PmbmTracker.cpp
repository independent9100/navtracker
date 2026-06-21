#include "core/pmbm/PmbmTracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <utility>

#include <Eigen/Dense>

#include "core/association/Murty.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/pipeline/BiasCorrection.hpp"
#include "core/pipeline/SourceTouchPopulate.hpp"

namespace navtracker::pmbm {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// TPMBM trajectory append + window trim. Called after detection
// (post-update state) and after misdetection (post-predict state).
// Skips when window=0 (TPMBM disabled).
void appendTrajectoryPoint(Bernoulli& b, std::size_t window_scans,
                           Timestamp t) {
  if (window_scans == 0) return;
  TrajectoryPoint p;
  p.time = t;
  p.state = b.mean;
  p.covariance = b.covariance;
  b.trajectory.push_back(std::move(p));
  if (b.trajectory.size() > window_scans) {
    const std::size_t drop = b.trajectory.size() - window_scans;
    b.trajectory.erase(b.trajectory.begin(),
                       b.trajectory.begin() + drop);
  }
}

// Numerically stable log(Σ exp(x_i)). Returns -inf for empty input.
double logSumExp(const std::vector<double>& v) {
  if (v.empty()) return -kInf;
  double m = -kInf;
  for (double x : v)
    if (x > m) m = x;
  if (m == -kInf) return -kInf;
  double s = 0.0;
  for (double x : v) s += std::exp(x - m);
  return m + std::log(s);
}

}  // namespace

PmbmTracker::PmbmTracker(const IEstimator& estimator, Config cfg,
                         BirthModelFn birth_model)
    : estimator_(estimator),
      cfg_(cfg),
      birth_model_(std::move(birth_model)) {}

void PmbmTracker::predict(Timestamp to) {
  if (!has_current_time_) {
    current_time_ = to;
    has_current_time_ = true;
    if (birth_model_) {
      auto births = birth_model_(to, 0.0);
      density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
    }
    return;
  }

  const double dt = to.secondsSince(current_time_);
  if (dt <= 0.0) {
    current_time_ = to;
    return;
  }

  for (auto& c : density_.ppp) {
    c.weight *= cfg_.survival_probability;
    Track t = toTrack(c, current_time_);
    estimator_.predict(t, to);
    fromTrack(c, t);
  }

  for (auto& h : density_.mbm) {
    for (auto& b : h.bernoullis) {
      b.existence_probability *= cfg_.survival_probability;
      Track t = toTrack(b);
      estimator_.predict(t, to);
      fromTrack(b, t);
    }
  }

  if (birth_model_) {
    auto births = birth_model_(to, dt);
    density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
  }

  current_time_ = to;
}

// Build one Gaussian "new-target Bernoulli" per measurement by folding
// the current PPP through the measurement likelihood (§3.2 of
// docs/algorithms/pmbm-design.md).
//
// For each measurement z_l:
//   ρ_target_l = Σ_i p_D · w_i^u · ℓ(z_l | c_i)
//   ρ_total_l  = ρ_target_l + λ_FA(z_l)
//   r_new_l    = ρ_target_l / ρ_total_l            (existence | detection)
//   f_new_l(x) = Σ_i p_D · w_i^u · ℓ(z_l | c_i) · g(x | c_i, z_l) / ρ_target_l
//
// Phase 1 collapses the posterior mixture {g(x|c_i, z_l)} to a
// moment-matched single Gaussian. Tractable, standard, and what the
// García-Fernández/Williams Phase 1 baseline does.
std::vector<PmbmTracker::NewTargetCandidate>
PmbmTracker::buildNewTargetCandidates(
    const std::vector<Measurement>& scan) const {
  std::vector<NewTargetCandidate> out;
  out.reserve(scan.size());

  for (const auto& z : scan) {
    NewTargetCandidate cand;

    // Per-sensor (P_D, λ_C) when a detection model is wired; Config
    // fallback otherwise. Matches MhtTracker's per-measurement
    // paramsFor lookup so PMBM is on the same footing for the bench A/B.
    const double pD_z = detection_model_
        ? detection_model_->paramsFor(z).probability_of_detection
        : cfg_.probability_of_detection;
    const double lambda_z = detection_model_
        ? detection_model_->paramsFor(z).clutter_intensity
        : cfg_.clutter_intensity;

    if (density_.ppp.empty()) {
      cand.rho_target = 0.0;
      cand.rho_total = lambda_z;
      out.push_back(std::move(cand));
      continue;
    }

    std::vector<double> log_weights;
    std::vector<Track> updated_tracks;
    log_weights.reserve(density_.ppp.size());
    updated_tracks.reserve(density_.ppp.size());

    const double log_pD = std::log(pD_z);

    for (const auto& c : density_.ppp) {
      if (c.weight <= 0.0) continue;
      Track t = toTrack(c, current_time_);
      const double log_lik = estimator_.logLikelihood(t, z);
      Track t_upd = t;
      estimator_.update(t_upd, z);
      log_weights.push_back(std::log(c.weight) + log_pD + log_lik);
      updated_tracks.push_back(std::move(t_upd));
    }

    const double log_rho_target = logSumExp(log_weights);
    cand.rho_target = std::exp(log_rho_target);
    cand.rho_total = cand.rho_target + lambda_z;

    if (cand.rho_target > 0.0 && !updated_tracks.empty()) {
      // Moment-match the posterior mixture to a single Gaussian.
      const auto n_state = updated_tracks.front().state.size();
      Eigen::VectorXd mean = Eigen::VectorXd::Zero(n_state);
      double wsum = 0.0;
      for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        mean += w * updated_tracks[i].state;
        wsum += w;
      }
      if (wsum > 0.0) mean /= wsum;
      Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n_state, n_state);
      for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        const Eigen::VectorXd d = updated_tracks[i].state - mean;
        cov += w * (updated_tracks[i].covariance + d * d.transpose());
      }
      if (wsum > 0.0) cov /= wsum;
      cand.mean = std::move(mean);
      cand.covariance = std::move(cov);

      // Per-mode moment-matched IMM mixture across all post-update
      // components (Phase 2). Each component has the same mode count
      // K (an IMM invariant); the merged mixture's per-mode mean and
      // covariance are weighted by w_i · μ_i[j], with merged mode
      // probabilities μ[j] = (Σ_i w_i · μ_i[j]) / Σ_i w_i.
      //
      // Drops components whose imm_* are empty (single-Gaussian PPP)
      // — they contributed via the moment-matched (mean, cov) above
      // but can't inform the IMM ensemble. The IMM-only-PPP common
      // case (every PPP component carries IMM, via the measurement-
      // driven birth path that calls estimator.initiate) sees every
      // component contribute, with the dominant-component case as the
      // single-contributor degenerate.
      int K = 0;
      int n_imm_state = 0;
      for (const auto& t : updated_tracks) {
        if (t.imm_means.cols() > 0) {
          K = static_cast<int>(t.imm_means.cols());
          n_imm_state = static_cast<int>(t.imm_means.rows());
          break;
        }
      }
      if (K > 0) {
        Eigen::VectorXd modep_acc = Eigen::VectorXd::Zero(K);
        std::vector<Eigen::VectorXd> mode_mean_acc(K, Eigen::VectorXd::Zero(n_imm_state));
        std::vector<double> mode_w_acc(K, 0.0);
        double wsum_imm = 0.0;
        for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
          const auto& t = updated_tracks[i];
          if (t.imm_means.cols() != K) continue;
          const double w = std::exp(log_weights[i] - log_rho_target);
          wsum_imm += w;
          for (int j = 0; j < K; ++j) {
            const double mw = w * t.imm_mode_probabilities(j);
            modep_acc(j) += mw;
            mode_mean_acc[j] += mw * t.imm_means.col(j);
            mode_w_acc[j] += mw;
          }
        }
        if (wsum_imm > 0.0) {
          Eigen::MatrixXd merged_means(n_imm_state, K);
          std::vector<Eigen::MatrixXd> merged_covs(
              K, Eigen::MatrixXd::Zero(n_imm_state, n_imm_state));
          Eigen::VectorXd merged_modep = modep_acc / wsum_imm;
          for (int j = 0; j < K; ++j) {
            if (mode_w_acc[j] > 0.0) {
              merged_means.col(j) = mode_mean_acc[j] / mode_w_acc[j];
            } else {
              merged_means.col(j).setZero();
            }
          }
          for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
            const auto& t = updated_tracks[i];
            if (t.imm_means.cols() != K) continue;
            const double w = std::exp(log_weights[i] - log_rho_target);
            for (int j = 0; j < K; ++j) {
              const double mw = w * t.imm_mode_probabilities(j);
              if (mw <= 0.0) continue;
              const Eigen::VectorXd d =
                  t.imm_means.col(j) - merged_means.col(j);
              merged_covs[j] +=
                  mw * (t.imm_covariances[j] + d * d.transpose());
            }
          }
          for (int j = 0; j < K; ++j) {
            if (mode_w_acc[j] > 0.0) merged_covs[j] /= mode_w_acc[j];
          }
          cand.imm_means = std::move(merged_means);
          cand.imm_covariances = std::move(merged_covs);
          cand.imm_mode_probabilities = std::move(merged_modep);
        }
      }
    } else {
      // No information from PPP; leave mean/cov empty — the candidate
      // contributes only clutter mass and would never be picked by an
      // assignment that produces a real Bernoulli (existence ≈ 0).
      cand.mean = Eigen::VectorXd::Zero(0);
      cand.covariance = Eigen::MatrixXd::Zero(0, 0);
    }

    out.push_back(std::move(cand));
  }

  return out;
}

void PmbmTracker::enumerateChildren(
    const GlobalHypothesis& parent,
    const std::vector<Measurement>& scan,
    const std::vector<NewTargetCandidate>& nts,
    std::vector<GlobalHypothesis>& out) {
  const int n = static_cast<int>(parent.bernoullis.size());
  const int m = static_cast<int>(scan.size());

  // Source-aware misdetection: collect this scan's source_ids once.
  // Bernoullis whose contribution-history sources are entirely absent
  // from this set get a no-op misdetection (state and r unchanged) —
  // see Config::source_aware_misdetection.
  std::set<std::string> scan_sources;
  if (cfg_.source_aware_misdetection) {
    for (const auto& z : scan) scan_sources.insert(z.source_id);
  }
  auto should_misdetect = [&](BernoulliId id) {
    if (!cfg_.source_aware_misdetection) return true;
    auto it = contribution_history_.find(id);
    if (it == contribution_history_.end() || it->second.empty()) {
      return true;  // no prior history; treat as observable
    }
    for (const auto& touch : it->second) {
      if (scan_sources.count(touch.source_id)) return true;
    }
    return false;
  };

  // Idle-decay factor for the no-information branch (source-aware skip
  // OR out-of-any-sensor-coverage). Uses the Bernoulli's most-recent
  // contribution-touch time as "last observed". last_update can't be
  // used directly because the estimator's predict advances it to the
  // current filter time even when no measurement was applied.
  // Returns 1 (no decay) when the knob is disabled, no prior touch
  // history exists, or no time has elapsed.
  const double scan_time_sec =
      scan.empty() ? current_time_.seconds() : scan.front().time.seconds();
  auto idle_decay_for = [&](const Bernoulli& b) {
    if (cfg_.idle_halflife_sec <= 0.0) return 1.0;
    auto it = contribution_history_.find(b.id);
    if (it == contribution_history_.end() || it->second.empty()) return 1.0;
    double last_touch_s = -std::numeric_limits<double>::infinity();
    for (const auto& touch : it->second) {
      const double ts = touch.time.seconds();
      if (ts > last_touch_s) last_touch_s = ts;
    }
    const double dt = scan_time_sec - last_touch_s;
    if (dt <= 0.0) return 1.0;
    return std::exp(-std::log(2.0) * dt / cfg_.idle_halflife_sec);
  };

  // Per-measurement P_D under the detection model (cost-matrix and
  // detection log-weight). Config fallback when no model wired.
  std::vector<double> pD_l(m, cfg_.probability_of_detection);
  if (detection_model_) {
    for (int l = 0; l < m; ++l) {
      pD_l[l] = detection_model_->paramsFor(scan[l]).probability_of_detection;
    }
  }
  // Per-Bernoulli effective miss P_D under the detection model.
  // Aggregate across scan sensors via "1 − Π (1 − p_D_s · cov_s)" =
  // probability that at least one in-coverage scan sensor would have
  // detected this Bernoulli. Coverage = missDetectionProbability !=
  // 0 (the detection model returns 0 when the Bernoulli is outside
  // the sensor's max_range / sector). Config fallback when no model.
  auto compute_miss_pD = [&](const Bernoulli& b) {
    if (!detection_model_) return cfg_.probability_of_detection;
    if (b.mean.size() < 2) return cfg_.probability_of_detection;
    const Eigen::Vector2d b_xy = b.mean.head<2>();
    double survive = 1.0;
    bool any_coverage = false;
    for (const auto& z : scan) {
      const double pD_s = detection_model_->missDetectionProbability(
          z.sensor, z.model, b_xy, z.sensor_position_enu, z.source_id);
      if (pD_s > 0.0) {
        any_coverage = true;
        survive *= (1.0 - pD_s);
      }
    }
    return any_coverage ? (1.0 - survive) : 0.0;
  };

  // Edge case: empty scan. Only one child = parent with all-miss updates.
  // Misdetection: r ← (1 − p_D)·r / (1 − r + (1 − p_D)·r), state unchanged.
  // Hypothesis weight gains a Σ log(1 − r·p_D) shift (relative ordering
  // among parents preserved; absolute log_weight tracked for pruning).
  if (m == 0) {
    GlobalHypothesis child;
    child.weight = parent.weight;
    child.log_weight = parent.log_weight;
    child.bernoullis.reserve(parent.bernoullis.size());
    for (const auto& b : parent.bernoullis) {
      Bernoulli updated = b;
      if (!should_misdetect(b.id)) {
        updated.existence_probability *= idle_decay_for(b);
        appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                              current_time_);
        child.bernoullis.push_back(std::move(updated));
        continue;
      }
      const double r = b.existence_probability;
      const double pD = compute_miss_pD(b);
      if (pD <= 0.0) {  // out of any sensor's coverage; no penalty
        updated.existence_probability *= idle_decay_for(b);
        appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                              current_time_);
        child.bernoullis.push_back(std::move(updated));
        continue;
      }
      const double miss_norm = 1.0 - r * pD;
      updated.existence_probability =
          (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
      child.log_weight += std::log(std::max(miss_norm, 1e-300));
      appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                            current_time_);
      child.bernoullis.push_back(std::move(updated));
    }
    out.push_back(std::move(child));
    return;
  }

  // Build cost matrix C: (n + m) × m.
  // - C[i, l]      = −log(r_i · ℓ_{i,l})            i = 0..n−1  (Bernoulli ↔ z_l)
  // - C[n+l', l]   = −log(ρ_total_l) iff l == l'                (new target absorbs z_l)
  //                  +∞ otherwise
  //
  // We also pre-compute the per-cell updated Bernoulli (state, cov) and
  // the per-cell log-likelihood, so the cheap K-best path doesn't redo
  // estimator.update.
  Eigen::MatrixXd C =
      Eigen::MatrixXd::Constant(n + m, m, kInf);
  std::vector<std::vector<double>> log_lik(n, std::vector<double>(m, -kInf));
  std::vector<std::vector<Bernoulli>> updated(
      n, std::vector<Bernoulli>(m));

  for (int i = 0; i < n; ++i) {
    const Bernoulli& b = parent.bernoullis[i];
    if (b.existence_probability <= 0.0) continue;
    Track t = toTrack(b);
    for (int l = 0; l < m; ++l) {
      const double ll = estimator_.logLikelihood(t, scan[l]);
      log_lik[i][l] = ll;
      Track t_upd = t;
      estimator_.update(t_upd, scan[l]);
      Bernoulli upd = b;
      fromTrack(upd, t_upd);
      upd.existence_probability = 1.0;  // existence given detection
      updated[i][l] = std::move(upd);
      const double r = b.existence_probability;
      const double cost_val = -(std::log(r) + ll);
      C(i, l) = std::isfinite(cost_val) ? cost_val : kInf;
    }
  }

  // New-target rows: diagonal block.
  for (int l = 0; l < m; ++l) {
    const double rho = nts[l].rho_total;
    if (rho > 0.0) {
      C(n + l, l) = -std::log(rho);
    }
  }

  // Murty K-best on the cost matrix.
  const KBestResult kb =
      murtyKBest(C, std::max(1, cfg_.k_best_per_hypothesis));

  for (std::size_t k = 0; k < kb.assignments.size(); ++k) {
    const auto& asn = kb.assignments[k];
    // Validity check: any assignment cell that hit the BIG_M padding
    // (cost ≥ kInf marker; in practice, any +∞ in the original matrix)
    // would yield an invalid child. Skip.
    bool feasible = true;
    for (int row = 0; row < static_cast<int>(asn.size()); ++row) {
      const int col = asn[row];
      if (col < 0) continue;  // unassigned row, no contribution
      if (!std::isfinite(C(row, col))) {
        feasible = false;
        break;
      }
    }
    if (!feasible) continue;

    GlobalHypothesis child;
    child.bernoullis.reserve(n + m);  // upper bound
    child.log_weight = parent.log_weight;

    // First: which Bernoullis got an assignment? Mark for misdetection
    // pass.
    std::vector<int> bernoulli_to_meas(n, -1);
    for (int row = 0; row < n; ++row) {
      const int col = asn[row];
      if (col >= 0 && col < m && std::isfinite(C(row, col))) {
        bernoulli_to_meas[row] = col;
      }
    }

    // Detected Bernoullis.
    for (int i = 0; i < n; ++i) {
      const Bernoulli& b = parent.bernoullis[i];
      if (bernoulli_to_meas[i] >= 0) {
        const int l = bernoulli_to_meas[i];
        Bernoulli det = updated[i][l];
        det.last_update = scan[l].time;
        appendTrajectoryPoint(det, cfg_.trajectory_window_scans,
                              scan[l].time);
        child.bernoullis.push_back(std::move(det));
        // Full per-Bernoulli detection contribution to log-weight:
        // log(r · p_D · ℓ).  P_D is the assigned measurement's
        // per-(sensor, source) P_D under the detection model.
        child.log_weight +=
            std::log(b.existence_probability) + std::log(pD_l[l]) +
            log_lik[i][l];
      } else {
        // Misdetection: r ← (1 − p_D) · r / (1 − r·p_D), state unchanged.
        // Skip the recursion entirely when source-aware misdetection
        // says no sensor in this scan could have observed this
        // Bernoulli (e.g. AIS broadcast from a different vessel) —
        // but still apply the idle-decay (knob: idle_halflife_sec) so
        // ghost Bernoullis whose target has stopped reporting do
        // eventually fall below r_min.
        Bernoulli miss = b;
        if (!should_misdetect(b.id)) {
          miss.existence_probability *= idle_decay_for(b);
          appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                current_time_);
          child.bernoullis.push_back(std::move(miss));
          continue;
        }
        const double r = b.existence_probability;
        const double pD = compute_miss_pD(b);
        if (pD <= 0.0) {  // out of any sensor's coverage; no penalty
          miss.existence_probability *= idle_decay_for(b);
          appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                current_time_);
          child.bernoullis.push_back(std::move(miss));
          continue;
        }
        const double miss_norm = 1.0 - r * pD;
        miss.existence_probability =
            (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
        // Misdetection contribution: log(1 − r · p_D).
        child.log_weight += std::log(std::max(miss_norm, 1e-300));
        appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                              current_time_);
        child.bernoullis.push_back(std::move(miss));
      }
    }

    // Measurements claimed by new-target rows → new Bernoullis.
    for (int row = n; row < n + m; ++row) {
      const int col = asn[row];
      if (col < 0 || col >= m) continue;
      if (!std::isfinite(C(row, col))) continue;
      const int l = col;
      const auto& nt = nts[l];
      const double r_new =
          (nt.rho_total > 0.0) ? (nt.rho_target / nt.rho_total) : 0.0;
      // Phantom-birth gate (Config::min_new_bernoulli_existence). The
      // measurement still consumes ρ_total mass (assignment stays
      // balanced) but no near-zero-r Bernoulli is materialised.
      if (r_new < cfg_.min_new_bernoulli_existence) {
        child.log_weight += std::log(std::max(nt.rho_total, 1e-300));
        continue;
      }
      Bernoulli nb;
      nb.id = next_bernoulli_id_++;
      nb.existence_probability = r_new;
      nb.mean = nt.mean;
      nb.covariance = nt.covariance;
      nb.imm_means = nt.imm_means;
      nb.imm_covariances = nt.imm_covariances;
      nb.imm_mode_probabilities = nt.imm_mode_probabilities;
      nb.last_update = scan[l].time;
      nb.birth_time = scan[l].time;
      appendTrajectoryPoint(nb, cfg_.trajectory_window_scans,
                            scan[l].time);
      child.bernoullis.push_back(std::move(nb));
      // New-target contribution: log(ρ_total).
      child.log_weight += std::log(nt.rho_total);
    }

    out.push_back(std::move(child));
  }
}

void PmbmTracker::pruneAndNormalise() {
  // Normalise mixture weights from log_weight (numerically stable).
  if (density_.mbm.empty()) return;

  std::vector<double> lws;
  lws.reserve(density_.mbm.size());
  for (const auto& h : density_.mbm) lws.push_back(h.log_weight);
  const double lse = logSumExp(lws);
  if (std::isfinite(lse)) {
    for (auto& h : density_.mbm) {
      h.weight = std::exp(h.log_weight - lse);
    }
  } else {
    // All -inf: degenerate, distribute uniformly to keep the filter alive.
    const double u = 1.0 / static_cast<double>(density_.mbm.size());
    for (auto& h : density_.mbm) h.weight = u;
  }

  // Drop hypotheses below the weight floor.
  density_.mbm.erase(
      std::remove_if(density_.mbm.begin(), density_.mbm.end(),
                     [&](const GlobalHypothesis& h) {
                       return h.weight < cfg_.hypothesis_weight_min;
                     }),
      density_.mbm.end());

  // Cap mixture size at max_global_hypotheses (keep top-weighted).
  if (density_.mbm.size() > cfg_.max_global_hypotheses) {
    std::partial_sort(
        density_.mbm.begin(),
        density_.mbm.begin() + cfg_.max_global_hypotheses,
        density_.mbm.end(),
        [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
          return a.weight > b.weight;
        });
    density_.mbm.resize(cfg_.max_global_hypotheses);
  }

  // Renormalise after pruning.
  double s = 0.0;
  for (const auto& h : density_.mbm) s += h.weight;
  if (s > 0.0) {
    for (auto& h : density_.mbm) h.weight /= s;
  } else if (!density_.mbm.empty()) {
    const double u = 1.0 / static_cast<double>(density_.mbm.size());
    for (auto& h : density_.mbm) h.weight = u;
  }

  // Within-hypothesis Bernoulli pruning by r_min.
  for (auto& h : density_.mbm) {
    h.bernoullis.erase(
        std::remove_if(h.bernoullis.begin(), h.bernoullis.end(),
                       [&](const Bernoulli& b) {
                         return b.existence_probability < cfg_.r_min;
                       }),
        h.bernoullis.end());
  }

  // PPP weight pruning.
  density_.ppp.erase(
      std::remove_if(density_.ppp.begin(), density_.ppp.end(),
                     [&](const PoissonComponent& c) {
                       return c.weight < cfg_.weight_min;
                     }),
      density_.ppp.end());

  // PPP count cap. Without it, every clutter measurement under
  // measurement-driven birth seeds a component that lives for several
  // scans, growing the PPP unbounded on long replays. Keep top-weighted.
  if (density_.ppp.size() > cfg_.max_ppp_components) {
    std::partial_sort(
        density_.ppp.begin(),
        density_.ppp.begin() + cfg_.max_ppp_components,
        density_.ppp.end(),
        [](const PoissonComponent& a, const PoissonComponent& b) {
          return a.weight > b.weight;
        });
    density_.ppp.resize(cfg_.max_ppp_components);
  }
}

void PmbmTracker::processBatch(const std::vector<Measurement>& scan_in) {
  // Apply per-sensor registration bias correction (item 9) + Schmidt-KF
  // R-inflation before any predict / update. Null provider =
  // bit-identical to legacy.
  std::vector<Measurement> scan_corrected;
  if (bias_provider_ != nullptr) {
    scan_corrected.reserve(scan_in.size());
    for (const auto& z : scan_in) {
      scan_corrected.push_back(applyBiasCorrection(z, bias_provider_));
    }
  }
  const std::vector<Measurement>& scan =
      bias_provider_ != nullptr ? scan_corrected : scan_in;

  // Predict to the latest measurement time in the scan (matches the
  // MhtTracker convention). Empty scan still advances the filter if a
  // future predict() is wired separately; here it's just a no-op for
  // both predict and update.
  if (!scan.empty()) {
    Timestamp t_max = scan.front().time;
    for (const auto& z : scan) {
      if (z.time.seconds() > t_max.seconds()) t_max = z.time;
    }
    predict(t_max);
  }

  if (scan.empty()) {
    // Apply misdetection to all Bernoullis (no measurements means every
    // Bernoulli was missed). Walks through enumerateChildren with m=0
    // for each parent to keep one code path.
    std::vector<GlobalHypothesis> children;
    children.reserve(density_.mbm.size());
    const std::vector<NewTargetCandidate> empty_nts;
    if (density_.mbm.empty()) {
      // No prior hypotheses: seed the MBM with the empty hypothesis so
      // future Bernoullis (from PPP births) have somewhere to land.
      GlobalHypothesis seed;
      seed.weight = 1.0;
      seed.log_weight = 0.0;
      density_.mbm.push_back(std::move(seed));
    } else {
      for (const auto& p : density_.mbm) {
        enumerateChildren(p, scan, empty_nts, children);
      }
      density_.mbm = std::move(children);
    }
    pruneAndNormalise();
    return;
  }

  // Seed MBM with the empty hypothesis on first update (no prior
  // detected targets) so the per-parent loop has something to extend.
  if (density_.mbm.empty()) {
    GlobalHypothesis seed;
    seed.weight = 1.0;
    seed.log_weight = 0.0;
    density_.mbm.push_back(std::move(seed));
  }

  // Measurement-driven birth: append one PPP component per initiable
  // measurement BEFORE building new-target candidates, so a fresh
  // measurement that no Bernoulli claims has somewhere to live (and
  // ρ_target is dominated by the just-birthed component centred on the
  // measurement). Bearing-only measurements cannot initiate a state
  // and are skipped, matching MhtTracker birth-on-measurement rules.
  if (cfg_.measurement_driven_birth) {
    for (const auto& z : scan) {
      if (!canInitiateTrack(z.model)) continue;

      // Smart birth: skip when an existing high-r Bernoulli already
      // gates to this measurement. Walks every hypothesis's Bernoullis
      // and uses the estimator's gate (Mahalanobis for single-Gaussian,
      // any-mode for IMM). The first match wins — cheap and sufficient
      // because the dominant hypothesis carries the highest-r Bernoulli
      // for each id in practice.
      bool claimed_by_existing = false;
      if (cfg_.smart_birth_skip_existing) {
        for (const auto& h : density_.mbm) {
          for (const auto& b : h.bernoullis) {
            if (b.existence_probability < cfg_.smart_birth_skip_r_min) continue;
            Track t = toTrack(b);
            if (estimator_.gate(t, z, cfg_.smart_birth_skip_gate)) {
              claimed_by_existing = true;
              break;
            }
          }
          if (claimed_by_existing) break;
        }
      }
      if (claimed_by_existing) continue;

      // PPP coverage gate. Σ w_c · ℓ(z | c) ≥ threshold ⇒ the
      // existing PPP intensity already covers this measurement;
      // skip the redundant injection that would otherwise inflate
      // the local ρ_target on the next scan.
      if (cfg_.smart_birth_skip_existing_ppp &&
          cfg_.smart_birth_skip_existing_ppp_threshold > 0.0) {
        double coverage = 0.0;
        for (const auto& cprev : density_.ppp) {
          if (cprev.weight <= 0.0) continue;
          Track tprev = toTrack(cprev, current_time_);
          coverage += cprev.weight *
                      std::exp(estimator_.logLikelihood(tprev, z));
          if (coverage >= cfg_.smart_birth_skip_existing_ppp_threshold) break;
        }
        if (coverage >= cfg_.smart_birth_skip_existing_ppp_threshold) continue;
      }

      Track t = estimator_.initiate(z);
      PoissonComponent c;
      c.weight = cfg_.birth_weight_per_measurement;
      fromTrack(c, t);
      density_.ppp.push_back(std::move(c));
    }
  }

  // Per-measurement new-target Bernoulli candidates from PPP.
  const auto nts = buildNewTargetCandidates(scan);

  // PPP undetected-mass decay (§3.3). Uniform p_D for Phase 1.
  for (auto& c : density_.ppp) {
    c.weight *= (1.0 - cfg_.probability_of_detection);
  }

  // Per-parent K-best enumeration.
  std::vector<GlobalHypothesis> children;
  children.reserve(density_.mbm.size() *
                   static_cast<std::size_t>(cfg_.k_best_per_hypothesis));
  for (const auto& p : density_.mbm) {
    enumerateChildren(p, scan, nts, children);
  }

  density_.mbm = std::move(children);

  // Within-hypothesis Bernoulli merge: fold near-duplicate Bernoullis
  // (Bhattacharyya position-block distance < threshold) into a single
  // Bernoulli keeping the older id.
  if (cfg_.bhattacharyya_merge_threshold > 0.0) {
    for (auto& h : density_.mbm) {
      mergeBernoulliDuplicates(h);
    }
  }

  pruneAndNormalise();

  // Source-touch history. Walk the highest-weighted hypothesis; for
  // each Bernoulli whose last_update equals this scan's time it was
  // detected, find the nearest measurement in this scan, and append
  // a SourceTouch under that Bernoulli's id. Same window-prune /
  // alive-id-keep semantics as MhtTracker::contribution_history_.
  if (!density_.mbm.empty() && !scan.empty()) {
    const auto& dom = *std::max_element(
        density_.mbm.begin(), density_.mbm.end(),
        [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
          return a.weight < b.weight;
        });
    for (const auto& b : dom.bernoullis) {
      // Find the scan measurement at b.last_update == scan time AND
      // nearest to b.mean in position. Detected Bernoullis carry their
      // measurement's timestamp; misdetected ones inherit the parent
      // pre-predict timestamp, so an exact-match filter cleanly
      // separates the two.
      double best_d = std::numeric_limits<double>::infinity();
      const Measurement* best = nullptr;
      for (const auto& z : scan) {
        if (!(z.time == b.last_update)) continue;
        // Use the measurement's ENU position (Position2D models) or
        // the sensor position fallback (range/bearing-only).
        Eigen::Vector2d z_xy = (z.value.size() >= 2)
            ? Eigen::Vector2d(z.value(0), z.value(1))
            : z.sensor_position_enu;
        const Eigen::Vector2d b_xy = b.mean.head<2>();
        const double d = (z_xy - b_xy).squaredNorm();
        if (d < best_d) { best_d = d; best = &z; }
      }
      if (!best) continue;
      Track::SourceTouch touch;
      touch.sensor = best->sensor;
      touch.source_id = best->source_id;
      touch.time = best->time;
      fillSourceTouchEnu(touch, *best);
      touch.sensor_position_enu = best->sensor_position_enu;
      touch.own_position_std_m = best->sensor_position_std_m;
      touch.covariance_is_default = best->covariance_is_default;
      auto& history = contribution_history_[b.id];
      history.push_back(std::move(touch));
      const std::int64_t window_ns =
          static_cast<std::int64_t>(kContributionWindowSec * 1e9);
      const std::int64_t now_ns = scan.front().time.nanos();
      auto first_keep = std::find_if(
          history.begin(), history.end(),
          [&](const Track::SourceTouch& st) {
            return (now_ns - st.time.nanos()) <= window_ns;
          });
      history.erase(history.begin(), first_keep);
    }
    // Drop history entries whose Bernoulli id is no longer present in
    // any hypothesis (the Bernoulli was pruned).
    std::set<BernoulliId> alive;
    for (const auto& h : density_.mbm) {
      for (const auto& b : h.bernoullis) alive.insert(b.id);
    }
    for (auto it = contribution_history_.begin();
         it != contribution_history_.end();) {
      if (alive.count(it->first) == 0) it = contribution_history_.erase(it);
      else ++it;
    }
  }

  aggregated_tracks_dirty_ = true;
  // Phase 4(B) push-based lifecycle events. Diffs against the prior-
  // scan emitted set. No-op when no sink is wired.
  if (track_sink_ != nullptr && !scan.empty()) {
    firePmbmLifecycleEvents(scan.front().time);
  }
}

namespace {

// Bhattacharyya distance between two 2-D Gaussians on the position
// block (first two state components). Standard form:
//   d_B = 1/8 · (m1-m2)' P^-1 (m1-m2) + 1/2 · ln(det(P) / sqrt(det(P1)·det(P2)))
// with P = (P1 + P2) / 2. Returns +∞ for degenerate covariances.
double bhattacharyya2D(const Eigen::VectorXd& mean_a,
                       const Eigen::MatrixXd& cov_a,
                       const Eigen::VectorXd& mean_b,
                       const Eigen::MatrixXd& cov_b) {
  if (mean_a.size() < 2 || mean_b.size() < 2) return kInf;
  if (cov_a.rows() < 2 || cov_b.rows() < 2) return kInf;
  const Eigen::Vector2d d = mean_a.head<2>() - mean_b.head<2>();
  const Eigen::Matrix2d Pa = cov_a.topLeftCorner<2, 2>();
  const Eigen::Matrix2d Pb = cov_b.topLeftCorner<2, 2>();
  const Eigen::Matrix2d P = 0.5 * (Pa + Pb);
  const double det_P = P.determinant();
  const double det_Pa = Pa.determinant();
  const double det_Pb = Pb.determinant();
  if (det_P <= 0.0 || det_Pa <= 0.0 || det_Pb <= 0.0) return kInf;
  const double mahal = d.transpose() * P.inverse() * d;
  return 0.125 * mahal +
         0.5 * std::log(det_P / std::sqrt(det_Pa * det_Pb));
}

}  // namespace

void PmbmTracker::mergeBernoulliDuplicates(GlobalHypothesis& h) const {
  // Sort by id ascending so the survivor (kept) carries the older id.
  std::sort(h.bernoullis.begin(), h.bernoullis.end(),
            [](const Bernoulli& a, const Bernoulli& b) {
              return a.id < b.id;
            });
  std::vector<bool> dead(h.bernoullis.size(), false);
  for (std::size_t i = 0; i < h.bernoullis.size(); ++i) {
    if (dead[i]) continue;
    for (std::size_t j = i + 1; j < h.bernoullis.size(); ++j) {
      if (dead[j]) continue;
      const double dB = bhattacharyya2D(
          h.bernoullis[i].mean, h.bernoullis[i].covariance,
          h.bernoullis[j].mean, h.bernoullis[j].covariance);
      if (dB > cfg_.bhattacharyya_merge_threshold) continue;

      // Merge j INTO i. Independent-existence fold:
      //   r_merged = 1 - (1 - r_i)(1 - r_j)
      // Mean / cov: r-weighted moment match.
      Bernoulli& a = h.bernoullis[i];
      const Bernoulli& b = h.bernoullis[j];
      const double r_a = a.existence_probability;
      const double r_b = b.existence_probability;
      const double r_m = 1.0 - (1.0 - r_a) * (1.0 - r_b);
      const double wa = r_a, wb = r_b;
      const double ws = wa + wb;
      if (ws > 0.0 && a.mean.size() == b.mean.size()) {
        const Eigen::VectorXd new_mean = (wa * a.mean + wb * b.mean) / ws;
        const Eigen::VectorXd d_a = a.mean - new_mean;
        const Eigen::VectorXd d_b = b.mean - new_mean;
        Eigen::MatrixXd new_cov =
            (wa * (a.covariance + d_a * d_a.transpose()) +
             wb * (b.covariance + d_b * d_b.transpose())) / ws;
        a.mean = new_mean;
        a.covariance = std::move(new_cov);
      }
      a.existence_probability = r_m;
      if (b.last_update.seconds() > a.last_update.seconds()) {
        a.last_update = b.last_update;
      }
      dead[j] = true;
    }
  }
  // Compact.
  std::vector<Bernoulli> kept;
  kept.reserve(h.bernoullis.size());
  for (std::size_t k = 0; k < h.bernoullis.size(); ++k) {
    if (!dead[k]) kept.push_back(std::move(h.bernoullis[k]));
  }
  h.bernoullis = std::move(kept);
}

const std::vector<Track>& PmbmTracker::tracks() const {
  if (aggregated_tracks_dirty_) {
    refreshAggregatedTracks();
    aggregated_tracks_dirty_ = false;
  }
  return aggregated_tracks_;
}

void PmbmTracker::firePmbmLifecycleEvents(Timestamp event_time) {
  if (track_sink_ == nullptr) return;
  // Compute current emitted track set + statuses by id.
  std::map<std::uint64_t, TrackStatus> current;
  const auto& ts = tracks();
  for (const auto& t : ts) current[t.id.value] = t.status;

  // Initiated (new ids, status Tentative) and Confirmed (new ids
  // status Confirmed, or prior Tentative → Confirmed transition).
  for (const auto& [id, status] : current) {
    const TrackId tid{id};
    const TrackLifecycleEvent e{tid, event_time, status};
    auto pit = prev_emitted_statuses_.find(id);
    if (pit == prev_emitted_statuses_.end()) {
      // Newly emitted this scan.
      if (status == TrackStatus::Confirmed) {
        track_sink_->onTrackInitiated(e);  // existence preceded
                                            // confirmation in one scan
        track_sink_->onTrackConfirmed(e);
      } else {
        track_sink_->onTrackInitiated(e);
      }
    } else {
      // Existed last scan; check status transition.
      if (pit->second != TrackStatus::Confirmed &&
          status == TrackStatus::Confirmed) {
        track_sink_->onTrackConfirmed(e);
      }
    }
    // Every track present this scan emits onTrackUpdated.
    track_sink_->onTrackUpdated(e);
  }

  // Deleted (prior id absent now). Emit BEFORE updating the snapshot.
  // Status reported is the pre-deletion status.
  for (const auto& [id, status] : prev_emitted_statuses_) {
    if (current.find(id) != current.end()) continue;
    const TrackId tid{id};
    track_sink_->onTrackDeleted({tid, event_time, status});
  }

  prev_emitted_statuses_ = std::move(current);
}

std::vector<TrajectoryPoint> PmbmTracker::trajectoryFor(BernoulliId id) const {
  // Pick the highest-weight hypothesis containing this id. Trajectory
  // is per-Bernoulli (per-hypothesis), so the dominant interpretation
  // is the right one to expose (matches refreshAggregatedTracks'
  // emission of the dominant-weighted state).
  const GlobalHypothesis* best_h = nullptr;
  const Bernoulli* best_b = nullptr;
  for (const auto& h : density_.mbm) {
    for (const auto& b : h.bernoullis) {
      if (b.id != id) continue;
      if (best_h == nullptr || h.weight > best_h->weight) {
        best_h = &h;
        best_b = &b;
      }
    }
  }
  if (best_b == nullptr) return {};
  return best_b->trajectory;
}

void PmbmTracker::refreshAggregatedTracks() const {
  // Aggregate (w^j · r^{j,id}) across hypotheses for each Bernoulli id.
  // std::map ordered by id keeps the emission order deterministic across
  // runs — critical for the determinism contract and for diff-friendly
  // bench output.
  struct Acc {
    double mass{0.0};
    Eigen::VectorXd mean_acc;       // Σ w·r · μ
    Eigen::MatrixXd cov_acc;        // Σ w·r · (P + μμ')
    Timestamp last_update{};
  };
  std::map<BernoulliId, Acc> by_id;

  for (const auto& h : density_.mbm) {
    for (const auto& b : h.bernoullis) {
      const double m = h.weight * b.existence_probability;
      if (m <= 0.0) continue;
      Acc& a = by_id[b.id];
      if (a.mean_acc.size() == 0) {
        a.mean_acc = Eigen::VectorXd::Zero(b.mean.size());
        a.cov_acc = Eigen::MatrixXd::Zero(b.mean.size(), b.mean.size());
      }
      a.mass += m;
      a.mean_acc += m * b.mean;
      a.cov_acc += m * (b.covariance + b.mean * b.mean.transpose());
      if (b.last_update.seconds() > a.last_update.seconds()) {
        a.last_update = b.last_update;
      }
    }
  }

  aggregated_tracks_.clear();
  aggregated_tracks_.reserve(by_id.size());
  for (const auto& [id, a] : by_id) {
    if (a.mass < cfg_.output_existence_floor) continue;
    Track t;
    t.id.value = id;
    t.last_update = a.last_update;
    t.existence_probability = std::min(a.mass, 1.0);
    t.status = (a.mass >= cfg_.confirm_threshold) ? TrackStatus::Confirmed
                                                   : TrackStatus::Tentative;
    t.state = a.mean_acc / a.mass;
    // Moment-matched covariance: E[P + μμ'] − mean·mean'
    t.covariance = (a.cov_acc / a.mass) - (t.state * t.state.transpose());
    auto hit = contribution_history_.find(id);
    if (hit != contribution_history_.end()) {
      t.recent_contributions = hit->second;
    }
    aggregated_tracks_.push_back(std::move(t));
  }
}

}  // namespace navtracker::pmbm
