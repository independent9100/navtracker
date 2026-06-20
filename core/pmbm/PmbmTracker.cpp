#include "core/pmbm/PmbmTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Dense>

#include "core/association/Murty.hpp"

namespace navtracker::pmbm {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

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

    if (density_.ppp.empty()) {
      cand.rho_target = 0.0;
      cand.rho_total = cfg_.clutter_intensity;
      out.push_back(std::move(cand));
      continue;
    }

    std::vector<double> log_weights;
    std::vector<Eigen::VectorXd> means;
    std::vector<Eigen::MatrixXd> covs;
    log_weights.reserve(density_.ppp.size());
    means.reserve(density_.ppp.size());
    covs.reserve(density_.ppp.size());

    const double log_pD = std::log(cfg_.probability_of_detection);

    for (const auto& c : density_.ppp) {
      if (c.weight <= 0.0) continue;
      Track t = toTrack(c, current_time_);
      const double log_lik = estimator_.logLikelihood(t, z);
      Track t_upd = t;
      estimator_.update(t_upd, z);
      log_weights.push_back(std::log(c.weight) + log_pD + log_lik);
      means.push_back(t_upd.state);
      covs.push_back(t_upd.covariance);
    }

    const double log_rho_target = logSumExp(log_weights);
    cand.rho_target = std::exp(log_rho_target);
    cand.rho_total = cand.rho_target + cfg_.clutter_intensity;

    if (cand.rho_target > 0.0 && !means.empty()) {
      // Moment-match the posterior mixture to a single Gaussian.
      Eigen::VectorXd mean = Eigen::VectorXd::Zero(means.front().size());
      double wsum = 0.0;
      for (std::size_t i = 0; i < means.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        mean += w * means[i];
        wsum += w;
      }
      if (wsum > 0.0) mean /= wsum;
      Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(mean.size(), mean.size());
      for (std::size_t i = 0; i < means.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        const Eigen::VectorXd d = means[i] - mean;
        cov += w * (covs[i] + d * d.transpose());
      }
      if (wsum > 0.0) cov /= wsum;
      cand.mean = std::move(mean);
      cand.covariance = std::move(cov);
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
      const double r = b.existence_probability;
      const double pD = cfg_.probability_of_detection;
      const double miss_norm = 1.0 - r * pD;
      updated.existence_probability =
          (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
      child.log_weight += std::log(std::max(miss_norm, 1e-300));
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

  const double log_pD = std::log(cfg_.probability_of_detection);

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
        child.bernoullis.push_back(std::move(det));
        // Full per-Bernoulli detection contribution to log-weight:
        // log(r · p_D · ℓ).
        child.log_weight +=
            std::log(b.existence_probability) + log_pD + log_lik[i][l];
      } else {
        // Misdetection: r ← (1 − p_D) · r / (1 − r·p_D), state unchanged.
        Bernoulli miss = b;
        const double r = b.existence_probability;
        const double pD = cfg_.probability_of_detection;
        const double miss_norm = 1.0 - r * pD;
        miss.existence_probability =
            (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
        // Misdetection contribution: log(1 − r · p_D).
        child.log_weight += std::log(std::max(miss_norm, 1e-300));
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
      Bernoulli nb;
      nb.id = next_bernoulli_id_++;
      nb.existence_probability =
          (nt.rho_total > 0.0) ? (nt.rho_target / nt.rho_total) : 0.0;
      nb.mean = nt.mean;
      nb.covariance = nt.covariance;
      nb.last_update = scan[l].time;
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
}

void PmbmTracker::processBatch(const std::vector<Measurement>& scan) {
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
  pruneAndNormalise();
}

}  // namespace navtracker::pmbm
