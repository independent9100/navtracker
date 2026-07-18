#include "core/tracking/TrackTree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>

#include <Eigen/LU>

#include "core/association/Gating.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {

namespace {

// IPDA / VIMM existence-recursion helpers.
//
// Notation: r̄, v̄ are the post-predict (Markov) existence and
// visibility-given-exists. P_D is the per-sensor detection probability
// (or the cfg global for the miss branch). P_G is the gate probability
// mass. g(z) is the predicted measurement density at the gated z
// (exp of the estimator's log-likelihood). λ_C is the per-sensor
// clutter intensity in the measurement's natural units.
//
// All four updates collapse to the no-op identity (r,v) → (r,v) when
// the corresponding update flag is off — branch() guards the call.
// Without visibility, v is held at 1.0 so the IPDA-only formulas
// reduce to plain Musicki 1994.

struct ExistenceVis {
  double existence;
  double visibility_given_exists;
};

// dt-scaled Markov prediction. The persistence parameters are per-second
// rates; raising the chain to dt keeps the decay rate independent of the
// scan cadence (per-scan application at 16 Hz would decay 16× faster
// than the same parameters at the classical 1 Hz).
//
// Existence is an absorbing 2-state chain (no birth): r̄ = π_e^dt · r.
// Visibility is a full 2-state chain with stay a = π_vv and recovery
// b = π_hv; its one-step map v ↦ a·v + b·(1−v) = λ·v + b with
// λ = a − b has the closed-form dt-step
//   v̄ = v∞ + λ^dt (v − v∞),   v∞ = b / (1 − λ)
// (dt = 1 reproduces the one-step map exactly; dt = 0 is the identity).
// λ is clamped to [0, 1) — a "chain" with a < b oscillates per step,
// which has no physical reading for obscuration; we collapse it to its
// stationary value instead.
inline ExistenceVis predictExistence(double r, double v,
                                     double pi_e,
                                     double pi_vv, double pi_hv,
                                     bool use_visibility,
                                     double dt) {
  const double t = std::max(dt, 0.0);
  const double r_pred = std::pow(pi_e, t) * r;
  if (!use_visibility) return {r_pred, v};
  const double lam = std::clamp(pi_vv - pi_hv, 0.0,
                                1.0 - std::numeric_limits<double>::epsilon());
  const double v_inf = pi_hv / (1.0 - lam);
  const double v_pred = v_inf + std::pow(lam, t) * (v - v_inf);
  return {r_pred, std::clamp(v_pred, 0.0, 1.0)};
}

// Hit posterior on (existence, visibility) given measurement z.
// IPDA-only (use_visibility=false): r' = r̄·L / (1 − r̄ + r̄·L),
//   where L = P_D · g(z) / λ_C; v' carried through unchanged.
// VIMM (use_visibility=true): r' = r̄·v̄·P_D·g / (r̄·v̄·P_D·g + (1−r̄)·λ_C);
//   v' = 1.0 (a hit means the target was visible this scan).
inline ExistenceVis updateHit(double r_pred, double v_pred,
                              double p_d, double exp_log_lik,
                              double lambda_c,
                              bool use_visibility) {
  const double p_d_g = p_d * exp_log_lik;
  if (use_visibility) {
    const double num = r_pred * v_pred * p_d_g;
    const double den = num + (1.0 - r_pred) * lambda_c;
    const double r_post = (den > 0.0) ? (num / den) : r_pred;
    return {std::clamp(r_post, 0.0, 1.0), 1.0};
  }
  const double L = (lambda_c > 0.0) ? (p_d_g / lambda_c) : 0.0;
  const double den = 1.0 - r_pred + r_pred * L;
  const double r_post = (den > 0.0) ? (r_pred * L / den) : r_pred;
  return {std::clamp(r_post, 0.0, 1.0), v_pred};
}

// Miss posterior on (existence, visibility).
// IPDA-only: r' = r̄·(1 − P_D·P_G) / (1 − r̄·P_D·P_G); v' carried.
// VIMM: split between visible (1 − P_D·P_G) and hidden (1) channels →
//   r' = r̄·(1 − v̄·P_D·P_G) / (1 − r̄·v̄·P_D·P_G);
//   v' = v̄·(1 − P_D·P_G) / (1 − v̄·P_D·P_G).
// VIMM under v̄ ≪ 1 (already-obscured): r' ≈ r̄ — existence barely
// decays, the obscuration-friendly behaviour.
inline ExistenceVis updateMiss(double r_pred, double v_pred,
                               double p_d, double p_g,
                               bool use_visibility) {
  const double p_d_pg = p_d * p_g;
  if (use_visibility) {
    const double r_factor = 1.0 - v_pred * p_d_pg;
    const double den_r = 1.0 - r_pred * v_pred * p_d_pg;
    const double r_post = (den_r > 0.0) ? (r_pred * r_factor / den_r) : r_pred;
    const double den_v = 1.0 - v_pred * p_d_pg;
    const double v_post =
        (den_v > 0.0) ? (v_pred * (1.0 - p_d_pg) / den_v) : v_pred;
    return {std::clamp(r_post, 0.0, 1.0), std::clamp(v_post, 0.0, 1.0)};
  }
  const double L = 1.0 - p_d_pg;
  const double den = 1.0 - r_pred + r_pred * L;
  const double r_post = (den > 0.0) ? (r_pred * L / den) : r_pred;
  return {std::clamp(r_post, 0.0, 1.0), v_pred};
}

}  // namespace

TrackTree::TrackTree(TrackId external_id, const TrackTreeNode& root)
    : external_id_(external_id) {
  nodes_.push_back(root);
}

std::vector<std::size_t> TrackTree::leafIndices() const {
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].is_leaf) out.push_back(i);
  }
  return out;
}

std::size_t TrackTree::bestLeafIndex() const {
  std::size_t best = TrackTreeNode::kNoParent;
  double best_score = 0.0;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (!nodes_[i].is_leaf) continue;
    if (best == TrackTreeNode::kNoParent || nodes_[i].score > best_score) {
      best = i;
      best_score = nodes_[i].score;
    }
  }
  return best;
}

void TrackTree::branch(const IEstimator& estimator,
                       const std::vector<Measurement>& scan,
                       Timestamp scan_time,
                       const BranchParams& params) {
  std::vector<std::size_t> current_leaves = leafIndices();
  for (std::size_t leaf_idx : current_leaves) {
    Track tmp_predicted;
    tmp_predicted.state = nodes_[leaf_idx].state;
    tmp_predicted.covariance = nodes_[leaf_idx].covariance;
    tmp_predicted.imm_means = nodes_[leaf_idx].imm_means;
    tmp_predicted.imm_covariances = nodes_[leaf_idx].imm_covariances;
    tmp_predicted.imm_mode_probabilities =
        nodes_[leaf_idx].imm_mode_probabilities;
    tmp_predicted.last_update = nodes_[leaf_idx].time;
    estimator.predict(tmp_predicted, scan_time);

    // Elapsed time driving the dt-scaled lifecycle prediction (per-second
    // Markov rates; see BranchParams docs).
    const double dt = scan_time.secondsSince(nodes_[leaf_idx].time);

    {
      // Per-sensor coverage-conditioned miss penalty: missed by every
      // DISTINCT (sensor, model, source) that surveyed this scan instant,
      //   Δscore = Σ_s log(1 − P_D^s(x)),
      // evaluated at this leaf's predicted position. A sensor whose
      // coverage (range or azimuth sector) excludes the track
      // contributes P_D^s = 0 → log 1 = 0. The source_id is part of the
      // key so that two physical sensors sharing a SensorKind (EO + IR
      // cameras) each charge their own calibrated miss penalty.
      const Eigen::Vector2d track_pos(tmp_predicted.state(0),
                                      tmp_predicted.state(1));
      double log_miss = 0.0;
      std::vector<std::tuple<SensorKind, MeasurementModel, std::string>> seen;
      for (const Measurement& z : scan) {
        std::tuple<SensorKind, MeasurementModel, std::string> key{
            z.sensor, z.model, z.source_id};
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
        seen.push_back(std::move(key));
        const double p_d = params.detection_model->missDetectionProbability(
            z.sensor, z.model, track_pos, z.sensor_position_enu, z.source_id);
        log_miss += std::log(std::max(1.0 - p_d, 1e-12));
      }
      // Effective scan-level P_D for the IPDA miss recursion:
      // P(detected by at least one surveying sensor).
      const double p_d_eff = 1.0 - std::exp(log_miss);

      TrackTreeNode miss;
      miss.parent = leaf_idx;
      miss.scan_idx = nodes_[leaf_idx].scan_idx + 1;
      miss.state = tmp_predicted.state;
      miss.covariance = tmp_predicted.covariance;
      miss.imm_means = tmp_predicted.imm_means;
      miss.imm_covariances = tmp_predicted.imm_covariances;
      miss.imm_mode_probabilities = tmp_predicted.imm_mode_probabilities;
      miss.time = scan_time;
      miss.score = nodes_[leaf_idx].score + log_miss;
      miss.is_leaf = true;
      miss.is_hit = false;
      miss.scan_meas_idx = TrackTreeNode::kNoMeasurement;
      miss.last_position_anchor = nodes_[leaf_idx].last_position_anchor;
      // IPDA / VIMM existence on the miss branch (carried through when
      // update_existence is off).
      if (params.update_existence) {
        const ExistenceVis pred = predictExistence(
            nodes_[leaf_idx].existence_probability,
            nodes_[leaf_idx].visibility_given_exists,
            params.existence_persistence,
            params.visibility_persistence, params.visibility_recovery,
            params.update_visibility, dt);
        const ExistenceVis post = updateMiss(
            pred.existence, pred.visibility_given_exists,
            p_d_eff,
            params.gate_probability_mass, params.update_visibility);
        miss.existence_probability = post.existence;
        miss.visibility_given_exists = post.visibility_given_exists;
      } else {
        miss.existence_probability = nodes_[leaf_idx].existence_probability;
        miss.visibility_given_exists =
            nodes_[leaf_idx].visibility_given_exists;
      }
      // W5.5: deferred-commitment protection is heritable for exactly one
      // scan. The global solve sets is_protected on the previous scan's
      // top-K leaves, but branch() (which runs FIRST next scan) demotes them
      // to internal — so the child must carry the flag or the pruning passes
      // that honour it (pruneKLocal/mergeBranches/pruneNScan) never see a
      // protected leaf. Self-limits: MhtTracker clears + re-sets all flags
      // after each solve, so an inherited flag is dropped next scan unless
      // the child is itself re-selected as top-K.
      miss.is_protected = nodes_[leaf_idx].is_protected;
      nodes_.push_back(miss);
    }

    for (std::size_t mi = 0; mi < scan.size(); ++mi) {
      const Measurement& z = scan[mi];
      // Per-sensor (P_D, λ_C, gate). Units of λ_C match z.model's
      // measurement space (m^-2 / (m·rad)^-1 / rad^-1) so the score
      // increment
      //   log P_D + log p(z|x) − log λ_C
      // is dimensionally consistent for this sensor — even when the
      // scan mixes sensors with different units.
      const DetectionParams dp = params.detection_model->paramsFor(z);
      // Gate + likelihood through IEstimator. For EKF/UKF/PF this is
      // the textbook Mahalanobis + log-N. For IMM the gate is
      // any-mode (Mazor 1998) and the likelihood is the mode-weighted
      // mixture — strictly more honest than the moment-matched
      // projection that this loop used to compute inline. The gate is
      // per-sensor when the entry declares one (sparse position
      // sensors widen it to recapture bearing-drifted tracks), and
      // position gates additionally scale with the leaf's position-
      // anchor age when the adaptive recapture gate is enabled.
      double gate =
          dp.gate_threshold > 0.0 ? dp.gate_threshold : params.gate_threshold;
      if (params.recapture_tau_s > 0.0 && canInitiateTrack(z.model)) {
        const double age = std::max(
            0.0,
            scan_time.secondsSince(nodes_[leaf_idx].last_position_anchor));
        gate *= std::min(params.recapture_max_scale,
                         1.0 + age / params.recapture_tau_s);
      }
      if (!estimator.gate(tmp_predicted, z, gate)) continue;
      const double log_likelihood = estimator.logLikelihood(tmp_predicted, z);

      Track child_tr = tmp_predicted;
      estimator.update(child_tr, z);

      TrackTreeNode hit;
      hit.parent = leaf_idx;
      hit.scan_idx = nodes_[leaf_idx].scan_idx + 1;
      hit.state = child_tr.state;
      hit.covariance = child_tr.covariance;
      hit.imm_means = child_tr.imm_means;
      hit.imm_covariances = child_tr.imm_covariances;
      hit.imm_mode_probabilities = child_tr.imm_mode_probabilities;
      hit.time = scan_time;
      hit.score = nodes_[leaf_idx].score +
                  std::log(dp.probability_of_detection) +
                  log_likelihood -
                  std::log(dp.clutter_intensity);
      hit.is_leaf = true;
      hit.is_hit = true;
      hit.scan_meas_idx = mi;
      // Position-sensor hits refresh the range anchor; bearing hits
      // carry it through (they observe angle, not range).
      hit.last_position_anchor = canInitiateTrack(z.model)
                                     ? scan_time
                                     : nodes_[leaf_idx].last_position_anchor;
      // IPDA / VIMM existence on the hit branch (carried through when
      // update_existence is off). Uses the same per-sensor (P_D, λ_C)
      // the score uses — calibrated quantity, not the raw score.
      if (params.update_existence) {
        const ExistenceVis pred = predictExistence(
            nodes_[leaf_idx].existence_probability,
            nodes_[leaf_idx].visibility_given_exists,
            params.existence_persistence,
            params.visibility_persistence, params.visibility_recovery,
            params.update_visibility, dt);
        const ExistenceVis post = updateHit(
            pred.existence, pred.visibility_given_exists,
            dp.probability_of_detection, std::exp(log_likelihood),
            dp.clutter_intensity, params.update_visibility);
        hit.existence_probability = post.existence;
        hit.visibility_given_exists = post.visibility_given_exists;
      } else {
        hit.existence_probability = nodes_[leaf_idx].existence_probability;
        hit.visibility_given_exists = nodes_[leaf_idx].visibility_given_exists;
      }
      // W5.5: inherit deferred-commitment protection (see the miss branch).
      hit.is_protected = nodes_[leaf_idx].is_protected;
      nodes_.push_back(hit);
    }

    nodes_[leaf_idx].is_leaf = false;
  }
}

// Bhattacharyya distance between two Gaussians evaluated on the
// position block of the kinematic state (rows/cols 0..1). Same-position
// covariances → 0; well-separated → grows. Returns +∞ if Σ is singular.
// Shared by within-tree leaf merging and the cross-tree duplicate merge
// in MhtTracker.
double bhattacharyyaPosition(const Eigen::VectorXd& mu_a,
                             const Eigen::MatrixXd& Sa,
                             const Eigen::VectorXd& mu_b,
                             const Eigen::MatrixXd& Sb) {
  const Eigen::Vector2d d = mu_a.head<2>() - mu_b.head<2>();
  const Eigen::Matrix2d Pa = Sa.topLeftCorner<2, 2>();
  const Eigen::Matrix2d Pb = Sb.topLeftCorner<2, 2>();
  const Eigen::Matrix2d P = 0.5 * (Pa + Pb);
  const double det_P = P.determinant();
  const double det_a = Pa.determinant();
  const double det_b = Pb.determinant();
  if (!(det_P > 0.0) || !(det_a > 0.0) || !(det_b > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  const double mahal = d.transpose() * P.inverse() * d;
  return 0.125 * mahal +
         0.5 * std::log(det_P / std::sqrt(det_a * det_b));
}

int TrackTree::countHitsInWindow(std::size_t leaf, int window) const {
  if (window <= 0) return 0;
  int count = 0;
  std::size_t cur = leaf;
  int steps = 0;
  while (cur != TrackTreeNode::kNoParent && steps < window) {
    if (nodes_[cur].is_hit) ++count;
    cur = nodes_[cur].parent;
    ++steps;
  }
  return count;
}

std::size_t TrackTree::pruneNScan(int n_scan) {
  std::vector<std::size_t> leaves = leafIndices();
  if (leaves.empty()) return 0;
  int max_scan = 0;
  for (std::size_t li : leaves) {
    if (nodes_[li].scan_idx > max_scan) max_scan = nodes_[li].scan_idx;
  }
  const int target_depth = max_scan - n_scan;
  if (target_depth <= 0) return 0;

  std::vector<std::size_t> ancestors(leaves.size());
  for (std::size_t k = 0; k < leaves.size(); ++k) {
    std::size_t cur = leaves[k];
    while (cur != TrackTreeNode::kNoParent &&
           nodes_[cur].scan_idx > target_depth) {
      cur = nodes_[cur].parent;
    }
    ancestors[k] = cur;
  }

  std::vector<std::size_t> unique_ancestors;
  std::vector<double> agg_scores;
  for (std::size_t k = 0; k < leaves.size(); ++k) {
    const std::size_t anc = ancestors[k];
    const double s = nodes_[leaves[k]].score;
    bool found = false;
    for (std::size_t u = 0; u < unique_ancestors.size(); ++u) {
      if (unique_ancestors[u] == anc) {
        if (s > agg_scores[u]) agg_scores[u] = s;
        found = true;
        break;
      }
    }
    if (!found) {
      unique_ancestors.push_back(anc);
      agg_scores.push_back(s);
    }
  }

  std::size_t winner = unique_ancestors[0];
  double best = agg_scores[0];
  for (std::size_t u = 1; u < unique_ancestors.size(); ++u) {
    if (agg_scores[u] > best) {
      best = agg_scores[u];
      winner = unique_ancestors[u];
    }
  }

  std::vector<bool> keep(nodes_.size(), false);
  std::size_t a = winner;
  while (a != TrackTreeNode::kNoParent) {
    keep[a] = true;
    a = nodes_[a].parent;
  }
  std::vector<std::size_t> frontier{winner};
  while (!frontier.empty()) {
    const std::size_t cur = frontier.back();
    frontier.pop_back();
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      if (!keep[i] && nodes_[i].parent == cur) {
        keep[i] = true;
        frontier.push_back(i);
      }
    }
  }
  // Protected nodes survive N-scan even when their ancestor lost the
  // score competition. The global solve has marked them (and their
  // ancestor chain back to the root) on the previous scan, so the
  // chain is already connected when we walk parent pointers below.
  // This is the load-bearing piece for deferred-commitment TOMHT:
  // without it, an alternative branch is killed at the trunk merge
  // before it ever gets a chance to overtake the K=1 best.
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].is_protected) keep[i] = true;
  }

  std::vector<std::size_t> new_index(nodes_.size(), TrackTreeNode::kNoParent);
  std::vector<TrackTreeNode> new_nodes;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (keep[i]) {
      new_index[i] = new_nodes.size();
      new_nodes.push_back(nodes_[i]);
    }
  }
  for (TrackTreeNode& n : new_nodes) {
    if (n.parent != TrackTreeNode::kNoParent) n.parent = new_index[n.parent];
  }
  const std::size_t removed = nodes_.size() - new_nodes.size();
  nodes_ = std::move(new_nodes);
  return removed;
}

std::size_t TrackTree::pruneKLocal(std::size_t k) {
  std::vector<std::size_t> leaves = leafIndices();
  if (leaves.size() <= k) return 0;
  std::sort(leaves.begin(), leaves.end(),
            [this](std::size_t a, std::size_t b) {
              return nodes_[a].score > nodes_[b].score;
            });
  // Skip protected leaves when demoting — they are alternative-hypothesis
  // leaves the global solve flagged for one-scan deferred commitment.
  // If all bottom-k are protected, k is effectively raised this scan;
  // self-limits since protection is cleared at the next branch().
  std::size_t dropped = 0;
  for (std::size_t i = k; i < leaves.size(); ++i) {
    if (nodes_[leaves[i]].is_protected) continue;
    nodes_[leaves[i]].is_leaf = false;
    ++dropped;
  }
  return dropped;
}

std::size_t TrackTree::mergeBranches(double threshold) {
  if (!(threshold > 0.0)) return 0;
  std::vector<std::size_t> leaves = leafIndices();
  if (leaves.size() < 2) return 0;
  // Order leaves by score descending so the higher-scoring leaf wins
  // any merge contest (its slot survives).
  std::sort(leaves.begin(), leaves.end(),
            [this](std::size_t a, std::size_t b) {
              return nodes_[a].score > nodes_[b].score;
            });
  std::size_t dropped = 0;
  for (std::size_t i = 0; i < leaves.size(); ++i) {
    if (!nodes_[leaves[i]].is_leaf) continue;  // already dropped
    for (std::size_t j = i + 1; j < leaves.size(); ++j) {
      if (!nodes_[leaves[j]].is_leaf) continue;
      const double b =
          bhattacharyyaPosition(nodes_[leaves[i]].state,
                          nodes_[leaves[i]].covariance,
                          nodes_[leaves[j]].state,
                          nodes_[leaves[j]].covariance);
      if (b < threshold) {
        // Don't merge away a protected leaf — it's an explicit "keep this
        // alternative alive one more scan" signal from the global solve.
        // The outer loop visits in score order, so leaves[i] is always
        // the higher-scoring of the pair; if either side is protected,
        // we leave both untouched.
        if (nodes_[leaves[i]].is_protected ||
            nodes_[leaves[j]].is_protected) continue;
        nodes_[leaves[j]].is_leaf = false;
        ++dropped;
      }
    }
  }
  return dropped;
}

}  // namespace navtracker
