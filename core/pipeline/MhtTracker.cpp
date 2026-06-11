#include "core/pipeline/MhtTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>

#include "core/association/Gating.hpp"
#include "core/association/Hungarian.hpp"
#include "core/association/Murty.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/tracking/SensorDetectionModels.hpp"

namespace navtracker {

MhtTracker::MhtTracker(const IEstimator& estimator, Config cfg,
                       std::shared_ptr<ISensorDetectionModel> detection_model)
    : estimator_(estimator), cfg_(cfg) {
  using_default_detection_model_ = (detection_model == nullptr);
  detection_model_ =
      detection_model
          ? std::move(detection_model)
          : std::make_shared<FixedSensorDetectionModel>(DetectionParams{
                cfg.probability_of_detection, cfg.clutter_density});
}

namespace {

TrackTreeNode rootFromMeasurement(const IEstimator& estimator,
                                  const Measurement& z) {
  Track seed = estimator.initiate(z);
  TrackTreeNode root;
  root.parent = TrackTreeNode::kNoParent;
  root.scan_idx = 0;
  root.state = seed.state;
  root.covariance = seed.covariance;
  // Carry IMM ensemble through to the tree so subsequent predicts
  // don't bail out on imm_means.cols()==0. Empty for single-mode
  // estimators (EKF/UKF/PF) — no-op.
  root.imm_means = seed.imm_means;
  root.imm_covariances = seed.imm_covariances;
  root.imm_mode_probabilities = seed.imm_mode_probabilities;
  root.time = z.time;
  root.score = 0.0;
  root.is_leaf = true;
  root.is_hit = true;  // root was born from a measurement
  root.scan_meas_idx = TrackTreeNode::kNoMeasurement;
  return root;
}

// Result of the global-hypothesis solve: per tree, the index of the
// chosen leaf node (or kNoParent if no leaf could be selected). For
// k_best > 1, also reports the leaves participating in the
// alternative top-K assignments so callers can protect them from
// pruning across N-scan trunk merging (deferred-commitment TOMHT).
struct GlobalAssignment {
  std::vector<std::size_t> chosen_leaf;
  // For each tree, the leaves seen in any of the K best global
  // assignments (including the best). Useful to protect alternative
  // hypotheses across scans. Empty entries for trees with no leaves.
  std::vector<std::vector<std::size_t>> top_k_leaves;
};

// Solve the K-best global hypothesis via Murty (for K=1 collapses to
// Hungarian — same behaviour as before). For each tree we pick one of
// its current leaves; the constraint is that no two trees may pick
// leaves that consumed the same scan measurement. Cost matrix:
//
//   T rows × (M + T) cols
//   first M cols   = scan measurements
//   trailing T cols = tree-specific "miss slots"
//
// For row t:
//   cost[t, j ∈ [0,M)] = -best_leaf_score(t, meas j)  or +∞ if no such leaf
//   cost[t, M + t]     = -best_leaf_score(t, miss)    or +∞ if no miss leaf
//   cost[t, M + t' ≠ t] = +∞                          (forbid)
//
// `score_delta_threshold` (Blackman 2004 §V): an alternative
// assignment (k ≥ 1) contributes to `top_k_leaves` only if its total
// cost exceeds the K=1 best by less than this delta. Without this
// filter, every Murty alternative gets protected regardless of how
// arbitrarily worse it is — in cooperative cases that means every
// "miss instead of hit" alternative is kept, doubling tree size each
// scan. <= 0 disables the filter (every K alternative protected).
GlobalAssignment solveGlobalHypothesis(
    const std::vector<TrackTree>& trees,
    std::size_t scan_size,
    int k_best,
    double score_delta_threshold) {
  GlobalAssignment out;
  const std::size_t T = trees.size();
  out.chosen_leaf.assign(T, TrackTreeNode::kNoParent);
  out.top_k_leaves.assign(T, {});
  if (T == 0) return out;

  const std::size_t M = scan_size;
  const std::size_t total_cols = M + T;
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(T, total_cols, kInf);

  // best_leaf[t][col] = index of the highest-scoring leaf in tree t
  // that "claims" the action represented by col, or kNoParent if none.
  std::vector<std::vector<std::size_t>> best_leaf(
      T, std::vector<std::size_t>(total_cols, TrackTreeNode::kNoParent));

  for (std::size_t t = 0; t < T; ++t) {
    const auto& nodes = trees[t].nodes();
    for (std::size_t li = 0; li < nodes.size(); ++li) {
      if (!nodes[li].is_leaf) continue;
      const double s = nodes[li].score;
      // Column for this leaf's action this scan:
      std::size_t col;
      if (nodes[li].scan_meas_idx == TrackTreeNode::kNoMeasurement) {
        col = M + t;  // miss slot for this tree
      } else if (nodes[li].scan_meas_idx < M) {
        col = nodes[li].scan_meas_idx;
      } else {
        continue;  // stale (shouldn't happen since branch just ran)
      }
      // Keep highest-scoring leaf per (tree, action).
      if (best_leaf[t][col] == TrackTreeNode::kNoParent ||
          s > nodes[best_leaf[t][col]].score) {
        best_leaf[t][col] = li;
        cost(t, col) = -s;  // negate: Hungarian minimizes.
      }
    }
    // Forbid other trees' miss slots:
    for (std::size_t t2 = 0; t2 < T; ++t2) {
      if (t2 != t) cost(t, M + t2) = kInf;
    }
    // If a tree has NO leaf at all (degenerate; pruned-empty), the row
    // is all +∞ → solver will leave it unassigned, and we set
    // chosen_leaf[t] = kNoParent.
  }

  // Murty K-best with k=1 reduces to plain Hungarian; we use Murty
  // uniformly to keep one code path and rely on K=1 ≡ Hungarian by
  // construction.
  const int K = std::max(1, k_best);
  const KBestResult kbest = murtyKBest(cost, K);
  if (kbest.assignments.empty()) return out;

  // K=1 (best) assignment → reported track per tree.
  const std::vector<int>& assign0 = kbest.assignments[0];
  for (std::size_t t = 0; t < T; ++t) {
    const int col = assign0[t];
    if (col < 0) continue;
    if (!std::isfinite(cost(t, col))) continue;
    const std::size_t leaf = best_leaf[t][static_cast<std::size_t>(col)];
    if (leaf == TrackTreeNode::kNoParent) continue;
    out.chosen_leaf[t] = leaf;
  }

  // Collect leaves participating in any of the K best assignments
  // per tree, subject to the Score-Δ window. Duplicates across
  // assignments are deduplicated. Used downstream as "protected from
  // local pruning this scan" so deferred-commitment alternatives
  // survive into N-scan trunk merge. The K=1 best is always included
  // (its leaves are what we just selected as chosen_leaf); higher-K
  // alternatives are admitted only if cost - best_cost < delta.
  const double best_cost = kbest.costs.empty() ? 0.0 : kbest.costs[0];
  for (std::size_t k = 0; k < kbest.assignments.size(); ++k) {
    if (k > 0 && score_delta_threshold > 0.0 &&
        kbest.costs[k] - best_cost > score_delta_threshold) {
      // Murty returns costs in non-decreasing order, so once we cross
      // the delta we're done.
      break;
    }
    const auto& a = kbest.assignments[k];
    for (std::size_t t = 0; t < T; ++t) {
      const int col = a[t];
      if (col < 0) continue;
      if (!std::isfinite(cost(t, col))) continue;
      const std::size_t leaf = best_leaf[t][static_cast<std::size_t>(col)];
      if (leaf == TrackTreeNode::kNoParent) continue;
      auto& bucket = out.top_k_leaves[t];
      if (std::find(bucket.begin(), bucket.end(), leaf) == bucket.end()) {
        bucket.push_back(leaf);
      }
    }
  }
  return out;
}

}  // namespace

void MhtTracker::processBatch(const std::vector<Measurement>& scan) {
  if (scan.empty()) return;
  const Timestamp t = scan.front().time;
  if (has_high_water_ && t < high_water_) {
    if (cfg_.reject_stale_measurements) {
      stale_dropped_ += scan.size();
      return;
    }
  } else {
    high_water_ = t;
    has_high_water_ = true;
  }

  // Footgun diagnostic: ≥2 distinct sensor keys on the auto-installed
  // single-default detection model (see defaultDetectionModelWarning).
  if (using_default_detection_model_ && !default_detection_warning_) {
    for (const Measurement& z : scan) {
      seen_sensor_keys_.insert({z.sensor, z.model});
    }
    if (seen_sensor_keys_.size() >= 2) default_detection_warning_ = true;
  }

  // NB: do NOT clear is_protected here. The pruning passes below need
  // to see the flags set by the PREVIOUS scan's solveGlobalHypothesis.
  // Protection is cleared and refreshed at the end of this method,
  // after the new solve has identified this scan's top-K leaves.

  // Hit-branch (P_D, λ_C) is looked up per measurement inside branch()
  // from the detection model — so each sensor contributes its own units
  // and rate to the score. Miss-branch P_D is likewise per-sensor (and
  // coverage-conditioned) via the detection model: each tree is only
  // charged for the sensors that actually surveyed this scan instant
  // and could have seen it (see TrackTree::BranchParams docs).
  TrackTree::BranchParams bp{
      detection_model_.get(),
      cfg_.gate_threshold,
      cfg_.use_ipda_lifecycle,
      cfg_.use_visibility && cfg_.use_ipda_lifecycle,
      cfg_.ipda_persistence,
      cfg_.ipda_gate_probability_mass,
      cfg_.visibility_persistence,
      cfg_.visibility_recovery};
  for (TrackTree& tt : trees_) {
    tt.branch(estimator_, scan, t, bp);
    tt.pruneKLocal(cfg_.k_max_leaves);
    if (cfg_.merge_bhattacharyya_threshold > 0.0) {
      tt.mergeBranches(cfg_.merge_bhattacharyya_threshold);
    }
    // Per-tree adaptive N-scan: trees that had multiple protected
    // alternatives on the previous scan (i.e., genuine deferred-
    // commitment ambiguity) get an extended trunk-merge delay so the
    // alternatives have more time to accumulate evidence. Trees with
    // a single dominant leaf use the base n_scan.
    const int n_eff =
        (tt.protectedAlternativesLastScan() > 1)
            ? cfg_.n_scan + cfg_.n_scan_extension_when_protected
            : cfg_.n_scan;
    tt.pruneNScan(n_eff);
  }

  // Spawn new tentative trees from measurements that didn't gate to any
  // existing tree's best leaf. (This is a track-birth heuristic, distinct
  // from the global-hypothesis solve below — birth precedes assignment.)
  std::vector<bool> measurement_explained(scan.size(), false);
  for (std::size_t j = 0; j < scan.size(); ++j) {
    bool gated_to_any = false;
    for (TrackTree& tt : trees_) {
      const std::size_t best = tt.bestLeafIndex();
      if (best == TrackTreeNode::kNoParent) continue;
      Track gate_tr;
      gate_tr.state = tt.nodes()[best].state;
      gate_tr.covariance = tt.nodes()[best].covariance;
      gate_tr.last_update = tt.nodes()[best].time;
      // NB: this gate uses the moment-matched single-Gaussian path —
      // we don't have the IMM per-mode means/covariances on a
      // TrackTreeNode (only on the Track during branch()). For birth
      // gating that's acceptable; the per-tree branch loop above uses
      // estimator.gate() with the full IMM state.
      if (estimator_.gate(gate_tr, scan[j], cfg_.gate_threshold)) {
        gated_to_any = true;
        break;
      }
    }
    measurement_explained[j] = gated_to_any;
  }
  for (std::size_t j = 0; j < scan.size(); ++j) {
    if (measurement_explained[j]) continue;
    // Passive bearing-only measurements can't seed a new tree (range
    // unobservable); they only extend existing trees via branch(). Drop
    // unassociated ones — see canInitiateTrack.
    if (!canInitiateTrack(scan[j].model)) continue;
    const TrackId id{next_external_id_++};
    TrackTreeNode root = rootFromMeasurement(estimator_, scan[j]);
    // Seed IPDA / VIMM state on birth. Off → defaults (1.0, 1.0) =
    // no-op sentinel. On → ipda_init_existence as the prior r₀ (a
    // single detection is weak evidence; default 0.5) and
    // visibility_init as v₀ (default 1.0 = just detected so visible).
    if (cfg_.use_ipda_lifecycle) {
      root.existence_probability = cfg_.ipda_init_existence;
      root.visibility_given_exists =
          cfg_.use_visibility ? cfg_.visibility_init : 1.0;
    }
    trees_.emplace_back(id, std::move(root));
  }

  // Feed the scan outcome to the detection model, bucketed per
  // (sensor, model). Measurements that gated to no existing tree are
  // the clutter proxy (most unassociated returns are false alarms).
  // Each sensor's bucket grows its own EWMA + surveyed area; one noisy
  // sensor no longer pollutes another's λ_C estimate. Fixed models
  // ignore.
  {
    using Key = std::tuple<SensorKind, MeasurementModel>;
    std::map<Key, ISensorDetectionModel::ScanObservation> by_sensor;
    for (std::size_t j = 0; j < scan.size(); ++j) {
      const Key k{scan[j].sensor, scan[j].model};
      auto it = by_sensor.find(k);
      if (it == by_sensor.end()) {
        ISensorDetectionModel::ScanObservation obs;
        obs.sensor = scan[j].sensor;
        obs.model = scan[j].model;
        obs.num_unassociated = 0;
        it = by_sensor.emplace(k, std::move(obs)).first;
      }
      // Pure-bearing measurements carry no ENU position — exclude from
      // surveyed-area but still count unassociated as a clutter proxy.
      if (canInitiateTrack(scan[j].model) && scan[j].value.size() >= 2)
        it->second.positions.emplace_back(scan[j].value(0), scan[j].value(1));
      if (!measurement_explained[j]) ++it->second.num_unassociated;
    }
    std::vector<ISensorDetectionModel::ScanObservation> bundle;
    bundle.reserve(by_sensor.size());
    for (auto& kv : by_sensor) bundle.push_back(std::move(kv.second));
    detection_model_->observe(bundle);
  }

  // Drop trees whose best-leaf score is below the delete threshold,
  // UNLESS any leaf in the tree is protected — in which case an
  // alternative hypothesis (flagged on the previous scan's global
  // solve) is still in play and deserves one more scan to either
  // recover or genuinely fall below threshold. Protection is cleared
  // each scan so this is bounded: a tree can survive at most one
  // below-threshold scan via protection alone.
  std::vector<TrackTree> kept;
  kept.reserve(trees_.size());
  for (TrackTree& tt : trees_) {
    const std::size_t best = tt.bestLeafIndex();
    if (best == TrackTreeNode::kNoParent) continue;
    const bool score_dead =
        tt.nodes()[best].score < cfg_.score_delete_threshold;
    // IPDA-existence delete gate (active only when use_ipda_lifecycle):
    // if every leaf's existence has decayed below ipda_delete_threshold
    // the tree is unrecoverable. We require *every* leaf to be below
    // — a single high-existence leaf is enough to keep the tree, since
    // it represents a still-viable hypothesis (mirrors the bestLeaf
    // pattern used for the score gate).
    bool existence_dead = false;
    if (cfg_.use_ipda_lifecycle) {
      double max_r = 0.0;
      for (const TrackTreeNode& n : tt.nodes()) {
        if (n.is_leaf && n.existence_probability > max_r)
          max_r = n.existence_probability;
      }
      existence_dead = max_r < cfg_.ipda_delete_threshold;
    }
    if (score_dead || existence_dead) {
      bool any_protected = false;
      for (const TrackTreeNode& n : tt.nodes()) {
        if (n.is_leaf && n.is_protected) { any_protected = true; break; }
      }
      if (!any_protected) continue;
    }
    kept.push_back(std::move(tt));
  }
  trees_ = std::move(kept);

  // Global hypothesis: pick one leaf per tree such that no scan
  // measurement is consumed by more than one tree. K>1 also collects
  // alternative-top-K leaves for downstream deferred-commitment
  // (consumed by callers that want to protect those leaves from
  // pruning; the reported track itself comes from the K=1 best).
  const GlobalAssignment assign = solveGlobalHypothesis(
      trees_, scan.size(), cfg_.k_best, cfg_.score_delta_threshold);

  // Clear all stale is_protected flags from the previous scan, then
  // set them fresh on this scan's top-K leaves and their ancestor
  // chain. Doing both here (post-solve) is the correctness fix: an
  // earlier version cleared at the top of processBatch, which meant
  // the prune passes above never actually saw the flags. With this
  // ordering, the flags set in this block survive into the NEXT
  // scan's pruneKLocal/mergeBranches/pruneNScan and the score-delete
  // sweep — exactly the deferred-commitment carry-over we want.
  //
  // Also record per-tree |top_k_leaves| for the adaptive-N-scan
  // signal used at the top of the next processBatch.
  for (TrackTree& tt : trees_) {
    for (TrackTreeNode& n : tt.mutableNodes()) n.is_protected = false;
  }
  for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
    auto& nodes = trees_[ti].mutableNodes();
    for (std::size_t li : assign.top_k_leaves[ti]) {
      std::size_t cur = li;
      while (cur != TrackTreeNode::kNoParent) {
        nodes[cur].is_protected = true;
        cur = nodes[cur].parent;
      }
    }
    trees_[ti].setProtectedAlternativesLastScan(
        assign.top_k_leaves[ti].size());
  }

  tracks_.clear();
  tracks_.reserve(trees_.size());
  for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
    std::size_t leaf = assign.chosen_leaf[ti];
    // Fall back to the tree's local best leaf if the global solver
    // could not assign this tree (e.g. all its leaves' columns were
    // taken by other trees). This is the K=1 limitation; Murty K-best
    // would consider alternatives. For now report the local best so
    // we never silently drop a track.
    if (leaf == TrackTreeNode::kNoParent) {
      leaf = trees_[ti].bestLeafIndex();
    }
    if (leaf == TrackTreeNode::kNoParent) continue;

    // Confirmation gate. Precedence: IPDA-existence (when enabled) →
    // SPRT (when enabled) → M-of-N (default).
    TrackStatus status;
    if (cfg_.use_ipda_lifecycle) {
      status = (trees_[ti].nodes()[leaf].existence_probability >=
                cfg_.ipda_confirm_threshold)
                   ? TrackStatus::Confirmed
                   : TrackStatus::Tentative;
    } else if (cfg_.use_sprt_confirm) {
      const double t_confirm = std::log(
          (1.0 - cfg_.sprt_beta) / std::max(cfg_.sprt_alpha, 1e-12));
      status = (trees_[ti].nodes()[leaf].score >= t_confirm)
                   ? TrackStatus::Confirmed
                   : TrackStatus::Tentative;
    } else {
      const int hits = trees_[ti].countHitsInWindow(
          leaf, cfg_.confirm_hits_window);
      status = (hits >= cfg_.confirm_hits_needed) ? TrackStatus::Confirmed
                                                  : TrackStatus::Tentative;
    }

    Track view;
    view.id = trees_[ti].externalId();
    view.state = trees_[ti].nodes()[leaf].state;
    view.covariance = trees_[ti].nodes()[leaf].covariance;
    view.imm_means = trees_[ti].nodes()[leaf].imm_means;
    view.imm_covariances = trees_[ti].nodes()[leaf].imm_covariances;
    view.imm_mode_probabilities =
        trees_[ti].nodes()[leaf].imm_mode_probabilities;
    view.last_update = trees_[ti].nodes()[leaf].time;
    view.status = status;
    view.existence_probability =
        trees_[ti].nodes()[leaf].existence_probability;
    view.visibility_given_exists =
        trees_[ti].nodes()[leaf].visibility_given_exists;
    tracks_.push_back(std::move(view));
  }
}

}  // namespace navtracker
