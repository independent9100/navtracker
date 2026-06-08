#include "core/pipeline/MhtTracker.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/association/Gating.hpp"
#include "core/association/Hungarian.hpp"
#include "core/association/Murty.hpp"

namespace navtracker {

MhtTracker::MhtTracker(const IEstimator& estimator, Config cfg)
    : estimator_(estimator), cfg_(cfg) {}

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
GlobalAssignment solveGlobalHypothesis(
    const std::vector<TrackTree>& trees,
    std::size_t scan_size,
    int k_best) {
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
  // per tree. Duplicates across assignments are deduplicated. Used
  // downstream as "protected from local pruning this scan" so
  // deferred-commitment alternatives survive into N-scan trunk merge.
  for (const auto& a : kbest.assignments) {
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

  // Clear last scan's protection markers. Protection is exactly one
  // scan deep: leaves flagged after the previous global solve survive
  // this scan's pruning, then start unprotected so this scan's solve
  // can re-flag a fresh set. See TrackTreeNode::is_protected.
  for (TrackTree& tt : trees_) {
    for (TrackTreeNode& n : tt.mutableNodes()) n.is_protected = false;
  }

  TrackTree::BranchParams bp{
      cfg_.probability_of_detection,
      cfg_.clutter_density,
      cfg_.gate_threshold};
  for (TrackTree& tt : trees_) {
    tt.branch(estimator_, scan, t, bp);
    tt.pruneKLocal(cfg_.k_max_leaves);
    if (cfg_.merge_bhattacharyya_threshold > 0.0) {
      tt.mergeBranches(cfg_.merge_bhattacharyya_threshold);
    }
    tt.pruneNScan(cfg_.n_scan);
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
    const TrackId id{next_external_id_++};
    trees_.emplace_back(id, rootFromMeasurement(estimator_, scan[j]));
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
    if (tt.nodes()[best].score < cfg_.score_delete_threshold) {
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
  const GlobalAssignment assign =
      solveGlobalHypothesis(trees_, scan.size(), cfg_.k_best);

  // Mark all leaves participating in any of the K best global hypotheses
  // (and their ancestor chain up to the root) as protected. They survive
  // the *next* scan's pruneKLocal/mergeBranches/pruneNScan and the
  // score-delete sweep, giving deferred-commitment evidence one more
  // chance to elevate an alternative past the K=1 best. Protection is
  // cleared at the top of the next processBatch.
  for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
    auto& nodes = trees_[ti].mutableNodes();
    for (std::size_t li : assign.top_k_leaves[ti]) {
      std::size_t cur = li;
      while (cur != TrackTreeNode::kNoParent) {
        nodes[cur].is_protected = true;
        cur = nodes[cur].parent;
      }
    }
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

    // M-of-N confirmation gate.
    const int hits = trees_[ti].countHitsInWindow(
        leaf, cfg_.confirm_hits_window);
    const TrackStatus status =
        (hits >= cfg_.confirm_hits_needed) ? TrackStatus::Confirmed
                                            : TrackStatus::Tentative;

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
    tracks_.push_back(std::move(view));
  }
}

}  // namespace navtracker
