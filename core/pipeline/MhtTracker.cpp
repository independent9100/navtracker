#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/BiasCorrection.hpp"
#include "core/pipeline/SourceTouchPopulate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>

#include "core/association/Gating.hpp"
#include "core/association/Hungarian.hpp"
#include "core/association/Murty.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/geo/Datum.hpp"
#include "core/tracking/DatumShift.hpp"
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
  // Births come from position-capable measurements (canInitiateTrack),
  // so the seed measurement is itself the first range anchor.
  root.last_position_anchor = z.time;
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
//
// `non_exclusive` (may be empty = none): measurements exempted from
// exclusivity (shared ambiguous bearings, Config::
// share_ambiguous_bearings). A hit leaf on such a measurement maps to
// the tree's PRIVATE column (M + t) instead of the shared measurement
// column, so it competes only against that tree's own miss/shared
// alternatives and any number of trees can consume the measurement.
GlobalAssignment solveGlobalHypothesis(
    const std::vector<TrackTree>& trees,
    std::size_t scan_size,
    int k_best,
    double score_delta_threshold,
    const std::vector<bool>& non_exclusive = {}) {
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
        const std::size_t mi = nodes[li].scan_meas_idx;
        // Shared ambiguous bearing: private column, no exclusivity.
        col = (mi < non_exclusive.size() && non_exclusive[mi]) ? M + t : mi;
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

void MhtTracker::processBatch(const std::vector<Measurement>& scan_arg) {
  if (scan_arg.empty()) return;
  // Backlog #15: a batch is one scan at its EARLIEST instant, but the canonical
  // fixed-rate consumer (collect everything since the last tick, hand it over)
  // produces an unsorted batch. Order it by time here so the caller need not:
  // otherwise scan.front().time is the wrong instant and, with the stale guard
  // on, a front older than the high-water mark drops the whole batch. stable_sort
  // keeps it deterministic; the is_sorted fast-path makes already-sorted input a
  // true no-op (bit-identical — the common case and every existing test/bench).
  std::vector<Measurement> scan_ordered;
  const auto by_time = [](const Measurement& a, const Measurement& b) {
    return a.time < b.time;
  };
  const bool need_sort =
      !std::is_sorted(scan_arg.begin(), scan_arg.end(), by_time);
  if (need_sort) {
    scan_ordered = scan_arg;
    std::stable_sort(scan_ordered.begin(), scan_ordered.end(), by_time);
  }
  const std::vector<Measurement>& scan_in = need_sort ? scan_ordered : scan_arg;
  const Timestamp t = scan_in.front().time;
  if (has_high_water_ && t < high_water_) {
    if (cfg_.reject_stale_measurements) {
      stale_dropped_ += scan_in.size();
      return;
    }
  } else {
    high_water_ = t;
    has_high_water_ = true;
  }

  // Apply per-sensor registration bias correction (item 9) + Schmidt-KF
  // R-inflation (item 9 acceptance criterion 5). Null provider =
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
      cfg_.visibility_recovery,
      cfg_.gate_recapture_tau_s,
      cfg_.gate_recapture_max_scale};
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
    // Per-sensor gate override (DetectionParams::gate_threshold) plus
    // the adaptive recapture scaling — the same gate branch() used, so
    // a return that can extend a tree never births a duplicate next to
    // it.
    const double dp_gate =
        detection_model_->paramsFor(scan[j]).gate_threshold;
    const double base_gate = dp_gate > 0.0 ? dp_gate : cfg_.gate_threshold;
    bool gated_to_any = false;
    for (TrackTree& tt : trees_) {
      const std::size_t best = tt.bestLeafIndex();
      if (best == TrackTreeNode::kNoParent) continue;
      double gate = base_gate;
      if (cfg_.gate_recapture_tau_s > 0.0 &&
          canInitiateTrack(scan[j].model)) {
        const double age = std::max(
            0.0, t.secondsSince(tt.nodes()[best].last_position_anchor));
        gate *= std::min(cfg_.gate_recapture_max_scale,
                         1.0 + age / cfg_.gate_recapture_tau_s);
      }
      Track gate_tr;
      gate_tr.state = tt.nodes()[best].state;
      gate_tr.covariance = tt.nodes()[best].covariance;
      gate_tr.last_update = tt.nodes()[best].time;
      // NB: this gate uses the moment-matched single-Gaussian path —
      // we don't have the IMM per-mode means/covariances on a
      // TrackTreeNode (only on the Track during branch()). For birth
      // gating that's acceptable; the per-tree branch loop above uses
      // estimator.gate() with the full IMM state.
      if (estimator_.gate(gate_tr, scan[j], gate)) {
        gated_to_any = true;
        break;
      }
    }
    measurement_explained[j] = gated_to_any;
  }
  // Birth bookkeeping for the post-solve clutter labeling: which
  // measurement seeded which tree (keyed by external id — trees_ gets
  // reshuffled by deletion/merge before the solve).
  std::map<std::size_t, std::uint64_t> birth_id_for_meas;
  for (std::size_t j = 0; j < scan.size(); ++j) {
    if (measurement_explained[j]) continue;
    // Passive bearing-only measurements can't seed a new tree (range
    // unobservable); they only extend existing trees via branch(). Drop
    // unassociated ones — see canInitiateTrack.
    if (!canInitiateTrack(scan[j].model)) continue;
    const TrackId id{next_external_id_++};
    birth_id_for_meas.emplace(j, id.value);
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
    // Seed fused identity from the birth measurement. The root node
    // carries no attributes, so identity must live alongside the tree
    // (see tree_attributes_/tree_sources_); later committed hits refine
    // it in the contribution loop below.
    if (scan[j].hints.mmsi.has_value())
      tree_attributes_[id].mmsi = scan[j].hints.mmsi;
    tree_sources_[id].push_back(scan[j].source_id);
  }

  // NB: the detection-model observe() feed happens at the END of this
  // method — clutter evidence is labeled from the chosen global
  // hypothesis (existence-weighted claims), not from the birth gate.

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

  // Cross-tree duplicate merge: retire the younger of two trees whose
  // best leaves have stayed within the Bhattacharyya bound for
  // duplicate_merge_seconds of sustained stream time (time-based, not
  // scan-counted — see Config::duplicate_merge_bhattacharyya). Runs
  // before the global solve so a retired duplicate never emits this
  // scan. trees_ preserves spawn order and external ids are monotonic,
  // so trees_[i] is the older of any pair (i < j).
  if (cfg_.duplicate_merge_bhattacharyya > 0.0 && trees_.size() > 1) {
    std::vector<bool> retire(trees_.size(), false);
    for (std::size_t i = 0; i < trees_.size(); ++i) {
      if (retire[i]) continue;
      const std::size_t li = trees_[i].bestLeafIndex();
      if (li == TrackTreeNode::kNoParent) continue;
      for (std::size_t j = i + 1; j < trees_.size(); ++j) {
        if (retire[j]) continue;
        const std::size_t lj = trees_[j].bestLeafIndex();
        if (lj == TrackTreeNode::kNoParent) continue;
        const auto key = std::make_pair(trees_[i].externalId().value,
                                        trees_[j].externalId().value);
        const double d = bhattacharyyaPosition(
            trees_[i].nodes()[li].state, trees_[i].nodes()[li].covariance,
            trees_[j].nodes()[lj].state, trees_[j].nodes()[lj].covariance);
        if (d < cfg_.duplicate_merge_bhattacharyya) {
          const auto it = duplicate_close_since_.find(key);
          if (it == duplicate_close_since_.end()) {
            duplicate_close_since_.emplace(key, t);  // streak begins
          } else if (t.secondsSince(it->second) >=
                     cfg_.duplicate_merge_seconds) {
            retire[j] = true;
          }
        } else {
          duplicate_close_since_.erase(key);  // separated: clock resets
        }
      }
    }
    std::vector<TrackTree> survivors;
    survivors.reserve(trees_.size());
    for (std::size_t i = 0; i < trees_.size(); ++i) {
      if (!retire[i]) survivors.push_back(std::move(trees_[i]));
    }
    trees_ = std::move(survivors);
  }
  // Prune streak entries whose trees no longer exist (retired here or
  // deleted by score/existence above).
  if (!duplicate_close_since_.empty()) {
    std::set<std::uint64_t> live;
    for (const TrackTree& tt : trees_) live.insert(tt.externalId().value);
    for (auto it = duplicate_close_since_.begin();
         it != duplicate_close_since_.end();) {
      if (live.count(it->first.first) == 0 ||
          live.count(it->first.second) == 0) {
        it = duplicate_close_since_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Shared ambiguous bearings (Config::share_ambiguous_bearings): a
  // Bearing2D measurement whose hit branches exist in ≥ 2 trees is
  // angularly unresolved — it can't support identity, and the camera
  // detection genuinely merges both targets at small separations.
  // Mark it non-exclusive so the solve below lets every gating tree
  // consume it (identity stays with the exclusive position sensors).
  std::vector<bool> shared_bearing(
      cfg_.share_ambiguous_bearings ? scan.size() : 0, false);
  if (cfg_.share_ambiguous_bearings) {
    std::vector<int> tree_count(scan.size(), 0);
    for (const TrackTree& tt : trees_) {
      std::vector<bool> seen(scan.size(), false);  // one vote per tree
      for (const TrackTreeNode& n : tt.nodes()) {
        if (!n.is_leaf || !n.is_hit) continue;
        if (n.scan_meas_idx == TrackTreeNode::kNoMeasurement ||
            n.scan_meas_idx >= scan.size() || seen[n.scan_meas_idx])
          continue;
        seen[n.scan_meas_idx] = true;
        ++tree_count[n.scan_meas_idx];
      }
    }
    for (std::size_t j = 0; j < scan.size(); ++j) {
      shared_bearing[j] =
          scan[j].model == MeasurementModel::Bearing2D && tree_count[j] >= 2;
    }
  }

  // Global hypothesis: pick one leaf per tree such that no scan
  // measurement is consumed by more than one tree (shared ambiguous
  // bearings excepted). K>1 also collects alternative-top-K leaves for
  // downstream deferred-commitment (consumed by callers that want to
  // protect those leaves from pruning; the reported track itself comes
  // from the K=1 best).
  const GlobalAssignment assign = solveGlobalHypothesis(
      trees_, scan.size(), cfg_.k_best, cfg_.score_delta_threshold,
      shared_bearing);

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

  // Emit InnovationEvents for the global hypothesis just chosen — one
  // per surviving tree whose chosen leaf is a hit. The pre-update
  // predicted state is reconstructed by re-running estimator.predict on
  // the parent leaf's stored state up to the scan time; deterministic
  // and bit-exact to what TrackTree::branch passed to estimator.update.
  if (innov_sink_ != nullptr) {
    for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
      std::size_t leaf = assign.chosen_leaf[ti];
      if (leaf == TrackTreeNode::kNoParent) {
        leaf = trees_[ti].bestLeafIndex();
      }
      if (leaf == TrackTreeNode::kNoParent) continue;
      const TrackTreeNode& n = trees_[ti].nodes()[leaf];
      if (!n.is_hit) continue;  // miss-branch — no innovation
      if (n.parent == TrackTreeNode::kNoParent) continue;  // root spawn — no prior
      if (n.scan_meas_idx == TrackTreeNode::kNoMeasurement ||
          n.scan_meas_idx >= scan.size()) {
        continue;
      }
      const Measurement& z = scan[n.scan_meas_idx];
      const TrackTreeNode& p = trees_[ti].nodes()[n.parent];
      Track tr_pred;
      tr_pred.id = trees_[ti].externalId();
      tr_pred.state = p.state;
      tr_pred.covariance = p.covariance;
      tr_pred.imm_means = p.imm_means;
      tr_pred.imm_covariances = p.imm_covariances;
      tr_pred.imm_mode_probabilities = p.imm_mode_probabilities;
      tr_pred.last_update = p.time;
      estimator_.predict(tr_pred, n.time);
      const auto pred = predictMeasurement(z.model, tr_pred.state,
                                           z.sensor_position_enu);
      if (pred.z_pred.size() == 0 || pred.H.rows() == 0) continue;
      const Eigen::VectorXd nu =
          measurementResidual(z.model, z.value, pred.z_pred);
      if (z.covariance.rows() < nu.size() || z.covariance.cols() < nu.size()) {
        continue;
      }
      const Eigen::MatrixXd R =
          z.covariance.topLeftCorner(nu.size(), nu.size());
      const Eigen::MatrixXd S =
          pred.H * tr_pred.covariance * pred.H.transpose() + R;
      InnovationEvent ev;
      ev.time = z.time;
      ev.track_id = trees_[ti].externalId();
      ev.sensor = z.sensor;
      ev.source_id = z.source_id;
      ev.model = z.model;
      ev.residual = nu;
      ev.S = S;
      ev.R = R;
      ev.dim = static_cast<std::size_t>(nu.size());
      innov_sink_->onInnovation(ev);
    }
  }

  // Per-tree contribution history: append a SourceTouch for this scan's
  // chosen-leaf hit (if any), then prune entries older than the window.
  // Mirrors what Tracker.cpp does for the single-hypothesis pipeline so
  // downstream pair extractors (AisArpaPairExtractor,
  // SensorBiasPairExtractor) see equivalent track provenance.
  {
    const std::int64_t window_ns =
        static_cast<std::int64_t>(kContributionWindowSec * 1e9);
    for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
      std::size_t leaf = assign.chosen_leaf[ti];
      if (leaf == TrackTreeNode::kNoParent) continue;
      const TrackTreeNode& n = trees_[ti].nodes()[leaf];
      if (!n.is_hit) continue;
      if (n.scan_meas_idx == TrackTreeNode::kNoMeasurement ||
          n.scan_meas_idx >= scan.size()) {
        continue;
      }
      const Measurement& z = scan[n.scan_meas_idx];
      // Refine fused identity from the committed hit (parity with the
      // single-hypothesis Tracker, which sets mmsi on initiate and
      // appends distinct source_ids on each hit).
      const TrackId ext_id = trees_[ti].externalId();
      if (z.hints.mmsi.has_value())
        tree_attributes_[ext_id].mmsi = z.hints.mmsi;
      {
        auto& srcs = tree_sources_[ext_id];
        if (std::find(srcs.begin(), srcs.end(), z.source_id) == srcs.end())
          srcs.push_back(z.source_id);
      }
      // A committed hit past birth means a second position in time →
      // velocity is now observed, not pure init prior (review #13). The
      // birth root has scan_meas_idx == kNoMeasurement and is skipped above,
      // so this fires only on real updates.
      tree_velocity_observed_[ext_id] = true;
      Track::SourceTouch touch;
      touch.sensor = z.sensor;
      touch.source_id = z.source_id;
      touch.time = z.time;
      fillSourceTouchEnu(touch, z);
      touch.sensor_position_enu = z.sensor_position_enu;
      touch.own_position_std_m = z.sensor_position_std_m;
      touch.covariance_is_default = z.covariance_is_default;
      auto& history = contribution_history_[trees_[ti].externalId()];
      history.push_back(std::move(touch));
      // Prune.
      auto first_keep = std::find_if(
          history.begin(), history.end(),
          [&](const Track::SourceTouch& st) {
            return (t.nanos() - st.time.nanos()) <= window_ns;
          });
      history.erase(history.begin(), first_keep);
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

    // Confirmation gate. Precedence: IPDA-existence (when enabled) →
    // SPRT (when enabled) → M-of-N (default).
    TrackStatus status;
    if (cfg_.use_ipda_lifecycle) {
      // Hysteresis: first confirmation needs r ≥ confirm; once
      // confirmed, the track holds Confirmed down to the demote
      // threshold, and re-confirmation needs the full confirm
      // threshold again (ever-confirmed memory lives on the tree).
      const double r = trees_[ti].nodes()[leaf].existence_probability;
      const double confirm = cfg_.ipda_confirm_threshold;
      const double demote =
          std::min(cfg_.ipda_demote_threshold, confirm);
      if (trees_[ti].everConfirmed()) {
        status = (r >= demote) ? TrackStatus::Confirmed
                               : TrackStatus::Tentative;
        if (r < demote) trees_[ti].setEverConfirmed(false);
      } else {
        status = (r >= confirm) ? TrackStatus::Confirmed
                                : TrackStatus::Tentative;
        if (r >= confirm) trees_[ti].setEverConfirmed(true);
      }
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
    auto hit = contribution_history_.find(trees_[ti].externalId());
    if (hit != contribution_history_.end()) {
      view.recent_contributions = hit->second;
    }
    // Fused identity + provenance (TrackTreeNode carries neither).
    auto attr_it = tree_attributes_.find(trees_[ti].externalId());
    if (attr_it != tree_attributes_.end()) view.attributes = attr_it->second;
    auto src_it = tree_sources_.find(trees_[ti].externalId());
    if (src_it != tree_sources_.end())
      view.contributing_sources = src_it->second;
    auto vobs_it = tree_velocity_observed_.find(trees_[ti].externalId());
    if (vobs_it != tree_velocity_observed_.end())
      view.velocity_observed = vobs_it->second;
    tracks_.push_back(std::move(view));
  }
  // Drop history for trees that no longer exist (kept above with the
  // history map; trees_ may have shrunk).
  {
    std::set<TrackId> alive;
    for (const auto& tt : trees_) alive.insert(tt.externalId());
    for (auto it = contribution_history_.begin();
         it != contribution_history_.end();) {
      if (alive.count(it->first) == 0) it = contribution_history_.erase(it);
      else ++it;
    }
    for (auto it = tree_attributes_.begin(); it != tree_attributes_.end();) {
      if (alive.count(it->first) == 0) it = tree_attributes_.erase(it);
      else ++it;
    }
    for (auto it = tree_sources_.begin(); it != tree_sources_.end();) {
      if (alive.count(it->first) == 0) it = tree_sources_.erase(it);
      else ++it;
    }
    for (auto it = tree_velocity_observed_.begin();
         it != tree_velocity_observed_.end();) {
      if (alive.count(it->first) == 0) it = tree_velocity_observed_.erase(it);
      else ++it;
    }
  }

  // Feed the scan outcome to the detection model, labeled from the
  // global hypothesis just chosen: a return claimed by some tree's
  // selected hit leaf (or that birthed a still-alive tree this scan)
  // carries clutter weight 1 − r of that hypothesis; an unclaimed
  // return carries 1.0. Clutter-born trees keep low existence, so the
  // clutter signal survives, while returns explained by confident
  // tracks contribute ≈ 0 — unlike the old birth-gate proxy, which
  // counted every birthing return at full weight (the clean-scene
  // "birth self-poisoning" tax) and couldn't grade confidence.
  {
    std::vector<double> claim_r(scan.size(), -1.0);  // < 0 = unclaimed
    std::map<std::uint64_t, double> tree_r;  // external id → leaf r
    for (std::size_t ti = 0; ti < trees_.size(); ++ti) {
      std::size_t leaf = assign.chosen_leaf[ti];
      if (leaf == TrackTreeNode::kNoParent) {
        leaf = trees_[ti].bestLeafIndex();  // same fallback as readout
      }
      if (leaf == TrackTreeNode::kNoParent) continue;
      const TrackTreeNode& n = trees_[ti].nodes()[leaf];
      tree_r[trees_[ti].externalId().value] = n.existence_probability;
      if (n.is_hit && n.scan_meas_idx != TrackTreeNode::kNoMeasurement &&
          n.scan_meas_idx < scan.size()) {
        claim_r[n.scan_meas_idx] =
            std::max(claim_r[n.scan_meas_idx], n.existence_probability);
        if (n.scan_meas_idx < shared_bearing.size() &&
            shared_bearing[n.scan_meas_idx]) {
          ++shared_bearing_assignments_;
        }
      }
    }
    // Births: roots carry no scan_meas_idx, so credit the seeding
    // measurement through the id map (the tree may have been retired
    // between spawn and solve — then the claim simply lapses).
    for (const auto& [j, id] : birth_id_for_meas) {
      const auto it = tree_r.find(id);
      if (it != tree_r.end())
        claim_r[j] = std::max(claim_r[j], it->second);
    }

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
        obs.time = scan.front().time;
        it = by_sensor.emplace(k, std::move(obs)).first;
      }
      const bool claimed = claim_r[j] >= 0.0;
      const double weight =
          claimed ? std::clamp(1.0 - claim_r[j], 0.0, 1.0) : 1.0;
      // Pure-bearing measurements carry no ENU position — exclude from
      // surveyed-area but still contribute weighted clutter evidence.
      if (canInitiateTrack(scan[j].model) && scan[j].value.size() >= 2) {
        it->second.positions.emplace_back(scan[j].value(0), scan[j].value(1));
        if (weight > 0.0) {
          it->second.clutter_positions.emplace_back(scan[j].value(0),
                                                    scan[j].value(1));
          it->second.clutter_position_weights.push_back(weight);
        }
      }
      if (scan[j].model == MeasurementModel::Bearing2D &&
          scan[j].value.size() >= 1) {
        it->second.bearings.push_back(scan[j].value(0));
        if (weight > 0.0) {
          it->second.clutter_bearings.push_back(scan[j].value(0));
          it->second.clutter_bearing_weights.push_back(weight);
        }
      }
      if (!claimed) ++it->second.num_unassociated;
    }
    std::vector<ISensorDetectionModel::ScanObservation> bundle;
    bundle.reserve(by_sensor.size());
    for (auto& kv : by_sensor) bundle.push_back(std::move(kv.second));
    detection_model_->observe(bundle);
  }
}

void MhtTracker::onDatumRecentered(const geo::Datum& old_datum,
                                   const geo::Datum& new_datum) {
  for (TrackTree& tt : trees_) {
    for (TrackTreeNode& n : tt.mutableNodes()) {
      shiftStateOnDatumChange(n.state, n.covariance, n.imm_means,
                              n.imm_covariances, old_datum, new_datum);
    }
  }
  // The published Track views are rebuilt from tree nodes on the next
  // processBatch; the live snapshot would otherwise be stale.
  for (Track& tr : tracks_) {
    shiftStateOnDatumChange(tr.state, tr.covariance, tr.imm_means,
                            tr.imm_covariances, old_datum, new_datum);
  }
  // Transient bias-extraction history holds ENU positions in the old
  // frame; it ages out within kContributionWindowSec, so drop it rather
  // than shift (a recenter is rare and the bias path tolerates a gap).
  contribution_history_.clear();
}

}  // namespace navtracker
