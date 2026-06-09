#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

class IEstimator;
class ISensorDetectionModel;

// One node in a track-tree hypothesis. Index into the tree's nodes vector
// serves as the node_id. Parent index of std::numeric_limits<std::size_t>::max()
// marks the root.
struct TrackTreeNode {
  static constexpr std::size_t kNoParent =
      std::numeric_limits<std::size_t>::max();
  static constexpr std::size_t kNoMeasurement =
      std::numeric_limits<std::size_t>::max();

  std::size_t parent;       // index into nodes vector; kNoParent for root
  int scan_idx;             // 0 at root, +1 per scan
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  Timestamp time;
  double score;             // cumulative log-likelihood-ratio
  bool is_leaf;             // true if current leaf (not yet branched)
  bool is_hit{true};        // true if this node was created by a measurement
                            // (false = missed-detection branch). Roots count
                            // as hits since they were spawned from a meas.
  std::size_t scan_meas_idx{kNoMeasurement};
                            // index into the scan that created this node;
                            // kNoMeasurement for miss / root. Used by the
                            // global-hypothesis solver to enforce
                            // "each detection used at most once across trees".

  // Optional IMM ensemble carrier (parallel to Track::imm_means etc.).
  // Empty for single-mode estimators (EKF/UKF/PF). Required for IMM:
  // ImmEstimator::predict/update bail out when imm_means.cols()==0,
  // so without this the IMM mode mixture would not propagate across
  // scans and tracks would freeze at their seed velocity.
  Eigen::MatrixXd imm_means;
  std::vector<Eigen::MatrixXd> imm_covariances;
  Eigen::VectorXd imm_mode_probabilities;

  // One-scan-deep guard against pruning. Set by MhtTracker on leaves
  // participating in any of the Murty top-K global hypotheses (and
  // their ancestor chain up to the root) after solveGlobalHypothesis;
  // cleared at the top of the next processBatch. Honoured by
  // pruneKLocal (won't demote), mergeBranches (won't merge away), and
  // pruneNScan (won't delete). Implements deferred-commitment TOMHT:
  // alternative hypotheses survive one more scan so evidence has a
  // chance to elevate them past the current K=1 best.
  bool is_protected{false};

  // Existence probability r_k ∈ [0,1] (IPDA / Musicki-Evans-Stankovic
  // 1994). Carried per leaf so MHT can read a calibrated lifecycle
  // signal instead of the raw cumulative LLR score (which is dominated
  // by measurement-fit, not the target-vs-clutter ratio). Updated in
  // TrackTree::branch when BranchParams::update_existence is on; left
  // at 1.0 otherwise (a "no information" sentinel that makes IPDA-off
  // behaviour bit-identical to the legacy code path).
  double existence_probability{1.0};

  // Visibility-given-exists v_k ∈ [0,1] (Brekke & Wilthil 2019,
  // VIMM-JIPDA). Separates "track is gone" (existence drops) from
  // "track is currently obscured" (visibility drops, existence stable).
  // Updated in TrackTree::branch when BranchParams::update_visibility
  // is on. Default 1.0 = "fully visible" so VIMM-off math collapses
  // back to plain IPDA.
  double visibility_given_exists{1.0};
};

// A per-track hypothesis tree.
class TrackTree {
 public:
  TrackTree(TrackId external_id, const TrackTreeNode& root);

  TrackId externalId() const { return external_id_; }
  const std::vector<TrackTreeNode>& nodes() const { return nodes_; }
  std::vector<TrackTreeNode>& mutableNodes() { return nodes_; }

  std::vector<std::size_t> leafIndices() const;
  std::size_t bestLeafIndex() const;

  // Branching parameters for one scan.
  //
  // Hit branches use per-measurement (P_D, λ_C) looked up from the
  // detection model (correct units per sensor — see
  // ISensorDetectionModel). The miss branch uses a single
  // `miss_probability_of_detection` value because the miss is a
  // *per-track* event, not per-measurement; deciding which sensor "should
  // have detected" the track would require multi-sensor footprint
  // accounting that we don't carry yet. The default is the configured
  // global P_D — equivalent to today.
  //
  // IPDA / VIMM lifecycle: when `update_existence` is set, each new
  // child node carries an existence_probability updated by a Bayes
  // recursion using (P_D, λ_C) from the detection model + the
  // estimator's measurement likelihood. When `update_visibility` is
  // also set, the joint (existence, visibility) update of Brekke &
  // Wilthil 2019 (VIMM-JIPDA) applies — missed detections under
  // obscuration decay visibility rather than existence. Both default
  // to false → tree-node existence/visibility stay at their 1.0
  // sentinels and lifecycle math is a no-op (bit-identical to legacy).
  struct BranchParams {
    const ISensorDetectionModel* detection_model;
    double miss_probability_of_detection;
    double gate_threshold;
    // IPDA / VIMM controls. The `existence_persistence` and
    // `visibility_*` knobs are only consulted when the corresponding
    // update flag is on.
    bool update_existence{false};
    bool update_visibility{false};
    double existence_persistence{0.99};       // P(e_k=1 | e_{k-1}=1)
    double gate_probability_mass{0.99};       // P_G — gate-captured mass
                                              // under target-present
    double visibility_persistence{0.95};      // P(v_k=1 | v_{k-1}=1)
    double visibility_recovery{0.3};          // P(v_k=1 | v_{k-1}=0)
  };

  // N-scan pruning. For each current leaf, walk back N steps via parent
  // pointers to find its scan-(t-N) ancestor. Among all (leaf, ancestor)
  // pairs, find the ancestor with the highest descendant leaf score; mark
  // all other ancestors at that depth (and their descendants) for removal.
  // Returns the number of nodes removed.
  std::size_t pruneNScan(int n_scan);

  // Keep only the top-k highest-scoring leaves. Lower-scoring leaves are
  // marked is_leaf = false but kept in nodes_ (for future N-scan ancestor
  // walks). Returns the number of leaves dropped.
  std::size_t pruneKLocal(std::size_t k);

  // Branch every current leaf by:
  //   - generating one missed-detection child (state advanced via predict only;
  //     score += log(1 - P_D))
  //   - for each gated measurement, generating one child with EKF-updated
  //     state + score += log(P_D) + log N(z|x,P) - log lambda_C
  // The old leaves get is_leaf = false; new nodes are is_leaf = true.
  void branch(const IEstimator& estimator,
              const std::vector<Measurement>& scan,
              Timestamp scan_time,
              const BranchParams& params);

  // Count how many nodes on the ancestry chain of `leaf` (inclusive) are
  // hits (created from a measurement, not a missed-detection branch),
  // walking back at most `window` levels. Used by M-of-N confirmation
  // gating on the MhtTracker.
  int countHitsInWindow(std::size_t leaf, int window) const;

  // Merge near-identical leaves within this tree by Bhattacharyya
  // distance between their (state, covariance) Gaussians (position
  // block only — kinematic similarity, not score similarity). For each
  // pair with B(p,q) < `threshold`, the lower-scoring leaf is marked
  // is_leaf = false. Returns the number of leaves dropped.
  // Bhattacharyya for two Gaussians (μ_a, Σ_a), (μ_b, Σ_b):
  //   B = 1/8 (μ_a − μ_b)ᵀ Σ^{-1} (μ_a − μ_b)
  //       + 1/2 ln(|Σ| / √(|Σ_a||Σ_b|)),  Σ = (Σ_a + Σ_b)/2.
  // Smaller B = more similar. Typical threshold 0.5–2.0.
  std::size_t mergeBranches(double threshold);

  // Number of distinct alternative-hypothesis leaves protected on the
  // previous scan (== |top_k_leaves[this_tree]| from the last
  // solveGlobalHypothesis call, after the Score-Δ filter). Used by
  // adaptive N-scan to delay trunk merging on trees with surviving
  // alternatives. >1 means "more than just the K=1 best was protected"
  // — extend n_scan.
  std::size_t protectedAlternativesLastScan() const {
    return protected_alternatives_last_scan_;
  }
  void setProtectedAlternativesLastScan(std::size_t n) {
    protected_alternatives_last_scan_ = n;
  }

 private:
  TrackId external_id_;
  std::vector<TrackTreeNode> nodes_;
  std::size_t protected_alternatives_last_scan_ = 0;
};

}  // namespace navtracker
