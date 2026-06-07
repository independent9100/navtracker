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
  struct BranchParams {
    double probability_of_detection;
    double clutter_density;
    double gate_threshold;
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

 private:
  TrackId external_id_;
  std::vector<TrackTreeNode> nodes_;
};

}  // namespace navtracker
