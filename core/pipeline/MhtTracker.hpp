#pragma once

#include <cstdint>
#include <vector>

#include "core/tracking/TrackTree.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {

// Track-oriented MHT (TOMHT). Per scan:
//   1. Branch each tree's leaves on every gated measurement + missed-detection.
//   2. Prune K_local-worst leaves per tree.
//   3. Merge near-duplicate leaves within each tree by Bhattacharyya distance.
//   4. Prune N-scan-old branches (trunk merging).
//   5. Spawn new trees for measurements that gated to no tree.
//   6. Drop trees whose best-leaf score falls below score_delete_threshold.
//   7. Solve the global hypothesis (Hungarian K=1): pick one leaf per tree
//      so that no scan measurement is consumed by more than one tree.
//   8. Apply M-of-N confirmation gate: emit only trees with at least
//      `confirm_hits_needed` hits in the last `confirm_hits_window` nodes.
//   9. Materialize one Track per surviving tree (state from selected leaf,
//      id from tree's externalId) into `tracks_` for downstream consumers.
//
// Track identity is stable: each tree gets a fresh monotonic id at spawn
// time; ids are never reused across the MhtTracker's lifetime. The
// Tentative/Confirmed status of an emitted Track reflects the M-of-N gate.
//
// References:
//   - Blackman & Popoli (1999), Modern Tracking Systems ch. 16
//   - Blackman (2004), IEEE AES Mag 19(1) §III–§V
//   - Kuhn (1955) / Munkres (1957) for the assignment solver
class MhtTracker {
 public:
  struct Config {
    double probability_of_detection = 0.9;
    double clutter_density = 1e-4;
    double gate_threshold = 9.0;
    int n_scan = 3;
    std::size_t k_max_leaves = 5;
    double score_delete_threshold = -15.0;

    // M-of-N confirmation. A tree's best selected leaf must have at
    // least `confirm_hits_needed` hits among its last
    // `confirm_hits_window` nodes for the tree to be reported as
    // Confirmed; otherwise the emitted Track stays Tentative.
    int confirm_hits_needed = 2;
    int confirm_hits_window = 3;

    // Bhattacharyya distance threshold for within-tree leaf merging.
    // Set <= 0 to disable. Typical 0.5–2.0 (smaller = stricter, fewer
    // merges).
    double merge_bhattacharyya_threshold = 1.0;
  };

  MhtTracker(const IEstimator& estimator, Config cfg);

  void processBatch(const std::vector<Measurement>& scan);

  const std::vector<Track>& tracks() const { return tracks_; }
  std::size_t treeCount() const { return trees_.size(); }

 private:
  const IEstimator& estimator_;
  Config cfg_;
  std::vector<TrackTree> trees_;
  std::vector<Track> tracks_;
  std::uint64_t next_external_id_{1};
};

}  // namespace navtracker
