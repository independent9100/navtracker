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
//   3. Prune N-scan-old branches (trunk merging).
//   4. Spawn new trees for measurements that gated to no tree.
//   5. Drop trees whose best-leaf score falls below score_delete_threshold.
//   6. Materialize one Track per tree (state from best leaf, id from tree's
//      externalId) into `tracks_` for downstream consumers.
//
// Track identity is stable: each tree gets a fresh monotonic id at spawn
// time; ids are never reused across the MhtTracker's lifetime.
class MhtTracker {
 public:
  struct Config {
    double probability_of_detection = 0.9;
    double clutter_density = 1e-4;
    double gate_threshold = 9.0;
    int n_scan = 3;
    std::size_t k_max_leaves = 5;
    double score_delete_threshold = -15.0;
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
