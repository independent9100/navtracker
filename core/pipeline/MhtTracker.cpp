#include "core/pipeline/MhtTracker.hpp"

#include <utility>

#include "core/association/Gating.hpp"

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
  root.time = z.time;
  root.score = 0.0;
  root.is_leaf = true;
  return root;
}

}  // namespace

void MhtTracker::processBatch(const std::vector<Measurement>& scan) {
  if (scan.empty()) return;
  const Timestamp t = scan.front().time;

  TrackTree::BranchParams bp{
      cfg_.probability_of_detection,
      cfg_.clutter_density,
      cfg_.gate_threshold};
  for (TrackTree& tt : trees_) {
    tt.branch(estimator_, scan, t, bp);
    tt.pruneKLocal(cfg_.k_max_leaves);
    tt.pruneNScan(cfg_.n_scan);
  }

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
      const double d2 = mahalanobisDistance(gate_tr, scan[j]);
      if (d2 <= cfg_.gate_threshold) { gated_to_any = true; break; }
    }
    measurement_explained[j] = gated_to_any;
  }
  for (std::size_t j = 0; j < scan.size(); ++j) {
    if (measurement_explained[j]) continue;
    const TrackId id{next_external_id_++};
    trees_.emplace_back(id, rootFromMeasurement(estimator_, scan[j]));
  }

  std::vector<TrackTree> kept;
  kept.reserve(trees_.size());
  for (TrackTree& tt : trees_) {
    const std::size_t best = tt.bestLeafIndex();
    if (best == TrackTreeNode::kNoParent) continue;
    if (tt.nodes()[best].score < cfg_.score_delete_threshold) continue;
    kept.push_back(std::move(tt));
  }
  trees_ = std::move(kept);

  tracks_.clear();
  tracks_.reserve(trees_.size());
  for (const TrackTree& tt : trees_) {
    const std::size_t best = tt.bestLeafIndex();
    Track view;
    view.id = tt.externalId();
    view.state = tt.nodes()[best].state;
    view.covariance = tt.nodes()[best].covariance;
    view.last_update = tt.nodes()[best].time;
    view.status = TrackStatus::Confirmed;
    tracks_.push_back(std::move(view));
  }
}

}  // namespace navtracker
