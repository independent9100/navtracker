#include "core/tracking/TrackTree.hpp"

namespace navtracker {

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

}  // namespace navtracker
