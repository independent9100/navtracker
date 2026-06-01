#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// One node in a track-tree hypothesis. Index into the tree's nodes vector
// serves as the node_id. Parent index of std::numeric_limits<std::size_t>::max()
// marks the root.
struct TrackTreeNode {
  static constexpr std::size_t kNoParent =
      std::numeric_limits<std::size_t>::max();

  std::size_t parent;       // index into nodes vector; kNoParent for root
  int scan_idx;             // 0 at root, +1 per scan
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  Timestamp time;
  double score;             // cumulative log-likelihood-ratio
  bool is_leaf;             // true if current leaf (not yet branched)
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

 private:
  TrackId external_id_;
  std::vector<TrackTreeNode> nodes_;
};

}  // namespace navtracker
