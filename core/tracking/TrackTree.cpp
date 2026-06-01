#include "core/tracking/TrackTree.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>

#include "core/association/Gating.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/types/Track.hpp"
#include "ports/IEstimator.hpp"

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

void TrackTree::branch(const IEstimator& estimator,
                       const std::vector<Measurement>& scan,
                       Timestamp scan_time,
                       const BranchParams& params) {
  std::vector<std::size_t> current_leaves = leafIndices();
  for (std::size_t leaf_idx : current_leaves) {
    Track tmp_predicted;
    tmp_predicted.state = nodes_[leaf_idx].state;
    tmp_predicted.covariance = nodes_[leaf_idx].covariance;
    tmp_predicted.last_update = nodes_[leaf_idx].time;
    estimator.predict(tmp_predicted, scan_time);

    {
      TrackTreeNode miss;
      miss.parent = leaf_idx;
      miss.scan_idx = nodes_[leaf_idx].scan_idx + 1;
      miss.state = tmp_predicted.state;
      miss.covariance = tmp_predicted.covariance;
      miss.time = scan_time;
      miss.score = nodes_[leaf_idx].score +
                   std::log(1.0 - params.probability_of_detection);
      miss.is_leaf = true;
      nodes_.push_back(miss);
    }

    for (const Measurement& z : scan) {
      Track gate_tr;
      gate_tr.state = tmp_predicted.state;
      gate_tr.covariance = tmp_predicted.covariance;
      const double d2 = mahalanobisDistance(gate_tr, z);
      if (d2 > params.gate_threshold) continue;

      const MeasurementPrediction pred =
          predictMeasurement(z.model, tmp_predicted.state, z.sensor_position_enu);
      const Eigen::MatrixXd S =
          pred.H * tmp_predicted.covariance * pred.H.transpose() + z.covariance;
      const int d = static_cast<int>(z.value.size());
      const double det = S.determinant();
      const double safe_det = (det > 0.0 && std::isfinite(det)) ? det : 1e-300;
      const double log_norm =
          -0.5 * static_cast<double>(d) * std::log(2.0 * M_PI) -
          0.5 * std::log(safe_det);
      const double log_likelihood = log_norm - 0.5 * d2;

      Track child_tr = tmp_predicted;
      estimator.update(child_tr, z);

      TrackTreeNode hit;
      hit.parent = leaf_idx;
      hit.scan_idx = nodes_[leaf_idx].scan_idx + 1;
      hit.state = child_tr.state;
      hit.covariance = child_tr.covariance;
      hit.time = scan_time;
      hit.score = nodes_[leaf_idx].score +
                  std::log(params.probability_of_detection) +
                  log_likelihood -
                  std::log(params.clutter_density);
      hit.is_leaf = true;
      nodes_.push_back(hit);
    }

    nodes_[leaf_idx].is_leaf = false;
  }
}

std::size_t TrackTree::pruneNScan(int n_scan) {
  std::vector<std::size_t> leaves = leafIndices();
  if (leaves.empty()) return 0;
  int max_scan = 0;
  for (std::size_t li : leaves) {
    if (nodes_[li].scan_idx > max_scan) max_scan = nodes_[li].scan_idx;
  }
  const int target_depth = max_scan - n_scan;
  if (target_depth <= 0) return 0;

  std::vector<std::size_t> ancestors(leaves.size());
  for (std::size_t k = 0; k < leaves.size(); ++k) {
    std::size_t cur = leaves[k];
    while (cur != TrackTreeNode::kNoParent &&
           nodes_[cur].scan_idx > target_depth) {
      cur = nodes_[cur].parent;
    }
    ancestors[k] = cur;
  }

  std::vector<std::size_t> unique_ancestors;
  std::vector<double> agg_scores;
  for (std::size_t k = 0; k < leaves.size(); ++k) {
    const std::size_t anc = ancestors[k];
    const double s = nodes_[leaves[k]].score;
    bool found = false;
    for (std::size_t u = 0; u < unique_ancestors.size(); ++u) {
      if (unique_ancestors[u] == anc) {
        if (s > agg_scores[u]) agg_scores[u] = s;
        found = true;
        break;
      }
    }
    if (!found) {
      unique_ancestors.push_back(anc);
      agg_scores.push_back(s);
    }
  }

  std::size_t winner = unique_ancestors[0];
  double best = agg_scores[0];
  for (std::size_t u = 1; u < unique_ancestors.size(); ++u) {
    if (agg_scores[u] > best) {
      best = agg_scores[u];
      winner = unique_ancestors[u];
    }
  }

  std::vector<bool> keep(nodes_.size(), false);
  std::size_t a = winner;
  while (a != TrackTreeNode::kNoParent) {
    keep[a] = true;
    a = nodes_[a].parent;
  }
  std::vector<std::size_t> frontier{winner};
  while (!frontier.empty()) {
    const std::size_t cur = frontier.back();
    frontier.pop_back();
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      if (!keep[i] && nodes_[i].parent == cur) {
        keep[i] = true;
        frontier.push_back(i);
      }
    }
  }

  std::vector<std::size_t> new_index(nodes_.size(), TrackTreeNode::kNoParent);
  std::vector<TrackTreeNode> new_nodes;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (keep[i]) {
      new_index[i] = new_nodes.size();
      new_nodes.push_back(nodes_[i]);
    }
  }
  for (TrackTreeNode& n : new_nodes) {
    if (n.parent != TrackTreeNode::kNoParent) n.parent = new_index[n.parent];
  }
  const std::size_t removed = nodes_.size() - new_nodes.size();
  nodes_ = std::move(new_nodes);
  return removed;
}

std::size_t TrackTree::pruneKLocal(std::size_t k) {
  std::vector<std::size_t> leaves = leafIndices();
  if (leaves.size() <= k) return 0;
  std::sort(leaves.begin(), leaves.end(),
            [this](std::size_t a, std::size_t b) {
              return nodes_[a].score > nodes_[b].score;
            });
  std::size_t dropped = 0;
  for (std::size_t i = k; i < leaves.size(); ++i) {
    nodes_[leaves[i]].is_leaf = false;
    ++dropped;
  }
  return dropped;
}

}  // namespace navtracker
