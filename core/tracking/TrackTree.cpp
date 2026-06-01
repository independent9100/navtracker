#include "core/tracking/TrackTree.hpp"

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
          predictMeasurement(z.model, tmp_predicted.state);
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

}  // namespace navtracker
