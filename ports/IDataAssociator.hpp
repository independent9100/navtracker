#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Output of associating a batch of measurements to existing tracks. Indices
// refer into the input `tracks` and `measurements` vectors.
struct AssociationResult {
  // Hard-association path (GNN, etc.). Empty if the associator returned soft
  // probabilities instead.
  std::vector<std::pair<std::size_t, std::size_t>> matches;  // (track_idx, meas_idx)
  std::vector<std::size_t> unmatched_tracks;
  std::vector<std::size_t> unmatched_measurements;

  // Soft-association path (JPDA, JIPDA). betas(j, t) = P(measurement j came
  // from track t | data). beta_0(t) = P(track t has no measurement this
  // scan). Empty if the associator returned hard matches instead.
  Eigen::MatrixXd betas;       // shape M x T, rows = measurements
  Eigen::VectorXd beta_0;      // shape T,    per-track no-detection prob

  // PDA parameters the associator used. Forwarded into the estimator's
  // soft update so multi-mode estimators can compute the textbook
  // mixture likelihood with proper V·P_D normalization. Zero on
  // hard-association results.
  double p_d{0.0};
  double gate_threshold{0.0};
};

// Data-association strategy: assign measurements to tracks.
class IDataAssociator {
 public:
  virtual ~IDataAssociator() = default;
  virtual AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements) const = 0;
};

}  // namespace navtracker
