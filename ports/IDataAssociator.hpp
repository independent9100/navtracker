#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Output of associating a batch of measurements to existing tracks. Indices
 * refer into the input `tracks` and `measurements` vectors.
 */
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

  // True when a soft associator (JPDA) exceeded its joint-event budget for
  // some cluster and fell back to greedy hard assignment. Diagnostic only —
  // betas/beta_0 stay valid (hard 0/1 for the fallen-back cluster). Lets
  // the harness flag real-clutter scans that outran full enumeration.
  bool overflow_fallback{false};
};

class IEstimator;

/**
 * Data-association strategy: assign measurements to tracks.
 *
 * `estimator` is optional; when non-null the associator routes gate
 * and log-likelihood through it (`estimator->gate(...)` and
 * `estimator->logLikelihood(...)`). This is what makes the IMM/JPDA
 * and IMM/MHT combinations honest — multi-mode estimators expose
 * any-mode gating and mode-mixture likelihoods that no single-Gaussian
 * surrogate can reproduce. When null, the associator falls back to
 * the single-Gaussian path computed from `track.state, track.covariance`
 * directly — same behaviour as before this parameter existed, suitable
 * for unit tests and EKF/UKF/PF callers.
 */
class IDataAssociator {
 public:
  virtual ~IDataAssociator() = default;
  /**
   * Associate `measurements` to `tracks`, optionally routing gate and
   * log-likelihood through `estimator`. Returns hard matches or soft
   * betas per the concrete strategy.
   */
  virtual AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements,
      const IEstimator* estimator = nullptr) const = 0;
};

}  // namespace navtracker
