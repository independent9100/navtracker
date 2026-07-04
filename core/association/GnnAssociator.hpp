#pragma once

#include "ports/IDataAssociator.hpp"

namespace navtracker {

/**
 * Greedy global-nearest-neighbor association over gated in-gate cost.
 * Repeatedly assigns the globally smallest in-gate pair, where the cost is the
 * squared Mahalanobis distance without an estimator and −logLikelihood with
 * one (equivalent for single-mode Gaussians).
 */
class GnnAssociator : public IDataAssociator {
 public:
  explicit GnnAssociator(double gate_threshold);

  /**
   * Associate `measurements` to `tracks` by repeatedly committing the
   * globally smallest in-gate cost pair — squared Mahalanobis distance
   * without an estimator, −logLikelihood when `estimator` is supplied (which
   * then also provides the gate/likelihood evaluation for each pair).
   */
  AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements,
      const IEstimator* estimator = nullptr) const override;

 private:
  double gate_threshold_;  // chi-square gate on squared Mahalanobis distance
};

}  // namespace navtracker
