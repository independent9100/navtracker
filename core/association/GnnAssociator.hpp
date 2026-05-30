#pragma once

#include "ports/IDataAssociator.hpp"

namespace navtracker {

// Greedy global-nearest-neighbor association over gated squared Mahalanobis
// distance. Repeatedly assigns the globally smallest in-gate pair.
class GnnAssociator : public IDataAssociator {
 public:
  explicit GnnAssociator(double gate_threshold);

  AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements) const override;

 private:
  double gate_threshold_;  // chi-square gate on squared Mahalanobis distance
};

}  // namespace navtracker
