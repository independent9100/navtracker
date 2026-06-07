#pragma once

#include "ports/IDataAssociator.hpp"

namespace navtracker {

// Classical Joint Probabilistic Data Association. For each (track,
// measurement) pair within the validation gate, computes the marginal
// probability beta_jt by enumerating all feasible joint assignments.
class JpdaAssociator : public IDataAssociator {
 public:
  JpdaAssociator(double gate_threshold,
                 double probability_of_detection,
                 double clutter_density);

  AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements,
      const IEstimator* estimator = nullptr) const override;

 private:
  double gate_threshold_;
  double p_d_;
  double lambda_c_;
};

}  // namespace navtracker
