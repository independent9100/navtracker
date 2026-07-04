#pragma once

#include "ports/IDataAssociator.hpp"

namespace navtracker {

class ISensorDetectionModel;

/**
 * Classical Joint Probabilistic Data Association. For each (track,
 * measurement) pair within the validation gate, computes the marginal
 * probability beta_jt by enumerating all feasible joint assignments.
 *
 * Two modes:
 *
 *  - Scalar (P_D, λ_C): the historical single-sensor formulation. Use for
 *    unit tests and single-sensor pipelines. λ_C must be in the
 *    measurement-model's natural units (m^-2 / (m·rad)^-1 / rad^-1).
 *
 *  - Per-sensor via ISensorDetectionModel: the multi-sensor formulation.
 *    Each gated measurement contributes log P_D^s + log p(z|x) − log λ_C^s
 *    in its own sensor's units (this is what makes
 *    log p(z|x) − log λ_C dimensionally consistent across mixed sensors,
 *    see ports/ISensorDetectionModel.hpp). The per-track miss factor is
 *    aggregated over distinct (sensor, model, source_id) tuples present
 *    in the scan via missDetectionProbability — the same convention used
 *    by TrackTree::branch in the MHT path. Brings the single-hypothesis
 *    JPDA path to parity with MHT on per-sensor (P_D, λ_C); step 1 of the
 *    JIPDA upgrade (sota-roadmap.md §2).
 */
class JpdaAssociator : public IDataAssociator {
 public:
  /** Scalar form: legacy single-sensor JPDA. */
  JpdaAssociator(double gate_threshold,
                 double probability_of_detection,
                 double clutter_density);

  /**
   * Per-sensor form: P_D and λ_C resolved per measurement via the model.
   * The pointer must outlive the associator.
   */
  JpdaAssociator(double gate_threshold,
                 const ISensorDetectionModel* detection_model);

  /**
   * Compute the JPDA soft-association result: per (track, measurement)
   * marginal probabilities β_jt over all feasible joint events within the
   * gate, using `estimator` for the gate/likelihood of each pair. Joint-event
   * enumeration is capped at 1e6 events; if a cluster would exceed the cap the
   * result degrades to a greedy mutual-exclusion assignment (β_jt → 0/1) and
   * sets `overflow_fallback = true`.
   */
  AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements,
      const IEstimator* estimator = nullptr) const override;

 private:
  double gate_threshold_;
  double p_d_;
  double lambda_c_;
  const ISensorDetectionModel* detection_model_{nullptr};
};

}  // namespace navtracker
