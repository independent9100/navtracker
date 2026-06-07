#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Context for PDAF/JPDA soft updates. Carries the same parameters the
// associator used to compute the betas, so the estimator can build the
// textbook per-mode mixture likelihood
//   Λ = β₀ + (1−β₀)/(V·P_D) · Σ_m β_m N(y_m; 0, S)
// where V is the (per-mode) gate volume derived from `gate_threshold` and
// the mode's innovation covariance S. If `p_d` ≤ 0 the estimator falls
// back to the unnormalized proxy `Λ ∝ β₀ + Σ_m β_m N(...)` — relative
// mode ordering is preserved, only absolute magnitudes change.
struct PdaContext {
  double p_d{0.0};              // detection probability used by the associator
  double gate_threshold{0.0};   // chi-square gate threshold (squared Mahalanobis)
};

// Recursive state estimator strategy. Implementations advance and correct a
// track's kinematic state/covariance.
class IEstimator {
 public:
  virtual ~IEstimator() = default;

  // Advance the track's state and covariance to time `to`.
  virtual void predict(Track& track, Timestamp to) const = 0;

  // Fold a measurement into the track. Assumes the track was already
  // predicted to z.time.
  virtual void update(Track& track, const Measurement& z) const = 0;

  // Create a new Tentative track seeded from a position-type measurement.
  virtual Track initiate(const Measurement& z) const = 0;

  // Soft update for probabilistic data association (PDAF / JPDA). Default
  // is no-op; estimators that support soft updates override this.
  // `betas(j)` = P(measurement j came from this track | data),
  // `beta_0` = P(no measurement assigned to this track this scan).
  // sum_j betas(j) + beta_0 == 1. `ctx` carries P_D and the gate
  // threshold so multi-mode estimators can normalize the per-mode
  // mixture likelihood properly; a default-constructed ctx selects
  // the unnormalized proxy.
  virtual void softUpdate(Track& /*track*/,
                          const std::vector<Measurement>& /*gated_measurements*/,
                          const Eigen::VectorXd& /*betas*/,
                          double /*beta_0*/,
                          const PdaContext& /*ctx*/ = {}) const {}

  // Gate test: is `z` plausible enough to be a real association
  // candidate for `track`? `gate_threshold` is the squared-Mahalanobis
  // chi-square cutoff (caller passes the same value it uses elsewhere
  // — e.g., associator's gate_threshold_).
  //
  // For single-mode estimators this is the standard Mahalanobis gate.
  // For IMM, the default implementation (any-mode gating, Mazor 1998
  // §V) returns true iff *any* mode's per-mode innovation gate passes
  // — which is what the textbook recommends, since the dominant mode
  // determines which spread is operationally correct.
  //
  // Default implementation works through track.state / track.covariance
  // (the moment-matched projection) — correct for EKF/UKF/PF, biased
  // loose for IMM. IMM overrides to compute per-mode gates.
  virtual bool gate(const Track& track,
                    const Measurement& z,
                    double gate_threshold) const;

  // Scalar log-likelihood log p(z | track) used by data associators
  // (JPDA hypothesis weighting, MHT branch scoring). For single-mode
  // estimators this is the standard Gaussian log N(z; h(x), H P H' + R).
  // For IMM, the default implementation returns the mode-mixture
  // log Σⱼ μⱼ · N(z; h(xⱼ), Sⱼ) — strictly more honest than the
  // moment-matched log N which double-counts inter-mode spread.
  virtual double logLikelihood(const Track& track,
                               const Measurement& z) const;
};

}  // namespace navtracker
