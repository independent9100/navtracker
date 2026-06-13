#pragma once

#include <memory>
#include <vector>

#include "ports/IEstimator.hpp"
#include "ports/IMeasurementNoiseModel.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Interacting Multiple Model estimator. Carries K modes per track; each
// mode runs an EKF predict/update against its own motion model. The Track
// kinematic carrier (state, covariance) is the moment-matched projection
// of the per-mode mixture. State is unified 5-d: [px, py, vx, vy, omega].
//
// Mixing happens inside `predict`, which advances the mode probabilities
// to the predicted prior c = π(dt)ᵀ μ; `update` consumes that prior and
// folds in the per-mode measurement likelihoods (μ⁺ ∝ c_j Λ_j).
//
// dt semantics: the configured transition matrix is the ONE-SECOND TPM;
// predict applies π(dt) = π^dt (matrix fractional power — equivalent to
// expm(logm(π)·dt) for embeddable chains, which diagonally-dominant
// tracking TPMs are). Rationale: per-call application makes the mode
// mixing rate proportional to the measurement cadence — at 16 Hz the
// modes wash toward stationarity between every measurement and the IMM
// degenerates to a blurred single model (observed on AutoFerry replays).
// π^dt keeps the mixing rate a property of the configuration, not of the
// sensor event rate, and makes prediction a semigroup: two 0.5 s
// predicts equal one 1.0 s predict.
class ImmEstimator : public IEstimator {
 public:
  // motions.size() == K; transition_matrix is K×K with rows summing to 1
  // (pi[i][j] = P(mode j next | mode i now)) — interpreted as the TPM at
  // dt = 1 s; initial_mode_probabilities is K, sums to 1.
  ImmEstimator(std::vector<std::shared_ptr<IMotionModel>> motions,
               Eigen::MatrixXd transition_matrix,
               Eigen::VectorXd initial_mode_probabilities,
               double init_speed_std = 10.0,
               double init_omega_std = 0.1,
               std::shared_ptr<const IMeasurementNoiseModel> noise = nullptr,
               bool bearing_range_guard = false);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;
  void softUpdate(Track& track,
                  const std::vector<Measurement>& gated_measurements,
                  const Eigen::VectorXd& betas,
                  double beta_0,
                  const PdaContext& ctx = {}) const override;
  Track initiate(const Measurement& z) const override;

  // Any-mode gate (Mazor 1998 §V): true iff ANY mode's per-mode
  // innovation gate passes. Tighter than the moment-matched gate in
  // maneuvering regimes; looser than most-likely-mode gating.
  bool gate(const Track& track,
            const Measurement& z,
            double gate_threshold) const override;

  // Mode-weighted log-likelihood:
  //   log Σⱼ μⱼ · N(z; h(x_j), H_j P_j H_j' + R).
  // Computed via log-sum-exp. Strictly more honest than the
  // moment-matched log N when modes disagree.
  double logLikelihood(const Track& track,
                       const Measurement& z) const override;

 private:
  void projectMixtureToTrack(Track& track) const;

  // π^dt — the 1-second TPM raised to dt. Falls back to the per-call
  // matrix when the fractional power is not well-defined (NaNs from a
  // non-embeddable chain); clamps tiny negative entries and
  // re-normalises rows.
  Eigen::MatrixXd transitionFor(double dt) const;

  std::vector<std::shared_ptr<IMotionModel>> motions_;
  Eigen::MatrixXd pi_;
  Eigen::VectorXd mu0_;
  double init_speed_std_;
  double init_omega_std_;
  std::shared_ptr<const IMeasurementNoiseModel> noise_;
  bool bearing_range_guard_;
};

}  // namespace navtracker
