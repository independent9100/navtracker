#pragma once

#include <memory>
#include <random>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

/**
 * Bootstrap (SIR) particle filter behind IEstimator. Carries a weighted
 * ensemble on the Track itself; projects to a Gaussian (mean, covariance) so
 * downstream consumers (gating, sinks) are estimator-agnostic.
 *
 * Determinism: a single internal RNG drives the three stochastic steps —
 * `initiate` draws the initial ensemble (only when the birth covariance is
 * PD), `predict` draws process noise (only when Q(dt) is PD; skipped when
 * q == 0 / singular), and `update` draws resampling offsets (only when the
 * ESS drops below threshold). A deterministic step advances the RNG the same
 * way every time, so replaying the same message stream against a
 * freshly-seeded instance reproduces identical particles, weights, and
 * projected state.
 *
 * State layout follows the motion model: `initiate` sizes the ensemble
 * from `motion_->stateDim()` (CV2D → 4-state; CT / CV5State → 5-state with
 * ω as the trailing entry, seeded with `init_omega_std`). `predict` applies
 * the model's per-particle nonlinear `propagate(x, dt)` so a coordinated-
 * turn model actually turns each particle, rather than collapsing to the
 * linear CV limit.
 */
class ParticleFilterEstimator : public IEstimator {
 public:
  /**
   * Wire the filter with its `motion` model. `particle_count` is the ensemble
   * size N. `init_speed_std` seeds per-axis velocity 1-σ on initiate;
   * `init_omega_std` seeds turn-rate 1-σ for 5-state motion models.
   * `ess_fraction_threshold` is the resampling trigger as a fraction of
   * `particle_count` — resampling fires when the effective sample size drops
   * below `ess_fraction_threshold · particle_count`. `seed` fixes the
   * deterministic RNG (equal seeds → identical output for identical input).
   */
  ParticleFilterEstimator(std::shared_ptr<const IMotionModel> motion,
                          int particle_count = 500,
                          double init_speed_std = 10.0,
                          double ess_fraction_threshold = 0.5,
                          std::uint64_t seed = 0,
                          double init_omega_std = 0.1);

  /** Propagate each particle through the motion model to time `to`. */
  void predict(Track& track, Timestamp to) const override;
  /** Reweight the ensemble by the measurement likelihood, resample if the ESS drops below threshold, and reproject. */
  void update(Track& track, const Measurement& z) const override;
  /** Seed a new ensemble from measurement `z`, sized to the motion model's state dimension. */
  Track initiate(const Measurement& z) const override;

 private:
  void projectToGaussian(Track& track) const;

  std::shared_ptr<const IMotionModel> motion_;
  int particle_count_;
  double init_speed_std_;
  double ess_threshold_;     // absolute (= fraction · N)
  double init_omega_std_;
  mutable std::mt19937_64 rng_;
};

}  // namespace navtracker
