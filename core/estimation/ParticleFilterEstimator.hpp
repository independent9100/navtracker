#pragma once

#include <memory>
#include <random>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Bootstrap (SIR) particle filter behind IEstimator. Carries a weighted
// ensemble on the Track itself; projects to a Gaussian (mean, covariance) so
// downstream consumers (gating, sinks) are estimator-agnostic.
//
// Determinism: a single internal RNG is advanced by every predict / update /
// initiate call. Replaying the same message stream against a freshly-seeded
// instance reproduces identical particles, weights, and projected state.
class ParticleFilterEstimator : public IEstimator {
 public:
  ParticleFilterEstimator(std::shared_ptr<const IMotionModel> motion,
                          int particle_count = 500,
                          double init_speed_std = 10.0,
                          double ess_fraction_threshold = 0.5,
                          std::uint64_t seed = 0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;
  Track initiate(const Measurement& z) const override;

 private:
  void projectToGaussian(Track& track) const;

  std::shared_ptr<const IMotionModel> motion_;
  int particle_count_;
  double init_speed_std_;
  double ess_threshold_;     // absolute (= fraction · N)
  mutable std::mt19937_64 rng_;
};

}  // namespace navtracker
