#pragma once

#include <functional>
#include <vector>

#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IEstimator.hpp"

// Poisson Multi-Bernoulli Mixture (PMBM) tracker. Sibling to MhtTracker,
// implementing the same multi-target tracking goal via the Random Finite
// Set (RFS) formulation rather than a hypothesis tree.
//
// See:
//   docs/algorithms/pmbm-design.md   — equation-level reference
//   docs/learning/23-pmbm.md         — plain-English introduction
//   docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md
//                                    — phased engineering plan
//
// Phase 1 (this file): GM-PMBM. Per-Bernoulli single-target density is a
// single Gaussian; estimator is whatever the caller injects (typically
// EKF or UKF). The standard point-target model is assumed (one
// measurement per target per scan, Poisson clutter, Poisson birth).
//
// Phase 2 swaps the per-Bernoulli density to an IMM mixture by
// extending the Bernoulli adapter to round-trip the imm_* fields on
// Track. Phase 3 (TPMBM) extends each Bernoulli with a trajectory
// (state history); structure unchanged otherwise.

namespace navtracker::pmbm {

class PmbmTracker {
 public:
  struct Config {
    // Per-scan target survival probability p_S. Applied multiplicatively
    // to every Bernoulli's existence probability and every PPP
    // component's weight during predict. 0.99 is the standard textbook
    // value; deployment may want sensor-conditional values for ships
    // entering/leaving radar coverage (future work).
    double survival_probability = 0.99;

    // Pruning thresholds. Components below threshold are dropped at the
    // end of each update. Phase 1 applies them only via accessors
    // (Bernoulli::isAlive) and tests; the predict path does not prune.
    double r_min = 1e-3;
    double weight_min = 1e-4;
    double hypothesis_weight_min = 1e-4;
  };

  // Birth intensity callback. Called once per predict() (after the
  // survival decay), supplying the new filter time and the elapsed dt.
  // Returns PPP components to add to the current PPP intensity. Empty
  // (default-constructed std::function) installs a no-birth model — the
  // tracker still runs but no new targets can ever appear. Useful for
  // closed-set tests; real deployments must supply a model.
  using BirthModelFn =
      std::function<std::vector<PoissonComponent>(Timestamp, double)>;

  PmbmTracker(const IEstimator& estimator, Config cfg,
              BirthModelFn birth_model = {});

  // Advance the PPP intensity, every Bernoulli in every global
  // hypothesis, and the filter time to `to`. Birth intensity (per the
  // BirthModelFn) is added after the survival decay.
  //
  // The first call after construction initialises the filter time from
  // `to` (dt = 0; birth model is called with dt = 0 to allow seeding an
  // initial PPP); subsequent calls compute dt from the previous filter
  // time. Calls with `to` ≤ currentTime() advance the clock only (no
  // propagation, no birth) — matches the MhtTracker stale-input
  // convention without raising.
  void predict(Timestamp to);

  const PmbmDensity& density() const noexcept { return density_; }
  Timestamp currentTime() const noexcept { return current_time_; }
  bool hasCurrentTime() const noexcept { return has_current_time_; }
  const Config& config() const noexcept { return cfg_; }

  // Test / introspection hooks.
  PmbmDensity& mutableDensityForTesting() { return density_; }

 private:
  const IEstimator& estimator_;
  Config cfg_;
  BirthModelFn birth_model_;
  PmbmDensity density_;
  Timestamp current_time_{};
  bool has_current_time_{false};
};

}  // namespace navtracker::pmbm
