#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

// Data structures for the Poisson Multi-Bernoulli Mixture (PMBM) filter.
// Header-only at this stage — the types are POD-like value semantics
// containers used by PmbmTracker and tested in isolation.
//
// See docs/algorithms/pmbm-design.md for the math and
// docs/learning/23-pmbm.md for the intuition. The plain-English
// recap:
//
//   PPP (Poisson Point Process) — a Gaussian-mixture intensity over
//   single-target state space representing targets that *may* exist but
//   have not been detected yet. New Bernoullis are born from this when a
//   measurement falls on PPP mass.
//
//   Bernoulli — one possible detected target. (r, mean, covariance) with
//   r ∈ [0,1] the existence probability.
//
//   GlobalHypothesis — one complete "who is who" interpretation of the
//   measurement history. Carries a weight and the Bernoullis that exist
//   under this interpretation.
//
//   PmbmDensity — the full posterior: the PPP plus the weighted mixture
//   of GlobalHypotheses (MBM).

namespace navtracker::pmbm {

// One point along a Bernoulli's trajectory (TPMBM, Phase 4).
//
// In standard PMBM each Bernoulli carries only the *current* state
// `(mean, covariance)` as a single-target density. Trajectory-PMBM
// (García-Fernández, Williams, Granström, Svensson 2020,
// arXiv:1912.08718) extends this so each Bernoulli carries the
// posterior over the *entire* trajectory from birth to now —
// concretely a vector of `TrajectoryPoint` recording the
// (smoothed-or-filtered) state at each scan the Bernoulli was alive.
//
// Phase 4 incremental: forward-pass filter trajectory (no smoothing
// yet). The state recorded here is the post-update state at each
// observation, or the predicted state on misdetection scans. RTS
// smoothing back through the trajectory is the next increment
// (Phase 4(B)). Doesn't change the per-scan tracking math but
// (a) gives ITrackSink consumers the full track history on
// `onTrackDeleted` and (b) is the necessary scaffold for smoothing.
struct TrajectoryPoint {
  Timestamp time;
  Eigen::VectorXd state;        // n_state (post-update at this scan)
  Eigen::MatrixXd covariance;   // n_state × n_state (post-update)

  // Phase 4(C) RTS smoother inputs. The post-PREDICT, pre-UPDATE
  // state and covariance at this scan's time (i.e., x_{k|k-1} and
  // P_{k|k-1} in textbook notation). For the FIRST trajectory point
  // (birth), there is no prior, so predicted_* equal state /
  // covariance (smoother gain G = I at that step, no effect).
  // Required by the backward RTS pass: gain
  //   G_{k-1} ≈ P_filt_{k-1} · P_pred_k^{-1}  (F ≈ I approximation)
  // operates on the PREVIOUS scan's filtered cov and the CURRENT
  // scan's predicted cov. Carrying both per-point keeps the smoother
  // stateless w.r.t. the filter run.
  Eigen::VectorXd predicted_state;
  Eigen::MatrixXd predicted_covariance;
};

// Backward Rauch-Tung-Striebel smoother over a trajectory.
//
// **Math.** Starting from the final point (smoothed = filtered),
// walk backward k = T-1 .. 0:
//   G_k = P_filt_k · F_k^T · P_pred_{k+1}^{-1}
//   x_smooth_k = x_filt_k + G_k · (x_smooth_{k+1} − x_pred_{k+1})
//   P_smooth_k = P_filt_k + G_k · (P_smooth_{k+1} − P_pred_{k+1}) · G_k^T
// where x_pred_{k+1} / P_pred_{k+1} come from TrajectoryPoint at
// k+1's `predicted_*` fields.
//
// **Assumption (this implementation).** F_k ≈ I. Correct for
// stationary targets, BIASED for moving targets (position-velocity
// cross-terms lost). The covariance-weighted blend G ≈ P_filt ·
// P_pred^{-1} still does useful work — the smoothed estimate at k
// is pulled toward the better-informed posterior at k+1 — but the
// magnitude of correction is conservative. A correct F (e.g.,
// constant-velocity F derived from dt) would require either
// extending IEstimator with a `transitionMatrix(track, t)` method
// (clean) or hard-coding the CV transition (rigid). Left as
// Phase 4(C)' refinement.
//
// **Rationale.** RTS only helps PAST states (k < T). The
// current-scan filter estimate at T is already optimal, so per-
// scan GOSPA is unchanged. The win comes through T-GOSPA, which
// integrates trajectory error across all k — there RTS smoothing
// of past points reduces accumulated error.
//
// **Ways to improve.** Add `transitionMatrix` to IEstimator, then
// pass the per-step F through the smoother. IMM-aware smoothing
// (per-mode RTS then mode mixing) is a further refinement.
//
// In-place modification of `trajectory`. No-op when fewer than 2
// points (nothing to smooth).
void rtsSmoothTrajectory(std::vector<TrajectoryPoint>& trajectory);

// Stable identifier for a Bernoulli component across hypotheses and
// scans. Two Bernoullis in different global hypotheses with the same id
// refer to the *same* target — they are alternative state distributions
// for that target conditional on different association histories. Ids
// are minted by the tracker; never reused. Track output aggregation
// (TrackOutput) keys on this id.
//
// Distinct from TrackId because PMBM may suppress a Bernoulli (existence
// → 0) before a TrackId is ever assigned, and conversely an aggregated
// TrackId may correspond to several Bernoullis-of-the-same-id across
// hypotheses. The mapping is one TrackId ↔ one BernoulliId, finalised
// at output time.
using BernoulliId = std::uint64_t;
constexpr BernoulliId kInvalidBernoulliId = 0;

// One Gaussian component of the PPP intensity λ^u(x). The full PPP is
// λ^u(x) = Σ_i weight_i · 𝒩(x; mean_i, covariance_i).
//
// `weight` is an intensity, not a probability — it can sum to > 1
// across components. It represents the expected number of undetected
// targets per unit state-space volume contributed by this component.
struct PoissonComponent {
  double weight{0.0};
  Eigen::VectorXd mean;        // n_state
  Eigen::MatrixXd covariance;  // n_state × n_state

  // Optional IMM ensemble carrier — same shape and semantics as
  // Bernoulli::imm_*. Populated when the injected estimator is IMM
  // (the PMBM birth path round-trips imm_* from estimator.initiate),
  // empty otherwise. Carried so the next scan's estimator.predict /
  // estimator.update sees the IMM fields and does real work instead
  // of the cols()==0 fast-path no-op.
  Eigen::MatrixXd imm_means;
  std::vector<Eigen::MatrixXd> imm_covariances;
  Eigen::VectorXd imm_mode_probabilities;

  // Convenience: integrated mass under this component (just `weight`
  // for a normalised Gaussian — provided as a name for readability at
  // call sites).
  double mass() const noexcept { return weight; }
};

// One Bernoulli component — a single possible detected target.
//
// Existence probability r ∈ [0,1]. State density is Gaussian
// 𝒩(x; mean, covariance) at this stage (Phase 1, GM-PMBM). Phase 2
// upgrades this to an IMM mixture (one mean/cov per motion mode + mode
// probabilities) — to be added when Phase 2 lands.
struct Bernoulli {
  BernoulliId id{kInvalidBernoulliId};
  double existence_probability{0.0};  // r ∈ [0,1]
  Eigen::VectorXd mean;               // n_state (moment-matched projection)
  Eigen::MatrixXd covariance;         // n_state × n_state

  // Optional IMM ensemble carrier. When populated, the Bernoulli's
  // single-target density is a Gaussian mixture indexed by motion
  // mode; `mean` / `covariance` above are the moment-matched
  // projection consumed by the cost-matrix log-weight terms and by
  // the output aggregation. When empty, the Bernoulli is a single
  // Gaussian and works with any IEstimator whose update/predict reads
  // `state` / `covariance` (EKF / UKF / particle-filter projection).
  // Round-tripped through toTrack/fromTrack so estimators that expect
  // IMM fields (ImmEstimator) see them on the Track. Phase 2's "IMM
  // per Bernoulli" path uses these fields end-to-end; Phase 1 uses
  // them too when the injected estimator is IMM, so the Phase 1 bench
  // A/B can compare like-for-like against imm_cv_ct_mht.
  Eigen::MatrixXd imm_means;
  std::vector<Eigen::MatrixXd> imm_covariances;
  Eigen::VectorXd imm_mode_probabilities;

  // Timestamp of the last measurement that updated this Bernoulli.
  // Used by output aggregation and by the predict step to compute dt.
  Timestamp last_update{};

  // TPMBM (Phase 4). Birth time = first scan time the Bernoulli was
  // alive (set at new-target row materialisation, preserved
  // thereafter). Trajectory = forward-pass post-update / post-predict
  // state history, capped at Config::trajectory_window_scans (the
  // most recent N). Empty when TPMBM is disabled
  // (Config::trajectory_window_scans == 0) — keeps Phase 3 bit-
  // identical for consumers that don't opt in.
  Timestamp birth_time{};
  std::vector<TrajectoryPoint> trajectory;

  // Convenience: a Bernoulli is "alive" if r is above the supplied
  // pruning threshold. Caller picks the threshold per the tracker's
  // configured ipda_delete_threshold equivalent. Phase 1 default
  // r_min = 1e-3 (see PmbmTracker::Config when added).
  bool isAlive(double r_min) const noexcept {
    return existence_probability >= r_min;
  }
};

// One complete scan-consistent assignment of measurement history to
// targets. Mixture weight is the Bayesian posterior probability of this
// interpretation; the MBM is the weighted sum across all stored
// GlobalHypotheses (weights summing to 1 after normalisation).
//
// The Bernoullis are kept in a flat vector; ids are unique within a
// hypothesis (each target appears at most once). Across hypotheses, the
// same id may appear with different `(r, mean, covariance)`.
struct GlobalHypothesis {
  double weight{0.0};                 // mixture weight w^j, [0,1]
  double log_weight{                  // unnormalised log weight,
      -std::numeric_limits<double>::infinity()};  // primary scoring quantity
  std::vector<Bernoulli> bernoullis;  // possibly empty (no detected targets)

  // Convenience: count of Bernoullis above r_min.
  std::size_t aliveCount(double r_min) const noexcept {
    std::size_t n = 0;
    for (const auto& b : bernoullis) {
      if (b.isAlive(r_min)) ++n;
    }
    return n;
  }
};

// The full PMBM posterior: PPP (undetected) plus a mixture (MBM) of
// global hypotheses (detected). The two parts evolve under the same
// Bayesian recursion (PmbmTracker::predict, PmbmTracker::update) but
// are stored separately because they have very different shapes —
// PPP is a flat Gaussian mixture, MBM is a *mixture of multi-Bernoulli
// sets*.
struct PmbmDensity {
  std::vector<PoissonComponent> ppp;
  std::vector<GlobalHypothesis> mbm;

  // The MBM is empty at filter start (no detected targets yet). The PPP
  // carries the prior birth intensity — also empty until the user wires
  // a birth model.
  bool empty() const noexcept { return ppp.empty() && mbm.empty(); }

  // Convenience: total mixture weight (should sum to 1 after each
  // update; tests use this to verify normalisation).
  double totalMbmWeight() const noexcept {
    double s = 0.0;
    for (const auto& h : mbm) s += h.weight;
    return s;
  }
};

// IEstimator interop. The repository's IEstimator interface operates on
// Track values; PMBM stores Bernoullis and PoissonComponents as smaller
// value types. These adapters round-trip the kinematic state so the
// existing predict/update/initiate/logLikelihood implementations work
// unchanged inside the PMBM pipeline.
//
// last_update on Bernoulli matches Track's semantics: it is the filter
// time, advanced by predict and update alike. Phase 2 (IMM-per-Bernoulli)
// will extend these adapters to round-trip imm_means/imm_covariances/
// imm_mode_probabilities — the underlying fields already exist on Track.

inline Track toTrack(const Bernoulli& b) {
  Track t;
  t.state = b.mean;
  t.covariance = b.covariance;
  t.last_update = b.last_update;
  t.imm_means = b.imm_means;
  t.imm_covariances = b.imm_covariances;
  t.imm_mode_probabilities = b.imm_mode_probabilities;
  return t;
}

inline void fromTrack(Bernoulli& b, const Track& t) {
  b.mean = t.state;
  b.covariance = t.covariance;
  b.last_update = t.last_update;
  b.imm_means = t.imm_means;
  b.imm_covariances = t.imm_covariances;
  b.imm_mode_probabilities = t.imm_mode_probabilities;
}

inline Track toTrack(const PoissonComponent& c, Timestamp filter_time) {
  Track t;
  t.state = c.mean;
  t.covariance = c.covariance;
  t.last_update = filter_time;
  t.imm_means = c.imm_means;
  t.imm_covariances = c.imm_covariances;
  t.imm_mode_probabilities = c.imm_mode_probabilities;
  return t;
}

inline void fromTrack(PoissonComponent& c, const Track& t) {
  c.mean = t.state;
  c.covariance = t.covariance;
  c.imm_means = t.imm_means;
  c.imm_covariances = t.imm_covariances;
  c.imm_mode_probabilities = t.imm_mode_probabilities;
  // weight is unchanged by predict — caller scales it separately
  // (PPP weight decays by p_S, Bernoulli existence decays by p_S).
}

}  // namespace navtracker::pmbm
