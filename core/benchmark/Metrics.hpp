#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "core/benchmark/BenchRunner.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Ids.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Per-truth-id breakdown of the metrics computeContinuity and
 * computeRmse already compute internally. Exposed so the bench can
 * emit one CSV row per (scenario, truth_id, metric) — surfaces "is
 * every target tracked, or is one bad target dragging the scenario
 * mean" without re-running the bench.
 */
struct PerTruthMetrics {
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, this truth only
  double id_switches{0.0};      // count, this truth only
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
  std::size_t rmse_n{0};        // # contributing samples (Confirmed only)
  // ADR-0002 rule-3 (static->moving promotion) latency, in present-steps
  // (scans at the truth cadence): the count from this truth's motion onset
  // (first present step with |velocity| > threshold) to the first present
  // step where it is BOTH moving AND assigned to a track. 0 = tracked through
  // the transition (or never moves); a large value = "never promoted while
  // moving" (== the truth's moving-phase duration). See computeContinuity.
  double promotion_latency{0.0};  // scans
};

/**
 * The full metric bundle computed for one bench run: OSPA and GOSPA
 * (mean/p95/RMS + localization/missed/false decomposition + cardinality
 * error), trajectory-aligned T-GOSPA (raw and RTS-smoothed), continuity
 * (lifetime / breaks / id-switches) and RMSE (position / SOG / COG), plus
 * the per-truth-id breakdown of the same numbers.
 */
struct MetricsResult {
  double ospa_mean{0.0};        // metres
  double ospa_p95{0.0};         // metres
  // Generalized OSPA (Rahmathullah et al. 2017). Same per-step grid as
  // OSPA, but missed/false targets are charged c^p / α each so the
  // metric grows with cardinality error instead of saturating at c.
  // Convention matches Helgesen et al. 2022 on AutoFerry: c = 20 m,
  // p = α = 2.
  double gospa_mean{0.0};       // metres, arithmetic mean over steps
  double gospa_p95{0.0};        // metres
  // RMS over per-step GOSPA: √(mean(GOSPA_k²)). This is the aggregation
  // Helgesen 2022 reports in Tables 6 / 7 ("GOSPA is reported as RMS")
  // and is what to compare against the paper's headline numbers.
  double gospa_rms{0.0};        // metres
  // GOSPA decomposition (Rahmathullah 2017): arithmetic mean over steps of
  // the three pre-root sub-costs (power-p space, p=2 convention).
  //   gospa_localization = mean_k Σ_{matched pairs k} d(x_i,y_j)²
  //   gospa_missed       = mean_k n_missed_k * c²/α
  //   gospa_false        = mean_k n_false_k  * c²/α
  // Identity: gospa_localization + gospa_missed + gospa_false ≈ mean_k(GOSPA_k²)
  //   (exact when gospa_mean = mean(√total_k) ≈ √(mean(total_k))).
  // card_err_mean = mean_k(|est_k| − |truth_k|), signed: positive means
  //   over-counting (more estimates than truth); negative means under-counting.
  double gospa_localization{0.0};  // m² (pre-root, p=2)
  double gospa_missed{0.0};        // m² (pre-root, p=2)
  double gospa_false{0.0};         // m² (pre-root, p=2)
  double card_err_mean{0.0};       // tracks (signed mean cardinality error)
  // Trajectory-aligned T-GOSPA over BenchResult.steps. Stitches truth
  // and estimated positions into time-indexed trajectories keyed by
  // truth_id / TrackId.value, then runs core/scenario/TGospa.hpp. The
  // switch penalty γ defaults to gospa_cutoff_m. Surfaces fragmentation
  // and id switching directly (the things per-scan OSPA/GOSPA hide).
  double tgospa_raw_m{0.0};
  // T-GOSPA computed against the RTS-smoothed trajectories the caller
  // optionally hands in (PMBM-only path via
  // PmbmTracker::collectSmoothedTrajectories). 0.0 sentinel when no
  // smoothed trajectories were supplied (typical for MHT / non-TPMBM
  // runs). The pair (tgospa_raw_m, tgospa_smooth_m) on the same
  // PMBM run quantifies the smoother win.
  double tgospa_smooth_m{0.0};
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, mean across truth
  double id_switches{0.0};      // count, mean across truth
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
  // Per-truth-id breakdown — same numbers, sliced by truth slot.
  // Ordered map (truth_id ascending) so CSV emission is deterministic.
  std::map<std::uint64_t, PerTruthMetrics> per_truth;
};

/** Cutoff / gate parameters for the metric computations (OSPA, GOSPA, assoc gate). */
struct MetricsParams {
  double ospa_cutoff_m{500.0};
  // GOSPA cutoff. 20 m matches Helgesen et al. 2022 §5.8 ("a Cartesian
  // distance cut-off parameter of 20 m to match the track-truth
  // assignment threshold used in other metrics"). Smaller than the
  // OSPA cutoff because GOSPA isn't trying to bound cardinality
  // penalty into a per-step ceiling.
  double gospa_cutoff_m{20.0};
  double assoc_gate_m{100.0};
};

/** Per-step OSPA values across the run (one per step in result.steps). */
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m);

/** Per-step GOSPA values (p=2, alpha=2 per literature convention). */
std::vector<double> computeGospaPerStep(const BenchResult& result,
                                        double cutoff_m);

/** Arithmetic mean of `v` (0 for empty). */
double mean(const std::vector<double>& v);

/** Linear-interpolated percentile, q in [0,1]. Sorts a copy of v. */
double percentile(std::vector<double> v, double q);

/**
 * Per-step assignment: for each truth index in step.truth, the assigned
 * TrackId from step.tracks (or std::nullopt if no track within gate).
 * Greedy nearest-neighbour under the gate. Hungarian is equivalent for
 * the small target counts in our scenarios; swap if profiling flags it.
 */
using StepAssignment = std::vector<std::optional<TrackId>>;
std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m);

/** Per-truth-id continuity triple (lifetime ratio, breaks, id switches). */
struct PerTruthContinuity {
  double lifetime_ratio{0.0};
  double track_breaks{0.0};
  double id_switches{0.0};
  double promotion_latency{0.0};  // scans; ADR-0002 rule-3 (see computeContinuity)
};
/** Continuity counts averaged across truths, plus the per-truth-id breakdown. */
struct ContinuityCounts {
  double lifetime_ratio;  // mean across truths in [0, 1]
  double track_breaks;    // mean count per truth
  double id_switches;     // mean count per truth
  std::map<std::uint64_t, PerTruthContinuity> per_truth;  // by truth_id
};
/**
 * Continuity keyed by truth_id (read from result.steps[k].truth), so it
 * supports time-varying truth cardinality and per-step slot reordering.
 * Lifetime is assigned-steps / present-steps per truth id; a truth id
 * appearing or disappearing is neither a break nor a switch.
 */
ContinuityCounts computeContinuity(const BenchResult& result,
                                   const std::vector<StepAssignment>& assigns);

/** Per-truth-id RMSE triple (position / SOG / COG) with its sample count. */
struct PerTruthRmse {
  double pos_rmse_m{0.0};
  double sog_rmse_mps{0.0};
  double cog_rmse_deg{0.0};
  std::size_t n{0};
};
/** RMSE aggregated across truths (position / SOG / COG), plus per-truth-id slices. */
struct RmseResult {
  double pos_rmse_m;
  double sog_rmse_mps;
  double cog_rmse_deg;
  std::map<std::uint64_t, PerTruthRmse> per_truth;  // by truth_id
};

/** Position / SOG / COG RMSE over Confirmed tracks under the per-step assignment. */
RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns);

/** Compute the full metric bundle for one bench run. */
MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params);

/**
 * PMBM overload — accepts the RTS-smoothed trajectories from
 * PmbmTracker::collectSmoothedTrajectories. Computes everything the
 * scalar overload does, plus tgospa_smooth_m by replacing the
 * per-step est positions in tgospa_raw_m's stitching with the
 * smoothed positions sampled at each step's time.
 */
MetricsResult computeMetrics(
    const BenchResult& result,
    const MetricsParams& params,
    const std::map<std::uint64_t, std::vector<pmbm::TrajectoryPoint>>&
        smoothed_trajectories);

}  // namespace benchmark
}  // namespace navtracker
