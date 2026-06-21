#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "core/benchmark/BenchRunner.hpp"
#include "core/types/Ids.hpp"

namespace navtracker {
namespace benchmark {

// Per-truth-id breakdown of the metrics computeContinuity and
// computeRmse already compute internally. Exposed so the bench can
// emit one CSV row per (scenario, truth_id, metric) — surfaces "is
// every target tracked, or is one bad target dragging the scenario
// mean" without re-running the bench.
struct PerTruthMetrics {
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, this truth only
  double id_switches{0.0};      // count, this truth only
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
  std::size_t rmse_n{0};        // # contributing samples (Confirmed only)
};

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
  // Trajectory-aligned T-GOSPA over BenchResult.steps. Stitches truth
  // and estimated positions into time-indexed trajectories keyed by
  // truth_id / TrackId.value, then runs core/scenario/TGospa.hpp. The
  // switch penalty γ defaults to gospa_cutoff_m. Surfaces fragmentation
  // and id switching directly (the things per-scan OSPA/GOSPA hide).
  double tgospa_raw_m{0.0};
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

// Per-step OSPA values across the run (one per step in result.steps).
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m);

// Per-step GOSPA values (p=2, alpha=2 per literature convention).
std::vector<double> computeGospaPerStep(const BenchResult& result,
                                        double cutoff_m);

double mean(const std::vector<double>& v);

// Linear-interpolated percentile, q in [0,1]. Sorts a copy of v.
double percentile(std::vector<double> v, double q);

// Per-step assignment: for each truth index in step.truth, the assigned
// TrackId from step.tracks (or std::nullopt if no track within gate).
// Greedy nearest-neighbour under the gate. Hungarian is equivalent for
// the small target counts in our scenarios; swap if profiling flags it.
using StepAssignment = std::vector<std::optional<TrackId>>;
std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m);

struct PerTruthContinuity {
  double lifetime_ratio{0.0};
  double track_breaks{0.0};
  double id_switches{0.0};
};
struct ContinuityCounts {
  double lifetime_ratio;  // mean across truths in [0, 1]
  double track_breaks;    // mean count per truth
  double id_switches;     // mean count per truth
  std::map<std::uint64_t, PerTruthContinuity> per_truth;  // by truth_id
};
// Continuity keyed by truth_id (read from result.steps[k].truth), so it
// supports time-varying truth cardinality and per-step slot reordering.
// Lifetime is assigned-steps / present-steps per truth id; a truth id
// appearing or disappearing is neither a break nor a switch.
ContinuityCounts computeContinuity(const BenchResult& result,
                                   const std::vector<StepAssignment>& assigns);

struct PerTruthRmse {
  double pos_rmse_m{0.0};
  double sog_rmse_mps{0.0};
  double cog_rmse_deg{0.0};
  std::size_t n{0};
};
struct RmseResult {
  double pos_rmse_m;
  double sog_rmse_mps;
  double cog_rmse_deg;
  std::map<std::uint64_t, PerTruthRmse> per_truth;  // by truth_id
};

RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns);

MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params);

}  // namespace benchmark
}  // namespace navtracker
