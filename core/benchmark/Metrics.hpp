#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "core/benchmark/BenchRunner.hpp"
#include "core/types/Ids.hpp"

namespace navtracker {
namespace benchmark {

struct MetricsResult {
  double ospa_mean{0.0};        // metres
  double ospa_p95{0.0};         // metres
  // Generalized OSPA (Rahmathullah et al. 2017). Same per-step grid as
  // OSPA, but missed/false targets are charged c^p / α each so the
  // metric grows with cardinality error instead of saturating at c.
  // Convention here matches the PMBM / autoferry literature: c = 30 m,
  // p = α = 2 (Helgesen et al. 2022).
  double gospa_mean{0.0};       // metres
  double gospa_p95{0.0};        // metres
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, mean across truth
  double id_switches{0.0};      // count, mean across truth
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
};

struct MetricsParams {
  double ospa_cutoff_m{500.0};
  // GOSPA cutoff (default 30 m matches Helgesen et al. 2022 on the
  // AutoFerry dataset — the convention we want to reproduce). Smaller
  // than the OSPA cutoff because GOSPA isn't trying to bound cardinality
  // penalty into a per-step ceiling.
  double gospa_cutoff_m{30.0};
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

struct ContinuityCounts {
  double lifetime_ratio;  // mean across truths in [0, 1]
  double track_breaks;    // mean count per truth
  double id_switches;     // mean count per truth
};
// Continuity keyed by truth_id (read from result.steps[k].truth), so it
// supports time-varying truth cardinality and per-step slot reordering.
// Lifetime is assigned-steps / present-steps per truth id; a truth id
// appearing or disappearing is neither a break nor a switch.
ContinuityCounts computeContinuity(const BenchResult& result,
                                   const std::vector<StepAssignment>& assigns);

struct RmseResult {
  double pos_rmse_m;
  double sog_rmse_mps;
  double cog_rmse_deg;
};

RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns);

MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params);

}  // namespace benchmark
}  // namespace navtracker
