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
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, mean across truth
  double id_switches{0.0};      // count, mean across truth
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
};

struct MetricsParams {
  double ospa_cutoff_m{500.0};
  double assoc_gate_m{100.0};
};

// Per-step OSPA values across the run (one per step in result.steps).
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m);

double mean(const std::vector<double>& v);

// Linear-interpolated percentile, q in [0,1]. Sorts a copy of v.
double percentile(std::vector<double> v, double q);

}  // namespace benchmark
}  // namespace navtracker
