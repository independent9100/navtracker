#include "core/benchmark/Metrics.hpp"

// Metrics — bench-side aggregates and helpers feeding MetricsResult.
//
// This file accumulates the metric implementations across Tasks 5–9
// (OSPA aggregates, per-step assignment, continuity, RMSE,
// computeMetrics bundle). Each function carries its own
// Math/Assumptions/Rationale/Improve-next block per CLAUDE.md.

#include <algorithm>
#include <cmath>
#include <numeric>

#include "core/scenario/Ospa.hpp"

namespace navtracker {
namespace benchmark {

// Math:        per-step OSPA via greedy assignment (existing repo impl).
// Assumptions: result.steps[i].truth and .tracks are valid;
//              cutoff_m > 0; positions in metres.
// Rationale:   reuse ospaGreedy() from core/scenario/Ospa.hpp so the
//              metric matches the existing harness exactly.
// Improve next: OSPA(2) window-based variant (penalises ID switches
//               directly); switch to true Hungarian assignment if the
//               greedy version dominates regressions in pathological
//               crossing cases.
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m) {
  std::vector<double> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    std::vector<Eigen::Vector2d> truth;
    truth.reserve(step.truth.size());
    for (const auto& t : step.truth) truth.push_back(t.position);
    std::vector<Eigen::Vector2d> est;
    est.reserve(step.tracks.size());
    for (const auto& tr : step.tracks) est.push_back(tr.position);
    out.push_back(ospaGreedy(truth, est, cutoff_m));
  }
  return out;
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

// Math:        linear-interpolated percentile;
//              idx = q * (n-1), result = v[floor(idx)]*(1-frac) +
//              v[ceil(idx)]*frac where frac = idx - floor(idx).
// Assumptions: q in [0,1]; v non-empty (returns 0 if empty).
// Rationale:   matches NumPy's "linear" interpolation method; standard
//              and predictable for downstream tooling.
// Improve next: support alternative methods (nearest, lower, higher)
//               if a future metric needs them.
double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = q * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  const double frac = idx - static_cast<double>(lo);
  return v[lo] * (1.0 - frac) + v[hi] * frac;
}

}  // namespace benchmark
}  // namespace navtracker
