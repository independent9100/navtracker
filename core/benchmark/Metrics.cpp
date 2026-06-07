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
#include <unordered_set>

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

// Math:        per-truth greedy NN: for each truth index i, scan
//              unclaimed tracks, pick the one minimising
//              ||tr.position - truth[i].position|| strictly less than
//              gate_m; mark it claimed.
// Assumptions: truth and track positions are in the same ENU frame and
//              metres; gate_m is the exclusive ceiling.
// Rationale:   greedy NN matches the existing countIdSwitches behaviour
//              and is O(N*M); for our small N,M (<= 10 typical) the
//              difference vs. full Hungarian is below noise. Single
//              shared assignment function across continuity, id
//              switches, and per-track RMSE keeps metrics consistent.
// Improve next: swap to full Hungarian via munkres if pathological
//               crossing geometry causes assignment churn that affects
//               ID-switch counts.
std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m) {
  std::vector<StepAssignment> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    StepAssignment a(step.truth.size(), std::nullopt);
    std::unordered_set<std::uint64_t> claimed;
    for (std::size_t i = 0; i < step.truth.size(); ++i) {
      double best = gate_m;
      std::optional<TrackId> best_id;
      for (const auto& tr : step.tracks) {
        if (claimed.count(tr.id.value)) continue;
        const double d = (tr.position - step.truth[i].position).norm();
        if (d < best) {
          best = d;
          best_id = tr.id;
        }
      }
      if (best_id) {
        claimed.insert(best_id->value);
        a[i] = best_id;
      }
    }
    out.push_back(std::move(a));
  }
  return out;
}

// Math:        per truth i, walk the time series of StepAssignment[i].
//                - lifetime_ratio_i = (#steps where a[i] is Some) / total
//                - track_breaks_i  = # of maximal Some -> None transitions
//                - id_switches_i   = # of adjacent Some -> Some transitions
//                  where the TrackId changes
//              Reported as plain means across truths.
// Assumptions: assigns is the output of assignPerStep; each entry's size
//              matches n_truths; n_truths > 0. Bad inputs return zeros.
// Rationale:   CLAUDE.md names ID stability as an architectural
//              guarantee; OSPA alone wouldn't catch silent ID churn or
//              brief drops that don't change cardinality. Sharing the
//              assignment with assignPerStep keeps every metric in
//              agreement about which track represents which truth.
// Improve next: replace per-step assignment with a run-level longest
//               common subsequence over (truth, track_id) — better at
//               distinguishing brief swap-then-swap-back from real
//               permanent ID churn.
ContinuityCounts computeContinuity(const std::vector<StepAssignment>& assigns,
                                   std::size_t n_truths) {
  if (n_truths == 0 || assigns.empty()) return {0, 0, 0};
  std::vector<double> life(n_truths, 0.0);
  std::vector<double> breaks(n_truths, 0.0);
  std::vector<double> switches(n_truths, 0.0);
  std::vector<bool> in_gap(n_truths, true);
  std::vector<std::optional<TrackId>> prev(n_truths, std::nullopt);

  for (const auto& step : assigns) {
    for (std::size_t i = 0; i < n_truths && i < step.size(); ++i) {
      const auto& a = step[i];
      if (a.has_value()) {
        life[i] += 1.0;
        if (in_gap[i]) in_gap[i] = false;
        if (prev[i].has_value() && prev[i]->value != a->value) {
          switches[i] += 1.0;
        }
        prev[i] = a;
      } else {
        if (!in_gap[i]) {
          breaks[i] += 1.0;  // exited an assigned interval
          in_gap[i] = true;
        }
        prev[i] = std::nullopt;
      }
    }
  }

  ContinuityCounts c{};
  for (std::size_t i = 0; i < n_truths; ++i) {
    c.lifetime_ratio += life[i] / static_cast<double>(assigns.size());
    c.track_breaks += breaks[i];
    c.id_switches += switches[i];
  }
  c.lifetime_ratio /= static_cast<double>(n_truths);
  c.track_breaks /= static_cast<double>(n_truths);
  c.id_switches /= static_cast<double>(n_truths);
  return c;
}

}  // namespace benchmark
}  // namespace navtracker
