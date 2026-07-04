#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "core/benchmark/BenchRunner.hpp"
#include "core/types/Ids.hpp"
#include "ports/IInnovationSink.hpp"

namespace navtracker {
namespace benchmark {

// Source-keyed bucket for per-update NIS accumulation. Matches the
// (SensorKind, MeasurementModel, source_id) tuple used by the
// per-sensor detection-model table — items 4 + spec §3.
using ConsistencySourceKey =
    std::tuple<SensorKind, MeasurementModel, std::string>;

/**
 * Per-source normalized innovation squared (NIS) summary. Measures filter
 * consistency: whether the innovation ν sits inside its predicted covariance S.
 * `alpha_hat` and `trace_ratio_HPH_over_R` diagnose measurement-noise scaling;
 * the Wilson-Hilferty `band_lo`/`band_hi` give a closed-form χ² acceptance band
 * for the mean.
 */
struct NisStats {
  std::size_t n{0};
  double mean{0.0};                 // ε̄_NIS
  double coverage_95{0.0};          // fraction below upper χ²_m 95%
  double coverage_99{0.0};          // fraction below upper χ²_m 99%
  double alpha_hat{1.0};            // ε̄ / m — fitted-R scaling under R≫HPHᵀ
  double trace_ratio_HPH_over_R{0.0};  // mean tr(HPHᵀ)/tr(R) — regime
                                       // tell (high → α̂ unreliable)
  // Closed-form Wilson-Hilferty band for ε̄. NaN when n < kMinSamples.
  double band_lo{0.0};
  double band_hi{0.0};
  bool low_sample{false};
  std::size_t dim{0};
  std::size_t dropped_singular{0};  // S un-LDLT-able or pivot < kMinPivot
};

/**
 * Position normalized estimation error squared (NEES) summary. Measures how the
 * estimated position error compares to its posterior covariance against ground
 * truth (2-d). Reports median / p95 / p99 / coverage alongside the mean because
 * ID-switch transients drag the raw mean; `beta_hat` is the gross posterior-cov
 * scaling ε̄ / 2.
 */
struct NeesStats {
  std::size_t n{0};
  double mean{0.0};                 // ε̄_NEES (position only, 2-d). On
                                    // benches with frequent metric ID
                                    // reassignments (e.g. close parallel
                                    // targets) the mean is dragged by
                                    // post-switch transient samples;
                                    // see median + p95 + coverage_95
                                    // for the central-tendency reads
                                    // that are robust to that artefact.
  double median{0.0};               // p50 of εⁿᵉᵉˢ samples — central
                                    // tendency robust to ID-switch
                                    // transient spikes.
  double p95{0.0};                  // p95 of εⁿᵉᵉˢ samples
  double p99{0.0};                  // p99 of εⁿᵉᵉˢ samples — pins
                                    // tail magnitude separately from
                                    // p95; useful for distinguishing
                                    // "fat tail" from "extreme outlier"
                                    // when mean ≫ p95.
  double coverage_95{0.0};
  double beta_hat{1.0};             // ε̄ / 2 — gross posterior-cov
                                    // scaling. Inherits the mean's
                                    // tail-drag caveat; prefer
                                    // median / coverage_95 for honesty.
  double band_lo{0.0};
  double band_hi{0.0};
  bool low_sample{false};
  std::size_t dropped_singular{0};  // P_xy un-LDLT-able or pivot < kMinPivot
};

/** Bundle of both consistency reads for one run: per-source NIS + position NEES. */
struct ConsistencyResult {
  std::map<ConsistencySourceKey, NisStats> per_source;
  NeesStats nees;
};

/**
 * Sink-side accumulator. Streams (ν, S, R) into per-source Welford
 * means and per-source χ² coverage. finalize() computes Wilson-Hilferty
 * bands and α̂ / trace-ratio summaries.
 */
class NisCollector : public IInnovationSink {
 public:
  /** Accumulate one innovation event into its per-source bucket. */
  void onInnovation(const InnovationEvent& e) override;

  /** Snapshot per-source stats. Cheap; safe to call mid-run for tests. */
  std::map<ConsistencySourceKey, NisStats> finalize() const;

  /** Diagnostics — total innovation samples seen. */
  std::size_t totalSamples() const { return total_samples_; }
  /** Diagnostics — total samples dropped for singular S. */
  std::size_t totalDroppedSingular() const { return total_dropped_; }

 private:
  struct Accum {
    std::size_t n = 0;
    double mean = 0.0;                   // Welford running mean
    double m2 = 0.0;                     // unused, reserved for future variance
    std::size_t below_95 = 0;
    std::size_t below_99 = 0;
    double sum_trace_ratio = 0.0;
    std::size_t dim = 0;
    std::size_t dropped_singular = 0;
  };
  std::map<ConsistencySourceKey, Accum> per_key_;
  std::size_t total_samples_ = 0;
  std::size_t total_dropped_ = 0;
};

/**
 * Walks BenchResult.steps + per-step assignment and computes
 * per-(truth, track) position NEES. The bench's snapshotAt already
 * filters to Confirmed tracks, so the returned NeesStats is the
 * "Confirmed-only" quantity from the design spec (no separate
 * Tentative arm needed — those are not in BenchStep::tracks).
 *
 * gate_m is the 100 m assignPerStep gate; reused so RMSE / NEES / id-
 * switches all see the same truth↔track pairing.
 */
NeesStats computeNees(const BenchResult& result, double gate_m);

/** One-shot helper bundling both NIS and NEES into a ConsistencyResult. */
ConsistencyResult computeConsistency(const NisCollector& nis,
                                     const BenchResult& result,
                                     double assoc_gate_m);

}  // namespace benchmark
}  // namespace navtracker
