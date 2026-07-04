#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * Time-ordered buffer that releases measurements once they are older than
 * `window_seconds` behind the latest timestamp seen. Late arrivals (older
 * than that cutoff) are dropped.
 *
 * === Math ===
 *   Let t_latest = max time seen so far. On drain(), every queued
 *   measurement m with m.time <= (t_latest - W) is released in
 *   non-decreasing m.time order (multimap iteration). On push(m), if
 *   m.time < (t_latest - W), the measurement is rejected as late and
 *   counted in dropped().
 *
 * === Assumptions ===
 *   1. Measurement.time carries the truth timestamp of the observation,
 *      not the arrival time. The buffer corrects only ordering, not skew.
 *   2. The user-chosen window W exceeds the maximum expected arrival skew
 *      (lag + jitter). Otherwise late drops are by design.
 *   3. drain() is called frequently enough that the queue size stays
 *      bounded by the input rate * W.
 *
 * === Rationale ===
 *   Fixed-window release was chosen over alternatives:
 *     - Per-source ordering buffer: forces the consumer to know sensor
 *       counts up front and complicates cross-sensor monotonicity.
 *     - Statistical late-drop: requires a noise model per source and adds
 *       state; deferred until we have calibrated lag distributions.
 *     - Reorder-on-drain (sort the whole queue lazily): equivalent here
 *       since the multimap is already sorted; we keep the eager form
 *       because it lets drain() be a single forward sweep.
 *
 * === Ways to improve / what to test next ===
 *   - Per-source windows: lets AIS use a larger W than ARPA without
 *     paying the AIS latency on radar-only consumers.
 *   - Statistical late-drop heuristic: drop on
 *     P(late | observed history) > threshold instead of a hard window.
 *   - Bounded-queue back-pressure for misuse (caller forgets to drain).
 *   - Reorder-on-drain with std::vector + nth_element if the multimap
 *     allocation cost ever shows up in profiling.
 */
class ReorderBuffer {
 public:
  /** Construct with the release window `window_seconds` (seconds behind t_latest). */
  explicit ReorderBuffer(double window_seconds);

  /** Returns true if the measurement was accepted; false if dropped as late. */
  bool push(const Measurement& m);

  /**
   * Release all measurements with time <= (latest_seen - window), in
   * chronological order.
   */
  std::vector<Measurement> drain();

  /** Number of measurements currently held (not yet released). */
  std::size_t pending() const { return queue_.size(); }
  /** Cumulative count of measurements rejected as too late. */
  std::size_t dropped() const { return dropped_; }

 private:
  std::int64_t window_nanos_;
  bool seen_{false};
  Timestamp latest_;
  std::multimap<Timestamp, Measurement> queue_;
  std::size_t dropped_{0};
};

}  // namespace navtracker
