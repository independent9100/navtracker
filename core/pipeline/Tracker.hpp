#pragma once

#include <vector>

#include "core/types/Measurement.hpp"
#include "ports/IBearingInnovationSink.hpp"
#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {

class TrackManager;

// Single-measurement orchestrator. Drives the predict-associate-update/initiate
// cycle, then ages unobserved tracks via a miss-timeout policy.
class Tracker {
 public:
  Tracker(const IEstimator& estimator,
          const IDataAssociator& associator,
          TrackManager& manager,
          double miss_timeout_seconds);

  void process(const Measurement& z);

  // Process a batch of measurements that share a common timestamp ("scan").
  // Calls associator with the full batch, then applies either hard matches
  // (if `betas` empty) or soft updates via `softUpdate` (if `betas` filled).
  void processBatch(const std::vector<Measurement>& scan);

  // Optional. When non-null, every successful hard-match update on a
  // Bearing2D or RangeBearing2D measurement triggers an
  // onBearingInnovation callback computed from the PRE-update predicted
  // state. Soft (JPDA) updates are not emitted in this revision.
  void setBearingInnovationSink(IBearingInnovationSink* sink) {
    bearing_innov_sink_ = sink;
  }

  // Stale-input guard, ON by default. The engine is time-driven: a
  // measurement older than the high-water mark of everything already
  // processed would be applied against newer state (predict is a dt≤0
  // no-op) and rewind track.last_update, inflating the next predict's
  // dt. Such measurements are dropped and counted. Use a ReorderBuffer
  // upstream to *recover* (not just reject) late data; opt out only if
  // your input is guaranteed time-ordered and you want zero checks.
  void setRejectStaleMeasurements(bool on) { reject_stale_ = on; }
  std::size_t staleDropped() const { return stale_dropped_; }

 private:
  const IEstimator& estimator_;
  const IDataAssociator& associator_;
  TrackManager& manager_;
  double miss_timeout_seconds_;
  IBearingInnovationSink* bearing_innov_sink_{nullptr};
  bool reject_stale_{true};
  std::size_t stale_dropped_{0};
  bool has_high_water_{false};
  Timestamp high_water_{};
};

}  // namespace navtracker
