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

 private:
  const IEstimator& estimator_;
  const IDataAssociator& associator_;
  TrackManager& manager_;
  double miss_timeout_seconds_;
  IBearingInnovationSink* bearing_innov_sink_{nullptr};
};

}  // namespace navtracker
