#pragma once

#include <vector>

#include "core/types/Measurement.hpp"
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

 private:
  const IEstimator& estimator_;
  const IDataAssociator& associator_;
  TrackManager& manager_;
  double miss_timeout_seconds_;
};

}  // namespace navtracker
