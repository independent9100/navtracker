#pragma once

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

 private:
  const IEstimator& estimator_;
  const IDataAssociator& associator_;
  TrackManager& manager_;
  double miss_timeout_seconds_;
};

}  // namespace navtracker
