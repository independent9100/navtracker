#pragma once

#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

struct SensorBiasPairExtractorConfig {
  // Pairs from contributions older than this (relative to `cycle_time`)
  // are ignored.
  double cycle_window_seconds{0.5};
  // 1-sigma isotropic anchor (AIS) position uncertainty fallback when
  // the AIS measurement covariance is degenerate.
  double anchor_position_std_fallback_m{10.0};
  // 1-sigma isotropic non-anchor sensor position uncertainty fallback.
  double sensor_position_std_fallback_m{10.0};
};

// Extract position-bias pair observations from a fused-track snapshot.
// For each track whose `recent_contributions` (within the cycle window
// of `cycle_time`) include both an AIS Position2D contribution and at
// least one *other* Position2D contribution (radar, lidar, ARPA), emit
// one PositionBiasPairObservation per (AIS, non-AIS) pair.
//
// Bearing-only contributions (EO/IR Bearing2D) are not extracted here;
// they need a different observation kind, which a future iteration
// adds via extractBearingPairs(...).
std::vector<PositionBiasPairObservation> extractPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg = {});

}  // namespace navtracker
