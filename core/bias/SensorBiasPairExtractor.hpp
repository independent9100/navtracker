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
std::vector<PositionBiasPairObservation> extractPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg = {});

// Extract bearing-bias pair observations. For each track whose
// `recent_contributions` (within cycle_window_seconds of cycle_time)
// include both an AIS Position2D contribution and at least one
// Bearing2D contribution (EO / IR), emit one BearingBiasPairObservation
// per (AIS, bearing) pair.
//
// The bearing residual is computed as r = wrap(α_obs − α_pred), where
// α_pred is the bearing from the camera's sensor position to the
// AIS anchor's reported ENU position. The bias estimator's bearing
// update consumes this residual directly.
//
// Why this exists separately from the heading-bias estimator's bearing
// path: heading bias is shared across all sensors on the platform; the
// per-sensor mounting bias is sensor-specific. Both can coexist —
// heading bias is removed upstream by OwnShipProvider; the residual
// per-sensor bearing offset is what this estimator catches.
std::vector<BearingBiasPairObservation> extractBearingPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg = {});

}  // namespace navtracker
