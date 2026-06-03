#pragma once

#include <vector>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

struct AisArpaPairExtractorConfig {
  // Pairs from contributions older than this (relative to `cycle_time`)
  // are ignored. Default 0.5 s — anything in the current cycle.
  double cycle_window_seconds{0.5};
  // 1-sigma isotropic AIS position uncertainty assumed when the AIS
  // measurement covariance is degenerate; used as a fallback only.
  double ais_position_std_fallback_m{10.0};
  // 1-sigma ARPA bearing uncertainty fallback (rad).
  double arpa_bearing_std_fallback_rad{1.0 * 3.14159265358979323846 / 180.0};
};

// Extract AIS+ARPA pair observations from a fused-track snapshot. For
// each track in `tracks` whose recent_contributions (within the cycle
// window of cycle_time) include both an AIS Position2D contribution
// and an ARPA Position2D contribution, emit one observation that
// pairs them.
std::vector<AisArpaPairObservation> extractPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    AisArpaPairExtractorConfig cfg = {});

}  // namespace navtracker
