#pragma once

#include <vector>

#include "ports/ISensorDetectionModel.hpp"  // ScanObservation

namespace navtracker {

/**
 * Sink for the PMBM per-scan clutter-labeled feed, routed to a live
 * occupancy/structure model. Deliberately SEPARATE from ISensorDetectionModel:
 * the occupancy layer must influence the BIRTH channel only (via its
 * IStaticObstacleModel::birthSuppression face), never λ_C / p_D. Wiring an
 * occupancy feed therefore does NOT couple the learned map into association —
 * that indiscriminate coupling is exactly what the Stage 1b design rejects
 * (it caused the clutter-map spike's dense_clutter regression).
 *
 * The payload is the same per-(sensor,model) ScanObservation bundle the
 * feed_clutter_map producer already builds: clutter_positions carry the
 * unclaimed (weight 1.0) and weakly-claimed (weight 1 − r) returns; returns
 * claimed by a near-certain track (r ≈ 1) are excluded (weight 0). Nullable:
 * unwired ⇒ today's behaviour, bit-identical.
 */
class ILiveOccupancyFeed {
 public:
  virtual ~ILiveOccupancyFeed() = default;

  /** Feed one per-scan bundle of clutter-labeled observations by sensor. */
  virtual void observe(
      const std::vector<ISensorDetectionModel::ScanObservation>& by_sensor) = 0;
};

}  // namespace navtracker
