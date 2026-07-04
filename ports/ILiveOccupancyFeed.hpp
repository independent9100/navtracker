#pragma once

#include <vector>

#include <Eigen/Core>

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

  /**
   * Feed one corroborating vessel FIX — an AIS / Cooperative / RemoteTrack
   * position (isNonScanningSource) — so the occupancy model can VETO
   * birth-suppression near a known vessel (R9 item 1b). `position_enu` is in the
   * tracker's working ENU frame; `t_unix` is the fix time (s) for the recency
   * window. Kept SEPARATE from observe() because a vessel fix is external
   * knowledge (a positive vessel presence), not a clutter-labeled return, and it
   * feeds only the birth-veto face — never λ_C / p_D. Default no-op: an occupancy
   * model without the veto, or a consumer that doesn't wire it, is unaffected —
   * the same nullable-sink contract as observe().
   */
  virtual void observeVesselFix(double /*t_unix*/,
                                const Eigen::Vector2d& /*position_enu*/) {}
};

}  // namespace navtracker
