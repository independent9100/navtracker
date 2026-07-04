#pragma once

#include <cstddef>
#include <map>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IStaticHazardSink.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

/** Tuning for `StaticHazardEvaluator`: the exit-hysteresis multiplier on the
 *  keep-clear radius and whether per-cycle Updated events are emitted. */
struct StaticHazardEvaluatorConfig {
  // Exit when distance exceeds keep_clear * exit_hysteresis (> 1.0), so a
  // vessel loitering at the boundary does not flap Entered/Exited.
  double exit_hysteresis = 1.1;
  // Emit an Updated event each cycle while inside the keep-clear ring.
  bool emit_updates = false;
};

/**
 * Emits static-hazard proximity events per (own-ship × charted-obstacle) with
 * hysteresis. Static geometry: distance from own-ship to obstacle centre vs
 * the obstacle's keep-clear radius. Mirrors CpaEvaluator's per-pair state +
 * hysteresis, minus the trajectory/CPA math.
 */
class StaticHazardEvaluator {
 public:
  using Config = StaticHazardEvaluatorConfig;

  explicit StaticHazardEvaluator(const IStaticObstacleModel* model,
                                 Config cfg = {})
      : model_(model), cfg_(cfg) {}

  /** Register the sink that receives static-hazard events; null = no emission. */
  void setSink(IStaticHazardSink* s) { sink_ = s; }

  /**
   * Evaluate all obstacles against the current own-ship ENU position (metres,
   * in `datum`'s frame). Fires Entered/Exited/Updated with hysteresis.
   */
  void evaluate(const Eigen::Vector2d& own_ship_enu, const geo::Datum& datum,
                Timestamp t);

 private:
  const IStaticObstacleModel* model_{nullptr};
  IStaticHazardSink* sink_{nullptr};
  Config cfg_;
  // R7.3: keyed by obstacle index (stable within a run), NOT hazard_id — two
  // co-located ENC records share a hazard_id, and sharing hysteresis state
  // there lets one obstacle's Exit mask another's Enter. Events still carry
  // hazard_id for operator identification.
  std::map<std::size_t, bool> inside_;  // obstacle index -> inside ring
};

}  // namespace navtracker
