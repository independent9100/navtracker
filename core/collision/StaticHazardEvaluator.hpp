#pragma once

#include <cstdint>
#include <map>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IStaticHazardSink.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

struct StaticHazardEvaluatorConfig {
  // Exit when distance exceeds keep_clear * exit_hysteresis (> 1.0), so a
  // vessel loitering at the boundary does not flap Entered/Exited.
  double exit_hysteresis = 1.1;
  // Emit an Updated event each cycle while inside the keep-clear ring.
  bool emit_updates = false;
};

// Emits static-hazard proximity events per (own-ship × charted-obstacle) with
// hysteresis. Static geometry: distance from own-ship to obstacle centre vs
// the obstacle's keep-clear radius. Mirrors CpaEvaluator's per-pair state +
// hysteresis, minus the trajectory/CPA math.
class StaticHazardEvaluator {
 public:
  using Config = StaticHazardEvaluatorConfig;

  explicit StaticHazardEvaluator(const IStaticObstacleModel* model,
                                 Config cfg = {})
      : model_(model), cfg_(cfg) {}

  void setSink(IStaticHazardSink* s) { sink_ = s; }

  // Evaluate all obstacles against the current own-ship ENU position (metres,
  // in `datum`'s frame). Fires Entered/Exited/Updated with hysteresis.
  void evaluate(const Eigen::Vector2d& own_ship_enu, const geo::Datum& datum,
                Timestamp t);

 private:
  const IStaticObstacleModel* model_{nullptr};
  IStaticHazardSink* sink_{nullptr};
  Config cfg_;
  std::map<std::uint64_t, bool> inside_;  // hazard_id -> currently inside ring
};

}  // namespace navtracker
