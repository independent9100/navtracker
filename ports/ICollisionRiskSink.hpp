#pragma once

#include "core/collision/Cpa.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * Push-based collision-risk observer. CpaEvaluator emits these on
 * per-pair (own-ship × track) state transitions with hysteresis.
 *
 * === Transitions ===
 *   Entered - P(CPA < d_threshold) crossed enter_probability from below
 *   Exited  - P fell below exit_probability OR the track was deleted
 *   Updated - still risky; per-cycle refresh (gated by emit_updates)
 */
enum class CollisionRiskTransition {
  Entered,
  Exited,
  Updated,
};

/**
 * One collision-risk transition for a single (own-ship × track) pair,
 * carrying the full CPA prediction at the moment the transition fired.
 */
struct CollisionRiskEvent {
  CollisionRiskTransition transition;
  TrackId other;             // the non-own-ship track in the pair
  Timestamp time;
  CpaPrediction prediction;  // full CPA at this moment
};

/**
 * Push-based sink for collision-risk events. Consumers (alarms, UI,
 * loggers) implement this and register with the CpaEvaluator; null =
 * today's behaviour with no overhead.
 */
class ICollisionRiskSink {
 public:
  virtual ~ICollisionRiskSink() = default;
  /** Receive one per-pair collision-risk transition event. */
  virtual void onCollisionRisk(const CollisionRiskEvent& event) = 0;
};

}  // namespace navtracker
