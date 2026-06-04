#pragma once

#include "core/collision/Cpa.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Push-based collision-risk observer. CpaEvaluator emits these on
// per-pair (own-ship × track) state transitions with hysteresis.
//
// === Transitions ===
//   Entered - P(CPA < d_threshold) crossed enter_probability from below
//   Exited  - P fell below exit_probability OR the track was deleted
//   Updated - still risky; per-cycle refresh (gated by emit_updates)
enum class CollisionRiskTransition {
  Entered,
  Exited,
  Updated,
};

struct CollisionRiskEvent {
  CollisionRiskTransition transition;
  TrackId other;             // the non-own-ship track in the pair
  Timestamp time;
  CpaPrediction prediction;  // full CPA at this moment
};

class ICollisionRiskSink {
 public:
  virtual ~ICollisionRiskSink() = default;
  virtual void onCollisionRisk(const CollisionRiskEvent& event) = 0;
};

}  // namespace navtracker
