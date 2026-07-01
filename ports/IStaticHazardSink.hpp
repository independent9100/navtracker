#pragma once

#include <cstdint>

#include "core/types/Timestamp.hpp"

namespace navtracker {

// Push-based static-hazard proximity observer. StaticHazardEvaluator emits
// these per (own-ship × charted-obstacle) on keep-clear crossings, with
// hysteresis. Distinct from ICollisionRiskSink: a static-geometry range check,
// not a trajectory CPA (ADR 0002 — obstacles have no velocity).
enum class StaticHazardTransition { Entered, Exited, Updated };

struct StaticHazardEvent {
  StaticHazardTransition transition;
  std::uint64_t hazard_id;
  Timestamp time;
  double distance_m;    // own-ship to obstacle centre (m)
  double keep_clear_m;  // the obstacle's keep-clear radius (m)
};

class IStaticHazardSink {
 public:
  virtual ~IStaticHazardSink() = default;
  virtual void onStaticHazard(const StaticHazardEvent& event) = 0;
};

}  // namespace navtracker
