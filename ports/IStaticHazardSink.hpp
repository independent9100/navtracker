#pragma once

#include <cstdint>

#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * Which keep-clear boundary crossing a StaticHazardEvent reports: the
 * own-ship first breached the radius (Entered), cleared it (Exited), or
 * moved while still inside it (Updated).
 */
enum class StaticHazardTransition { Entered, Exited, Updated };

/**
 * One static-hazard proximity event: an own-ship crossing of a charted
 * obstacle's keep-clear radius, carrying the current separation and the
 * obstacle's keep-clear distance.
 */
struct StaticHazardEvent {
  StaticHazardTransition transition;
  std::uint64_t hazard_id;
  Timestamp time;
  double distance_m;    // own-ship to obstacle centre (m)
  double keep_clear_m;  // the obstacle's keep-clear radius (m)
};

/**
 * Push-based static-hazard proximity observer. StaticHazardEvaluator emits
 * events per (own-ship × charted-obstacle) on keep-clear crossings, with
 * hysteresis. Distinct from ICollisionRiskSink: a static-geometry range check,
 * not a trajectory CPA (ADR 0002 — obstacles have no velocity).
 */
class IStaticHazardSink {
 public:
  virtual ~IStaticHazardSink() = default;
  /** Receive one keep-clear crossing event for a charted obstacle. */
  virtual void onStaticHazard(const StaticHazardEvent& event) = 0;
};

}  // namespace navtracker
