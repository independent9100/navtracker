#pragma once

namespace navtracker {

struct NavHealth;  // core/own_ship/NavInputGuard.hpp

/**
 * Sink for own-ship nav-input health events (backlog #18). Fired at the
 * OwnShipProvider edge when an incoming pose trips a sanity check — the guard's
 * job is to make nav trouble an EVENT, not a silent smear. Pure notification: it
 * MUST NOT be wired to anything that rewrites the pose or the tracker state (the
 * poses are still trusted; the operator is merely told the nav is degraded).
 * Nullable — unwired ⇒ no events and no overhead (bit-identical to today).
 */
class INavHealthSink {
 public:
  virtual ~INavHealthSink() = default;
  virtual void onNavHealth(const NavHealth& health) = 0;
};

}  // namespace navtracker
