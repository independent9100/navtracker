#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

// A joint event: for each measurement j in 0..M-1, event[j] is either
// 0 (clutter) or 1..T (assigned to track event[j]-1).
using JointEvent = std::vector<int>;

// Enumerate all feasible joint events for the M x T binary validation
// matrix V (V(j, t) = 1 iff measurement j is gated to track t). Each
// returned event has size M. Within an event, no two measurements may
// share the same non-zero track index.
std::vector<JointEvent> enumerateJointEvents(const Eigen::MatrixXi& V);

// Budgeted variant. Stops and returns an empty vector if the number of
// feasible joint events would exceed `max_events`. Since the normal
// enumeration always yields at least one event (the all-clutter event),
// an empty return unambiguously signals overflow, letting the caller
// fall back to a tractable approximation (e.g. greedy hard assignment).
// This is the standard cluster-size safeguard for full-enumeration JPDA
// without an EHM solver (Blackman & Popoli §6): dense clutter or large
// track clusters make exhaustive enumeration O(M^T) intractable.
std::vector<JointEvent> enumerateJointEvents(const Eigen::MatrixXi& V,
                                             std::size_t max_events);

}  // namespace navtracker
