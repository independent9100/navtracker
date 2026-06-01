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

}  // namespace navtracker
