#include "core/association/JointEvents.hpp"

#include <limits>

namespace navtracker {
namespace {

// Returns false if the running event count exceeds `max_events`, which
// aborts the recursion early (overflow). `out` is left in a partial state
// the caller discards.
bool recurse(const Eigen::MatrixXi& V,
             int j,
             std::vector<bool>& track_used,
             JointEvent& current,
             std::vector<JointEvent>& out,
             std::size_t max_events) {
  const int M = static_cast<int>(V.rows());
  const int T = static_cast<int>(V.cols());
  if (j == M) {
    out.push_back(current);
    return out.size() <= max_events;
  }
  current[j] = 0;
  if (!recurse(V, j + 1, track_used, current, out, max_events)) return false;
  for (int t = 0; t < T; ++t) {
    if (V(j, t) == 0) continue;
    if (track_used[t]) continue;
    track_used[t] = true;
    current[j] = t + 1;
    const bool ok = recurse(V, j + 1, track_used, current, out, max_events);
    track_used[t] = false;
    if (!ok) return false;
  }
  current[j] = 0;
  return true;
}

}  // namespace

std::vector<JointEvent> enumerateJointEvents(const Eigen::MatrixXi& V,
                                             std::size_t max_events) {
  const int M = static_cast<int>(V.rows());
  const int T = static_cast<int>(V.cols());
  std::vector<JointEvent> out;
  if (M == 0) {
    out.push_back(JointEvent{});
    return out;
  }
  std::vector<bool> track_used(T, false);
  JointEvent current(M, 0);
  const bool ok = recurse(V, 0, track_used, current, out, max_events);
  if (!ok) return {};  // overflow → empty signals caller to approximate
  return out;
}

std::vector<JointEvent> enumerateJointEvents(const Eigen::MatrixXi& V) {
  return enumerateJointEvents(V, std::numeric_limits<std::size_t>::max());
}

}  // namespace navtracker
