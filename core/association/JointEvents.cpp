#include "core/association/JointEvents.hpp"

namespace navtracker {
namespace {

void recurse(const Eigen::MatrixXi& V,
             int j,
             std::vector<bool>& track_used,
             JointEvent& current,
             std::vector<JointEvent>& out) {
  const int M = static_cast<int>(V.rows());
  const int T = static_cast<int>(V.cols());
  if (j == M) {
    out.push_back(current);
    return;
  }
  current[j] = 0;
  recurse(V, j + 1, track_used, current, out);
  for (int t = 0; t < T; ++t) {
    if (V(j, t) == 0) continue;
    if (track_used[t]) continue;
    track_used[t] = true;
    current[j] = t + 1;
    recurse(V, j + 1, track_used, current, out);
    track_used[t] = false;
  }
  current[j] = 0;
}

}  // namespace

std::vector<JointEvent> enumerateJointEvents(const Eigen::MatrixXi& V) {
  const int M = static_cast<int>(V.rows());
  const int T = static_cast<int>(V.cols());
  std::vector<JointEvent> out;
  if (M == 0) {
    out.push_back(JointEvent{});
    return out;
  }
  std::vector<bool> track_used(T, false);
  JointEvent current(M, 0);
  recurse(V, 0, track_used, current, out);
  return out;
}

}  // namespace navtracker
