#include "core/scenario/Metrics.hpp"

namespace navtracker {

int countIdSwitches(const std::vector<ScenarioStep>& steps, double cutoff) {
  if (steps.empty()) return 0;
  const std::size_t k = steps.front().truth.size();
  std::vector<std::uint64_t> last(k, 0);
  int switches = 0;
  for (const auto& s : steps) {
    for (std::size_t i = 0; i < s.truth.size() && i < k; ++i) {
      std::uint64_t best_id = 0;
      double best_d = cutoff;
      for (const auto& snap : s.tracks) {
        const double d = (s.truth[i] - snap.position).norm();
        if (d < best_d) {
          best_d = d;
          best_id = snap.id.value;
        }
      }
      if (last[i] != 0 && best_id != 0 && best_id != last[i]) ++switches;
      if (best_id != 0) last[i] = best_id;
    }
  }
  return switches;
}

}  // namespace navtracker
