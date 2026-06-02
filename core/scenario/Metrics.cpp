#include "core/scenario/Metrics.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

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

PerWindowOspa computePerWindowOspa(const ScenarioResult& result,
                                   Timestamp t0,
                                   double window_dt_s) {
  PerWindowOspa out;
  if (result.steps.empty() || result.ospa_per_step.size() != result.steps.size())
    return out;

  std::vector<int> bucket_keys;
  std::vector<double> bucket_sums;
  std::vector<int> bucket_counts;
  std::unordered_map<int, std::size_t> key_to_idx;

  for (std::size_t i = 0; i < result.steps.size(); ++i) {
    const double rel = result.steps[i].time.secondsSince(t0);
    if (rel < 0.0) continue;
    const int key = static_cast<int>(rel / window_dt_s);
    auto it = key_to_idx.find(key);
    if (it == key_to_idx.end()) {
      key_to_idx.emplace(key, bucket_keys.size());
      bucket_keys.push_back(key);
      bucket_sums.push_back(result.ospa_per_step[i]);
      bucket_counts.push_back(1);
    } else {
      bucket_sums[it->second]  += result.ospa_per_step[i];
      bucket_counts[it->second] += 1;
    }
  }

  std::vector<std::size_t> order(bucket_keys.size());
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) {
              return bucket_keys[a] < bucket_keys[b];
            });

  out.per_window.reserve(order.size());
  for (std::size_t idx : order) {
    out.per_window.push_back(bucket_sums[idx] /
                             static_cast<double>(bucket_counts[idx]));
  }
  if (out.per_window.empty()) return out;

  double sum = 0.0;
  for (double v : out.per_window) sum += v;
  out.mean = sum / static_cast<double>(out.per_window.size());
  if (out.per_window.size() < 2) return out;
  double sse = 0.0;
  for (double v : out.per_window) sse += (v - out.mean) * (v - out.mean);
  out.stddev = std::sqrt(sse / static_cast<double>(out.per_window.size() - 1));
  return out;
}

}  // namespace navtracker
