#include "core/scenario/Ospa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {

double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff) {
  const std::size_t n = std::max(truth.size(), est.size());
  if (n == 0) return 0.0;

  std::vector<bool> t_used(truth.size(), false);
  std::vector<bool> e_used(est.size(), false);
  double sum_sq = 0.0;
  std::size_t pairs = 0;

  while (true) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t bi = 0, bj = 0;
    bool found = false;
    for (std::size_t i = 0; i < truth.size(); ++i) {
      if (t_used[i]) continue;
      for (std::size_t j = 0; j < est.size(); ++j) {
        if (e_used[j]) continue;
        const double d = (truth[i] - est[j]).norm();
        if (d < best) {
          best = d;
          bi = i;
          bj = j;
          found = true;
        }
      }
    }
    if (!found) break;
    const double clipped = std::min(best, cutoff);
    sum_sq += clipped * clipped;
    t_used[bi] = true;
    e_used[bj] = true;
    ++pairs;
  }

  const std::size_t unmatched = n - pairs;
  sum_sq += static_cast<double>(unmatched) * cutoff * cutoff;
  return std::sqrt(sum_sq / static_cast<double>(n));
}

}  // namespace navtracker
