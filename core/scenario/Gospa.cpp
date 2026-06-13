#include "core/scenario/Gospa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {

double gospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                   const std::vector<Eigen::Vector2d>& est,
                   double cutoff,
                   double p,
                   double alpha) {
  if (truth.empty() && est.empty()) return 0.0;

  std::vector<bool> t_used(truth.size(), false);
  std::vector<bool> e_used(est.size(), false);
  double cost = 0.0;
  std::size_t matched = 0;

  // Greedy nearest-neighbour assignment. Only pairs whose raw distance
  // is < cutoff contribute as matches; a pair at the cutoff or beyond
  // is no cheaper than the per-truth + per-track miss penalty of
  // 2 · c^p / α = c^p / (α/2) — under α=2 that is exactly c^p, the same
  // as a clipped match cost — so the greedy stops there. For α≠2 the
  // exact tradeoff differs, but greedy still terminates on raw distance
  // ≥ cutoff: beyond the cutoff the match contributes c^p (clipped)
  // versus leaving them unmatched contributes 2c^p/α; either way no
  // further match strictly improves the objective when distances are
  // already ≥ c, so iterating past it is wasted work.
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
    if (!found || best >= cutoff) break;
    cost += std::pow(best, p);
    t_used[bi] = true;
    e_used[bj] = true;
    ++matched;
  }

  // Cardinality penalty: c^p / α per missed truth and per false track.
  const std::size_t missed = truth.size() + est.size() - 2 * matched;
  cost += static_cast<double>(missed) * std::pow(cutoff, p) / alpha;
  return std::pow(cost, 1.0 / p);
}

}  // namespace navtracker
