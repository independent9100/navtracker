#include "core/scenario/Ospa.hpp"

#include <algorithm>
#include <cmath>

#include "core/association/Hungarian.hpp"

namespace navtracker {

double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff) {
  const std::size_t n = std::max(truth.size(), est.size());
  if (n == 0) return 0.0;

  // Optimal (min-cost) assignment via Hungarian, not greedy NN. Clipped
  // distance d̄ = min(‖·‖, c) is ≤ c for every cell, so matching is never
  // worse than the per-target cutoff penalty — the optimal matching pairs
  // exactly min(|X|,|Y|) targets, and the |‖X‖−‖Y‖| surplus each costs c².
  // Build a (rows = truth) × (cols = est) cost matrix of d̄²; surplus rows
  // return −1 from the solver.
  double sum_sq = 0.0;
  std::size_t pairs = 0;
  if (!truth.empty() && !est.empty()) {
    Eigen::MatrixXd cost(truth.size(), est.size());
    for (std::size_t i = 0; i < truth.size(); ++i) {
      for (std::size_t j = 0; j < est.size(); ++j) {
        const double d = std::min((truth[i] - est[j]).norm(), cutoff);
        cost(static_cast<int>(i), static_cast<int>(j)) = d * d;
      }
    }
    const std::vector<int> row_to_col = hungarianAssignment(cost);
    for (std::size_t i = 0; i < row_to_col.size(); ++i) {
      const int j = row_to_col[i];
      if (j < 0) continue;
      sum_sq += cost(static_cast<int>(i), j);
      ++pairs;
    }
  }

  const std::size_t unmatched = n - pairs;
  sum_sq += static_cast<double>(unmatched) * cutoff * cutoff;
  return std::sqrt(sum_sq / static_cast<double>(n));
}

}  // namespace navtracker
