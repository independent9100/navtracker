#include "core/scenario/Gospa.hpp"

#include <cmath>
#include <limits>

#include "core/association/Hungarian.hpp"

namespace navtracker {

double gospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                   const std::vector<Eigen::Vector2d>& est,
                   double cutoff,
                   double p,
                   double alpha) {
  if (truth.empty() && est.empty()) return 0.0;

  const int n = static_cast<int>(truth.size());
  const int m = static_cast<int>(est.size());
  const double miss = std::pow(cutoff, p) / alpha;  // c^p / α per dropped target

  // Optimal (min-cost) assignment via Hungarian, not greedy NN. GOSPA only
  // wants a pair matched when d^p < 2·c^p/α (else dropping both targets to
  // their cardinality penalties is cheaper). Encode that as a square
  // (n+m)×(n+m) augmented cost matrix:
  //   - block [0:n, 0:m]   : raw d(x_i,y_j)^p (truth↔est match).
  //   - block [0:n, m:m+n] : truth_i's own "miss" slot on the diagonal
  //                          (cost c^p/α), ∞ elsewhere.
  //   - block [n:n+m, 0:m] : est_j's own "false" slot on the diagonal
  //                          (cost c^p/α), ∞ elsewhere.
  //   - block [n:n+m, m:m+n]: dummy↔dummy filler (cost 0).
  // The solver matches i↔j only when d^p beats routing both to dummies, so
  // the total is Σ_matched d^p + (#missed + #false)·c^p/α — exactly GOSPA.
  const int K = n + m;
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(K, K, kInf);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j)
      cost(i, j) = std::pow((truth[i] - est[j]).norm(), p);
  for (int i = 0; i < n; ++i) cost(i, m + i) = miss;        // truth miss slot
  for (int j = 0; j < m; ++j) cost(n + j, j) = miss;        // est false slot
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      cost(n + i, m + j) = 0.0;                             // dummy filler

  const std::vector<int> row_to_col = hungarianAssignment(cost);
  double total = 0.0;
  for (int i = 0; i < K; ++i) {
    const int j = row_to_col[i];
    if (j >= 0) total += cost(i, j);
  }
  return std::pow(total, 1.0 / p);
}

}  // namespace navtracker
