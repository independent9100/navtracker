#include "core/scenario/Gospa.hpp"

#include <cmath>
#include <limits>

#include "core/association/Hungarian.hpp"

namespace navtracker {

// Math:        Augmented (n+m)×(n+m) Hungarian solve. Cost blocks:
//   [0:n, 0:m]   — d(x_i,y_j)^p for truth↔est pairs.
//   [0:n, m:m+n] — diagonal: c^p/α (truth_i's miss slot), ∞ elsewhere.
//   [n:n+m, 0:m] — diagonal: c^p/α (est_j's false slot), ∞ elsewhere.
//   [n:n+m, m:m+n] — 0 (dummy↔dummy filler, always preferred over ∞).
// After hungarianAssignment, each row assignment falls into one of:
//   (a) i<n and j<m:     matched pair → add d^p to localization.
//   (b) i<n and j=m+i:   truth i missed  → increment n_missed.
//   (c) i=n+j and j<m:   est j is false  → increment n_false.
//   (d) i≥n and j≥m:     dummy↔dummy    → no cost (already 0).
// missed  = n_missed * c^p/α
// false_  = n_false  * c^p/α
// total   = localization + missed + false_
//
// Assumptions: p>0, alpha>0, cutoff>0; truth and est positions are in
//              the same Cartesian frame (metres ENU). Empty truth or
//              est sets are valid and yield pure false/missed costs.
// Rationale:   reuses the exact same augmented-cost matrix as gospaGreedy
//              (zero code duplication) and adds only the bucket
//              classification step after the solve. gospaGreedy becomes
//              a one-liner wrapping this function (DRY).
// Improve next: expose per-target labels (which truth_i was missed,
//               which est_j was false) for per-target diagnostic logging.
GospaComponents gospaComponents(const std::vector<Eigen::Vector2d>& truth,
                                const std::vector<Eigen::Vector2d>& est,
                                double cutoff,
                                double p,
                                double alpha) {
  if (truth.empty() && est.empty()) return {};

  const int n = static_cast<int>(truth.size());
  const int m = static_cast<int>(est.size());
  const double miss = std::pow(cutoff, p) / alpha;  // c^p / α per dropped target

  // Augmented (n+m)×(n+m) cost matrix — same construction as gospaGreedy.
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
      cost(n + i, m + j) = 0.0;                              // dummy filler

  const std::vector<int> row_to_col = hungarianAssignment(cost);

  GospaComponents g{};
  for (int i = 0; i < K; ++i) {
    const int j = row_to_col[i];
    if (j < 0) continue;
    const double c_ij = cost(i, j);
    if (i < n && j < m) {
      // (a) matched pair
      g.localization += c_ij;
    } else if (i < n && j == m + i) {
      // (b) truth i routed to its miss slot
      g.n_missed += 1;
    } else if (i >= n && j < m && i == n + j) {
      // (c) est j routed to its false slot
      g.n_false += 1;
    }
    // (d) dummy↔dummy: cost 0, skip
  }
  g.missed = g.n_missed * miss;
  g.false_ = g.n_false * miss;
  g.total  = g.localization + g.missed + g.false_;
  return g;
}

double gospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                   const std::vector<Eigen::Vector2d>& est,
                   double cutoff,
                   double p,
                   double alpha) {
  const GospaComponents g = gospaComponents(truth, est, cutoff, p, alpha);
  if (g.total == 0.0) return 0.0;
  return std::pow(g.total, 1.0 / p);
}

}  // namespace navtracker
