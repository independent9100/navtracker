#include "core/association/Hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {

// Rectangular Hungarian algorithm via potentials and shortest
// augmenting paths (a.k.a. Jonker–Volgenant). Handles N rows × M cols
// with N <= M (we pad with +∞ cols if N > M, then drop unassigned).
//
// Implementation follows the canonical 1-indexed formulation; we keep
// the off-by-one to minimize bugs vs published pseudocode.
std::vector<int> hungarianAssignment(const Eigen::MatrixXd& cost) {
  const int N = static_cast<int>(cost.rows());
  const int M = static_cast<int>(cost.cols());
  if (N == 0 || M == 0) return std::vector<int>(N, -1);

  // Pad to square. Replace +∞ with a finite BIG_M larger than any
  // feasible assignment so the algorithm makes progress on degenerate
  // rows without UB on -1 indexing. BIG_M cells will be preferred only
  // when no feasible assignment exists; the caller can detect this by
  // re-checking the original cost matrix against the returned assignment.
  const int K = std::max(N, M);
  double max_finite = 0.0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const double c = cost(i, j);
      if (std::isfinite(c) && std::abs(c) > max_finite) max_finite = std::abs(c);
    }
  }
  const double BIG_M = 1.0 + max_finite * 1e3 + 1e9;
  Eigen::MatrixXd C = Eigen::MatrixXd::Constant(K, K, BIG_M);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const double c = cost(i, j);
      C(i, j) = std::isfinite(c) ? c : BIG_M;
    }
  }
  // Padding rows: zero cost so the square LSAP finds a full assignment.
  for (int i = N; i < K; ++i) C.row(i).setZero();
  // Padding cols (only when N > M): also zero — surplus real rows can
  // slot into them; the caller filters them out at return time.
  for (int j = M; j < K; ++j) C.col(j).setZero();

  // u[i], v[j]: dual potentials; p[j]: row assigned to col j; way[j]:
  // predecessor column on the augmenting path.
  std::vector<double> u(K + 1, 0.0);
  std::vector<double> v(K + 1, 0.0);
  std::vector<int> p(K + 1, 0);
  std::vector<int> way(K + 1, 0);

  for (int i = 1; i <= K; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(K + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(K + 1, 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      int j1 = -1;
      for (int j = 1; j <= K; ++j) {
        if (used[j]) continue;
        const double cur = C(i0 - 1, j - 1) - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= K; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j]    -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  // p[j] = row+1 assigned to col j. Invert.
  std::vector<int> row_to_col(N, -1);
  for (int j = 1; j <= K; ++j) {
    const int i = p[j] - 1;
    if (i < 0 || i >= N) continue;
    if (j - 1 < M) row_to_col[i] = j - 1;
    // else: row was assigned to a padding column → leave -1.
  }
  return row_to_col;
}

}  // namespace navtracker
