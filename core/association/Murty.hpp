#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

// Result of Murty's K-best ranked-assignment algorithm. Up to K
// assignments are returned, in non-decreasing order of total cost
// against the ORIGINAL cost matrix (partition modifications inside the
// algorithm don't affect reported costs).
//
// `assignments[k]` is a `row -> col` mapping (size N = cost.rows()),
// matching the contract of `hungarianAssignment`: `assignments[k][r]`
// is the column assigned to row r, or -1 if row r was unassigned in a
// rectangular problem.
struct KBestResult {
  std::vector<std::vector<int>> assignments;
  std::vector<double> costs;
};

// Murty 1968 (Miller-Stone-Cox 1997 partition variant) K-best ranked
// assignments on top of the existing `hungarianAssignment` LSAP solver.
//
// Math: enumerate the assignment space via a priority queue of
// partition nodes. Each partition carries a modified cost matrix
// (with forbidden cells set to +∞ and required-edge rows/cols locked).
// At each pop, generate up to N children by forbidding each edge of
// the popped assignment while locking previously-considered edges.
// See docs/superpowers/specs/2026-06-08-murty-k-best-design.md.
//
// Complexity: O(K · N) LSAP solves, each O(N³). For K=3 and N ≤ 30
// this is well under a millisecond.
//
// Sanity properties (locked in by tests):
//   - K=1 result is identical to `hungarianAssignment` for any cost
//     matrix without all-+∞ rows.
//   - costs[0] <= costs[1] <= … <= costs[K-1].
//   - For K larger than the number of feasible assignments, returns
//     the feasible ones and stops (no hang, no UB).
//   - +∞ cells never appear in any returned assignment.
KBestResult murtyKBest(const Eigen::MatrixXd& cost, int k);

}  // namespace navtracker
