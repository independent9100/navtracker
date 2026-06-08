#include "core/association/Murty.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "core/association/Hungarian.hpp"

namespace navtracker {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Evaluate the cost of an assignment against the ORIGINAL cost matrix
// (not the partition-modified one). Returns +∞ if any assigned cell
// is itself +∞ in the original — i.e. the assignment crossed a
// genuinely-forbidden edge (should not happen for feasible solutions,
// but the Hungarian's BIG_M fallback could in principle produce such
// an "assignment"; we reject it).
double totalCostAgainstOriginal(const std::vector<int>& assignment,
                                const Eigen::MatrixXd& C0) {
  double total = 0.0;
  for (int r = 0; r < static_cast<int>(assignment.size()); ++r) {
    const int c = assignment[r];
    if (c < 0) continue;  // unassigned row in rectangular problem
    const double v = C0(r, c);
    if (!std::isfinite(v)) return kInf;
    total += v;
  }
  return total;
}

// Feasibility against a partition's working cost matrix: every assigned
// cell must be finite. Hungarian falls back to BIG_M for infeasible
// rows; we reject those so the K-best loop doesn't propagate impossible
// children.
bool feasibleAgainst(const std::vector<int>& assignment,
                     const Eigen::MatrixXd& C) {
  for (int r = 0; r < static_cast<int>(assignment.size()); ++r) {
    const int c = assignment[r];
    if (c < 0) continue;
    if (!std::isfinite(C(r, c))) return false;
  }
  return true;
}

struct Partition {
  Eigen::MatrixXd cost;            // working matrix with locks applied
  std::vector<int> assignment;     // row -> col
  double total_cost{0.0};          // cost evaluated against the ORIGINAL

  // std::priority_queue is a max-heap; flip the comparison so the
  // lowest-cost partition pops first.
  bool operator<(const Partition& other) const {
    return total_cost > other.total_cost;
  }
};

}  // namespace

KBestResult murtyKBest(const Eigen::MatrixXd& C0, int K) {
  KBestResult out;
  if (K <= 0 || C0.rows() == 0 || C0.cols() == 0) return out;

  // Seed: solve LSAP on the unconstrained cost.
  const std::vector<int> seed_assign = hungarianAssignment(C0);
  if (!feasibleAgainst(seed_assign, C0)) return out;

  std::priority_queue<Partition> heap;
  heap.push(Partition{C0, seed_assign,
                      totalCostAgainstOriginal(seed_assign, C0)});

  while (!heap.empty() && static_cast<int>(out.assignments.size()) < K) {
    // Pop best partition. Note: std::priority_queue exposes the top by
    // const reference; we need a mutable working copy of its cost
    // matrix for the forbid-and-lock loop below.
    Partition popped = heap.top();
    heap.pop();
    out.assignments.push_back(popped.assignment);
    out.costs.push_back(popped.total_cost);

    Eigen::MatrixXd C_locked = popped.cost;
    const int N = static_cast<int>(popped.assignment.size());

    // Generate children by forbidding each edge of `popped.assignment`
    // in turn, while locking previously-considered edges as required.
    for (int r = 0; r < N; ++r) {
      const int c = popped.assignment[r];
      if (c < 0) continue;  // unassigned row, nothing to forbid

      // Child: forbid (r, c) in the current locked matrix.
      Eigen::MatrixXd C_child = C_locked;
      C_child(r, c) = kInf;
      const std::vector<int> child_assign = hungarianAssignment(C_child);
      if (feasibleAgainst(child_assign, C_child)) {
        Partition child;
        child.cost = std::move(C_child);
        child.assignment = child_assign;
        child.total_cost = totalCostAgainstOriginal(child_assign, C0);
        if (std::isfinite(child.total_cost)) {
          heap.push(std::move(child));
        }
      }

      // Lock (r, c) as required for subsequent siblings: set row r
      // and col c to +∞ everywhere except (r, c).
      for (int rr = 0; rr < C_locked.rows(); ++rr) {
        if (rr != r) C_locked(rr, c) = kInf;
      }
      for (int cc = 0; cc < C_locked.cols(); ++cc) {
        if (cc != c) C_locked(r, cc) = kInf;
      }
    }
  }
  return out;
}

}  // namespace navtracker
