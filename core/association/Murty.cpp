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

// Per-row degradation (backlog #34 M3): unassign (set to -1) every row whose
// assigned cell is +∞ in `C`. A +∞ in the Hungarian result is the BIG_M
// fallback for a row with no feasible edge; keeping the finite remainder yields
// a valid partial assignment instead of discarding the whole result. No-op when
// the assignment is already fully feasible.
void dropInfeasibleEdges(std::vector<int>& assignment,
                         const Eigen::MatrixXd& C) {
  for (int r = 0; r < static_cast<int>(assignment.size()); ++r) {
    const int c = assignment[r];
    if (c >= 0 && !std::isfinite(C(r, c))) assignment[r] = -1;
  }
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

  // A measurement column is "explainable" if it has ANY finite cell in the
  // ORIGINAL cost. A column that is all-+∞ in C0 (e.g. a rho==0 measurement with
  // no in-gate Bernoulli) is genuinely unexplainable — every assignment must drop
  // it. Used by the #34 F2 child-degradation to tell a genuinely-partial child
  // (keep) from a dead partition branch that a lock/forbid stranded (reject).
  std::vector<char> col_explainable(C0.cols(), 0);
  for (int cc = 0; cc < C0.cols(); ++cc)
    for (int rr = 0; rr < C0.rows(); ++rr)
      if (std::isfinite(C0(rr, cc))) { col_explainable[cc] = 1; break; }

  // Seed: solve LSAP on the unconstrained cost.
  std::vector<int> seed_assign = hungarianAssignment(C0);
  // Per-row degradation (backlog #34 M3): drop any infeasible (+∞) seed edge and
  // keep the feasible subset, rather than returning EMPTY and silently dropping
  // the whole cluster's children. Both callers (PmbmTracker, MhtTracker) filter
  // per-cell on isfinite and skip unassigned (-1) rows — the contract they were
  // built against. No-op on a fully-feasible seed (byte-identical; the common
  // case — Phase-0 probe measured 0 infeasible-seed hits across the gauntlet).
  dropInfeasibleEdges(seed_assign, C0);

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

    // The K-th accepted assignment needs no children: they would only feed
    // the heap for pops that never happen. At K=1 this skips one LSAP solve
    // per assigned row per call — the dominant PMBM cost on real workloads
    // (2026-07-05 runtime probe, docs/baselines/2026-07-05_pmbm_runtime_frontier.md).
    if (static_cast<int>(out.assignments.size()) == K) break;

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
      std::vector<int> child_assign = hungarianAssignment(C_child);
      // Per-row degradation (backlog #34 F2): let a child be a partial in exactly
      // the same way the seed (M3) can — but ONLY where the infeasibility is
      // genuine. Hungarian routes a column onto BIG_M only when that column has no
      // finite cell under the partition's locks. Two cases:
      //   - the column is unexplainable in the ORIGINAL C0 (all-+∞, e.g. rho==0):
      //     no assignment can explain it → drop the edge and keep the partial, so
      //     the K genuine siblings survive (review Finding 2);
      //   - the column IS explainable in C0 but a lock/forbid stranded it here:
      //     this partition branch has no feasible completion → reject the whole
      //     child. This preserves correct Murty pruning (feasibleAgainst was
      //     load-bearing, not a bug) so ordinary matrices enumerate unchanged.
      // On every real config rho_l>0 for all l → col_explainable is all true →
      // this reduces exactly to the pre-#34 wholesale rejection (byte-identical;
      // 0 infeasible children were seen across the gauntlet).
      bool dead_branch = false;
      for (int rr = 0; rr < static_cast<int>(child_assign.size()); ++rr) {
        const int cc = child_assign[rr];
        if (cc >= 0 && !std::isfinite(C_child(rr, cc))) {
          if (col_explainable[cc]) { dead_branch = true; break; }
          child_assign[rr] = -1;  // genuinely unexplainable column → degrade
        }
      }
      if (!dead_branch) {
        Partition child;
        child.cost = std::move(C_child);
        child.assignment = std::move(child_assign);
        child.total_cost = totalCostAgainstOriginal(child.assignment, C0);
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
