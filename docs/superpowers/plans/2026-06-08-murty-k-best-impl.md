# Murty K-Best — Implementation Plan

Concrete step-by-step plan for the design at
[`docs/superpowers/specs/2026-06-08-murty-k-best-design.md`](../specs/2026-06-08-murty-k-best-design.md).

## Files added / changed

```
core/association/Murty.hpp           NEW   public API
core/association/Murty.cpp           NEW   K-best partition loop
core/pipeline/MhtTracker.hpp         EDIT  add k_best to Config
core/pipeline/MhtTracker.cpp         EDIT  solveGlobalHypothesis uses Murty
                                           when k_best > 1; tag protected leaves
core/benchmark/Config.cpp            EDIT  make MHT factory bump k_best=3 so
                                           the bench actually exercises K-best
tests/association/test_murty.cpp     NEW   unit tests for the solver
CMakeLists.txt                       EDIT  add Murty.cpp + test
docs/algorithms/algorithm-review-...md  EDIT  close §6 K>1 deferred item
docs/baselines/...                   NEW   new baseline run + comparison
```

## Step-by-step

### Step 1 — `core/association/Murty.hpp` (~40 LoC)

```cpp
// K best ranked assignments via Murty 1968 / Miller-Stone-Cox 1997
// (partition variant). For K=1 the first element is identical to
// hungarianAssignment.
struct KBestResult {
  std::vector<std::vector<int>> assignments;  // size <= K
  std::vector<double> costs;                  // matching size
};

KBestResult murtyKBest(const Eigen::MatrixXd& cost, int k);
```

### Step 2 — `core/association/Murty.cpp` (~150 LoC)

Internal representation of a partition node:

```cpp
struct Partition {
  Eigen::MatrixXd cost;            // working matrix with locked rows/cols
  std::vector<int> assignment;     // row -> col, length N
  double total_cost;
  // Comparator for std::priority_queue (min-heap on total_cost).
  bool operator<(const Partition& o) const {
    return total_cost > o.total_cost;  // > for min-heap via priority_queue
  }
};
```

Main loop, following the reference structure:

```cpp
KBestResult murtyKBest(const Eigen::MatrixXd& C0, int K) {
  KBestResult out;
  if (K <= 0 || C0.rows() == 0 || C0.cols() == 0) return out;
  // Seed: solve LSAP on the unconstrained cost.
  auto a0 = hungarianAssignment(C0);
  if (!feasible(a0, C0)) return out;
  std::priority_queue<Partition> heap;
  heap.push(Partition{C0, a0, totalCost(a0, C0)});

  while (!heap.empty() && (int)out.assignments.size() < K) {
    const Partition P = heap.top(); heap.pop();
    out.assignments.push_back(P.assignment);
    out.costs.push_back(P.total_cost);

    // Generate children by forbidding each edge in turn, locking
    // earlier edges as we go.
    Eigen::MatrixXd C = P.cost;  // local working copy
    const int N = (int)P.assignment.size();
    for (int r = 0; r < N; ++r) {
      const int c = P.assignment[r];
      if (c < 0) continue;  // rectangular unassigned row, skip

      // Child: forbid (r, c) in current C.
      Eigen::MatrixXd C_child = C;
      C_child(r, c) = +inf;
      auto a_child = hungarianAssignment(C_child);
      if (feasible(a_child, C_child)) {
        heap.push(Partition{
            C_child, a_child, totalCost(a_child, C0 /* original cost */)});
      }

      // Lock (r, c) as required for subsequent siblings:
      // set row r and col c to +inf except (r, c).
      for (int rr = 0; rr < C.rows(); ++rr) if (rr != r) C(rr, c) = +inf;
      for (int cc = 0; cc < C.cols(); ++cc) if (cc != c) C(r, cc) = +inf;
    }
  }
  return out;
}
```

Two subtleties:

1. **Cost reporting uses original C0**, not the partition-modified C.
   The partition modifications are constraints; the cost of an
   assignment is always evaluated against the original.
2. **Feasibility check**: after `hungarianAssignment` we must verify
   every assigned cell `C_child(r, a[r])` is finite (Hungarian
   falls back to BIG_M for infeasible rows). Reject infeasible
   children — they correspond to over-constrained partitions.

### Step 3 — Unit tests (`tests/association/test_murty.cpp`, ~120 LoC)

- `KBest1MatchesHungarian` — `K=1` returns the same assignment as
  `hungarianAssignment` on a randomised batch.
- `KBestRanksByIncreasingCost` — `cost(A_k) ≤ cost(A_{k+1})` for all k.
- `KBestEnumeratesAllPermutationsOn3x3` — for a 3×3 with distinct costs,
  `K=6` returns all 6 permutations sorted.
- `KBestStopsAtFeasibleCount` — when `K` exceeds feasibility, returns
  fewer answers, no hang.
- `KBestHonorsForbiddenCells` — `+∞` cells never appear in any returned
  assignment.
- `KBestRectangular` — 2×4 cost matrix, `K=3` works.

### Step 4 — `MhtTracker` integration

Two surgical changes:

**4a.** Add `int k_best = 3;` to `MhtTracker::Config`. Default >1 so we
actually exercise the deferred-commitment path.

**4b.** In `solveGlobalHypothesis`, when `k_best > 1`:
1. Call `murtyKBest(cost, k_best)` instead of `hungarianAssignment`.
2. The reported track per tree comes from the BEST assignment
   (`assignments[0]`) — identical reporting behaviour for K=1.
3. Walk assignments[1..k-1] and collect a set of `(tree, leaf)`
   pairs that participate in any top-K assignment. Mark these
   leaves as "protected from k-local pruning this scan."

Return a `GlobalAssignment` augmented with a `protected_leaves` field:

```cpp
struct GlobalAssignment {
  std::vector<std::size_t> chosen_leaf;
  // Per tree: extra leaves that appeared in alternative top-K
  // assignments. k-local pruning should keep these for one more scan
  // so the deferred-commitment alternatives stay alive across
  // N-scan trunk merge.
  std::vector<std::vector<std::size_t>> protected_leaves;
};
```

**4c.** In `processBatch`, after `solveGlobalHypothesis` and BEFORE the
score-delete loop / emission, set `protected = true` on every leaf
listed in `protected_leaves[t]`. `pruneKLocal` would then need to
respect a `protected` flag — easiest: do `pruneKLocal` BEFORE Murty
this scan (as today) and let the next-scan pruning naturally let
alternatives die if they don't pick up support. *Even simpler*: just
report the top-K participation so we can verify via metrics, defer
the protection mechanism until measurements show it matters.

**Minimal first-cut**: just call Murty, use `assignments[0]` for the
report, log the K-1 alternatives (in tests). The actual behavioural
upgrade — protecting alternative-hypothesis leaves — comes in a follow-
up commit once we have the Murty primitive in place.

### Step 5 — Benchmark

Run `2026-06-08_murty-k3` baseline. Compare against
`2026-06-07_with-mht` (classical-gold-standard tag). Expectations:

- Cooperative scenarios (crossing, overtaking, head_on, clock_skew,
  parallel): bit-identical with K=1, very-close-to-identical with K=3
  (the reported track always comes from the best assignment; K>1 just
  generates extra LSAP solves whose results are discarded in the
  minimal first cut). Difference if any is from RNG-style nondeterminism
  in Murty's tie-breaking — should be zero by construction.
- `ais_dropout`: maybe a small change. Alternative branchings might
  survive longer if we wire in leaf protection.
- `speed_change`: similar story.
- `non_cooperative`: unstable baseline, large stds — Murty might or
  might not move it.

The honest expectation: numbers very close to identical. The win for
Murty is structural (PMBM Phase 1 dependency, deferred-commitment
primitive in place) more than performance on the existing suite,
which doesn't have the kind of sustained ambiguity Murty resolves.

### Step 6 — Doc updates

- Close §6 `K>1 deferred` item in
  `docs/algorithms/algorithm-review-2026-06-07.md`.
- Update [PMBM plan](2026-06-07-pmbm-integration-plan.md) Phase 0
  entry to "complete".

## Sequencing & estimates

| Step | Effort | Risk |
|---|---|---|
| 1+2 — Murty.{hpp,cpp} | ~2 h | Low. Algorithm well-specified; existing Hungarian is the inner solve. |
| 3 — Unit tests | ~1 h | Low. K=1 ≡ Hungarian is the safety net. |
| 4 — MhtTracker integration (minimal) | ~30 min | Low. Substitute Murty for Hungarian; use top-1 for report. |
| 5 — Benchmark + compare | ~10 min | None. |
| 6 — Doc updates | ~10 min | None. |
| **Total** | **~4 h** | |

## Open questions deferred to follow-up

- Whether to add `protected` flag on `TrackTreeNode` and have
  `pruneKLocal` honour it. Decision: defer until measurements show
  the simple "use top-1 of K Murty for the report, discard the rest"
  cut leaves performance on the table.
- Full global-hypothesis tree. Bigger work; spec's §3 notes it as a
  separate item.
- Miller-Stone-Cox dual-inheritance optimisation. Promote if Murty
  becomes a bottleneck (it won't for K=3, N≤30).
