# Murty K-Best Assignment — Design

Spec for adding Murty's K-best ranked-assignment algorithm on top of the
existing `hungarianAssignment` LSAP solver, and integrating it into
`MhtTracker` to deliver true deferred-commitment TOMHT. Same algorithm
will service PMBM Phase 1 hypothesis enumeration.

Follows the project documentation standard: Math / Assumptions /
Rationale / Ways to improve.

## 1. The math

### What it computes

Given an N×M cost matrix `C` and a positive integer `K`, return the `K`
lowest-cost feasible assignments in ranked order:

```
A_1, A_2, …, A_K   with   cost(A_1) ≤ cost(A_2) ≤ … ≤ cost(A_K)
```

Each `A_k` is a column-index per row (or -1 if rectangular and the row
goes unassigned), exactly the same shape as `hungarianAssignment`. For
`K=1` this is the standard LSAP and the result matches today's solver
bit-for-bit.

### Murty 1968 partition recipe (Miller-Stone-Cox 1997 variant)

The algorithm enumerates the assignment space via a tree of *partition
nodes*. Each partition node stores:

- a cost matrix `C'` (the original `C` with some cells overwritten),
- two edge sets: **forbidden** (`E_forbid`) and **required** (`E_force`),
- the best LSAP solution `S` under those constraints, and its cost `v`.

Encoding the constraints:

- **Forbidden** edge `(r, c)`: set `C'(r, c) = +∞` (the BIG_M trick our
  Hungarian already handles).
- **Required** edge `(r, c)`: set `C'(r, *) = +∞` and `C'(*, c) = +∞`
  except `C'(r, c)`. Forcing the LSAP through `(r, c)` because no
  competing cell is feasible.

The K-best loop:

```
root = solve LSAP on C; constraints = ∅
heap = { (cost(root), root) }   # min-heap on cost
answers = []
while heap non-empty and |answers| < K:
  P = heap.pop_min()
  answers.append(P.S)
  # Generate up to |P.S| children by forbidding each edge of P.S
  # while locking previously-considered edges as required.
  C' = P.C
  for i in 0 .. len(P.S) - 1:
    (r, c) = P.S[i]
    # Child: forbid (r, c), force earlier edges (already locked in C').
    C_child = C' with C_child(r, c) = +∞
    S_child = solve LSAP on C_child
    if feasible:
      heap.push((cost(S_child), C_child, S_child))
    # Now lock (r, c) as required in C' for subsequent siblings.
    C' = C' with row r and col c set to +∞ everywhere except (r, c)
```

The forbid-then-lock loop produces exactly the partition Murty 1968
proves enumerates each feasible assignment at most once. Termination is
guaranteed because each child is strictly more constrained than its
parent.

### Sanity properties

- K=1 reproduces Hungarian.
- For K > number of feasible assignments, returns the available
  feasible answers (fewer than K) without hanging.
- Ties in cost: any consistent tie-break is acceptable; the algorithm
  doesn't guarantee a specific ordering within a tie.
- All-+∞ rows: the underlying Hungarian's BIG_M handling means the
  algorithm returns some K answers but the caller must verify
  feasibility via `std::isfinite(C(r, S[r]))` (same contract as
  today's `hungarianAssignment` per `MhtTracker::solveGlobalHypothesis`).

## 2. Assumptions

- `hungarianAssignment` exists and is correct for rectangular cost
  matrices with `+∞` cells (it is, per `core/association/Hungarian.cpp`
  + `tests/association/test_hungarian.cpp`).
- Cost matrix is reasonably small. Murty does `O(K · N · LSAP)` LSAPs,
  each `O(N³)`. For TOMHT use cases `N ≤ ~30` trees and `K ≤ ~5`, so
  this is well under a millisecond. For very large cost matrices (e.g.,
  raw detection-to-cell-of-Bernoulli matrices in PMBM with hundreds of
  detections) the asymptotic cost matters; not addressed in this first
  cut.
- We are NOT yet implementing the Miller-Stone-Cox 1997 optimisations:
  - Inheriting dual variables / partial solutions across partitions,
  - Sorting subproblems by lower-bound cost,
  - Pruning by best-cost-seen-so-far.
  These can be added later; the straightforward partition variant is
  correct and good enough for our K and N ranges.

## 3. Rationale

Why Murty over alternatives:

- **K=1 ≡ Hungarian**: zero-regression composition with existing code.
- **Conceptually local change**: builds on the LSAP primitive we already
  have. The PMBM port (Phase 1 in the integration plan) needs the same
  primitive, so one implementation serves both consumers.
- **Bayes-optimal for deferred commitment**: in TOMHT, carrying the K
  best per-scan assignments forward as alternative global hypotheses
  is the textbook deferred-decision recipe (Kurien 1990 §3.3;
  Blackman 2004 §V). At trunk merge time the right answer wins on
  N scans of evidence rather than one.
- **Same primitive PMBM uses**: PMBM's hypothesis enumeration truncates
  the exponential mixture by taking the K best joint associations per
  scan (García-Fernández et al. 2018 §VII; reference repo
  `Agarciafernandez/MTT`). Doing it once buys both consumers.

Why the straightforward partition variant (not Miller-Stone-Cox):

- Code complexity. The MSC optimisations require carrying LSAP duals
  across partition nodes — substantial restructuring of the Hungarian
  solver. For our `N · K` ranges the straightforward version is fast
  enough; we can promote later if benchmarks show LSAP solve time
  dominating.
- Existing Hungarian is dual-free in its public API. Restructuring it
  to expose duals would touch every caller. Defer.

Why we are NOT yet implementing the full deferred-commitment global-
hypothesis tree:

- That requires a parallel data structure (the *global hypothesis
  tree*) on top of the per-tree `TrackTree` we already have. Each
  global-hypothesis node is "one leaf per tree such that no
  measurement is shared this scan." Each scan, each global hypothesis
  spawns K children via Murty. After N scans, prune all but the
  top-scoring global hypothesis. This is substantial new state and
  data flow (~500 LoC + significant test work).
- The simpler near-term integration: at each scan, use Murty K-best
  to identify leaves that participate in any of the top-K cross-tree
  assignments. Tag those as "protected" from k-local pruning for one
  scan. Alternative-hypothesis leaves then persist N scans through
  the existing N-scan pruning, giving trunk merge a richer leaf set
  to choose from. This delivers most of the deferred-commitment win
  with a small change.
- We will note the full global-hypothesis-tree variant as a follow-up.

## 4. Ways to improve / what to test next

- **Inherit duals across partitions** (Miller-Stone-Cox 1997). Cuts
  LSAP solves from full-`O(N³)` to amortised `O(N²)` per child.
  Promote if Murty becomes a bottleneck.
- **Lazy partition expansion**. Instead of generating all `|S|`
  children of a popped partition immediately, generate only one per
  pop and re-push the parent with a "next-edge-to-forbid" cursor.
  Cuts heap size and memory.
- **Full global-hypothesis tree TOMHT**. The bigger structural
  upgrade. Worth doing only after we measure whether the protected-
  leaf shortcut closes the practical gap.
- **Murty for PMBM**. Phase 1 of the PMBM plan; reuse this solver
  unchanged. The cost matrix shape is different (rows are existing
  Bernoullis + new-target hypotheses, cols are measurements + miss),
  but the K-best primitive is identical.
- **Tests**:
  - K=1 ≡ Hungarian (bit-identical) on randomised cost matrices.
  - K=N! returns every permutation in non-decreasing cost order
    for small N.
  - K > feasible-count returns the feasible answers and stops.
  - Forbidden cells (+∞) honoured.
  - Ranked-order property: `cost(A_k) ≤ cost(A_{k+1})` for all k.

## 5. Sources & reference comparison

- Murty, K. G. (1968). *An Algorithm for Ranking all the Assignments
  in Order of Increasing Cost*. Operations Research 16(3), 682–687.
  The original algorithm.
- Miller, M. L., Stone, H. S., Cox, I. J. (1997). *Optimizing Murty's
  ranked assignment method*. IEEE Trans. AES 33(3), 851–862.
  The standard partition-tree variant used in modern trackers.
- Crouse, D. F. (2016). *On Implementing 2D Rectangular Assignment
  Algorithms*. IEEE Trans. AES 52(4), 1679–1696. The MATLAB
  TrackerComponentLibrary reference; also covers Murty.
- García-Fernández, Williams, Granström, Svensson (2018). *Poisson
  Multi-Bernoulli Mixture Filter: Direct Derivation and
  Implementation*. IEEE TAES 54(4). PMBM's use of Murty for
  hypothesis truncation, §VII.
- **Reference C++ implementation:** `fbaeuerlein/MurtyAlgorithm`
  GitHub (BSD-style); `Miller.h` is "inspired by Miller's
  pseudocode" and uses an auction solver instead of Hungarian.
  Verified via WebFetch on 2026-06-08; the partition data
  structure and main K-best loop match the recipe above.
  Our implementation uses our existing Hungarian solver as the inner
  LSAP, but the partition / forbid-and-lock loop is structurally
  identical.

## 6. Cross-references

- [`Hungarian.{hpp,cpp}`](../../../core/association/Hungarian.hpp) —
  inner LSAP solver. Murty calls into this.
- [`MhtTracker`](../../../core/pipeline/MhtTracker.hpp) — primary
  consumer for the TOMHT deferred-commitment use case.
- [PMBM integration plan](../plans/2026-06-07-pmbm-integration-plan.md)
  Phase 0 — Murty K-best is the explicit dependency.
- [`algorithm-review-2026-06-07.md`](../../algorithms/algorithm-review-2026-06-07.md)
  §6 — the deferred K>1 item from the MHT review is closed by this
  spec's implementation.
