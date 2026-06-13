# 11 — Gating, GNN, Hungarian

> Prerequisites: [04 — Kalman filter](04-kalman-filter.md),
> [10 — Measurement models](10-measurements-frames-time.md).
> Next: [12 — JPDA](12-jpda.md).

A single Bayes filter assumes the measurement clearly belongs to
*its* track. In a multi-target world that is not given. At every
time step we may have:

- 5 tracks alive,
- 7 measurements just arrived,
- some pairs of (track, measurement) are obvious matches,
- some measurements are clutter (false alarms),
- some tracks miss out (no measurement this scan).

We need to decide *which measurement updates which track*. This is
**data association**. This chapter covers three pieces:

- **Gating** — the cheap filter that throws away (track, z) pairs
  that are obviously not from each other.
- **GNN** — Global Nearest Neighbour. The simplest greedy
  association.
- **Hungarian** — the optimal-pairing algorithm we use when greedy
  is not good enough.

## 1. Gating — Mahalanobis distance

For a candidate (track, measurement) pair, compute the
**innovation** `ŷ = z − h(x̂⁻)` (bearing-wrapped). The innovation
covariance is `S = H P⁻ Hᵀ + R`. The **Mahalanobis distance
squared** is:

```
d² = ŷᵀ S⁻¹ ŷ
```

If `d²` is larger than a chosen **gate** `γ`, this measurement
is too surprising to plausibly belong to this track. Throw the
pair away.

Why is this the right quantity?

Under the Gaussian hypothesis, `d²` is **chi-squared distributed**
with degrees of freedom equal to the measurement dimension `m`.
A `0.99` quantile gives:

| m   | χ²_m (p=0.99) |
|-----|---------------|
| 1   | 6.63          |
| 2   | 9.21          |
| 4   | 13.28         |

So with `m = 2` (range/bearing) and gate `γ = 9.21`, you expect to
keep 99 % of the *real* (track, z) pairs and reject most clutter.

### Picture: the gate as an ellipse, with in-gate and out-of-gate measurements

![Mahalanobis gate around predicted measurement](figures/11-gating-ellipse.png)

The "+" is the predicted measurement `ẑ = H·x̂⁻`. The ellipse
shape comes from `S = H P Hᵀ + R`. Green measurements fall
inside the gate (kept as candidates); red ones outside are
dropped. Notice `z_2` and `z_1` both look "close" on the chart,
but `z_1` is in-gate because the ellipse is elongated in that
direction. **The Mahalanobis distance `d² = ŷᵀ S⁻¹ ŷ` is the
*scaled* distance, where the scaling is `S`.** Two equal raw
distances on the chart can give very different `d²` if `S` is
elongated.

A measurement that falls inside the ellipse is "in-gate". Outside,
"out-of-gate".

### Cost-conscious gating

We do not actually invert `S`. We Cholesky-factor `S = L Lᵀ` and
solve `L u = ŷ` by forward substitution. Then `d² = uᵀ u`. This is
both faster and more numerically stable.

We also use a **score-style cost** `c = d² + log |S|` in the MHT
chapter so that the cost is a true log-likelihood. Pure `d²` is
fine for greedy GNN where the `log |S|` term cancels across pairs
of the same dimension.

### Code pointer

`core/association/Gating.{hpp,cpp}` — `mahalanobisDistance`.
`docs/algorithms/association.md` §1.

## 2. Greedy GNN — the baseline associator

Once gating has thrown away the obviously-wrong pairs, we have a
small list of candidate (track, z) pairs each with a cost `d²`.

Greedy GNN says: **take the smallest-cost pair, lock it in,
remove both track and z from consideration, repeat**.

```
1. Compute cost[t][z] for all in-gate pairs.
2. Find (t*, z*) = argmin cost.
3. Assign z* to t*. Remove t* and z*.
4. Goto 2 until no in-gate pair remains.
5. Remaining tracks → unmatched. Remaining z's → potential new tracks.
```

This is `O(T · M · log(T·M))` for `T` tracks and `M`
measurements. Fast, deterministic, simple.

When does GNN fail?

```
           z_1   z_2
   t_1     2     1
   t_2     3     ∞ (out of gate)
```

Greedy picks `(t_1, z_2)` with cost 1, leaving `t_2` to pair with
`z_1` at cost 3. Total cost: 4.

But the optimal assignment is `(t_1, z_1) cost 2`, `(t_2, z_2)
∞ — invalid`, so actually no, in this case greedy *is* optimal.
Let's try another:

```
           z_1   z_2
   t_1     1     2
   t_2     1     3
```

Greedy picks `(t_1, z_1) cost 1`. Then `(t_2, z_2) cost 3`. Total: 4.
Optimal is `(t_1, z_2) cost 2, (t_2, z_1) cost 1`, total 3.

So greedy can pick suboptimal globally. That's when we use
Hungarian.

### Code pointer

`core/association/GnnAssociator.{hpp,cpp}`.
`docs/algorithms/association.md` §2.

## 3. The Hungarian algorithm — globally optimal pairing

The Hungarian algorithm solves the **assignment problem** in
polynomial time: given a cost matrix `C[i][j]`, find the
permutation that minimises `Σ_i C[i, π(i)]`.

For our case, `i` ranges over tracks, `j` over measurements, and
`C[i][j]` is the Mahalanobis cost (or `+∞` if out of gate). The
algorithm produces the globally optimal one-to-one assignment.

In practice we use a rectangular variant: typically `|T| ≠ |M|`,
and unmatched is allowed by padding the matrix with dummy rows
or columns with cost `0` (or a configurable miss/birth penalty).

Hungarian is `O(n³)` where `n = max(|T|, |M|)`. At our typical
sizes (dozens of tracks and dozens of measurements per scan)
this is fast enough.

```
       z_1   z_2   z_3   ø
 t_1   2.1   ∞     8.0   M
 t_2   ∞     1.4   ∞     M
 t_3   3.0   9.0   2.5   M
 ø      B     B     B    0

   ø = dummy. B = "birth": cost of treating z as a new track.
   M = "miss": cost of leaving t unmatched.
```

The Hungarian solver returns assignments like `(t_1, z_1)`,
`(t_2, z_2)`, `(t_3, z_3)` or marks pairs as ø (unmatched).

### Why not always Hungarian?

Hungarian is more expensive per scan than greedy GNN. For very
small scans the difference is negligible. The reason we still
have both is that **greedy is the textbook starting point**, and
in many runs the two give identical assignments. Hungarian wins
when there is geometric ambiguity (parallel tracks, crossing
tracks). The codebase configures Hungarian by default for
production but keeps greedy as a fallback / for unit tests.

### Code pointer

`core/association/Hungarian.{hpp,cpp}`.
`core/association/JointEvents.{hpp,cpp}` (used in JPDA / MHT).

## 4. The bigger picture: associator hierarchy

In navtracker the associator is an **interface** (`IDataAssociator`).
The concrete implementations form a ladder:

```
GNN              ←  simplest. Hard 1:1. Greedy.
Hungarian        ←  hard 1:1 globally optimal.
JPDA             ←  soft 1:N: every (t,z) gets a probability β.
MHT              ←  deferred decision: keep multiple hypotheses,
                    prune later.
```

Each is the strict superset of the previous (in terms of
information retained) and the strict subset (in terms of
discarded ambiguity). MHT keeps the most ambiguity, so it is
the most powerful but also the most expensive.

The next two chapters (12, 14) cover JPDA and MHT. This chapter
ends here.

## 5. Assumptions of gating + GNN/Hungarian

| Assumption                                       | When it pinches                                   |
|--------------------------------------------------|---------------------------------------------------|
| Gaussian innovations                             | Sharp manoeuvres → real innovations bigger than `S` predicts → false rejects |
| `S` correct                                      | Bad `Q` or `R` → too-tight or too-loose gates    |
| 1:1 matching (one z per track per scan)          | Closely-spaced targets share blobs → MHT needed  |
| Costs additive across pairs                      | Holds for log-likelihoods under independence     |
| Same measurement dimension across pairs in batch | We keep dimensions consistent per associator call|

## 6. Why we can use this here

For the AIS path the (track, measurement) ambiguity is essentially
zero because the MMSI in the AIS message hints which track to
choose. For ARPA in moderate traffic the ambiguity is small. GNN
or Hungarian suffices.

When traffic is dense (e.g. a harbour with closely-spaced ferries)
the assignment becomes ambiguous and we step up to JPDA or MHT.
The library lets you pick the associator at composition time.

## 7. Where this lives in code

- `core/association/Gating.{hpp,cpp}` — Mahalanobis distance,
  gating helpers.
- `core/association/GnnAssociator.{hpp,cpp}` — greedy GNN.
- `core/association/Hungarian.{hpp,cpp}` — Kuhn-Munkres.
- `ports/IDataAssociator.hpp` — interface.

## 8. What we did not pick (yet)

- **Auction algorithm** — alternative to Hungarian, often faster in
  practice for sparse problems. Backlog.
- **2-D linear assignment with `k`-best solutions** (Murty) —
  needed for MHT (chapter 14), already implemented.
- **Feature-aided gating** — fold in target size / class into the
  cost. Future work.

---

Previous: [10 — Measurements, frames, time](10-measurements-frames-time.md)
Next: [12 — JPDA](12-jpda.md) →
