# Association & Track-Management Algorithms

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec sections 7 and 11.3.

## 1. Gating (`mahalanobisDistance`)

**Math.** `(ẑ,H)=h(x)`; innovation `y=z−ẑ` (bearing wrapped); innovation
covariance `S=H P Hᵀ + R`; gating statistic `d²=yᵀ S⁻¹ y` (χ²-distributed with
DOF = measurement dimension under the Gaussian hypothesis).

**Assumptions.** Gaussian innovations; `S` positive-definite (true when `P` and
`R` are PD).

**Rationale.** Standard χ² gate; `d²` doubles as the association cost.

**Ways to improve / test next.** Reuse the computed `S`/inverse in the EKF
update; add the `0.5·ln|S|` term for a proper log-likelihood cost.

## 2. Greedy GNN association (`GnnAssociator`)

**Math.** Cost between a track and a measurement is the gated `d²`. Greedy
assignment: repeatedly select the globally smallest in-gate pair (`d² ≤ gate`),
assign it, remove both from consideration, repeat until no in-gate pair remains.
Unselected tracks and measurements are returned as unmatched.

**Assumptions.** At most one measurement per track per call; the gate is a χ²
quantile for the measurement dimension (e.g. 9.21 ≈ χ²₂ at 0.99); pairwise
independent costs.

**Rationale.** Deterministic and simple; sufficient to validate the fusion
pipeline before investing in optimal assignment (spec D5).

**Ways to improve / test next.** Optimal 2D assignment (Hungarian/auction) so a
locally cheap greedy pick can't block a better global solution; JPDA for
ambiguous clutter; MHT for deferred decisions; MMSI / sensor-track-ID hint
locking before kinematic gating; feature-aided (size/type) costs.

## 3. Track lifecycle (`TrackManager`)

**Math/Logic.** New track: `Tentative`, `hits=1`, `misses=0`. `recordHit`:
`hits++`, `misses=0`; `hits ≥ confirm_hits` → `Confirmed`. `recordMiss`:
`misses++`, `hits=0`; `misses ≥ delete_misses` → deleted (removed); else
`Coasting`. IDs: monotonic `uint64` from 1, never reused.

**Assumptions.** Consecutive-count policy (not sliding window); one hit/miss per
track per cycle; orchestration lives in the pipeline.

**Rationale.** Simplest deterministic M-of-N machine; stable never-reused IDs
satisfy the core identity invariant independent of MMSI (spec D1).

**Ways to improve / test next.** Score-based (log-likelihood-ratio)
confirmation/deletion; sliding-window M-of-N; re-promote Coasting → Confirmed on
re-acquisition.

## 4. Joint Probabilistic Data Association (`JpdaAssociator`)

**Math.** Soft data association via classical joint event enumeration.
M measurements, T tracks. Validation matrix `V` (M×T binary) gates pairs
by Mahalanobis distance. A feasible joint event θ is a function
{1..M} → {0, 1..T} where `θ(j) = 0` means "clutter" and `θ(j) = t` means
"assigned to track t"; no two measurements share a non-zero target.

Joint event weight (un-normalized):

```
w(θ) ∝ λ_C^N_FA · P_D^N_D · (1−P_D)^(T−N_D) · Π_{j: θ(j)≥1} N(z_j; ẑ_{θ(j)}, S_{θ(j)})
```

where N_D = |{j: θ(j) ≥ 1}|, N_FA = M − N_D. Normalize over all feasible
events (log-sum-exp). Marginal:
- `β(j, t) = Σ_{θ: θ(j) = t+1} w(θ)`
- `β_0(t) = Σ_{θ: t not detected} w(θ)`

**PDAF soft update (in `EkfEstimator::softUpdate`):**

- `y_combined = Σ_j β_jt y_jt`
- `x_t ← x_t + K_t y_combined`
- `spread = K_t [Σ_j β_jt y_jt y_jtᵀ − y_combined y_combinedᵀ] K_tᵀ`
- `P_t ← β_0_t P_t + (1−β_0_t)(I − K_t H_t) P_t + spread`

**Assumptions.** Full enumeration is tractable for the cluster sizes the
caller passes in (≤ ~6 tracks × ~6 measurements per scan). Beyond that,
K-best or Markov-chain approximations are needed. `P_D` (detection
probability per track per scan) and `λ_C` (clutter spatial density) are
configuration; they should match the sensor's empirical detection and
clutter characteristics. All gated measurements share a measurement model
(`softUpdate` uses `H`, `R` from the first).

**Rationale.** GNN commits to a hard pairing per scan; in clutter or
near-crossing geometries it commits wrong and the wrong measurement
contaminates the track, leading to ID switches and ghost tracks from
clutter. JPDA spreads the update across all gated assignments weighted by
their joint probability — no commitment, no contamination. Classical
enumeration is the exact baseline against which cheaper approximations
are compared. **Measured: 64% reduction in ID switches and 60% reduction
in ghost-track count vs GNN on the clutter-crossing scenario** (see
evaluation log).

**Ways to improve / test next.** (1) K-best joint events (Murty / Auction)
for tractable JPDA at larger cluster sizes. (2) JIPDA — also tracks
per-track existence probability, ties in with M-of-N lifecycle.
(3) UKF / PF / IMM `softUpdate` implementations (currently EKF-only;
others fall back to the no-op default). (4) Per-measurement-model
`R`-and-`H` support in `softUpdate` (currently assumes uniform model
across the gated set). (5) MHT as the natural next step — defers
commitment further by maintaining a hypothesis tree.

**Measured behaviour.** See `docs/algorithms/evaluation-log.md`.

## 5. Multiple Hypothesis Tracking — TOMHT (`MhtTracker`)

**Math.** Track-oriented MHT. Each track owns a `TrackTree` of hypothesis
nodes. Each node carries `(state, covariance, parent, scan_idx, score)`
where score is the cumulative log-likelihood-ratio (LLR).

Per scan:

- **Branch.** For each leaf in each tree, produce one missed-detection
  child (`Δscore = log(1 − P_D)`) and one child per gated measurement
  (`Δscore = log P_D + log N(z; ẑ, S) − log λ_C`). The EKF update is
  applied per measurement-assigned child.
- **K_local prune.** Drop the lowest-scoring leaves per tree until at most
  `k_max_leaves` remain.
- **N-scan trunk-merge.** For each tree, find each current leaf's
  scan-(t−N) ancestor. The ancestor with the highest descendant-leaf
  score wins; prune all other branches at that depth and their
  descendants. Commits the past ≥ N scans to a single hypothesis while
  the most recent N scans stay multi-hypothesis.
- **Spawn.** Any measurement not gated to any existing tree's best leaf
  starts a new track tree.
- **Output.** For each tree, the best-scoring leaf's `(state, covariance)`
  is the externally-visible track.

**Assumptions.** EKF backend (score formula assumes Gaussian likelihood);
state space matches `IEstimator::initiate`. Track identity is stable:
each tree gets a monotonic external id at spawn that is never reused.
Tracks are independent across trees: this first-cut MHT does **not**
enforce global non-conflict (the same measurement can in principle
contribute to two trees' winning leaves within one scan). The
K-best-global / Murty's-algorithm extension is captured as future work.
Tracks are dropped when their best leaf's score falls below
`score_delete_threshold`.

**Rationale.** GNN commits per scan and stays wrong; JPDA spreads update
mass per scan and stays uncommitted; MHT carries multiple full-history
hypotheses across N scans before any commitment. The structural advantage
shows up when the optimal assignment is ambiguous *at the current scan*
but becomes clear after seeing 2-3 more scans — exactly the
crossing-with-dropout case. **Measured: 7x lower OSPA than JPDA, zero ID
switches vs JPDA's 2, correct final track count (2 vs JPDA's 3)** on the
documented scenario.

**Ways to improve / test next.** (1) **K-best global non-conflict** via
Murty's or auction-style assignment over current leaves — the single
largest practical improvement and the standard TOMHT formulation.
(2) Score-based confirmation/deletion as a first-class alternative to
M-of-N. (3) UKF / IMM / PF backends for the per-leaf predict + update +
likelihood. (4) MHT with track merging (when two trees converge to
nearly the same state for several scans, merge them). (5) Sensitivity
sweep over (dropout length, N_scan, closest-approach distance).

**Configuration choices for the documented scenario.** P_D = 0.9,
λ_C = 1e-4, gate = 9.0 (χ²₂ at 0.99), N_scan = 3, K_max_leaves = 5,
score_delete_threshold = −15.0. N_scan = 3 covers the 4-scan dropout
because the leaf depth grows by 1 per processed scan (no growth during
the dropout window where there are no measurements to process), so the
trunk extends through the gap.

**Measured behaviour.** See `docs/algorithms/evaluation-log.md`.
