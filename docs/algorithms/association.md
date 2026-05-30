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
