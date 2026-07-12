# Implementer prompt — Cl-4: the user's two-threshold scheme — shape-collapse proof + pending-band census

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
endgame (`docs/baselines/2026-07-11_cl4_cliff_price_list.md` Parts 1+2, the
2-D W_off × floor surface + phantom map, merged 7080949). The user proposed
a generalized near-shore scheme (2026-07-12): instead of one ramp + one
fixed bar, use distance-dependent accept/reject thresholds — and, crucially,
TWO thresholds, implying a middle "pending" zone between reject and accept.
This ticket (a) closes the pure-shape half with a short proof + empirical
check, and (b) measures the genuinely new half — the pending band — against
the existing Pareto front. Measurement only, zero shipped changes. Budget
~1–1.5 days. North-star tag: Cl-4 endgame.

**The key structural fact (state it in the write-up, verify it in Step 1):**
under adaptive birth, every birth candidate's pre-suppression existence is
PINNED to `birth_existence_target` = 0.1 (the λ_C-cancellation invariant,
`pmbm-design.md` §3.2.2). There is no per-return score variation. Distance
to shore is therefore the ONLY input any ramp/bar shape can use — so every
one-shot scheme (any monotone ramp shape × any monotone bar(d)) collapses
to a single admit/reject boundary distance d*, which the 2-D sweep already
swept. A shape cannot beat the measured front. What CAN beat it is a scheme
that uses information distance doesn't have — TIME (re-detection across
scans), which is exactly what the user's two-bar middle zone adds.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-cl4pb
-b cl4-pending-band-probe`, own build dir, fixtures inner-level symlinked,
0-skip runs; commit on your branch, never merge/push master). Extend the
census tool (`tools/cl4_a3_census.cpp` — it already has geo/chain/rnew/
phantomtrack modes and the Phase-1b chain data structures).

## Step 1 — shape-collapse verification (half day max)

Empirically confirm the collapse claim: pick two deliberately non-linear
suppression shapes (e.g. quadratic ramp; two-segment strict-then-lenient
ramp) with the same effective admit boundary d* as an existing sweep cell
(e.g. W25/f0.10 → d* = 25 m), run the philos + env-2 rows, and show the
results match the corresponding linear cell (within seed noise). One table.
If they do NOT match, stop and report — the collapse argument has a hole
and the shape family reopens.

## Step 2 — the pending-band census (the new half)

Scheme under test: in-band birth candidates (0 < d < 50 m, charted
coastline only — chart-free workloads untouched by construction) are
neither killed nor confirmed at first sight; they confirm only after
re-detection on ≥ K scans within a short window. Offline census first — no
tracker code:

- **philos (the guard):** split the in-band OFFSHORE population (the 345
  water/offshore returns — the 1361 inland returns are hard-gated
  regardless; keep them out of the denominator) into re-detection chains
  (Phase-1b chainer). For K ∈ {2, 3, 5, 8}: how many chains reach length K
  (= would confirm through the pending band)? Split by distance (0–10 /
  10–25 / 25–50 m) and by stationary vs moving. This is the decisive
  number: the transient water clutter should die in pending; the question
  is how much persistent offshore return (moored objects, stable
  reflectors) survives.
- **env-2 (the revival):** the vessels re-detect on ~100% of scans, so the
  cost is pure latency ≈ K scans — report it per K in seconds, and confirm
  no target fails to reach K within its first in-band window.
- **harbor + env-1 (no-collateral):** by construction the band only exists
  where a coastline is loaded — state it, and spot-confirm harbor/env-1
  rows are untouched (harbor is chart-free; env-1 vessels are offshore).

## Step 3 — score against the existing front

For each K: projected philos phantom count (chains ≥ K) vs the measured
front — today's best one-shot cell W25/f0.10 costs **+10.45 tracks/scan,
strip-confined** at 8/8 env-2 revival. The pending band is INTERESTING iff
some K lands meaningfully below that (fewer phantoms at the same 8/8
revival, placement still strip-confined), after honestly counting the
persistent-offshore survivors. Also compare against the A2 anchor
(unconditional floor-lowering = +36.15 at 0.05): the pending band IS
floor-lowering + a persistence requirement, so it must sit far below A2 to
be real. One combined table; no recommendation — the price decision stays
with the user.

**Known risk to measure, not argue:** persistent stationary offshore
returns near real charted shores (anchored/moored objects, stable
reflectors) pass any pure-persistence pending band. Phase 1b measured
philos in-band chains as "dominated by stationary structure" but never
split them inland-vs-offshore — this census does. If the persistent
offshore population is large, the pending band inherits the A2 failure and
dies; if it is small (most persistence is inland, already hard-gated), the
band may genuinely beat the front. Either answer closes the question.

## Constraints

1. Zero shipped behavior change; census/bench tooling only (the sanctioned
   env-knob method if a sweep needs a new lever; unset = byte-identical).
2. `birth_existence_target` / gate arithmetic untouched (§3.2.2 invariant);
   ADR 0002 presence guarantees untouched.
3. Full suite green at 0 skips; skips named BY NAME if any gate is
   data-absent. Commit on your branch; do not merge or push master.

## Checkpoint

Hand back: the Step-1 collapse table, the Step-2 K-census (philos split ×
distance × motion; env-2 latency), the Step-3 comparison vs W25/f0.10 and
the A2 anchor. The arbiter + user then pick the endgame: one-shot W25
integration, pending-band Phase-2 build, or keep today's default.
