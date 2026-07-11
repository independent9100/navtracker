# Implementer prompt — Cl-4 Phase 1b: does re-detection behavior separate near-shore vessels from near-shore clutter? (conditional coverage floor evidence probe)

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(one deployable PMBM configuration — `docs/algorithms/comparison-baselines.md`
§Cl-4). Phase 1 measured ranked path (a), A3 sensor-aware suppression, to a
clean **NO-BUILD** (2026-07-11, merged 93c31ec: the revival sensors and the
guard workload share no clutter-free sensor; camera is ~80% off-target near
shore). Ranked path (b) is now primary: a **conditional coverage floor**
(ADR 0001 §A2 family) — a suppressed near-shore birth that keeps getting
re-detected *and behaves like a vessel* ramps past the phantom-birth gate,
while structure and transient clutter do not. Its advantage over A3: the
discriminator is the target's own re-detection behavior, so it needs no
cross-workload sensor — every gauntlet workload can score it. This probe
measures — offline, before any birth-path code — whether the behavior
statistics actually separate. Budget ~2 days. North-star tag: Cl-4 Phase 1b.

**Two prior measurements bind this ticket (both are kill-anchors, not
trivia):**

1. The **unconditional** land-only A2 floor is measured-dead (2026-07-02 R1
   A/B: philos card_err +6.9→+40.15, gospa 73.1→106.9). Persistence-free
   flooring re-admits the philos near-shore clutter wholesale. Only a
   *conditioned* floor is on the table.
2. The harbor pier (13 points, P_D 0.9) is **persistent stationary
   structure** — a floor keyed on persistence alone re-admits it and
   regresses the freshly-reconciled harbor row (card_err +11.64 is REAL —
   `docs/baselines/2026-07-10_harbor_truthsort_reconcile.md`). So
   persistence cannot be the whole condition; the working hypothesis is
   **persistence + displacement** (a chain of re-detections that *moves*
   like a vessel).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4p1b -b cl4-phase1b-floor-probe`, own
build dir; autoferry + philos + harbor fixtures from the MAIN tree —
symlink; skips named BY NAME in the handoff). MEASUREMENT ONLY — zero
shipped behavior change. Extend/reuse `tools/cl4_a3_census.cpp`
(research-only bench target) rather than starting over; the gate radii,
in-band test, and per-scenario plumbing are already there.

## The question

For every workload, take the population of **suppressed birth candidates**
(measurements that would seed a birth but are killed by the coverage
stack's suppression + gate) and compute their **re-detection chain
statistics**: chains = consecutive-scan re-detections associable to the
same object (allow motion — chain by nearest-neighbour within a physically
justified radius per scan, state it up front), and per chain: length
(scans), duration (s), net displacement (m), and mean speed (m/s).

Does any fixed rule `(chain length ≥ M within N scans) AND (net
displacement ≥ D)` separate:

- **env-2 real shore-huggers** (must satisfy the rule, quickly) — from
- **philos near-shore suppressed clutter** (must not satisfy it) — and
- **harbor pier + sea clutter** (must not satisfy it)?

## Step 0 — suppression inventory (grounding; half a day max)

Per workload, *which mechanism* suppresses which births under the candidate
config (`imm_cv_ct_pmbm_coverage_land_ivgate`) — land ramp, coverage/
occupancy component, gate. Specifically:

- env-2: the collapse is the land-ramp no-birth zone (Phase-1 confirmed).
- philos: land ramp + gate (the A1-residual water clutter is the class the
  gate catches — identify it).
- harbor: chart-free by design, so the land ramp is inert — what does the
  coverage component suppress there (it moved card_err 11.1→8.0 in the
  dossier)? A conditional floor rides on *whatever* suppression fires, so
  the pier is in scope exactly to the extent the coverage stack suppresses
  it. Report precisely; if the pier population is NOT suppressed today
  (i.e. the 8.0 residual is all unsuppressed confirmations), say so — that
  changes K3 from "no re-admission" to "floor inert on harbor", both fine,
  but we must know which.

## Step 1 — chain census, three populations

Deliverable per workload: chain-statistics tables (length / duration /
displacement / speed distributions) for:

- env-2 (4 scenarios): the two real targets' in-band detections. Expected:
  long chains, vessel-like displacement. Also report **latency**: time from
  first in-band detection until a candidate rule (several (M,N,D) points)
  is first satisfied — a floor that revives a vessel 60 s late is a weaker
  claim; bound it.
- philos: suppressed in-band candidates, split by the Step-0 classes
  (stationary structure returns vs the transient water residual). Name the
  kill-risk explicitly: **drifting sea/wave clutter can move** — report the
  speed distribution of clutter chains so D is chosen against measured
  drift, not assumed-zero drift.
- harbor (5 seeds): pier returns (expected: very long chains, ~zero
  displacement), uniform sea clutter (expected: length-1 chains), and the
  3 anchored boats as a *reference* population (real, stationary — see
  scope note below).

## Step 2 — score the rule space

Sweep (M, N, D) over a small grid (state it before scoring) and report, per
point: env-2 targets revived (of 8) + revival latency; philos clutter
chains satisfying the rule (split by class) with first-order projected
philos damage (anchor: A1 re-admitted the full residual → gospa 73.1→100,
card_err +6.9→+36.2; scale by re-admission fraction, and A2-unconditional
→ +40.15 as the second anchor); harbor pier/clutter chains satisfying the
rule with projected card_err impact. One table, all three columns per
(M,N,D) point.

## Binding kill-criteria (agreed now; do not renegotiate after measuring)

**BUILD** requires ONE fixed (M, N, D) point satisfying ALL of:

- **K1 (revival):** ≥ 2/3 of the 8 env-2 in-band targets satisfy the rule,
  with revival latency ≤ 30 s from first in-band detection.
- **K2 (philos guard):** projected philos re-admission ≤ 10% of the A1
  residual (projected gospa ≤ ~76, card_err ≤ ~+10).
- **K3 (harbor guard):** zero pier chains and zero sea-clutter chains
  satisfy the rule (projected harbor card_err ≤ the 11.64 baseline — no
  regression), OR Step 0 shows the floor is inert on harbor (report which).

**NO-BUILD outcomes (write them up, don't force):**

- The (M,N,D) region satisfying K1 overlaps the region violating K2/K3
  (e.g. philos drift clutter moves as fast as slow shore-huggers) → path
  (b) dead as specified; report the overlap structure (which population
  bridges the gap). The arbiter then decides between path (c) re-pricing
  and accepting a documented per-geography residual in the Cl-4 claim.
- K1 fails on latency only → report the latency distribution; a slower
  bound is an arbiter decision, not yours.

**Scope note (state in the write-up, do not fix here):** a
displacement-conditioned floor deliberately does NOT revive *stationary*
near-shore vessels (anchored boats inside the band). That is unchanged
from today's behavior; their presence is covered by the static-hazard
channel (ADR 0002's accepted degraded mode). Use the harbor anchored-boat
reference population to quantify what such a rule would leave to that
channel — as information, not as a gate.

## Constraints (all binding)

1. **Zero shipped behavior change.** No config, no default, no birth-path
   code. Census tooling only (research bench target, like
   `navtracker_cl4_a3_census`).
2. Do not touch `birth_existence_target` / gate arithmetic (λ_C
   cancellation invariant, `pmbm-design.md` §3.2.2) or ADR 0002 presence
   guarantees.
3. Extraction boundary: plots/measurements as fed today; chaining is
   census-side analysis, not a new extraction stage.
4. Any Phase-2 build is judged on the FULL promotion-dossier gauntlet; this
   probe front-loads the three workloads where the trade lives.

## Checkpoint (mandatory before any Phase-2 work)

Hand back: Step-0 suppression inventory, Step-1 chain tables (three
populations), the Step-2 (M,N,D) score table, and the K1/K2/K3 verdict.
The arbiter writes the Phase-2 build ticket (or the pivot). Full suite
green in your worktree, skips named BY NAME vs the expected fixture-gated
set. Commit on your branch; do not merge or push master.
