# Implementer prompt — Cl-4 Phase 2: build the conditional floor + occupancy veto (the one-config candidate), gated on vessel-past-structure

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(`docs/algorithms/comparison-baselines.md` §Cl-4). The Phase-1 arc (a→d)
measured every candidate mechanism; Phase 1d was **BUILD-ELIGIBLE** (merged
de6464c): the conditional floor `(M scans in 30 s AND net displacement ≥ D)`
revives the env-2 shore-huggers, and an occupancy veto — no revival when the
chain endpoint sits in a cell the shipped `LiveOccupancyModel` flags as
persistent *extended* structure — excludes the harbor pier with an 11–27 s
race margin, full margin box, zero pier revivals across chain-level AND
real-PMBM-track measurement. The extent gate (`extended_cells_min = 4`)
keeps dwelling vessels unflagged (16/16 revived, +0 s latency); philos is
inert. Probe params that passed: **M = 8 in 30 s, D = 50 m (70 m also
clean), persistence_bar = 0.5, extent 4, default cell/ramp**.

**One measured gap gates everything (Phase-1d handoff, arbiter-adopted):**
no fixture exercises a vessel transiting within the 25 m ramp of extended
structure — env-2's shore is a polygon (no dense returns), harbor's
near-pier boat is stationary. The veto's env-2 pass is inert-by-data, not
vessel-vs-adjacent-structure discrimination. **Stage 0 closes that gap
before any tracker code. If Stage 0 fails, the build does not happen.**

Budget: Stage 0 ~1 day; Stage 1 ~3–5 days; Stage 2 ~2–3 days. North-star
tag: Cl-4 Phase 2 — this is the claim's build.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4p2 -b cl4-phase2-floor-veto`, own build
dir; ALL fixtures from the MAIN tree — symlink; skips named BY NAME).
Checkpoint to the arbiter after Stage 0 (mandatory, before Stage 1) and at
final handoff. Commit on your branch; never merge or push master.

## Stage 0 — the vessel-past-structure gate (census only, binding)

Extend the harbor scenario family with a **moving boat past the pier**: a
vessel transiting parallel to the 120 m pier, inside its 25 m suppression
ramp, at a representative harbor speed (2–4 m/s), plus a second pass at
1 m/s (slow transit = the hard case: more dwell per cell near flagged
structure). Census-level (extend `tools/cl4_a3_census.cpp`, same offline
occupancy + chainer as Phase 1d):

- **G1 (binding):** the transiting vessel satisfies the floor and passes
  the veto with added latency ≤ 15 s vs the same transit with no pier
  present — at both speeds, across 5 seeds, and across the Phase-1d margin
  box (bar ±0.15, 2× cell).
- **G2 (binding):** the pier remains 100% vetoed with the vessel present
  (the vessel's returns must not un-flag or fragment the pier's connected
  component in a way that revives it).
- **Optional corroboration arm (report-only, no gate):** HAXR carries real
  dense shore returns + real transiting traffic — one kattwyk hour through
  the same census, reporting veto behavior near real structure. Do this
  only if it fits the budget; say explicitly whether it ran.

**STOP-AND-REPORT if G1 or G2 fails** — hand the numbers back; the endgame
decision (per-geography residual vs re-price) is the arbiter's and user's,
not a reason to tune the veto until it passes.

## Stage 1 — build (tracker code, only after the Stage-0 checkpoint GO)

Compose floor + veto into the PMBM birth path:

- **Mechanism freedom, constraints fixed.** The natural home is the
  R1/A2 pre-suppression machinery (suppressed births already accumulate
  tiny existence on re-detection, obstacle-scoped) extended with the
  chain-history rule; but the design is yours — write the design note
  first (math / assumptions / rationale / ways-to-improve, house standard)
  and include the Stage-1 inventory of how the coverage stack currently
  wires `LiveOccupancyModel`, so the veto reuses the instance a consumer
  already feeds (no second grid, no double feeding).
- **Hard constraints:** per-instance ctor-threaded config (`FloorConfig`
  or equivalent — never global/static); default OFF and **byte-identical
  when OFF** (prove with the R3 two-class A/B method); no change to
  `birth_existence_target` / gate arithmetic (λ_C cancellation invariant,
  `pmbm-design.md` §3.2.2); ADR 0002 presence guarantees untouched;
  deterministic (replay test must stay green); bounded memory for chain
  history (candidates expire when the 30 s window passes without
  satisfaction).
- **One parameter set.** The probe's params (M=8/30 s, D=50, bar 0.5,
  extent 4) ship as the single default. Stage 2 may NOT tune them
  per-workload — a per-geography parameter set would rebuild the exact
  seam Cl-4 exists to remove. If one global set cannot pass Stage 2, that
  is a finding, not a tuning exercise.
- **Runtime:** the floor bookkeeping is per-suppressed-candidate — show
  the per-scan latency deltas (the R2 latency columns exist); the Cl-4
  claim includes realtime, so the decimated budget (148 ms interval)
  must hold.
- **Tests:** unit tests for the chain accumulator, the veto decision, and
  expiry; the Stage-0 moving-boat-past-pier scenario becomes the primary
  acceptance scenario test; assertions banded/structural per the #24
  standard (no exact pins, no epsilon-fragile bars). New named config
  (proposed: `imm_cv_ct_pmbm_coverage_land_ivgate_floor` — final name with
  the arbiter at checkpoint).

## Stage 2 — judgment on the FULL promotion-dossier gauntlet

Same commit, all seven workloads, champion (`imm_cv_ct_mht`) vs prior
candidate (`..._ivgate`) vs new candidate (`..._ivgate_floor`), single
parameter set. The Cl-4 definition-of-done targets:

- **env-2 channel: no collapse** — GOSPA within ~10% of `pmbm_land`'s
  17.74 (the per-geography best), lifetime > 0.
- **philos:** within noise of 73.1 / card_err ≈ +6.9 (the floor must not
  re-admit what four dead paths re-admitted).
- **harbor:** card_err ≤ 11.64 (no regression; report any improvement —
  the floor+veto should not touch confirmed pier phantoms, say so
  explicitly).
- **Everything else** (Imazu battery, autoferry, sim geometry/multisensor
  rows): no regression beyond noise; #25 gate + veto riders unchanged.
- **Realtime:** decimated within budget; raw reported.
- **OFF-state:** byte-identical to master on every workload.

Report the table with no winner declared — name each residual and when it
hurts (house rule); the promotion decision is above your pay grade by
design.

## Docs (same PR, house rules — a build without these is INCOMPLETE)

1. **ADR 0001 second amendment:** record the Phase-1 arc (A3 measured
   dead — camera is a clutter source near shore; kinematic floor dead —
   association launders linear-structure walks; floor+veto adopted with
   the Stage-0 gate result), and revise the <50 m no-birth-zone
   limitation statement.
2. **`pmbm-design.md`:** new section for the floor+veto — math,
   assumptions (incl. the sim-first caveat: the walking pier is synthetic;
   no real pier-walk observed to date), rationale-over-alternatives (the
   four dead paths, with baseline doc links), ways-to-improve.
3. **`docs/learning/`:** extend the gating chapter (or add a section) —
   the floor, the race, and the extent gate in easy English with a figure
   via `figures/generate.py` (never hand-edit PNGs).
4. **`docs/integration-guide.md`:** new config struct + wiring (the
   occupancy instance the veto rides, datum-sink reminder) + config-
   appendix rows — the drift-guard test enforces the mention.

## Review + handoff

Before the final handoff: an adversarial review pass on the birth-path
change (this touches the PMBM hot path — same standard as the T2T engine),
findings fixed and re-verified. Final handoff: Stage-0 tables, design note,
Stage-2 gauntlet table, docs list, full suite green in your worktree with
fixture-gated skips named BY NAME, OFF-state byte-identical proof.
