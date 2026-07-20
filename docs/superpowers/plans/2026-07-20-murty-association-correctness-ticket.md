# Implementer prompt — Murty association-correctness cycle (#34 M5 + M3): probe, fix, measure

Status: ready to hand off. Paste everything below the line. Origin: the
2026-07-09 review's M5/M3 findings, triaged PLAUSIBLE-OPEN into backlog #34
and parked at the batch-1 checkpoint as its-own-cycle material (F2 precedent —
they change association ranking on the deployable adaptive-K PMBM). #25 is
closed (ivgate, merged e1697f0), so this cycle stands alone. Budget ~2 days
(probe 0.5 / build 0.5 / gauntlet + write-up 1).

**Why probe-first is binding:** this repo has already found wrong math that
was *load-bearing* (the philos miss-P_D brake; see
`project`-level notes in `docs/algorithms/pmbm-design.md` §3.2.2 and the
clutter-birth campaign close-out). A correctness fix to association ranking
can shift cardinality behavior somewhere. The §5.0-probe precedent (clutter
campaign) killed a doomed build before it was written — same discipline here:
price the blast radius offline BEFORE writing behavior code.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-murty -b murty-association-correctness` off
current master, own build dir, fixtures inner-level symlinked — the
partially-tracked dirs `philos/`, `sim_multisensor/`, `rbad/` need per-entry
inner symlinks; set `SIMMS_DIR`/`RBAD_DIR`; suite under
`NAVTRACKER_REQUIRE_FIXTURES=1`, proof is 0 skips. Commit on your branch;
never merge/push master. Tear down your `../navtracker-bm1` worktree first.)

## The two defects (verified at these lines at batch-1 selection)

- **M5** — `core/pmbm/PmbmTracker.cpp:821`: the assignment cost Murty
  enumerates omits `log(p_D/(1−r·p_D))`, so K-best enumeration order ≠
  posterior hypothesis order → potentially wrong argmax under K=1 /
  adaptive-K floor (the deployable regime). NOTE: the 45a504d K=1 early-exit
  fix was a *runtime* change — a different thing; do not conflate.
- **M3** — `core/association/Murty.cpp:69`: `murtyKBest` returns EMPTY on any
  infeasible seed edge, defeating the per-row degradation contract both
  callers were built against (their `isfinite` re-checks are dead code).
  M3↔M6 are close: M6's new ctor guard (85cb0a4) fail-louds the
  `clutter_intensity=0` trigger, but any run-time all-inf row still empties
  the cluster's children silently.

## Phase 0 — offline probe (checkpoint; BINDING before any behavior code)

Instrument bench-side only (env-gated diagnostic à la `PMBM_*` knobs, or a
standalone tool over logged cost matrices — byte-identical with the knob
unset, prove it). On the full workload set (philos, harbor 5-seed keyed by
seed, env-1/env-2, autoferry, sim_multisensor, Imazu 14/15/17/19/20/22):

1. **M5 blast radius:** per scan, recompute the cost matrix WITH the term and
   report (a) K=1 winner-flip rate, (b) order changes within the used K,
   (c) where flips concentrate (close-pass ambiguity? clutter-dense scans?
   near-shore?).
2. **M3 occurrence:** count infeasible-seed hits per workload at head (is the
   empty-return path ever taken in practice post-M6?).

Hand the probe numbers to the arbiter WITH your read: near-zero flips ⇒
cheap cycle, expect ~byte-identical; material flips ⇒ the stop-and-report
gates below are live and the A/B focuses where the flips are. Build starts
on the GO.

## Phase 1 — build (TDD, one commit per item; NO new consumer-surface knob)

This is a correctness fix, not a feature: no runtime toggle — the A/B is
branch-vs-master. (Probe diagnostics stay bench-side/env-gated.)

1. **M3 first:** restore per-row degradation (skip infeasible seeds, return
   the feasible subset) so the callers' contract holds. TDD with hand-built
   cost matrices including the infeasible-seed repro; prove byte-identical
   on workloads where seeds are all feasible.
2. **M5:** add the missing term. TDD against hand-computed posterior weights:
   small 2-track/2-measurement cases where true posterior order differs from
   today's order; assert enumeration matches the posterior. Teeth per the
   #24 standard (mutate the term → RED → restore).

## Phase 2 — measured A/B (full gauntlet, banded assertions)

- Deployable-config (`imm_cv_ct_pmbm_coverage_land_ivgate`) rows = finding.
  Philos KEEP guard; harbor knife-edge watch (#21, banded only); env-1/env-2
  vs the ADR-0003 rows; Imazu family adds loss-seconds-overlapping-CPA +
  re-acquire-id-count (this touches the same hard-commit machinery ivgate
  mitigates — watch for interaction both ways).
- Decision rule (arbiter rulings of record): a confirmed correctness fix is
  never deferred to protect a buggy-baseline assertion — but STOP-AND-REPORT
  on any deployable-row movement or any direction flip, with the trade
  framed (name each failure mode and when it hurts). Recalibration of
  affected pinned tests is the arbiter's call, not yours.
- Adversarial review before handoff (association hot path — mandatory).

## Acceptance

1. Phase-0 probe numbers + arbiter GO on record; fixes TDD'd with teeth;
   A/B tables with raw numbers (arrows decoration only).
2. Full suite green under strict mode, 0 skips; adversarial review done.
3. Docs: four-part update where the Murty/assignment cost is specified
   (`docs/algorithms/pmbm-design.md` and/or the association doc); learning
   chapter gets a short plain-English note on why the detection-pricing term
   belongs in the cost; backlog #34 M5/M3 marked with outcome + sha.
4. Write-up `docs/baselines/2026-07-20_murty_association_correctness.md` +
   eval-log entry; commit on your branch; do not merge or push master.
