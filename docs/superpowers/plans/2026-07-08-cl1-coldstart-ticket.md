# Implementer prompt — Cl-1 cold-start: re-score the Helgesen head-to-head with the current stack, then diagnose or declare

Status: ready to hand off. Paste everything below the line. Origin: the last
open ❌ on the Cl-1 headline-claim card ("beat in cold deployment without
anchor", env-1) — item 12 of the pre-water selection doc
(`docs/superpowers/plans/2026-07-05-pre-water-window.md`). North-star tag:
**Cl-1** directly. The key fact motivating the ticket: the entire Cl-1
comparison table (`docs/baselines/helgesen2022_reference.md`, run
`gospa20m_20260613T174620Z.csv`) is frozen at **2026-06-13 on
`imm_cv_ct_mht`** — before the UKF promotion (2026-06-20: 9/9
autoferry-unanchored GOSPA wins, mean −12.3%), before PMBM Phases 4–7
(Phase 7 alone: autoferry unanchored −5..−32%), and before later harness
fixes. Nobody has re-scored the head-to-head since. The gap may already be
much smaller than ×2.1 — measure first, diagnose second, tune never.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first — the
parallel-work convention and fixture-trap note apply). Worktree:
`git worktree add ../navtracker-cl1 -b cl1-coldstart-rescore`, own build dir.
The autoferry fixtures live only in the MAIN tree — symlink them in and state
in every handoff which fixture-gated tests RAN vs skipped. Never build or
test in the shared main checkout.

This is a MEASUREMENT-then-DIAGNOSIS pass. No config is changed to make a
number pretty; the deliverable is numbers plus a mechanism, not a fix.
Budget: Phase 0+1 ≈ 1 day; checkpoint with the arbiter before Phase 2
(which is only entered if the gap survives).

## Phase 0 — reproduce the frozen baseline row (gate for everything else)

Reproduce the 2026-06-13 protocol per `docs/baselines/helgesen2022_reference.md`:
GOSPA c=20 m, p=α=2, reported as RMS, env-1 = sc2–6 open water,
env-2 = sc13/16/17/22 urban channel, config `imm_cv_ct_mht`, no-AIS
condition, single seed as documented. Expected: env-1 GOSPA RMS ≈ 43.4,
env-2 ≈ 33.9.

If it does NOT reproduce, that is a finding, not a blocker to hide: identify
which merged harness/config change moved it (candidates: truth-sort fix, UKF
default flip, metrics changes), quote the commit, and report BEFORE
continuing — the arbiter decides which number is the new baseline. Do not
silently rebase the table.

## Phase 1 — the current stack, same protocol (the decision-relevant table)

Same protocol, four conditions × two configs:

- Configs: today's `imm_cv_ct_mht` (UKF is now the default inner filter)
  and the PMBM canonical `imm_cv_ct_pmbm_land` (verify in
  `core/benchmark/Config.cpp` that this is still the KEEP config and that
  it wires on autoferry scenarios; if a different label is canonical now,
  say so and use it — do not guess silently).
- Conditions: no-AIS (the cold-start claim) and truth-AIS injected
  (the apples-to-apples calibration row), × env-1 and env-2.

Deliverable: the Cl-1 table extended with the new rows, per-scenario
breakdown for env-1 (sc2–6 individually — sc3/sc5/sc6 are the historically
pathological ones), plus breaks/lifetime/pos_rmse alongside GOSPA RMS.
Checksums + exact commands in a dated eval-log entry.

**CHECKPOINT: report Phase 0+1 numbers to the arbiter before Phase 2.**
Three possible verdicts, and the arbiter picks:
(a) gap closed or nearly closed (env-1 no-AIS ≲ 20.4-ish) → claim card
    flips ✅, no Phase 2;
(b) gap narrowed but open → Phase 2 diagnosis;
(c) gap unchanged → Phase 2 diagnosis with the mechanism question sharpened
    (why did the unanchored improvements not transfer?).

## Phase 2 — diagnosis (only on arbiter go)

Two threads, both measurement:

1. **Per-scenario mechanism on real data.** Re-run the eval-log 2026-06-16
   per-target diagnosis method on the worst env-1 scenarios under the
   CURRENT stack. The historical mechanism was re-scoped 2026-06-19: filter
   over-confidence on re-confirmed tracks after brief misses (sc3 unanchored
   median NEES 20 vs expected ~1.4), NOT BOT range collapse. The standing
   hypothesis (comparison-baselines Cl-2 #2) is that PMBM makes this moot
   because it collapses existence + association into one recursion — Phase 2
   tests exactly that: report NEES median/p95/cov95 per env-1 scenario for
   PMBM vs MHT.
2. **Observability control in sim.** Build ONE controlled scenario with the
   multi-sensor sim generator (`tests/fixtures/sim_multisensor/generator/`,
   explicit-placement path from the Imazu work): bearing-heavy open water,
   no anchor, a target geometry mimicking sc5/sc6. Independent truth by
   construction. Purpose: separate "the filter is over-confident" from "the
   geometry is unobservable without range" — if even a perfectly-modelled
   sim target can't be ranged from this geometry, the gap is structural,
   not a tuning target.

## Phase 3 — verdict framing (report, don't fix)

One of three write-ups for the arbiter:
- **Closed**: claim card flips ✅; north-star edit proposed in the handoff
  (arbiter lands north-star edits).
- **Open with named mechanism**: what it is, where it lives, what a fix
  would touch, what it would risk (knife-edge lessons apply — no proposal
  that pins association outcomes).
- **Structural observability limit**: the honest deployment statement —
  cold-start open-water bearing-heavy tracking needs a ranging/Doppler
  sensor or a cooperative anchor — which becomes an input to the
  deployment-facts thread, not further tuning. This outcome is NOT a
  failure; it is the claim card answered honestly.

## Acceptance

1. Phase 0 reproduction stated (reproduced, or drift root-caused with
   commit named).
2. Phase 1 table committed to `docs/baselines/helgesen2022_reference.md`
   (new dated section; the 2026-06-13 table stays untouched as the frozen
   record) + dated eval-log entry with checksums and exact commands.
3. Checkpoint honored — no Phase 2 without arbiter go.
4. No config or algorithm change anywhere in this ticket. Zero. Findings
   only.
5. Full suite green in YOUR worktree with fixtures symlinked;
   ran-vs-skipped stated in the handoff; strict pass check
   (`grep '^100% tests passed'`).
6. Stop-and-report: Phase 0 irreproducible with no identifiable cause;
   autoferry fixtures missing/corrupt; or truth-AIS injection path no
   longer runs.
