# Implementer prompt — backlog #25 Phase 1: localize WHY PMBM kills the track at close passes (measurement only)

Status: ready to hand off. Paste everything below the line. Origin: backlog
#25 (`docs/algorithms/improvement-backlog.md`), spun out of the #11 Imazu
forensics (`docs/baselines/2026-07-08_imazu22.md` §Q2b): PMBM
(`imm_cv_ct_pmbm_coverage_land`) loses tracks for 62–158 s at sustained close
passes, overlapping the own-ship CPA on imazu_15/22, re-acquiring under new
ids. North-star tag: Cl-3 — this is the deployment-choice discriminator and
ADR 0002's forbidden failure in temporal form. This ticket is PHASE 1 ONLY:
find the mechanism that kills the track. No fix, no config change, no lever —
the Phase-2 design happens at the arbiter after your report.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
parallel-work convention + fixture trap). Worktree:
`git worktree add ../navtracker-b25 -b backlog25-localization`, own build dir.
Imazu fixtures live in the MAIN tree (SIMMS_DIR or symlink — state
ran-vs-skipped). The Cl-1 re-score implementer is active in parallel on
autoferry data — disjoint, but the never-build-in-the-shared-main-tree rule
matters doubly right now. Budget ~1 day.

## The question

For the dying track in imazu_15 and imazu_22 (and one control: the same
target in a case where it survives), produce a scan-by-scan trace through the
close pass and answer ONE question: **what kills it?** The candidate
mechanisms, in prior order:

- **H1 — miss-starvation (the arbiter's working hypothesis):** the neighbor
  track claims the shared/contested measurements scan after scan; the losing
  track takes repeated miss updates; its existence r decays smoothly below
  the deletion/prune threshold. Signature: gradual r decay over many scans,
  while in-gate measurements EXIST but carry higher association mass to the
  other track.
- **H2 — abrupt structural death:** hypothesis-cap pruning, Murty K limit,
  or recycling drops the Bernoulli in one step. Signature: r healthy on scan
  k, track gone on scan k+1, no gradual decay.
- **H3 — estimator divergence first:** the state walks off during the
  ambiguity (coalescence-style), THEN misses follow because the gate is in
  the wrong place. Signature: innovation/NIS blow-up precedes the r decay;
  in-gate measurements stop existing at all.

H1 vs H2 vs H3 have entirely different Phase-2 levers — that's why
localization comes first. A mixture is a valid answer if the trace shows it.

## What to instrument

Per scan, for the dying Bernoulli and its surviving neighbor: r (existence),
which measurements fell in gate, the association mass split between the two
tracks for each contested measurement, whether the update was
detected-vs-missed, NIS of the applied update, and the
prune/recycle/cap events touching either track. Follow the existing pattern:
prefer consuming `--export-states-dir` output + a Python reproducer in
`tools/` (like the three #11 tools). If the needed quantities are not in the
export, a minimal ADDITIVE debug/export hook is acceptable (diagnostic
output only — byte-identical tracking behavior, default-off, per-instance
per the no-global-toggles rule) — say exactly what you added and show a
before/after byte-identical check on one non-imazu config.

## Constraint to respect in the write-up

The suspected H1 mechanism (miss penalty) is the SAME brake that suppresses
philos phantom over-count (pmbm-design cardinality work, 2026-06-24 line).
Your report must state, for whichever mechanism you find, what a fix would
have to be CONDITIONED on so it cannot weaken that brake globally — but do
NOT build anything. One paragraph, for the Phase-2 design.

## Acceptance

1. Scan-by-scan trace tables/plots for imazu_15 + imazu_22 + one survivor
   control, committed doc `docs/baselines/2026-07-08_b25_localization.md` +
   dated eval-log entry; reproducer in `tools/`; checksums + exact commands.
2. A verdict: H1 / H2 / H3 / mixture, with the signature evidence named,
   and the one-paragraph Phase-2 conditioning note.
3. Zero behavior changes. If you added an export hook: additive,
   default-off, byte-identical proof included.
4. Full suite green in YOUR worktree (imazu tests RAN; ran-vs-skipped
   stated; strict `grep '^100% tests passed'`).
5. Stop-and-report: the needed quantities can't be exposed without touching
   tracking behavior; or the trace contradicts the Q2b finding itself
   (loss doesn't reproduce — that's a determinism alarm, report immediately).
