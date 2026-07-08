# Implementer prompt — STOP-THE-LINE: master is red on 3 sunset 6c tests; upgrade the knife-edge assertions (the c0ac493 pattern)

Status: ready to hand off, HIGHEST priority — master's full suite is red and
every in-flight merge inherits it. Paste everything below the line. Origin:
found 2026-07-08 by the Cl-1 implementer, confirmed by the arbiter in an
independent isolated worktree incl. a serial 3-test run (deterministic — the
f11d6e7 "concurrent-mutation artifact" attribution was wrong for these; see
the eval-log correction entry 2026-07-08 and
`docs/algorithms/improvement-backlog.md` #24).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
worktree `git worktree add ../navtracker-6cfix -b sunset6c-assertions`, own
build dir, fixtures symlinked/env-pointed from the MAIN tree; these three
tests MUST RUN, not skip, before and after your change). Budget ~half day.

## What is broken (and what is NOT)

Red on clean master since f11d6e7 (LOS shadow-guard re-land):

- `PhilosCoverageDecay6c.SunsetCoverageAwareHoldsStructureAndProtectsUnsweptCells`
  — fails at `tests/replay/test_philos_occupancy_coverage_6c.cpp:274`:
  `Expected (cov_astern) > (uni_astern), actual: 0 vs 13`.
- `PhilosCoverageDecay6c.SunsetCameraObservedEmptyFlagsVacatedCells`
- `PhilosCoverageDecay6c.SunsetCameraEvictionRemovesDepartedPinsHoldsChartStructure`

The GUARD is not the defect: the shadow guard is ON by design in
`occupancy_detector_coverage`, merged deliberately, net-beneficial
(hazard-detection 51→100% on the probe, HAXR net-beneficial). The defect is
the ASSERTIONS: they pin a cross-config A/B outcome (`coverage-aware >
universal` on specific marginal regions) that backlog #24 rules an INVALID
invariant — `astern_blob`'s emission class flips with the guard margin
because the clutter-adaptive bar (median × factor) is non-monotone in
persistence (#24 case 2, measured table in the eval-log 2026-07-06/07).
c0ac493 already fixed the loiterer assertion for exactly this reason
(config-independent streak + empirical flip-guard) — these three rows were
missed.

## The task

1. For each of the three tests, identify what REAL invariant it protects
   (structure persists; vacated cells get flagged; departed pins evicted
   while chart structure holds) and re-express the assertion per the #24
   rules and the c0ac493 precedent: config-independent formulations,
   isolated fixed-input invariants, or robustly-banded aggregates — never
   an exact cross-config comparison on a marginal region that the adaptive
   bar can flip. If a test's property genuinely cannot be stated robustly,
   say so and propose splitting it (fixed-input unit invariant + banded
   replay aggregate) rather than weakening it to vacuous.
2. NO changes to the guard, LiveOccupancyModel behavior, or any config.
   Tests only. If you find yourself needing a behavior change to make an
   assertion true, STOP and report — that would mean the breakage is not
   assertion-fragility after all.
3. Full suite in your worktree: the three tests RAN and green; total suite
   green (strict `grep '^100% tests passed'`); AND include your skip list
   in the handoff diffed against the expected env-gated set (the new
   verification rule — skips must be named, not counted).
4. Dated eval-log entry: per-test, the old assertion, the new one, and why
   the new one is a valid invariant under #24's two shapes.

## Acceptance

1. Master-red resolved by assertion upgrade only; zero behavior/config diff.
2. Three tests green in a worktree WITH fixtures wired, plus one serial
   3-test-only run shown green (the arbiter will re-run exactly that).
3. Eval-log entry + skip-list statement in the handoff.
4. Stop-and-report: a test's failure turns out NOT to be the #24 flip
   (e.g. camera eviction genuinely broken by the guard) — that is a
   behavior finding, not an assertion fix; report immediately.
