# Implementer prompt — backlog #24: the knife-edge assertion sweep (upgrade fragile test assertions repo-wide)

Status: ready to hand off. Paste everything below the line. Origin: backlog
#24. The repo has a measured history of test assertions that are *correct
about the wrong thing*: they pin an exact value or compare two adaptive
quantities with a strict inequality, so they flip on epsilon-sized,
meaning-free changes. This has bitten four ways: (1) the sunset-6c incident
— 3 `PhilosCoverageDecay6c` tests genuinely red on master for a day because
an adaptive-bar `cov>uni` comparison flipped 0 vs 13; (2) the same
assertions later proved TOOLCHAIN-fragile (same commit green on g++13, red
elsewhere — floating-point differences move the bar); (3) the R3 perf work
flipped a harbor assertion on 1 non-KEEP config (the "knife-edge" name);
(4) the pre-release review's test-sufficiency audit (W3) confirmed several
vacuous or swap-blind assertions (e.g. continuity gated via a degenerate
metric, `EXPECT_GT(n, 0)` guards that cannot fail meaningfully). Fixed
worked examples of the VALID shapes already exist: commits b6e865a and
c0ac493 (banded floors, config-independent streak assertions, one-sided
inequalities). This ticket is the systematic sweep the incidents keep
asking for. Budget ~1–2 days. North-star tag: suite health (protects every
claim's evidence).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-b24 -b backlog24-assertion-sweep`, own
build dir; ALL fixtures from the MAIN tree — symlink at the INNER level per
the CLAUDE.md second-order fixture trap; skips named BY NAME — the proof of
a fully-wired run is 0 skips). **Tests-only change: zero production code,
zero config, zero fixture changes.**

## Step 1 — inventory and classify

Sweep every assertion in `tests/replay/`, `tests/benchmark/`,
`tests/scenario/` (and any other suite asserting on metric outcomes, incl.
`tests/t2t/` scenario gates). Classify each metric-bearing assertion:

- **(a) Knife-edge / epsilon-fragile:** exact pins on floating-point
  metrics; strict comparisons between two ADAPTIVE quantities (both move
  under FP/toolchain/config drift — the sunset-6c shape); counts that
  depend on seeds or association tie-breaks; thresholds sitting within
  noise of the measured value (check the margin: measured vs asserted).
- **(b) Vacuous / swap-blind:** cannot fail under any plausible defect
  (`EXPECT_GT(x, 0)` where x is structurally positive; asserting a metric
  exists rather than its value; symmetric assertions that pass if two
  quantities are swapped). Cross-check the W3 findings
  (`docs/reviews/2026-07-09-prerelease-review/20-test-sufficiency-findings.md`,
  assertion-quality items) — confirm or refute each in your inventory.
- **(c) Sound:** leave alone. Do not churn healthy assertions.

Deliverable: the inventory table (file:line, current shape, class, margin).

## Step 2 — upgrade

For each (a) and (b), upgrade to a valid shape, following the worked
examples (b6e865a, c0ac493):

- **Banded floors:** assert the metric stays within a band with real
  margin to measured values (state the margin; ≥20% of the measured
  headroom unless justified), not equal to a value.
- **Config-independent streaks / structural invariants:** assert the
  PROPERTY (e.g. "track survives the pass with one id", "40 complete truth
  groups", "A beats B") rather than the count that encodes it fragilely —
  and where "A beats B" compares adaptive quantities, require a margin,
  never a bare `>`.
- **One-sided inequalities** against fixed physical bounds, not against
  another measured quantity.
- FP-robust by construction: nothing may depend on the last digits of a
  float or on tie-break order (the toolchain lesson — you cannot test
  other compilers locally, so the SHAPE must carry the robustness).

## Step 3 — prove teeth (the anti-vacuousness gate)

An upgraded assertion that can no longer fail is worse than the fragile
one. For EVERY upgraded assertion, demonstrate one mutation that trips it
(a deliberate local perturbation — flip a sign, zero a covariance, drop a
sensor feed — run the test, watch it FAIL, revert). Record mutation →
observed failure in the write-up table. No exceptions: an upgrade without
a teeth proof does not land.

## Acceptance

1. Inventory table + old→new table with per-assertion rationale and teeth
   proof, as `docs/baselines/2026-07-11_b24_assertion_sweep.md`.
2. Tests-only diff. Full suite green in your worktree at **0 skips**
   (fixtures wired inner-level); any test whose upgrade changes what it
   guards gets a one-line comment in the test stating the invariant it now
   pins.
3. W3 assertion-quality findings each marked confirmed-fixed / refuted
   (with reason) in the write-up — the review synthesis will consume this.
4. Stop-and-report: an assertion that cannot be made both robust and
   toothy without changing production behavior (that's a design gap, not a
   test fix — name it for the backlog).
5. Commit on your branch; do not merge or push master.
