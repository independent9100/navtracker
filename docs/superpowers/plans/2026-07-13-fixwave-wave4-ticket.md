# Implementer prompt — fix wave, wave 4: RangeBearing2D initiation (CONVERT), UKF bearing wrap, CPA tcpa sign — plus Stage-0 Cl-4 row re-measure (the W2.4 reconciliation input)

Status: ready to hand off AFTER the arbiter confirms the wave-2 merge
(your W2.4 change is what Stage 0 measures, and wave 4's files are disjoint
from waves 2/3 — estimators + CPA vs adapters/geo vs bias chain — so all
three waves stay parallel-safe). Paste everything below the line. Origin:
the pre-release review synthesis (`docs/reviews/2026-07-09-prerelease-open-points.md`
§B Themes 2 + 4b, §G item 6). Budget ~2 days. TDD throughout; teeth proofs
per the #24 standard. Read each finding's verifier evidence in
`docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md` first.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-fw4 -b fixwave-wave4` off master AFTER the
wave-2 merge, own build dir, fixtures inner-level symlinked; run the suite
with `NAVTRACKER_REQUIRE_FIXTURES=1` — the wave-2 strict mode is now the
ceremony standard, 0 skips enforced by the harness. Commit on your branch;
never merge or push master. Keep `fixwave-wave1` (held F2) untouched.)

## Stage 0 — re-measure the frozen Cl-4 headline rows post-W2.4 (half day, FIRST)

Your wave-2 A/B showed the autoferry family moves under the corrected
sensor-activity coverage, but it scored the bench scenarios — not the two
frozen ADR-0003 headline rows. Re-run, on the post-merge master, the Cl-4
gauntlet's env-1 and env-2 rows (candidate `imm_cv_ct_pmbm_coverage_land_ivgate`,
same commands as the adoption freeze — `docs/baselines/` eval-log entry has
the reproduce lines) plus the harbor 5-seed row (expected unchanged;
confirm). Deliverable: a three-row before/after table (frozen value →
post-W2.4 value) with the env-2 8/8 revival status re-confirmed. Hand it to
the arbiter AS SOON AS IT EXISTS (do not wait for the rest of the wave) —
the arbiter writes the dated ADR-0003 reconciliation addendum from it.
STOP-AND-REPORT if env-2 revival drops below 8/8 or harbor moves: that
changes the Cl-4 claim, not just its numbers.

## W4.1 — RangeBearing2D initiation plants polar values as ENU Cartesian (HIGH; all four estimators)

`EkfEstimator.cpp:82`, `UkfEstimator.cpp:123`, `ImmEstimator.cpp:449`,
`ParticleFilterEstimator.cpp:127`: a RangeBearing2D measurement births a
track at literal (range_m, bearing_rad) with mixed m²/rad² covariance —
nothing gates to it, phantom proliferation per scan.

**Arbiter decision: CONVERT, not forbid.** The integration guide documents
range/bearing as birth-capable, and ADR-0002 requires a radar-only
detection to be able to initiate — forbidding would silently shrink the
consumer contract. Implement polar→ENU conversion in each `initiate()`
(position from `sensor_position_enu` + range/bearing; covariance via the
standard polar→Cartesian Jacobian at the measured point; state the bearing
convention explicitly per the W3/W4 angle rider — zero reference and turn
direction in the doc comment). One shared helper, four call sites — do NOT
implement it four times. Per-estimator TDD: a RangeBearing2D initiation
lands at the true ENU position (banded) with a sane ENU covariance, and the
next scan's measurement GATES to it (the anti-proliferation teeth — the
pre-fix code fails this). Also close the paired TODO at
`AutoferryJsonReplay.hpp:52` (synthesis §G.6 pairs it with this fix).

## W4.2 — UKF predicted bearing is a linear mean across the ±π wrap (MED)

`UkfEstimator.cpp:80`: sigma-point bearings averaged linearly corrupt the
update for targets ~due west of the sensor. Fix with a circular mean
(vector-sum atan2). Test: target due west, sigma-point spread straddling
±π → predicted bearing ≈ π (not ≈ 0); update converges instead of
diverging. Teeth: pre-fix code fails it.

## W4.3 — CPA tcpa-Jacobian chain-term sign (HIGH, operator-facing) + the spec's same error

`Cpa.cpp:135`: a sign error in the tcpa-Jacobian chain term makes published
`sigma_tcpa` ~3× wrong for converging pairs and corrupts the head-on
fallback `sigma_cpa`/probability. The CPA-uncertainty design spec §4.3
carries the SAME error — fix code AND spec in the same commit (the spec is
the four-part doc; its math section must show the corrected derivation).
The review noted existing tests pass because the error CANCELS in the
direction they exercise — your new tests must exercise the non-cancelling
direction: a converging-pair geometry where the analytic sigma_tcpa is
known, asserted banded; teeth: the pre-fix code is ~3× off and fails.
Check the full-stack integration test's CPA assertions still hold (they
may have been calibrated against the wrong sigma — if so, correct them
with a comment, same as the W2.3 test-pins-the-bug precedent).

## A/B duty (all of W4.1–W4.3)

Full bench A/B vs post-merge master: W4.1 is expected bench-inert IF no
bench path feeds raw RangeBearing2D (verify and STATE which paths do);
W4.3 changes published CPA uncertainty everywhere — report the collision/
CPA test-surface deltas explicitly. Any move on the Cl-4 candidate rows =
FINDING for the arbiter (same rule as wave 2).

## Docs (same PR)

CPA spec §4.3 corrected (W4.3); integration guide: RangeBearing2D
initiation behavior note (it now genuinely births — consumers should know
the conversion happens inside the estimator) + bearing-convention wording
per the angle rider; learning docs only if the CPA chapter shows the
sigma_tcpa formula (check; fix if so, regenerate figures via generate.py).

## Acceptance

1. Stage-0 table delivered to the arbiter early and separately.
2. TDD paper trail + teeth proofs per finding; the shared polar→ENU helper
   (one implementation); the non-cancelling-direction CPA test.
3. A/B table with the RangeBearing2D bench-path statement and CPA deltas.
4. Full suite green under `NAVTRACKER_REQUIRE_FIXTURES=1` (0 skips
   enforced); adversarial review before handoff (estimator init + CPA math
   are hot-path/operator-facing).
5. Write-up `docs/baselines/2026-07-13_fixwave_wave4.md` + eval-log entry.
6. Commit on your branch; do not merge or push master.
