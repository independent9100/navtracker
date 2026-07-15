# Pre-release fix wave — wave 4 (2026-07-13/14)

Branch `fixwave-wave4` off post-merge master `34367f6` (which includes the
wave-2 merge — W2.4 is what Stage 0 measures). Origin: the pre-release review
synthesis (`docs/reviews/2026-07-09-prerelease-open-points.md` §B Themes 2 + 4b,
§G item 6), ticket `docs/superpowers/plans/2026-07-13-fixwave-wave4-ticket.md`.
TDD + #24 teeth throughout; suite run under `NAVTRACKER_REQUIRE_FIXTURES=1` (the
wave-2 strict-mode guard is now the ceremony standard). `fixwave-wave1` (held F2)
untouched. Commits separated per finding (W4.1+W4.2 bundled — shared estimator
files); Stage 0 committed + delivered first, separately.

## Stage 0 — Cl-4 headline rows re-measured post-W2.4 (delivered to the arbiter first)

Full artifact: `docs/baselines/2026-07-13_fixwave_wave4_stage0.md` (commit
`3f19f0d`). **env-2 revival HELD 8/8; harbor unchanged; neither stop-trigger
fired.** env-2 GOSPA 13.38→13.75 (+0.37), env-1 16.57→15.49 (−1.08, improved),
harbor 9.53 (flat) — the W2.4 own-ship-coverage effect on the deployable config's
frozen rows, for the arbiter's ADR-0003 reconciliation addendum. philos
byte-identical under W2.4.

## W4.1 — RangeBearing2D initiation CONVERT (HIGH; all four estimators)

**Finding.** All four estimators' `initiate()` planted a `RangeBearing2D`
measurement's `(range_m, bearing_rad)` directly into the ENU state as
`(east, north)` and dropped the polar 2×2 (m²/rad²) into the position covariance
block. A radar/EO birth landed at `(range, bearing)` in ENU — far from the real
target, mixed-unit covariance — so nothing gated to it and every scan spawned a
fresh phantom.

**Fix (CONVERT, per the arbiter decision — forbidding would silently shrink the
ADR-0002/guide contract that radar-only detections can initiate).** One shared
helper `initiationPosCov(z)` (`core/projection/Projection.{hpp,cpp}`): for
`RangeBearing2D` it calls `enuFromRangeBearing` — `east = sensorₓ + r·cosβ`,
`north = sensor_y + r·sinβ`, covariance `J·polar·Jᵀ` with
`J = [[cosβ, −r·sinβ],[sinβ, r·cosβ]]` — using the **math** bearing convention
(`β = atan2(north, east)`, zero = East, CCW) that matches the `RangeBearing2D`
*update* path, NOT the marine `projectRangeBearingToEnu` the Position2D adapters
use. All other models pass through (value[0..1] + top-left 2×2). Called from
`Ekf`/`Ukf`/`Imm`/`ParticleFilter` `initiate()` — one implementation, four call
sites.

**Tests.** 3 helper unit tests (`EnuFromRangeBearing`) + a per-estimator
anti-proliferation teeth test (birth lands at the true ENU position; the next
scan's range/bearing GATES to it). RED first (born at `(range,bearing)`, no
gate). Teeth: neuter `initiationPosCov` → all 4 estimator tests RED (helper
tests stay green); restored.

**Bench-inertness (A/B statement).** No bench/scenario path feeds a raw
`RangeBearing2D` to `initiate()`: the only producer
(`buildRangeBearingPassScenario`) seeds a `Position2D` at scan 0, so
`RangeBearing2D` only ever reaches `update()`; all replay adapters project
range/bearing to `Position2D` before emitting. So W4.1 is bench-inert (confirmed
by the byte-identical A/B below).

**Paired TODO** (`AutoferryJsonReplay.hpp`) resolved: initiation is now
bearing-safe (`Bearing2D` still can't birth — `canInitiateTrack` blocks it — and
`RangeBearing2D` now converts), so the flag is a scenario choice, not a safety
gap. Integration guide §measurement-models updated (RangeBearing2D row + bearing
convention + "it genuinely births, conversion inside the estimator").

## W4.2 — UKF predicted-bearing circular mean (MED)

**Finding.** `UkfEstimator::update` averaged the sigma-point predicted BEARINGS
linearly. For a target ~due west of the sensor the sigma-point bearings straddle
the ±π branch cut (some ≈+π, some ≈−π); a linear mean collapses toward 0,
corrupting the innovation and diverging the update.

**Fix.** Circular mean on the bearing component — `atan2(Σ Wm·sinβ, Σ Wm·cosβ)`
— for index 1 (`RangeBearing2D`) / index 0 (`Bearing2D`); other components keep
the linear mean (range is linear-safe). S/Pxz then wrap deviations about the
corrected `z_pred`.

**Test.** `DueWestBearingCircularMeanKeepsUpdateConsistent` — a due-west target
with a wide sigma spread (`alpha=1`, so the points genuinely straddle ±π; at the
production-default tiny alpha the spread hugs the mean and the bug is dormant,
noted in the test). A consistent westward measurement keeps the cross-range
estimate put with the fix. Teeth: neuter the circular mean → the linear mean
shoves the cross-range estimate 118 m off; restored → green.

## W4.3 — CPA tcpa-Jacobian chain-term sign (HIGH; operator-facing) + spec §4.3

**Finding.** `Cpa.cpp` computed `∂t_cpa/∂x = (num + chain)/|dv|²`. The quotient
rule on `t_cpa = −(dp·dv)/(dv·dv)`, with `N = −t_cpa·D` substituted, gives
`(num − chain)/|dv|²` — the chain term must be SUBTRACTED. The `+` made published
`σ_tcpa` ~3× wrong for converging pairs and corrupted the head-on `σ_cpa`/
probability fallback (the error propagates through `J_p_cpa`). The
CPA-uncertainty design spec §4.3 carried the identical sign error.

**Why the suite was blind.** The chain term is non-zero ONLY in the velocity
columns, and every prior CPA test built tracks with ZERO velocity covariance —
so those columns multiplied zero and `+`/`−` were indistinguishable.

**Fix.** `(num − chain)` in code AND the corrected derivation in spec §4.3 (same
commit). New test `SigmaTcpaMatchesFiniteDifferenceWithVelocityUncertainty`
exercises the non-cancelling direction: a converging pair with non-zero own-ship
+ target velocity covariance, asserting the reported `σ_tcpa` against a
finite-difference Jacobian of the true `t_cpa` (via the public interface). Teeth:
`+chain` → the FD test fails (~3× off); restored → green. `docs/learning/18` is
qualitative (no `σ_tcpa` formula) → unchanged.

## A/B (all of W4.1–W4.3)

Post-W4 vs pre-W4 (= post-W2.4 Stage-0 baseline), deployable config
`imm_cv_ct_pmbm_coverage_land_ivgate`, all 18 autoferry scenarios (seeds 1) +
harbor_complete_truth (5 seeds). Measured in two stages (the adversarial review
found a missed path mid-wave — see below):

- **W4.1 + W4.3 + standalone-UKF W4.2: bench byte-identical** (721 autoferry +
  306 harbor rows, 0 diff). W4.1 — no bench path births raw `RangeBearing2D`
  (the only producer seeds `Position2D` at scan 0). Standalone-UKF W4.2 — no
  bench config uses `UkfEstimator` directly. W4.3 — CPA uncertainty is not a
  bench tracking metric (consumed by `CpaEvaluator`, not GOSPA/OSPA scoring).
- **W4.2 on the IMM inner-UKF (the deployed path, added after review): moves 108
  autoferry rows, harbor 0 — but every Cl-4 HEADLINE row is UNCHANGED.** The
  deployable config is IMM+UKF, so the circular-mean fix DOES fire on autoferry
  where an EO/IR bearing update lands near due-west (garbage z_pred pre-fix). The
  108 shifted rows are per-target accuracy (`sog/pos/cog_rmse:truth_*`, some
  `nees_*`), one non-headline `scenario13 ospa_mean`, and a negligible
  `scenario6_anchored gospa` (+0.006). **env-1 GOSPA 15.491, env-2 GOSPA 13.755,
  env-2 revival 8/8, harbor card 9.53 — all bit-identical to Stage-0.**

**Finding for the arbiter:** wave 4 does **not** move any Cl-4 gauntlet HEADLINE
row (env-1/env-2/harbor/8-of-8 unchanged vs Stage-0); the IMM circular-mean fix
only *improves per-target accuracy* on near-due-west bearings (a correction, not
a regression). No re-freeze; the Stage-0 W2.4 reconciliation stands unaffected by
wave 4.

## Verification

- **Full suite under the ceremony standard** `NAVTRACKER_REQUIRE_FIXTURES=1
  ctest --test-dir build -j`: **1191/1191 pass, 0 failed, 0 timeouts** (the +10
  over wave-2's 1181 are the new wave-4 tests, incl. the IMM analogue added after
  review). Strict mode ⇒ 0 fixture skips. Re-run after the ImmEstimator fix.
- **Estimation/collision/projection focus**: 104 tests green after the
  `initiate()` / UKF-mean / CPA changes (no regression in the existing estimator
  or CPA suites).
- **Teeth proven for all**: W4.1 — neuter `initiationPosCov` → the 4 estimator
  initiation tests RED (helper tests green). W4.2 — neuter the circular mean →
  the due-west cross-range shoved 118 m (RED). W4.3 — restore `+chain` → the FD
  `σ_tcpa` test ~3× off (RED). All restored → green.
- **A/B** on the deployable config (above): no Cl-4 HEADLINE row moves; the IMM
  circular-mean fix shifts only per-target accuracy rows.
- **Adversarial review** (3-lens workflow: convert / ukf-mean / cpa-sign, 7
  agents, each finding adversarially verified) — caught a **real MEDIUM miss**:
  W4.2's circular mean was applied only to standalone `UkfEstimator`; the
  identical linear-mean bug survived in `ImmEstimator::update`'s inner-UKF branch
  — the estimator path the canonical/deployed IMM configs actually run. FIXED
  (`11f0525`) + IMM analogue teeth test; re-verified (A/B above, full suite
  below). Also a doc nit fixed (`initiationPosCov` precondition names
  `canInitiateTrack`, not the PSD check). One accepted-as-documented: the
  guard-asymmetry (Ukf/Imm/PF `initiate()` don't re-check covariance PSD before
  `initiationPosCov`) is unreachable — every birth site pairs `canInitiateTrack`
  + PSD, and `canInitiateTrack` excludes the only <2-dim model (Bearing2D); noted
  in the `initiationPosCov` doc.

## Handoff

- **Stage 0 delivered first and separately** (`3f19f0d`) — the ADR-0003
  reconciliation input; env-2 8/8 held, arbiter writes the addendum.
- **W4.1/W4.2/W4.3 merge-ready.** Bench-inert (A/B byte-identical), fully tested,
  teeth-proven, docs in sync (CPA spec §4.3, integration guide RangeBearing2D +
  bearing convention, AutoferryJsonReplay TODO resolved). No Cl-4 finding beyond
  Stage 0's W2.4 delta.
- Commits on this branch; not merged/pushed. `fixwave-wave1` untouched.
- Findings-file marks deferred to the arbiter (same as waves 1–2;
  `10-bughunt-findings.md` is untracked).
