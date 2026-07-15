# Fix-wave wave 3 — the heading/sensor-bias chain (one repair project)

Branch: `fixwave-wave3` (from `master` @ `f9ad004`). Ticket:
`docs/superpowers/plans/2026-07-13-fixwave-wave3-ticket.md`. Origin:
`docs/reviews/2026-07-09-prerelease-open-points.md` §B Theme 3.

Four independently-confirmed correctness bugs in the heading/bias
estimation chain, fixed and tested together, plus the input
angle-convention audit (user rider 2026-07-12). TDD throughout; teeth
proofs per finding.

## What was wrong and what changed

### W3.4 — sign convention (load-bearing)
`HeadingBiasEstimator` fuses five observation kinds into ONE scalar bias
`b`, consumed by the ARPA/EO-IR adapter correction
`corrected_compass_bearing = measured − b`. The three gyro-vs-reference
kinds (v3) and that correction live in the **marine compass** frame
(`b = gyro − reference`, `gyro = true + b`, 0 = N, CW+). The AIS↔ARPA
pair (v1) and the bearing-innovation (v2) kinds compute in the **ENU-math**
frame (`atan2(dN,dE)`, 0 = E, CCW+), where a `+b` compass gyro bias appears
as `−b`. Pre-fix, v1/v2 fed `−b` into a `+b` state — they fought v3 and, if
they dominated, drove the estimate to `−b`, so the adapter *doubled* the
error instead of removing it. Empirically confirmed: a `+2°` bias drove v1
and v2 to `−2°` while v3 correctly reached `+2°`.

Chosen canonical convention: **compass `b`** (fewest changes; matches the
load-bearing adapter correction and v3). v1 and v2 are converted at the
estimator's `observe()` boundary (v1: `z = wrap(β_ais − β_arpa)`; v2:
`meas = −innovation_rad`). `BearingInnovation.innovation_rad` keeps its
literal meaning (raw ENU-math innovation) — the frame knowledge lives in
the estimator.

### W3.1 / W3.2 — closed-loop double-subtraction (heading & per-sensor)
In the documented wiring the adapter/pipeline subtracts the published `b̂`
from a measurement *before* the `SourceTouch` is recorded, so the pair the
extractor forms shows only the residual `(b_true − b_pub)`; subtracting `b̂`
again gives fixed point `b̂ = b_true/2`. The published bias decays to half,
overconfidently, and every corrected measurement keeps the other half.

Fix (single mechanism): each measurement carries the correction already
applied — `Measurement`/`Track::SourceTouch::applied_heading_bias_rad`
(compass, set by `ArpaAdapter`/`EoIrAdapter`), `applied_position_bias_enu`
and `applied_bearing_bias_rad` (set by `applyBiasCorrection`), copied into
the touch by `fillSourceTouchEnu` (uniform across Tracker/MHT/PMBM). The
pair extractors and the v1 path reconstruct the RAW observation by adding
the applied correction back before forming the innovation. The same
reconstruction on the cross-sensor anchor also removes the twin
"anchor debiased twice" defect (`extractCrossSensorPositionPairs`). Open-loop
callers (`applied_* = 0`) are byte-identical.

### W3.3 — AIS/ARPA pairs about the datum origin, not own-ship
`ArpaAdapter` never set `Measurement::sensor_position_enu`, so the touch
carried `(0,0)` and the estimator measured the angle subtended at the ENU
datum origin — geometry-diluted and wrong whenever own-ship is far from the
datum. Fixed: `ArpaAdapter` populates `sensor_position_enu` with own-ship
ENU (TTM and TLL). The replay adapters already set it; this makes ARPA
consistent with them.

### W3.5 — input angle-convention audit
Doc comments now name the **zero reference and turn direction** of every
angle field the chain consumes:
- `OwnShipPose` — all `*_heading_*` / `*_variation_*` fields are compass
  (deg, 0=N, CW+).
- `HeadingBiasObservations.hpp` (the three v3 kinds) — compass rad; header
  states the one convention + the v1/v2 boundary conversion.
- `Measurement.value` — bearing components are ENU-math rad (0=E, CCW+).
- `IBearingInnovationSink::innovation_rad` — raw ENU-math, negated by the
  estimator.
- `SensorBiasEstimator`'s `BearingBiasPairObservation.alpha_observed_rad` —
  ENU-math per-sensor offset (independent of the compass gyro bias).

**Audit (fields whose unit/frame was not self-evident from name/type):**
`OwnShipPose.heading_true_deg` and the `gps_true_heading_deg` /
`magnetic_heading_deg` / `magnetic_variation_deg` group (were "deg" with
unstated frame → now documented); `Measurement.value` bearing components
(model-dependent, frame unstated → now documented);
`IBearingInnovationSink.innovation_rad` (frame ambiguous → documented).
No one-line, call-site-free renames were available; the higher-value
follow-up (a typed `Bearing`/`CompassAngle` wrapper to make the frame
unmissable at the type level) is flagged for a separate wave — out of scope
here to avoid sprawl.

## TDD / teeth evidence
- `tests/bias/test_heading_bias_sign_convention.cpp` — v1/v2 converge to
  `+b` (pre-fix `−b`); v3 stays `+b`; mixed v1+v3 agree (through the real
  `projectRangeBearingToEnu`, so the frame is measured not assumed).
- `tests/bias/test_heading_bias_closed_loop.cpp` — full `b` with
  reconstruction; `b/2` without (same loop).
- `tests/bias/test_sensor_bias_closed_loop.cpp` — per-sensor position bias:
  full `b` with reconstruction, `b/2` without.
- `tests/bias/test_bias_off_datum.cpp` — ArpaAdapter carries own-ship ENU;
  the estimator recovers the true bias about own-ship, and is diluted to
  `< 0.5·b` under the datum-origin formula.
- Updated to the corrected convention: `test_heading_bias_estimator.cpp`
  (`makePair`), `test_heading_bias_bearing_innovation.cpp`,
  `test_bearing_bias_convergence.cpp`.

**Full suite: 1180/1180 passed, 0 skipped, 0 disabled** (fixtures symlinked
inner-level incl. `SIMMS_DIR`/`RBAD_DIR`; the previously fixture-skipped
`T2tScenarioRun.ConflictBiasedOverconfidentSourceCharacterization` ran).

## A/B on the deployable configs — FINDING for the arbiter

Config `imm_cv_ct_pmbm_coverage_land_ivgate` (the Cl-4 candidate) × all
autoferry scenarios, `fixwave-wave3` vs `master` (identical build/fixtures).

- **The heading-bias chain (W3.1/W3.3/W3.4) is wired in NO deployable
  config** — only in the sim/bus tests. Zero deployable impact by
  construction; validated by the sim bias tests (green) + the new teeth.
- **Deployment-realistic (non-anchored) autoferry: byte-identical** on every
  tracking/accuracy metric (GOSPA mean/false/missed, pos_rmse, id_switches,
  nees, …). Only wall-clock timing differs (noise).
- **Anchored diagnostic mode (`*_anchored`, injects a truth anchor so the
  per-sensor EO/IR bearing bias actually publishes): metrics MOVED, some
  worse** — e.g. `scenario16_anchored` gospa_mean 2.67→8.42, gospa_false
  10.2→130.8; `scenario17_anchored` gospa_mean 12.38→14.13. Mechanism: with
  the fix the per-sensor bearing bias converges to the *full* offset instead
  of half, and the env-2 EO/IR seed (7.0°/4.9° from
  `autoferry_r_calibration.py`, in `ReplayScenarioRun::seedSensorBiasEstimator`)
  was calibrated **against the buggy half-converging loop**, so it is now
  mis-scaled.

Interpretation: the fix is correct (teeth-proven); the anchored-mode move
is a downstream-baseline effect, not a tracking regression on the deployed
path. **Per the ticket: reported as a delta, NOT re-frozen, estimator NOT
tuned to match the old seed.** Recommended arbiter action: re-derive the
env-2 EO/IR bearing seed against the corrected loop, then re-run the Cl-4
gauntlet.

## Unblocks the F2 provenance cycle
`docs/superpowers/plans/2026-07-12-f2-provenance-cycle-ticket.md` must be
measured on a correct bias chain (its garbage-bias × broken-chain
attribution question). That chain is now correct and available.

## 2026-07-14 — Rebase onto master 34367f6 + adversarial-review cycle (arbiter acceptance)

The arbiter accepted the wave pending two items; both done here. Branch head
after this cycle sits on top of `34367f6` (wave-2 merged).

### Rebase onto 34367f6 (wave-2 collision)
Wave-2 (W2.1) made the four sensor adapters `IDatumChangeSink`s that swap the
datum and reproject buffered measurements on an auto-recenter — touching
`ArpaAdapter`, the same file as W3.3. The rebase was clean: all code files
auto-merged (disjoint regions), only `evaluation-log.md` needed a trivial
two-entry resolve. Both behaviours survive and were verified to compose *by
design*: wave-2's `reprojectMeasurementEnu` already reprojects
`sensor_position_enu` (the W3.3 field) on a recenter, and the wave-3 `applied_*`
fields are correctly left un-reprojected — `applied_heading_bias_rad` /
`applied_bearing_bias_rad` are angular *offsets* (frame-invariant under the ENU
rotation), and `applied_position_bias_enu` is only ever written downstream of
the adapter `buffer_` that a recenter reprojects (provably (0,0) there).

### Adversarial review (3 independent fresh-agent lenses: sign/frame,
reconstruction, geometry/KF)
Two lenses returned the runtime math CONFIRMED correct (sign/frame of all five
kinds; reconstruction sign/units/frame; plumbing uniform across
Tracker/MHT/PMBM; open-loop byte-identity). Findings and disposition:

- **B5 — v2 BearingInnovation outlier gate [FIXED].** The gate keyed on the
  absolute measurement `|meas|` (≈ `|b_true|`), so once b̂ converged to a large
  true bias it rejected every state-consistent observation and froze. Now gates
  on the innovation `|y| = |meas − b̂|` (a consistent obs → y≈0 always accepted;
  a genuine outlier still rejected, including a huge first innovation because the
  wide init prior keeps the cold-start threshold generous). Teeth:
  `BiasObsBearingInnovation.GateOnInnovationNotAbsoluteBiasDoesNotFreeze`.
- **TLL origin-unset fallback [FIXED].** An ARPA-TLL fix arriving before the
  first own-ship pose left `sensor_position_enu` at the (0,0) sentinel; the pair
  extractor would then measure the bearing about the datum origin (the W3.3 bug)
  and those cold-start obs are outlier-gate-exempt. `extractPairs` now SKIPS a
  pair whose ARPA touch carries the (0,0) "own-ship unset" sentinel — the same
  convention `DatumReproject` uses. (Cost: also skips the rare legitimate
  own-ship-exactly-at-datum pair, e.g. the instant after a recenter — harmless,
  since the origin bearing is *correct* there anyway.) Teeth:
  `AisArpaPairExtractorTest.SkipsWhenArpaOwnShipOriginUnset`.
- **Anchor add-back guard [FIXED].** `SensorBiasPairExtractor` reconstructed the
  anchor's RAW (biased) position by adding its applied bias back. The anchor is
  the TRUTH reference, so its CORRECTED position is what the estimator wants;
  the add-back would drive the sensor estimate to `b_sensor − b_anchor` if an
  anchor were ever position-calibrated. Now uses `anchor->value_enu` directly
  (byte-identical today — anchors never publish a position bias — and closes the
  latent hole). Teeth:
  `SensorBiasClosedLoop.InvariantToAnchorAppliedPositionBias`.
- **v2 emit-sign "not load-bearing" — VERIFIED FALSE POSITIVE.** The reviewer
  inspected only the `bias/` unit tests. The emit-side sign (`Tracker.cpp`
  `r = wrap(beta_obs − beta_pred)`) IS load-bearing: `TrackerBearingInnovation
  Emit.Bearing2DEmitsCorrectFields` pins it (a flip → −offset → fails), and
  `BearingBiasConvergence.AnchoredBearingStreamReconstructsTrueBias` pins the
  full composite v2 sign end-to-end through the real Tracker into a
  HeadingBiasEstimator (a flip converges to −b and fails). No new test needed.

### env-2 EO/IR seed — re-derivation delivered, adoption PARKED for the arbiter
Root cause is deeper than "calibrated against the half-loop":
`tools/autoferry_r_calibration.py` computes the mean **absolute** bearing
residual (line ~178 `copysign(1.0,1.0)` ⇒ always +1, on an already-`abs()`ed
value) and associates each detection to the **min-|residual|** truth — so the
`7.0°/4.9°` are the mean magnitude of the ~7–9° noise (≈0.8·σ), not a signed
bias. A corrected signed re-derivation (scratchpad, min-|residual| association
retained so magnitudes are conservative lower bounds) over sc13/16/17/22:
**EO mean +1.66° (median +0.97°), IR mean −1.92° (median −0.41°)**, both dwarfed
by σ≈8.9°/6.9° with frac>0 ≈ 0.58/0.44 — i.e. the true signed bias is small and
noise-dominated, and IR's sign is *opposite* the seeded `+4.9°`. This is exactly
why the fixed loop applying the full seed degraded the anchored metrics.

**Effort estimate:** re-deriving the *value* is < half a day (largely done
above; a production version needs a proper bearing-only truth association +
signed stats). But the right move may be to seed a small signed value or **not
seed at all**, and *adopting* any change perturbs the env-2 urban autoferry rows
that feed the frozen Cl-4 gauntlet (ADR-0003). Per the arbiter's rule, the
value is delivered and **adoption is parked as a named Cl-4 reconciliation
addendum item** (fix the script → decide seed value/none → re-run the anchored
autoferry gauntlet → reconcile the frozen env-2 rows). The frozen rows stay
safe meanwhile: the deployment-realistic path is byte-identical.

### Verification (strict, post-rebase + fixes)
`NAVTRACKER_REQUIRE_FIXTURES=1 ctest`: **1195/1195 passed, 0 failed, 0 skips**
(1192 + 3 new teeth). One earlier run showed a lone 300 s TIMEOUT on the wave-2
`VetoIsolationHaxrAB.VetoIsolatedOnAisArmThreeSites` (280 s standalone, starved
under `-j8` while other sessions' ctest/build competed); it passes standalone
and in the clean run — a wave-2 TIMEOUT knife-edge (280 s test / 300 s cap), not
a wave-3 regression. Flagged for wave-2/infra.

## Residual / out of scope (see `docs/algorithms/sensor-bias.md`)
- **Env-2 EO/IR seed adoption** — parked as the Cl-4 reconciliation addendum
  item above (re-derivation delivered; adoption is a frozen-row re-freeze).
- ~~v2 zero-centred outlier gate (finding B5)~~ — FIXED 2026-07-14 (above).
- **Cross-sensor position-pair closed-loop test** — the `extractCrossSensor
  PositionPairs` "twin anchor-debiased-twice" path is correct by two independent
  inspections but lacks its own closed-loop regression test (the path was not
  modified this wave). Hardening item.
- Combined heading-bias + per-sensor-bias on the same ARPA touch — untested,
  no production wiring hits it.
- VetoIsolationHaxrAB 300 s TIMEOUT knife-edge (wave-2 infra; above).
