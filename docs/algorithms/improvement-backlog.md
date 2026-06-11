# Improvement backlog (from the 2026-06-10 multi-sensor review)

Engineering-robustness gaps between the current tracker and a
commercial-grade fusion stack, found while fixing the AutoFerry
benchmark (see `evaluation-log.md`, 2026-06-10 entry). Ordered by
expected value. This list complements `sota-roadmap.md`, which covers
*algorithm-level* upgrades (JIPDA, PMBM, …); everything here is about
making the existing chain robust and honestly evaluated.

## 1. Out-of-order input: in-tracker guard + retrodiction decision

**STATUS: guard DONE (2026-06-11).** `Tracker` and `MhtTracker` now
reject measurements older than their high-water mark by default and
count drops (`staleDropped()`); opt out via
`setRejectStaleMeasurements(false)` / `Config::reject_stale_measurements`.
Verified bit-identical on in-order feeds (full suite + autoferry e2e
pins). The OOSM-retrodiction follow-up below remains open.

**Problem.** `Tracker::process` / `MhtTracker::processBatch` accept any
timestamp. A stale measurement (older than the track's `last_update`)
is silently applied against the newer state (predict is a dt≤0 no-op),
and — worse — `estimator.update` rewinds `track.last_update` to the
stale time, so the *next* predict spans an inflated dt (over-blown
process noise, widened gates). The `ReorderBuffer` solves this but is
an opt-in composition element; nothing protects a consumer who skips
it.

**Change (default flips to safe).** Reject measurements older than the
engine's high-water mark inside the tracker(s); count drops
(`stale_dropped()` diagnostic). Document that the reorder buffer is
the tool for *recovering* (not just rejecting) late data.

**Test.** Feed `[t=123] → [t=115, t=126]`; assert t=115 is dropped and
counted, track state and last_update never regress, and the t=126
update behaves identically to a clean in-order run.

**Later (separate decision):** OOSM retrodiction (Bar-Shalom Bl1/Bl2)
to *use* late measurements instead of dropping them. Only worth it if
real deployments show meaningful drop rates at acceptable windows —
measure first via `ReorderBuffer::dropped()` telemetry.

## 2. IPDA/VIMM confirmation latency → flip the canonical lifecycle

**STATUS: DONE (2026-06-11).** Root cause of the synthetic latency was
*not* r₀/threshold tuning but dishonest synthetic λ_C: clutter-free
scenarios scored with the legacy global 1e-4 m⁻², making a gated hit
on a young (unconverged, large-S) track evidence *against* existence
(measured: r dips 0.5 → 0.19 over scans 2–4, confirm at scan 7).
Fixes: (a) hysteresis (confirm 0.9 / demote 0.6, ever-confirmed flag
on TrackTree; demote == confirm reproduces the memoryless readout);
(b) honest per-scenario detection tables for all 10 synthetics
(clutter-free floor 1e-6 m⁻², dense_clutter 3.33e-5 measured, P_D
0.95); (c) r₀ stays 0.5 — with honest λ the first update is decisive
and raising it was unnecessary. Result: IPDA/VIMM bit-identical to
M-of-N on clean synthetics, dominant under misses/clutter →
`use_ipda_lifecycle = use_visibility = true` are now the MhtTracker
defaults, the canonical bench config, and the documented default;
M-of-N kept as `imm_cv_ct_mht_mofn` ablation; scenario2 pins
tightened to lifetime > 0.9, breaks < 10, switches < 120. See
evaluation-log 2026-06-11.

**Problem.** IPDA/VIMM dominate M-of-N on every real-data scenario
(scenario2: breaks 64.5 → 7, lifetime 0.77 → 0.945, best-in-class
OSPA) but cost OSPA on clean synthetics (crossing 19.7 → ~82,
p95 = 500) because a new track's existence starts at r₀ = 0.5 and must
climb past confirm = 0.9, and a confirmed track demotes the moment r
dips below the same threshold (no hysteresis).

**Change.** (a) Hysteresis: separate confirm (e.g. 0.9) and demote
(e.g. 0.6) thresholds. (b) Revisit r₀ given the now-calibrated
per-sensor (P_D, λ_C) — with honest clutter densities a first
detection is much stronger evidence than the legacy global λ implied.
(c) When clean-synthetic OSPA is within noise of M-of-N, make VIMM the
canonical bench config and the documented default
(`use_ipda_lifecycle = use_visibility = true`), and tighten the
scenario2 MHT pins in `test_replay_scenario_run.cpp`.

**Test.** Baseline sweep; acceptance = no synthetic regression beyond
noise, autoferry lifecycle metrics at or better than the 2026-06-10
IPDA/VIMM rows.

## 3. Cross-tree duplicate management

**STATUS: DONE (2026-06-11).** Merge pass in `MhtTracker::processBatch`
(before the global solve): when two trees' best leaves stay within a
position-block Bhattacharyya bound (`duplicate_merge_bhattacharyya`,
default 1.0) for `duplicate_merge_seconds` (default 3.0) of
**sustained stream time**, the younger tree is retired — older
external id survives. The clock resets on separation; threshold ≤ 0
disables. Time-based, not scan-counted: a scan-counted streak of 3 is
~0.19 s at AutoFerry's 16 Hz and merged real vessels passing close
(measured: scenario6 breaks 2.5 → 11.5 before the time-based rework
fixed it — same multi-rate lesson as scan-counted M-of-N). Measured
(2026-06-11_crossmerge): id_switches roughly halved on all autoferry
scenarios (sc16 68.5 → 10, sc17 27 → 9, sc2 59 → 39.5), dense_clutter
OSPA 245 → 103, clean synthetics bit-identical; residual lifetime cost
−0.02..−0.07 on 3 of 9 autoferry scenarios from genuinely-sustained
close passes (the Bhattacharyya bound widens while a track coasts
under occlusion — FOV/occlusion modelling, item 4, is the refinement).
Remaining residual switches (sc5 ~97) are not duplicates.

**Problem.** The global hypothesis only enforces per-scan measurement
exclusivity. Two trees latched onto one target both persist and both
emit Confirmed tracks; the per-step greedy metric assignment then
flips between them (residual ~57 id_switches for VIMM on scenario2),
and OSPA carries a permanent +1 cardinality error per duplicate.

**Change.** Track-to-track merge pass after the global solve: when two
trees' selected leaves stay within a Bhattacharyya/Mahalanobis bound
for M consecutive scans, retire the younger tree (keep the older
external id — ID-stability invariant).

**Test.** Unit: two trees seeded on one synthetic target merge within
M scans and the surviving id is the older one. Bench: autoferry
id_switches drop, no new breaks on parallel_targets (the
two-real-targets case must NOT merge).

## 4. Sensor coverage beyond range: FOV sectors + EO/IR split

**Problem.** Coverage conditioning is range-only (`max_range_m`).
Cameras have azimuth sectors; harbours have occlusion. Also EO and IR
share `SensorKind::EoIr`, so they share one detection-table entry
despite measured P_D differing (EO 0.7–0.87, IR 0.24–0.54).

**Change.** (a) Add azimuth-sector coverage to `DetectionParams`
(start/width about the sensor position, evaluated in
`missDetectionProbability`). (b) Key detection models by
(SensorKind, MeasurementModel, source_id?) or split the enum so EO and
IR calibrate independently. VIMM remains the statistical backstop for
unmodelled occlusion.

**Test.** Unit: out-of-sector track gets zero miss penalty. Bench:
autoferry with split EO/IR entries.

## 5. Spatially varying clutter (clutter map)

**Problem.** λ_C is one number per sensor per scenario. Harbour
clutter is strongly non-uniform (shoreline vs open water); a global
λ under-penalises shore clutter and over-penalises open-water hits.
`AdaptiveSensorDetectionModel` exists (EWMA per sensor) but is unwired
and still spatially global.

**Change.** Grid-based clutter map per (sensor, model): EWMA of
unassociated returns per cell / cell area; `paramsFor` interpolates at
the measurement position. Wire the adaptive path into the bench as an
ablation config first.

**Test.** Bench ablation vs fixed table on all autoferry scenarios;
acceptance = fewer confirmed false tracks (OSPA ↓) without lifetime
loss on true tracks.

## 6. Default-detection-model footgun diagnostic

**STATUS: DONE (2026-06-11).** `MhtTracker::defaultDetectionModelWarning()`
goes sticky-true when ≥2 distinct (SensorKind, MeasurementModel) keys
have been processed on the auto-installed single-default model. Never
set when any model was injected (an explicit single-default injection
is a stated choice). Composition roots should poll it once after
warm-up.

**Problem.** A multi-sensor consumer who doesn't inject a per-sensor
table silently runs every sensor at the default (P_D 0.9, λ 1e-4 m⁻²)
— the exact misconfiguration that produced the pre-fix AutoFerry
collapse, and dimensionally wrong for bearing sensors.

**Change.** One-shot diagnostic: when `MhtTracker` sees ≥2 distinct
(SensorKind, MeasurementModel) keys but the detection model is the
single-default `FixedSensorDetectionModel`, surface a warning through
a diagnostics hook (no logging in core — expose a counter/flag the
composition root can read, like `ReorderBuffer::dropped()`).

**Test.** Unit: flag set on the second distinct sensor kind; never set
with a table-backed model.

## 7. Philos replay: asynchronous truth resampling

**Problem.** All MHT configs score lifetime ≤ 0.015 on philos. Philos
truth is AIS-as-truth: asynchronous per-vessel messages with no scan
structure, so the AutoFerry per-scan truth fix does not apply — steps
are 1-truth fragments again, and the scenario still uses the legacy
scalar λ_C.

**Change.** Time-windowed truth resampling in the harness (interpolate
each vessel's AIS track onto a fixed evaluation clock, e.g. 1 Hz,
holding cardinality per window), plus a per-sensor detection table for
the philos radar.

**Test.** Same style as `AutoferryScenario2GnnMetricsAreSane`: pin
step cardinality and identity metrics on the real fixture.

## 8. JPDA path parity: per-sensor (P_D, λ_C)

**Problem.** `JpdaAssociator(gate, P_D, λ)` takes scalars; the
single-hypothesis pipeline never sees the per-sensor detection port.
Fine for single-sensor, inconsistent for fusion.

**Change.** Pass `ISensorDetectionModel` into the JPDA β computation
(per-measurement λ in its own units). This is also the first step of
the JIPDA upgrade already specced in `sota-roadmap.md` §2.

## 9. Inter-sensor registration biases

**Problem.** Only own-ship heading bias is estimated. Radar↔lidar↔
camera mounting offsets / range biases are unmodelled; on AutoFerry
they fold into pos_rmse and gate sizes.

**Change.** Per-sensor bias states (Schmidt-KF consideration already
sketched in `sota-roadmap.md` §5) fed by AIS-anchored residuals, like
the heading-bias estimator's v1 pair flow.

## 10. Benchmark hygiene

- OSPA cutoff 500 m compresses differences on harbour-scale scenes;
  report c = 100 m alongside (keep 500 for cross-baseline
  comparability).
- Replays are single-seed; add measurement-order/jitter perturbation
  seeds so replay rows get error bars.
- haxr remains gated out for full-enumeration JPDA/MHT — needs cluster
  decomposition (independent-subproblem partitioning) before it can
  join the matrix.
