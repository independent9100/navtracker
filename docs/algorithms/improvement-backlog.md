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

**STATUS: DONE (2026-06-11).** (a) `DetectionParams` gained
`sector_center_rad` / `sector_width_rad` (ENU math convention, default
full circle = legacy), evaluated in `missDetectionProbability`
alongside `max_range_m` — out-of-sector tracks charge no miss penalty.
(b) Detection entries are now source-keyed:
`ISensorDetectionModel::paramsFor(sensor, model, source_id)` with
fallback source → kind-wide → defaults; the TrackTree miss loop keys
distinct surveying sensors by the full (sensor, model, source) triple.
AutoFerry declares split camera entries calibrated per camera and per
environment (EO P_D 0.7 open / 0.8 urban; IR 0.5 / 0.4).

**Calibration lesson (recorded so it isn't re-tried):** the measured
per-environment camera clutter rate (open water 0.004–0.6 rad⁻¹ vs
urban channel 1.0–4.9 rad⁻¹) was fed in first and **collapsed urban
lifetime** (sc17 0.65 → 0.35, sc13 0.77 → 0.59, sc22 0.71 → 0.44;
baseline `2026-06-11_eoir_split_measured_lambda`). The urban excess is
persistent structured shoreline returns, not uniform Poisson clutter —
when the model family is wrong, the honestly-fitted parameter is not
the right operating point. Camera λ therefore stays at the kind-wide
0.5 rad⁻¹ (regression-pinned in
`ReplayScenarioRun.AutoferryDeclaresSplitEoIrDetectionEntries`) until
item 5 provides a spatial clutter model.

**Open residue → item 5:** spatially-resolved clutter for the urban
shoreline. VIMM remains the statistical backstop for unmodelled
occlusion; per-measurement sensor attitude (sectors on rotating
platforms) is future work.

## 5. Spatially varying clutter (clutter map)

**STATUS: DONE for position sensors (2026-06-12); bearing maps
shipped but OFF by default.** `ClutterMapSensorDetectionModel`
(association.md §6): decorator over the fixed table; sparse per-
(sensor, model) grids of time-based EWMA cells (τ = 20 s), virtual
`paramsFor(z)` interpolates λ at the measurement position, clamped to
[baseline/8, baseline·64]; untouched cells read back the table.
Bench ablation `imm_cv_ct_mht_cmap`. Measured
(`2026-06-12_clutter_map`): dense_clutter OSPA 103 → 64, philos
429 → 398 (switches → 0), autoferry lifetime preserved-or-up on all
9; clean synthetics pay +5–11 OSPA from birth self-poisoning (a new
target's first return bumps its own cell). **Key negative result:**
the bearing map alone collapsed camera-heavy lifetime (sc17
0.90 → 0.25; baseline `2026-06-12_clutter_map_bearing_spiral`,
per-sub-map ablation isolated it) — bearings can't initiate tracks,
so a trackless target's own bearings get labeled clutter and block
its re-confirmation, self-reinforcing. Bearing maps are opt-in
(`enable_bearing_map`) until the clutter proxy is fixed.

**Update 2026-06-12 (second iteration):** clutter evidence is now
labeled from the *global hypothesis* — each return carries weight
1 − r of the claiming track/birth, 1.0 unclaimed (was: binary
birth-gate). This removed the clean-scene birth tax but, against the
original hypothesis, did **not** make the bearing map safe: measured
strictly worse (sc17 lifetime 0.13, sc5 0.10), because a coasting or
re-born track's claimed bearings carry weight 1 − r exactly while r
is low. Bearing maps stay opt-in; the open path is a weight that
distinguishes "low-existence target" from "no target"
(visibility-conditioned weights or a hard zero for hypothesis-claimed
returns — association.md §6 ways-to-improve). Promotion of the
position map into the canonical config is a measurement question
after the hyplabel baseline. Pairs with backlog §8 (JPDA per-sensor
parity) for the single-hypothesis path.

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

**STATUS: DONE (2026-06-11).** `resampleTruthToClock(samples, period_s,
max_gap_s)` (`core/scenario/TruthResample.hpp`) interpolates each
vessel's asynchronous AIS track onto a shared evaluation clock: linear
position interpolation + segment-FD velocity between fixes, nearest-tick
snap (half-open ±period/2) at span endpoints so single-fix vessels get
one-step presence instead of becoming permanent cardinality errors, and
a max-gap guard so minutes-long dropouts are not bridged by a straight
line. PhilosScenarioRun resamples at 1 Hz / 30 s max gap and declares a
calibrated per-sensor table: radar 0.07 / 2.7e-6 m⁻² / 1000 m **per
sub-scan event** (the rotating sweep arrives as ~10 narrow azimuth
bursts/s — measured across 187 vessel × event opportunities), AIS
0.05 / 1e-9 (a broadcast detects one vessel per event, so per-event
P_D ≈ 1/N_vessels). Canonical MHT on philos: lifetime 0.015 → 0.295,
breaks 0.04, switches 0.17, OSPA 430 (off the 500 cutoff), pos_rmse
38 m; pinned in `ReplayScenarioRun.PhilosResampledTruthAndMhtLifecycle`.
The remaining lifetime ceiling is honest: the fixture is a ~20 s
snippet where most vessels report AIS only twice ~10 s apart, so
confirming at the second fix already costs half the presence window.
Boston-harbor radar caveat: unmatched plots are mostly persistent shore
structure — same uniform-λ limitation as the AutoFerry urban cameras
(item 5).

## 8. JPDA path parity: per-sensor (P_D, λ_C)

**STATUS: DONE (2026-06-13).** Second `JpdaAssociator` ctor
`(gate, ISensorDetectionModel*)`: each gated measurement contributes
`log P_D[s] + log p(z|x) − log λ_C[s]` in its sensor's natural units
(m⁻² / (m·rad)⁻¹ / rad⁻¹), and the per-track miss factor is aggregated
over distinct `(sensor, model, source_id)` tuples in the scan via
`missDetectionProbability(...)` — same coverage-conditioned convention
as `TrackTree::branch`. Two new bench ablations
(`ekf_cv_jpda_persensor`, `imm_cv_ct_jpda_persensor`) opt into the
scenario's detection table via a `PerSensorAssociatorFactory` on
`benchmark::Config`. Scalar ctor + existing scalar configs untouched
and bit-identical. Measured on the synthetic sweep
(`jpda_persensor_20260613T143004Z`): dense_clutter id_switches −71%
on EKF/CV (honest 3.33e-5 m⁻² vs legacy 1e-4), non_cooperative
pos_rmse −40% on IMM/CV+CT (calibrated 1e-2 rad⁻¹ vs mismatched
1e-4 m⁻²); no clean-synthetic regressions on either pipeline.
Replay measurement pending; see evaluation-log 2026-06-13 (later 2).

**Open follow-ups:** (a) per-sensor batch decomposition (true
sequential multi-sensor fusion) so mixed-sensor IMM gets a proper
per-batch P_D in the mixture-likelihood normalization instead of the
homogeneous-batch shortcut; deferred until JIPDA where per-track
existence covers it natively. (b) JIPDA (sota-roadmap.md §2): augment
each track with `r(k)` and run lifecycle off it on the JPDA path too.

**Problem (original).** `JpdaAssociator(gate, P_D, λ)` takes scalars;
the single-hypothesis pipeline never saw the per-sensor detection
port. Fine for single-sensor, dimensionally wrong on mixed sensors.

## 9. Inter-sensor registration biases — DONE (2026-06-15; Schmidt-KF + bearing pairs added 2026-06-16)

**Problem.** Only own-ship heading bias is estimated. Radar↔lidar↔
camera mounting offsets / range biases are unmodelled; on AutoFerry
they fold into pos_rmse and gate sizes.

**Change shipped.** Per-(sensor, source_id) bias filters fed by
AIS-anchored cross-sensor pair extraction. Position bias (2-D) on
radar / lidar; bearing bias (scalar) on EO / IR cameras. Random-walk
dynamics with very small Q. Three observability gates (time, range,
innovation) modelled directly on `HeadingBiasEstimator` G1-G2-G3.

**Schmidt-KF follow-up (2026-06-16).** The 2026-06-15 ship applied
only the mean `b̂` to incoming measurements; `P_b` was published but
unused. That makes the filter overconfident exactly when the bias
estimator has just published with `P_b` still wide. Fixed: the bias
correction now inflates the measurement covariance by
`H_b · P_b · H_bᵀ` — `R + P_b` for Position2D, `R + σ_b²` on the
bearing component for Bearing2D / RangeBearing2D. The state-bias
cross-covariance is dropped (the "considered" simplification);
that's exact here because the bias estimator only ingests
AIS-anchored pairs disjoint from per-track filter observations.
Implementation: `core/pipeline/BiasCorrection.hpp` (shared between
`Tracker` and `MhtTracker`). Unit tests:
`tests/bias/test_bias_correction.cpp`. Learning chapter §6.

**Bearing-pair extraction (2026-06-16).** The 2026-06-15 ship
extracted only Position2D pairs; EO/IR bearing biases were
unobserved. Per-target diagnostic on sc13_anchored revealed a
systematic ~7° camera bias that the bias estimator never saw —
sc13 NEES stayed at 75 anchored while sc16/17/22 dropped to 1–3.
Fixed: `extractBearingPairs(tracks, time)` walks the same
`recent_contributions` window for (AIS anchor) × (Bearing2D
contribution) pairs and emits `BearingBiasPairObservation`. To
carry the bearing payload through the SourceTouch side-channel,
`Track::SourceTouch` gained optional `alpha_rad` / `alpha_var_rad2`
fields (NaN sentinel = "this touch was not a bearing");
`fillSourceTouchEnu`'s Bearing2D case now populates them. Sweep's
PostScanHook calls both extractors. Tests:
`tests/bias/test_sensor_bias_estimator.cpp::SensorBiasPairExtractor.*`.

New ports / types: `ISensorBiasProvider`, `SensorBiasKey`,
`SensorBiasEstimator`, `NullBiasProvider`, `SensorBiasPairExtractor`.
Tracker / MhtTracker gained `setSensorBiasProvider`. New bench config
`imm_cv_ct_mht_biascal` wires the estimator post-scan. Spec:
`docs/superpowers/specs/2026-06-13-inter-sensor-registration-bias-design.md`.
Learning chapter: `docs/learning/21-sensor-registration-bias.md`.

## 10. Benchmark hygiene

- OSPA cutoff 500 m compresses differences on harbour-scale scenes;
  report c = 100 m alongside (keep 500 for cross-baseline
  comparability).
- Replays are single-seed; add measurement-order/jitter perturbation
  seeds so replay rows get error bars.
- haxr remains gated out for full-enumeration JPDA/MHT — needs cluster
  decomposition (independent-subproblem partitioning) before it can
  join the matrix.

## 11. Bearing-driven identity churn on angularly-unresolvable targets

**RE-DIAGNOSED 2026-06-12 — the 2026-06-11 hypothesis was wrong.**
Forensics on the ~91 sc5 switches (per-event dump): only 21 of 182
raw events are pair swaps. The dominant pattern is a **conveyor belt
of short-lived duplicate tracks**: 107 confirmed ids in 139 s for 2
truths; 45 of 48 near-truth confirmations happened with a live
confirmed track already within 50 m. Mechanism: a track carried by
16 Hz camera bearings drifts 10–30 m and turns overconfident; the
sparse (~0.6 Hz) radar return then misses the χ² gate, births a
duplicate, the young tree takes the stream, the old one starves —
identity hands off every 2–4 s.

**Knobs landed 2026-06-12 (all opt-in, defaults OFF — measured, none
promotable yet):**
- `share_ambiguous_bearings` — a Bearing2D return whose hit branches
  exist in ≥ 2 trees is exempted from solve exclusivity (candidate (b)
  in its cheapest form). Measured: sc5 bit-identical (each tree
  already takes its nearest bearing — per-scan swaps were not the
  churn), tiny moves elsewhere. Kept for genuine merged-detection use.
- `DetectionParams::gate_threshold` — per-sensor static gate. Flat 50
  on position sensors: switches roughly halved on all 9 scenarios,
  OSPA −15..−95, but lifetime −0.03..−0.07 on 4 scenarios.
- `gate_recapture_tau_s` / `gate_recapture_max_scale` — position gate
  scales with the hypothesis' position-anchor age (recapture exactly
  when bearing-carried). Measured: switches/OSPA improve strongly
  (sc5 91 → 38) but lifetime drops (sc3 0.87 → 0.63, sc17
  0.90 → 0.54) and rmse climbs to 30–60 m — the return gates back in
  but the Kalman gain (same overconfident P) barely corrects.

**Actual root cause → item 12.** NEES on sc5 near-truth confirmed
tracks: **mean 77.6** (consistent ≈ 2), 57% of samples above the 99%
χ² bound; claimed σ 1.2–3.8 m vs actual error 15.1 m mean. The
filter is structurally overconfident on real bearing-dominated data;
every symptom above is downstream. Fix the covariance first, then
re-evaluate these knobs (they may become unnecessary or finally
default-able).

**Re-measured 2026-06-16 on honest per-env R (item 12(a) landed).**

Base canonical (no item-11 knobs) already captured most of the
benefit:

| Sc | id_switches OLD R | id_switches NEW R | Δ |
|----|---:|---:|---:|
| sc5 | 91   | **49.5** | −46% |
| sc6 | 74.5 | **36.5** | −51% |
| sc3 | 40.5 | **17.5** | −57% |
| sc2 | 37.5 | **24.5** | −35% |

The conveyor mechanism shrank as predicted: with the lidar gate now
~4× wider (8 m vs 2 m R), more radar / lidar returns gate back into
existing tracks, fewer duplicates spawn.

Re-measuring `imm_cv_ct_mht_recapture` (τ = 2 s, max_scale = 8) on
top of honest R confirms the June 12 trade-off persists:

- GOSPA improves on **all 9** scenarios by 10–36% (sc3 −36%, sc22 −23%).
- ID switches drop further on 8/9 (sc17 −94%, sc6 additional −48%).
- BUT lifetime regresses: sc17 0.87 → 0.38 (catastrophic), sc3 0.88 → 0.66.
- pos_rmse climbs 9–162%; NEES spikes on most scenarios (track-loss
  events dominate the per-step average).

Honest R did *not* eliminate the gate-recapture trade-off — it only
shifted the operating point. The recapture knob makes a different
kind of mistake (over-aggressive association → state error → track
loss) instead of the original (gate too tight → duplicate birth).
**Decision: leave as opt-in; do not promote to default.** Item 11
closes here.

The remaining sc17 lifetime / sc22 NEES residuals are deeper than
gating — see SOTA-roadmap §2 (JIPDA, the paper's tracker class) and
§5 (Schmidt-KF residual-bias treatment).

## 12. Filter consistency on real data (NEES calibration)

**Problem (measured 2026-06-12, sc5).** Mean position NEES 77.6 vs
the ~2 of a consistent filter. Suspects, in test order:
(a) camera bearing R too small (calibrate σ_bearing against AIS-truth
residuals per camera); (b) bearing-update range collapse — 16 Hz
linearized bearing updates leak spurious range information through
the velocity states (classic BOT pathology; candidate guard: range-
direction variance must be non-decreasing under a Bearing2D update);
(c) IMM process noise tuned on synthetics, too small for real harbour
maneuvering.

**Change.** Instrument NEES per (scenario, sensor mix) in the bench;
calibrate R from measured residuals; add the bearing range-variance
guard if (b) confirmed.

**Test.** NEES → O(2) band on autoferry scenarios; then re-run the
item-11 knob sweeps — expect conveyor births to vanish at the BASE
gate (honest P keeps radar returns in gate), sc5/sc6 switches to
collapse without lifetime loss.

## 13. Cross-sensor anchored bias (AIS-less deployment) — DONE 2026-06-17 (math correct, fires; AutoFerry gospa-invariant)

**Problem.** `SensorBiasEstimator` (item 9) needs an unbiased anchor
to learn each sensor's mounting offset; today the only anchor it
consumes is AIS. Real maritime deployment is full of non-cooperative
targets (sailing boats, kayaks, jet skis, illegal vessels) that
broadcast no AIS. The tracker itself handles non-cooperative targets
fine — but the bias *calibration* path is blocked. Without
calibration the per-sensor offsets fold silently into pos_rmse and
gate sizes (the gap the Helgesen 2022 comparison surfaced).

**Change.** Cross-sensor anchoring: when no AIS contribution exists
on a track, use a high-confidence converged-track position sourced
from sensors *other than* the one being calibrated. Lidar tracks
calibrate radar bias and vice versa. The cyclic-anchor problem (a
track using sensor X cannot anchor X's bias) is sidestepped because
the anchor sensor differs from the sensor under calibration.

**Eligibility gates.** A contribution from sensor Y is anchor-
eligible for sensor X iff: (i) Y ≠ X (no self-anchoring),
(ii) the track has `existence_probability ≥ 0.95`, (iii) the track's
2x2 position covariance trace is below a tight threshold
(e.g. ≤ 25 m²), (iv) no other Y on the track this cycle (avoid
double-anchoring through the same sensor twice).

**Coupling.** Both biases are unknown, so a single pair gives one
equation in two unknowns. Two paths: (a) freeze one sensor's bias
once it has converged on an AIS subset (then it anchors the others),
(b) joint coordinate-descent across the (X, Y) bias pair, alternating
which is "anchor" each iteration. (a) is the simpler bootstrap; (b)
the steady-state solution.

**Test.** AutoFerry env 1 GOSPA RMS drops measurably from the AIS-
anchored option-1 baseline (target: 5-15% additional reduction once
the AIS-less anchor pulls in the AIS-absent stretches). Synthetic
non_cooperative scenario: estimator publishes a non-trivial bias
without AIS in the input stream.

**Outcome (2026-06-17).** Shipped (`extractCrossSensorPositionPairs`
+ unit tests + canonical wiring + design-doc + eval-log entry). The
math is correct and the path fires on every AIS-less AutoFerry
scenario (~100-1000 pairs per run; estimator publishes biases of the
right magnitude). **The 5-15% GOSPA target was not met** — measured
zero delta because cross-sensor coordinate descent under a symmetric
zero-mean prior allocates the relative bias as `(b̂_X, b̂_Y) =
(+δ/2, −δ/2)`, and a symmetric weighted-mean fusion is invariant
under that allocation (the corrections cancel in the fused estimate).
Item 13 unlocks value only when (a) intermittent AIS calibrates one
sensor absolutely, (b) sensor variances are asymmetric, or (c) one
sensor's bias is pre-seeded via `setKnownPositionBias` — none of
which is true on AutoFerry env-1 raw. See `sensor-bias.md
§Cross-sensor anchored extension` and eval-log 2026-06-17 (later).
Latent value: leave enabled by default; when item 14 (per-
measurement R) lands and produces asymmetric variance, item 13
will start contributing.

## 14. Sensor-provided per-measurement covariance (LOW PRIORITY)

**Problem.** The current adapters write a fixed-per-sensor R into
`Measurement::cov` (or leave it empty for the `pessimisticSensorDefaults`
fallback). Many real sensors *do* publish per-detection uncertainty
that we discard: AIS Class A position-accuracy categories, ARPA track-
quality bits, modern EO/IR computer-vision detection bounding-box
covariances, lidar per-cluster spread, GNSS HDOP/FOM. Throwing this
away is information loss — a single bad-quality return weights the
state estimate the same as a clean one.

**Change.** Per-sensor adapter opt-in: when the sensor reports a
covariance, populate `Measurement::cov` from it. The tracker enforces a
**covariance floor** against the item-12(a) per-env calibrated R:
`R_used = elementwise_max(R_reported, R_floor)` (or a more careful
PSD-preserving variant — `R_used = R_reported + R_floor_residual` where
`R_floor_residual = max(0, R_floor − R_reported)` along each eigen
direction). The floor is the safety net: if a vendor reports
optimistic noise we don't blindly trust it; if they report nothing
we use the calibration. Diagnostic flag `covariance_is_reported` on
output (alongside the existing `covariance_is_default`) so the eval
log can distinguish the two paths.

**Assumptions.** (a) Per-env calibrated R floors exist for every
sensor kind we use (item 12(a) shipped them for AutoFerry); (b)
adapter authors take responsibility for unit-converting the vendor
value into m² / rad²; (c) the bias estimator's gate-by-covariance
math is robust to per-measurement R variation (it already is, by
construction — R enters as an additive term in the innovation).

**Rationale.** Trusting sensor-reported R unconditionally is unsafe
(vendor specs are optimistic, NMEA epsilon fields often zero), but
trusting our fixed table when honest per-measurement noise is
available leaves gain on the table. The `max(reported, floor)`
policy gets the best of both: tight when the sensor knows, calibrated
when it doesn't.

**Test.** A synthetic scenario with sensor-quality variation per
measurement (e.g. radar plot SNR variation) should out-perform the
fixed-R baseline on pos_rmse. AutoFerry baseline numbers should not
move on sensors that don't yet populate per-measurement covariance
(bit-identity preserved).

**Lift.** Small (per-sensor adapter changes + one floor-policy
function in the tracker) — 1-2 days. Scheduling priority: LOW.
File-level entry point is `Measurement::cov` and the per-sensor
adapter populators in `adapters/`; the floor policy fits in
`core/types/Measurement.hpp` or as a new
`core/sensors/CovarianceFloor.hpp`.

**Where the trade-off lives.** Vendor R-quality varies per sensor
family. AIS Class A's position-accuracy enum is reasonably
calibrated; NMEA RMC's epsilon fields are often hard-coded zero;
ARPA quality bits are unstandardised across vendors. Start with the
sensors whose reported R has known meaning (AIS, modern lidar,
CV-based EO/IR with bbox covariance) and stay on the fixed table
for legacy ARPA / dumb GNSS until per-vendor calibration exists.

Raised 2026-06-17 in the item-9 closeout review.

## 15. Batch must be pre-sorted by the caller (QUICK FIX — schedule soon)

**STATUS: DONE (2026-07-02).** Both `MhtTracker::processBatch` and
`PmbmTracker::processBatch` now `std::stable_sort` the batch by time at the top,
behind an `is_sorted` fast-path (already-sorted input = no-op = bit-identical;
verified — 889 tests green, pmbm A/B unchanged). MHT was the real fix (it used
`scan.front().time` as the instant, so an unsorted batch mis-timed and could
whole-batch stale-drop); PMBM was already order-robust (predicts to `t_max`,
set-wise association) so its sort is defensive/uniform + pins the optional
idle-decay knob's `front()` read. Manual sort removed from
`app/mht_fusion_example.cpp`; contract restated ("the tracker orders the batch").
Test: `tests/pipeline/test_batch_ordering.cpp` (MHT RED→green equivalence, PMBM
order-independence guard, determinism). Cross-tick late-data recovery
(`ReorderBuffer`, item 1) and batch-resolution remain out of scope as noted below.

**Problem (ergonomics).** `processBatch` is documented and implemented as
"one scan at `scan.front().time`". The *very common* consumer pattern —
run a fixed-rate loop (e.g. 10 Hz), collect every measurement that arrived
since the last tick, hand the lot to the tracker — produces a batch whose
members are **not** timestamp-sorted (AIS, radar, EO-IR arrive
asynchronously). Today the caller must know to sort it themselves:
`app/mht_fusion_example.cpp:149-159` does exactly this (`std::sort` by time
before `processBatch`), and the requirement is only stated in a code
comment. A consumer who skips the sort gets a wrong scan instant
(`front().time` is not the earliest), and with `reject_stale_measurements`
on (the default, item 1) the out-of-order tail is **silently dropped and
counted**. This is a footgun disguised as a no-op — the tracker still
"works", it just quietly loses data. Confirmed present in **both**
`MhtTracker::processBatch` and `PmbmTracker::processBatch`
(`core/pmbm/PmbmTracker.cpp` uses `scan.front().time` as the scan instant).

**Change (quick fix).** Sort the batch inside `processBatch` — a
deterministic `std::stable_sort` on `time` at the top of the method, in
both trackers. Already-sorted input is a no-op → **bit-identical** for
every existing consumer and test (the app example's manual sort becomes
redundant, not wrong). Then delete the "caller must sort" burden from the
docs/example and state the new contract: *a batch may arrive in any order;
the tracker orders it.* Keeps the batch-as-one-scan model unchanged — this
only fixes which instant represents the scan and stops the stale-guard from
eating an unsorted tail.

**Scope boundary (NOT this item).** This does **not** address (a) whether a
100 ms batch should be one scan or several sub-scans (a real design
question — batching resolution vs. latency), nor (b) recovering genuinely
late data across ticks (that is item 1's `ReorderBuffer` / OOSM
retrodiction). It only removes the *within-tick pre-sort* requirement so the
naive fixed-rate consumer is correct by default.

**Test.** Feed one batch out of order (`[t=126, t=115, t=140]`) to each
tracker and assert the result is identical to the same batch pre-sorted;
assert nothing is stale-dropped; assert an already-sorted batch is
bit-identical to today (determinism pin).

**Lift.** Tiny — two `stable_sort` calls + doc/example cleanup + one
equivalence test per tracker. Half a day. **Scheduling priority: SOON**
(high ergonomics value, low risk, unblocks the canonical fixed-rate
integration pattern). File entry points: `core/pipeline/MhtTracker.cpp`
and `core/pmbm/PmbmTracker.cpp` `processBatch`; example cleanup in
`app/mht_fusion_example.cpp`.

Raised 2026-07-02 (user side-quest — "sensor data needs to be sorted" is a
general smell that will hurt naive consumers).

---

## 16. Per-pose heading σ on OwnShipPose (consistency wart) — **DONE 2026-07-06**

**Status (2026-07-06): SHIPPED.** `std::optional<double> heading_std_deg` added
to `OwnShipPose`. Composition rule `max(pose.heading_std_deg, floor)` in
quadrature, floor-can-only-widen, absent ⇒ bit-identical: wired into the bearing
builders (`makeMeasurementFromRelativeBearing` / `...TrueBearing` gain a trailing
`heading_std_floor_deg=0.0`), `ArpaAdapter` and `EoIrAdapter` (their
`cfg_.heading_std_deg` is now the floor). Bias-estimator assessment (steer): NO
change — its observations (`GyroVsGps*`, `AisArpaPairObservation`, …) already
carry their own per-source σ and compare heading *sources* to estimate gyro
bias, a different quantity from the fused-heading σ the projection uses; wiring
the new field there would be conceptually muddy. Tests: builder composition
(widen / floor-clamp / bit-identical) + one per-adapter. Guide §4/§5/§7 (wedge
tie-in) + appendix + pitfall updated same commit; no new Config struct ⇒
drift-guard green untouched. Follow-up (parked): `OwnShipNmeaAdapter` populating
`heading_std_deg` from a talker's reported quality (needs the fact-dependent
per-sensor mapping — same shopping-list gate as #18's fact half).

**What.** `OwnShipPose` carries per-fix σ for position
(`position_std_m`) and velocity (`velocity_std_m_per_s`) — but NOT for
the primary `heading_true_deg`. The random heading error instead lives
as static per-consumer config (`ArpaAdapterConfig.heading_std_deg`,
`EoIrAdapterConfig` equivalent), while the v3 auxiliary heading fields
(`gps_true_heading_deg`, `magnetic_heading_deg`) DO carry per-fix σ
because bias observations need them. Found 2026-07-03 when an
integrator asked "why is there no std deviation of heading?".

**Why it matters.** (a) Inconsistent API: two heading sources carry σ,
the one that projects every radar/EO-IR measurement doesn't. (b) The
per-adapter config default is 0.0 ("perfect gyro") — a documented
pitfall that exists only because the σ is detached from the pose.
(c) Per-fix σ cannot be expressed at all: a satellite compass degrades
with constellation geometry, a gyro during alignment is worse than
steady-state — today both must be priced at one static worst-case
number per adapter, or optimistically ignored.

**Proposal.** Add `heading_std_deg{0.0}` to `OwnShipPose`. Consumers
(`ArpaAdapter`, `EoIrAdapter`, measurement builders) use
`max(pose.heading_std_deg, cfg.heading_std_deg)` — pose value when the
nav source provides one, adapter config as the deployment-level floor,
and the existing bias-variance composition (`sigma_heading_eff`)
unchanged on top. `OwnShipNmeaAdapter` populates it from talker config.
Consumer-surface change ⇒ integration-guide entry required (heading-σ
split already has a MUST entry in the guide ticket; update it).

**Test.** A TTM projected with `pose.heading_std_deg = 2°` and adapter
config 0 gets the same cross-range σ as today's config-only path with
2°; the max() rule asserted both ways; bit-identical when both are 0.

**Lift.** Small — one field, two consumer sites, builder passthrough,
tests. Half a day. Priority: LOW-MEDIUM (correctness of σ bookkeeping,
no behavior change for correctly-configured deployments).

Raised 2026-07-03 (integrator question; the "why don't we have a std
deviation of heading?" wart).

---

## 17. Camera-only contact — options short of a distance sensor

**STATUS — option 1 SHIPPED 2026-07-05 (full suite 1032/1032).** `BearingWedgeModel`
(`core/static/BearingWedgeModel.hpp`) + `BearingWedgeOutput`: a camera-only
`Bearing2D` contact surfaces as a wedge from own-ship (half-width `max(2σ, floor)`,
σ = composed camera⊕heading; range `std::optional`, unbounded by default). Standalone
(not on the PMBM hot path). Handover is per-drain **suppression, not deletion**
(a confirmed track in the wedge's angle hides it; it reappears when the track
leaves; only camera silence removes it — avoids the ADR-0002 forbidden failure of
a crossing vessel erasing a still-seen contact). Anchor-frame apexes + IDatumChangeSink
(recenter-safe); reused/suspect contact ids mint a fresh never-reused wedge_id.
Docs: output-contract (BearingWedgeOutput), integration-guide §7 + datum-sink list,
learning ch. 28 (+figure), algorithm doc `bearing-wedge-hazard.md`, glossary. Options
2 (waterline monocular range) + 3 (range-parameterised bearing-only init) remain
open; PMBM auto-wiring of option 1 is the next step (see the algorithm doc). The
original ticket follows.

**Problem.** A target only the camera sees (kayak, small wooden boat —
radar-silent) currently becomes NOTHING: `Bearing2D` cannot initiate a
track (by design), so the object is invisible in the output. Sharpest
real-world violation of the ADR-0002 "never invisible" rule. The clean
fix is the planned distance sensor (camera+range = existing
range/bearing path, zero new architecture). If that slips or fails,
three fallbacks, in recommended order:

1. **Bearing-wedge hazard (safety net, cheapest).** "Never invisible"
   does not require a position. Emit a hazard that is a DIRECTION: a
   wedge from own-ship along the detection bearing (width = bearing σ,
   range unbounded or sensor-max). Operator-actionable ("keep clear of
   that line"); CPA not computable; fits presence-over-classification
   exactly. Needs an output-contract extension (hazard-as-sector).
2. **Waterline monocular range (pragmatic upgrade).** Camera height
   above water + the pixel row where target meets water ⇒ coarse range
   by flat-water geometry (r ≈ h/tan(depression)). Error grows ~r²;
   honest large σ_r is exactly what per-measurement R handles. Upgrades
   camera to coarse range+bearing with the EXISTING path. Needs
   calibrated pitch/roll + horizon; degrades in swell. Prior art:
   standard practice in USV monocular perception.
3. **Range-parameterized bearing-only initiation (heavy).** Birth a
   mixture of candidates along the ray at assumed ranges; motion +
   own-ship baseline kills the wrong ones (classic bearing-only TMA;
   PMBM can host it as a birth mixture). Real work; only if 1+2 prove
   insufficient.
Also procedural, costs nothing: the system can PROMPT a small course
alteration ("bearing-only contact — 10° alteration resolves range by
triangulation").

**Trigger.** Decide when the real deployment's camera/distance-sensor
facts arrive. If the distance sensor is confirmed, only item 1 is worth
considering (defense-in-depth for sensor failure).

Raised 2026-07-04 (integrator asked "any other idea?"; recorded so it
survives the two-week memory horizon).

---

## 18. Own-ship nav-input sanity guard (library-side, not just documented)

**STATUS — fact-free half SHIPPED 2026-07-05 (full suite green).** `NavInputGuard`
(`core/own_ship/NavInputGuard.hpp`, free `evaluateNavInput`) + `INavHealthSink` +
`OwnShipProvider::setNavHealthSink`. On each `update(pose)` with a sink wired, the
incoming pose is checked against the previous and, if it trips a flag, the sink is
fired — degrade VISIBLY, never rewrite the pose (validate at the edge, invariant
#6). Four fact-free checks with GENERIC thresholds (`NavInputGuardConfig`):
`heading_unreliable_low_sog` (SOG < steerage way, the COG-at-anchor own-ship
twin), `stale_gap` (pose gap too long), `position_jump`, `heading_jump`. Nullable
sink ⇒ inert. **Parked (fact-dependent half):** the calibrated per-sensor
thresholds and any hold-last/σ-inflation *policy* — those need the deployment's
heading-source type + GPS quality (still on the shopping list). Docs: guide §9
(pitfalls + guard subsection) + appendix `NavInputGuardConfig`. The original
ticket follows.

**Problem.** Three own-ship nav failure modes are currently only
*documented* pitfalls; the library accepts the bad input silently:
(a) COG wired as heading at SOG≈0 → heading jumps with GPS jitter →
every relative-bearing sensor smears targets in arcs (guide §9,
2026-07-04); (b) stale pose — nav feed stops, projection silently
continues with the old fix, error grows unbounded (guide §9);
(c) position/heading step-jumps (GPS glitch, gyro fault) pass through
unchallenged.

**Proposal.** A small `NavInputGuard` at the OwnShipProvider edge
(validate-at-the-edges invariant), all thresholds config:
- **SOG-gated heading acceptance:** if the pose's heading source is
  motion-derived (COG), reject/hold-last below a SOG threshold —
  mirror the discipline `GyroVsGpsCogObservation` already applies
  inside the bias estimator (SOG + turn-rate gates).
- **Staleness signal:** pose age > threshold ⇒ fire a diagnostic sink
  (operator-visible "nav degraded"), optionally inflate
  sensor_position_std_m with age rather than pretending freshness.
- **Jump detection:** implausible position/heading step vs elapsed
  time ⇒ flag, don't silently ingest.
Degrade VISIBLY, never silently: the guard's job is to make nav
trouble an event, not a smear. Recovery after an episode is already
graceful (phantoms starve in seconds–minutes, occupancy decays, bias
estimator's COG gate closed the pollution path) — the guard shrinks
the blind window itself, which is the only unrecoverable part.

**Trigger.** Design when the real-test nav facts arrive (heading
source type, GPS quality); implement before the first unattended
deployment. Consumer-surface change ⇒ integration-guide entry.

Raised 2026-07-04 (integrator: "COG could start to jump when
stationary, or?" — yes; and the episode window is the damage).

---

## 19. Sensor-id continuity as soft association evidence (camera disambiguates what radar can't)

**Idea (integrator, 2026-07-04).** Sensors with their own trackers emit
per-sensor track ids (`hints.sensor_track_id` — today write-only). A
camera can KEEP APART two objects that are one blob to radar (two moored
boats side by side; the stationary/anchored use case). If the camera
pipeline ran a detector-TRACKER (e.g. ByteTrack-style) instead of
per-frame detection, its stable per-object ids could stabilize OUR
identities exactly where backlog #11's residual lives (angularly-
unresolvable targets) and in stationary clusters.

**Mechanism sketch.** Extend the PMBM source-aware identity machinery —
which already keys on mmsi/platform_id — with per-(source_id,
sensor_track_id) CONTINUITY as a third, deliberately weaker signal: an
association score bonus for id-consistent assignments (soft prior),
never a hard key (invariant 5: ARPA ids swap on crossings; camera ids
switch on occlusion — the evidence must lose to kinematics when they
disagree).

**Prereqs.** (a) Camera fixture pipeline currently emits per-frame YOLO
detections with NO ids — needs a tracking pass (offline, cheap);
`EoIrAdapter`'s detection struct already carries `sensor_track_id`.
(b) A measurement of how often camera ids survive occlusion on our
clips, BEFORE trusting them with weight.

**Trigger.** After the corroboration/steady-state line closes; natural
pairing with the camera-axis follow-ups (backlog #17). Evaluate on the
close-moored-boats case + backlog #11's sc5-style scenarios.

---

## 20. Target-reported kinematics (AIS SOG/COG/heading/nav-status; TTM speed/course)

**STATUS — SHIPPED 2026-07-05 (3 increments, full suite 1022/1022).**
- **Increment 1 (AIS self-report + veto data path):** AisDynamicReport carries
  sog/cog/heading/nav-status; AisAdapter parses heading→`hints.heading_deg`
  (attribute) and nav-status→`hints.nav_status`, dropping AIS sentinels. Both
  surface last-write-wins through the R11 attribute sites (estimators, MHT
  tree_attributes, PMBM Bernoulli+Acc). Nav-status veto: ILiveOccupancyFeed::
  observeVesselFix gained a kind-agnostic `anchored` flag (PmbmTracker translates
  nav-status 1/5); an anchored fix is held for the longer
  `anchored_veto_window_s` (600 s) so a sparsely-reporting anchored vessel is
  never re-suppressed (ADR 0002 / R3 finally has a data path).
- **Increment 2 (SOG/COG measurement content):** AisAdapter emits
  PositionVelocity2D when SOG≥threshold & COG present, COG down-weighted at low
  SOG (falls back to Position2D below threshold; polar-Jacobian velocity cov +
  isotropic floor). New AisAdapterConfig. Bench/sim-safe (no existing caller
  sets SOG).
- **Increment 3 (ARPA TTM, our-own-smoothed-data half):** TTM speed/course →
  one-shot `hints.birth_velocity_enu` (consumed only at estimator initiate,
  never recurring — guide §3), plus a swap diagnostic
  (`hints.sensor_track_id_suspect` on a discontinuous course jump; diagnostic
  only, association doesn't consume sensor_track_id). ArpaAdapterConfig knobs
  `seed_birth_velocity_from_ttm` / `swap_course_jump_deg` / `swap_min_speed_mps`.
  Stabilisation (ground-vs-water course) flagged as a deployment confirm.
Docs: guide §3/§4/§6/§7 + appendix; output-contract attributes. The original
ticket follows.

**What.** We currently DROP most of what targets report about themselves:
`AisDynamicReport` carries only position+accuracy (no SOG/COG/heading/
nav-status); ArpaAdapter ignores TTM speed/course fields. Split by the
independence rule (guide §3 "derived data and double-counting"):
- **AIS (independent witness — the target's own GPS/gyro):** parse SOG/
  COG/heading/nav-status. Uses: heading as a TrackAttribute + output
  (fills "stationary, direction undefined" — anchored vessel points
  somewhere even when COG is garbage); **nav-status (1=anchor, 5=moored)
  as veto/corroboration input** (the R3 "nav-status → vessel, never
  suppress" rule finally gets a data path); SOG/COG as measurement
  content (PositionVelocity2D) with COG down-weighted at low SOG.
- **ARPA TTM speed/course (our own data smoothed — never a recurring
  measurement):** one-shot velocity prior at track birth (then
  discarded); course-vs-ours swap diagnostic (distrust sensor_track_id
  when they diverge hard). Mind stabilization (ground vs water course).
**Effort.** ~1–2 days. Consumer surface ⇒ guide + output-contract sync.
Raised 2026-07-04, queued 2026-07-05 (pre-water window).

## 21. Benchmark-config fragility: harbor/pmbm_adapt_k3 knife-edge birth decision

**Raised 2026-07-06** during perf round 3 (hot-path mechanical sympathy). Same
class as the freeze-rule fragile-assertion problem — this is a *benchmark
config* fragility, not a tracker defect.

**Problem.** On `harbor_complete_truth` the `imm_cv_ct_pmbm_adapt_k3` config has
a cardinality/birth decision sitting on a knife edge that a **1e-15**
floating-point perturbation flips: any epsilon-class change to the estimator hot
path (the perf round-3 state-path single-decomposition and 2×2 kernels, and any
future one) tips it, moving `card_err_mean` 9.85↔9.375 and `gospa_false`
2005↔1910 with a cascade into that config's derived per-truth metrics. It is
isolated to this one config — `dense_clutter_datum` (all configs), the `philos`
KEEP replays, and all other harbor configs stayed byte-identical under the same
changes — so it is a single borderline decision, not drift. It will re-flip
under every future epsilon-class change (and the clutter/birth-model campaign
will run many), each time re-raising a spurious "did the math change?" question.

**Fix direction (decided later, not by the perf tickets).** Either make the
scenario's assertion robust (tolerance / cardinality band instead of a pinned
point, per the freeze-commit rule), or nudge that config's birth threshold off
the borderline so the decision is not epsilon-sensitive. Cheap; do it before
the clutter/birth investigation so its replays don't trip on this.

---

## 22. Parallelize the PMBM hot loop (design item — determinism is the hard part)

**Problem.** The tracker is single-threaded (99% of one core). After the
2026-07-06 perf arc (Murty K=1 early exit 12.4×; mechanical sympathy 1.5–1.66×)
the remaining per-scan cost is the cost-matrix / per-mode measurement update,
which is embarrassingly parallel across (Bernoulli, measurement) pairs.
Plausible win on deployment-class multicore hardware: 3–6×. This is the last
big *mechanical* lever; everything cheaper has been done or priced
(see `docs/baselines/2026-07-06_perf_round3.md`).

**Why it is a DESIGN item, not a perf fix.** Two invariants stand in the way:

1. **Deterministic replay (architecture invariant 4).** Naive parallel
   reduction reorders floating-point sums → same log no longer replays to
   identical output. Any design must make merge order deterministic
   (fixed-partition scheduling, ordered reduction, or per-pair results
   written to indexed slots and reduced sequentially — the last one is the
   simple safe shape: parallelize the *independent* work, keep every
   *combining* step sequential and ordered).
2. **Library thread policy.** navtracker is a library; consumers must control
   threading (embedders may forbid spawned threads, or want to supply their
   own executor). Concurrency must be per-instance, ctor-threaded
   configuration (never global — same rule as all toggles), default =
   single-threaded = today's behavior bit-identical.

**Proposal sketch (when triggered).** Parallel map over cost-matrix rows into
preallocated indexed storage; sequential ordered reduction; worker count a
per-instance config (0/1 = off). Determinism test extended: same log, N
threads vs 1 thread → identical output (this becomes a standing gate).

**Trigger.** A measured deployment compute budget that the single-threaded
tracker fails — NOT speculative. Deployment-representative feed currently
runs at 10× realtime margin single-threaded; nothing is throttled. If
triggered: fresh profile first (round-3 changed the hot path; its profile is
already stale), then the lossless gate prefilter (cheap distance bound
skipping hopeless pairs — likely 1.3–1.5× at high density) BEFORE reaching
for threads.

Raised 2026-07-06 (post-round-3 review: "sure there are no more performance
gains?"). Cross-ref: north-star runtime row; `docs/baselines/2026-07-06_perf_round3.md`.

**#20 follow-up (2026-07-06, from the philos farcross pass):** the SOG/COG
velocity path (increment 2) is unreachable via replay — `loadAisCsv` emits
Position2D only and drops the SOG/COG columns the fixtures carry; #20 lives in
`AisAdapter` (NMEA path). Follow-up: populate `AisAdapterConfig`-equivalent
velocity emission in the replay loader behind a default-off toggle
(default = bit-identical), so the increment-2 path gets its first real-data
exercise. Not bit-identical when ON → prices through the standing gates.
Natural companion to the multi-sensor sim Layer-2 AIS emitter (which will
also carry SOG/COG).

**#20 follow-up DONE (2026-07-06, branch `ais-loader-velocity`):** loadAisCsv
now emits PositionVelocity2D + nav_status behind a default-off toggle, sharing
the polar-velocity math with AisAdapter via `core/estimation/PolarVelocity.hpp`
(no duplicated Jacobian). Default-off proven byte-identical. FIRST honest-truth
measurement of the velocity path (sim gates): it is NOT a promotion candidate —
cuts MHT id-switches on maneuvering targets but REGRESSES continuity (track
breaks / lifetime / OSPA) broadly, worst on clean head-on (0.5→30.5 breaks); no
id-switch benefit for PMBM. See eval-log 2026-07-06. Open sub-items surfaced:
(a) root-cause the continuity regression (hypothesis: velocity R too tight vs
sparse AIS + noisy radar position); (b) nav_status-gated velocity suppression
(force Position2D when nav_status ∈ {1,5}) so anchored vessels stay inert — the
loader now surfaces nav_status to enable it.
