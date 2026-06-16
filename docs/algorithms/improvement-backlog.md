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

## 9. Inter-sensor registration biases — DONE (2026-06-15)

**Problem.** Only own-ship heading bias is estimated. Radar↔lidar↔
camera mounting offsets / range biases are unmodelled; on AutoFerry
they fold into pos_rmse and gate sizes.

**Change shipped.** Per-(sensor, source_id) bias filters fed by
AIS-anchored cross-sensor pair extraction. Position bias (2-D) on
radar / lidar; bearing bias (scalar) on EO / IR cameras. Random-walk
dynamics with very small Q. Three observability gates (time, range,
innovation) modelled directly on `HeadingBiasEstimator` G1-G2-G3.
Deterministic shift application — Schmidt-KF "considered" treatment
of `P_b` in the per-track update remains in `sota-roadmap.md` §5,
deferred until pos_rmse plateaus.

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

## 13. Cross-sensor anchored bias (AIS-less deployment)

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
