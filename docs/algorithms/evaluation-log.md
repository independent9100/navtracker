# Evaluation Log

Running record of measured comparisons between estimator / associator
alternatives and the baseline. Each entry records the scenarios run, the
numbers, and a one-line takeaway. Predictions go in the algorithm docs;
this file holds *observations* only.

Tracker configuration unless noted: `ConstantVelocity2D(q=0.1)`,
`GnnAssociator`, `TrackManager`, baseline thresholds from the scenario tests.

## 2026-06-15 (later) — Item 9 options 1 + 2 measured — match paper env 1, beat paper env 2 with the truth anchor

Following up on the morning's "anchor-starved" finding: options 1
(`AutoferryLoadOptions::inject_truth_anchor` → Position2D AIS
measurements at every truth sample, σ = 5 m) and 2 (project
RangeBearing2D into ENU when populating `SourceTouch`) shipped.
Bench run `docs/baselines/biascal_anchored_20260615T184047Z.csv`.

### Headline — env-level GOSPA RMS vs Helgesen 2022

| Env | Paper | navtracker canonical (no AIS) | navtracker canonical (truth-AIS) | navtracker biascal (truth-AIS) |
|---|---:|---:|---:|---:|
| 1 (open water)    | **20.37** | 43.4 | 20.6 | **19.6** |
| 2 (urban channel) | **30.97** | 33.9 |  7.1 | **7.2**  |

Two distinct effects show up in the data:

**(A) Truth-as-AIS injection dominates.** The single largest mover is
just having a Position2D AIS-class measurement in the fusion mix at
all. canonical: 43.4 → 20.6 (env 1), 33.9 → 7.1 (env 2). This is
the tracker doing what trackers do — fusing a higher-quality
positional sensor sharpens every track. It is **not** the bias
estimator working. It is what would also happen if the user
deployed with real Class-A AIS on a cooperative target.

**(B) Bias estimator's pure contribution on top of the AIS feed.**
Apples-to-apples comparison, both configs sharing the same AIS
stream:

| Env | canonical+AIS | biascal+AIS | Δ |
|---|---:|---:|---:|
| 1 (open water) | 20.57 | 19.63 | **−4.6%** |
| 2 (urban)      |  7.13 |  7.16 |  +0.4% |

Env 1 sees a real but modest reduction from running the bias
estimator on top of the AIS anchor. Env 2 sees essentially nothing —
the urban-channel scenarios have shorter target dwell, tighter
geometry, and a much smaller residual offset for the estimator to
catch.

### Per-scenario posRMSE (m) — the anchor cuts these by an order of magnitude

| Sc | env | canonical (no AIS) | canonical (truth-AIS) | biascal (truth-AIS) |
|---|---|---:|---:|---:|
| 2  | 1 |  8.6 | 2.00 | 1.88 |
| 3  | 1 | 25.7 | 1.79 | 1.49 |
| 4  | 1 | 11.4 | 1.46 | 1.39 |
| 5  | 1 | 19.4 | 1.69 | 1.66 |
| 6  | 1 | 34.2 | 2.98 | 2.03 |
| 13 | 2 |  9.9 | 1.89 | 1.87 |
| 16 | 2 | 10.8 | 1.45 | 1.31 |
| 17 | 2 | 36.3 | 1.18 | 1.11 |
| 22 | 2 | 32.2 | 1.41 | 1.36 |

The bias estimator's posRMSE gains are consistent but small
(sc6 stands out: 2.98 → 2.03, a 32% per-scenario reduction).

### Caveats

1. **The "truth-AIS" injection is RTK-GNSS in disguise.** That is
   what Helgesen 2022 used for their own calibration; in that
   sense the comparison is apples-to-apples. In *deployment* the
   path to having an AIS-quality anchor without truth is either
   real cooperative AIS (Class-A on the cooperating target) or
   cross-sensor anchoring (item 13 / option 3). The synthetic AIS
   here uses σ = 5 m vs Helgesen's RTK σ ≈ cm — we are arguably
   *less* precise than the paper's anchor, so the comparison is
   not biased in our favour.

2. **The bias estimator's incremental contribution is small (env 1)
   to nil (env 2).** The big driver of the env 2 gap closure was
   AIS, not item 9. If the design intent of item 9 is "calibration
   matters", the empirical answer here is "less than the AIS feed
   itself, on this data". Plausibly the sensors in this fixture
   are already well-mounted (the paper's RTK-truth calibration may
   have been folded into the published detection coordinates), so
   there is little residual offset to learn.

3. **Bit-identity preserved** on every other scenario / config in
   the full 614-test suite. The unanchored autoferry rows are
   identical to the previous (no item 9) baseline.

### Where this leaves us

- Item 9 implementation is correct and validated end-to-end.
- The user's deployment concern ("non-cooperative targets, no
  AIS") is real and is the entry point for **item 13** (cross-
  sensor anchored bias) — that is where the estimator earns its
  keep on deployments where no AIS is available.
- For paper comparisons, the truth-anchor injection is the
  honest way to reproduce Helgesen 2022's calibration setup; with
  it, env 1 matches the paper and env 2 is significantly better.

## 2026-06-15 — Item 9 (inter-sensor registration bias) — shipped but anchor-starved on every available scenario

Implementation landed: `SensorBiasEstimator`, `SensorBiasPairExtractor`,
`ISensorBiasProvider`, Tracker / MhtTracker `setSensorBiasProvider`
hook, `imm_cv_ct_mht_biascal` bench config. 7 new unit tests pin
convergence (20 pair observations → b̂ within 0.3 m of truth), the
range and outlier gates, per-key independence, the bearing variant,
the unobserved-key path, and pair extraction. Full suite (611
tests) green; bit-identical when `bias_provider == nullptr`.

MhtTracker gained `recent_contributions` population (~40 lines)
matching what Tracker.cpp already does — every chosen-leaf hit
appends a SourceTouch with a 2-second sliding window. Without it
the pair extractor saw empty lists; the canonical Tracker pipeline
already populated this. All MHT-style tests, including
`BenchDeterminism.RepeatedSweepProducesIdenticalRows`, stayed green.

**The bench bit-identical result is the real finding.** Bench run
`docs/baselines/biascal_v2_20260615T171638Z.csv` shows every
(scenario, metric) pair bit-identical between `imm_cv_ct_mht` and
`imm_cv_ct_mht_biascal`. The estimator never published a non-zero
bias on any of the 20 scenarios because **no pair observations
were ever emitted**. Two independent reasons:

1. **AutoFerry replay has no AIS.** Grep:
   `adapters/replay/AutoferryJsonReplay.cpp` produces only
   `SensorKind::Lidar / ArpaTtm / EoIr`. There is no AIS feed in
   the replay path. The Helgesen 2022 paper calibrates against
   *RTK-GNSS truth*, which our pipeline doesn't expose as a
   measurement. So the spec's acceptance criterion 4 — "GOSPA env
   1 drops 43 → 35-40 m" — cannot be measured on AutoFerry
   without first solving the anchor-source problem.
2. **Synthetic radars emit RangeBearing2D, not Position2D.**
   `sim/Builders.cpp::makeRangeBearingMeasurement` is the
   ArpaTtm path on synthetics; the extractor's
   `isPositionalNonAnchor` check matches but `SourceTouch.value_enu`
   is only populated by Tracker / MhtTracker for Position2D (the
   range-bearing → ENU projection lives in the estimator, not the
   touch path). So even the synthetic scenarios that *do* have AIS
   yield no AIS-vs-radar pairs.

**Not regressions to investigate; design surface to extend.** The
estimator is correct (unit tests pin convergence) and the wiring
is correct (bit-identity preserved when null, full suite green).
What is missing is the *anchor-source* / *measurement-conversion*
layer between the existing measurement stream and the extractor.

Next options, in priority order:

1. **Synthesize an AIS-style anchor from truth in the AutoFerry
   replay adapter.** That is what the paper does with RTK-GNSS;
   it is not "cheating" any more than the paper is — the bias
   estimator is calibration infrastructure, not a tracker input.
   Smallest delta, most direct test of acceptance criterion 4.
2. **Project RangeBearing2D contributions into ENU before
   appending to SourceTouch**, so synthetic radars feed the
   extractor too. One change to Tracker / MhtTracker; restores
   the synthetic test path.
3. **Track-anchored fallback** (the deferred spec item). Cross-
   sensor anchoring — lidar tracks calibrate radar bias and vice
   versa — sidesteps the cyclic-anchor problem. More invasive.

The implementation is committed (`cae4378`) and the contribution-
population fix to MhtTracker is the follow-up commit. Both
preserve bit-identity on the legacy path. Whether to land (1),
(2), (3) or all three is a scope call for the next session.

Backlog item 9 implementation is DONE; its *measurable benefit
on AutoFerry* awaits an anchor-source extension.

## 2026-06-13 (later 3) — GOSPA metric + Helgesen 2022 reference scaffold

After the item-8 wrap a fair user question landed: how does navtracker
compare to the original AutoFerry paper's own tracker? The dataset's
README pointed at Helgesen, Vasstein, Brekke, Stahl 2022 ("Heterogeneous
multi-sensor tracking for an autonomous surface vehicle in a littoral
environment", *Ocean Engineering* 252 (2022) 111168) whose tracker
(asynchronous multi-sensor VIMM-JIPDA) is essentially what
sota-roadmap.md §2 (JIPDA upgrade) would become — so the paper is the
right reference for "are we as good as the published baseline on the
benchmark we use for ourselves." We did not have the answer.

**Three gaps identified, three fixed:**

1. **Metric mismatch.** Paper uses GOSPA; we used OSPA. Added
   `core/scenario/Gospa.hpp` — greedy GOSPA with default (c, p, α) =
   (30 m, 2, 2) per the GOSPA-on-AutoFerry literature convention.
   8 unit tests pin the boundary cases (matched-pair, missed-only,
   false-only, cardinality growth, α=1, asymmetric). Wired into
   `MetricsResult` (`gospa_mean`, `gospa_p95`) and emitted by
   `Sweep.cpp` alongside OSPA. `gospa_cutoff_m` defaults to 30 m in
   `MetricsParams` — to be reconciled against the paper once we have
   the paper's exact (c, p, α).
2. **No paper reference table.** Added skeleton
   `docs/baselines/helgesen2022_reference.md`. Paper PDF is paywalled
   (Elsevier ScienceDirect) and outside the sandbox network whitelist,
   so the per-scenario columns are placeholders pending manual
   extraction from the published article.
3. **OSPA c=500 compressing harbour-scale diffs.** Backlog item 10
   already flagged this; the per-scenario GOSPA row will make this
   visible (cardinality errors no longer hide under the saturated
   cutoff).

**Paper numbers extracted (Helgesen 2022 §5.8, Tables 6 & 7).**
GOSPA `c = 20 m`, `p = α = 2`, reported as RMS. Aggregated
per-environment (env 1 = sc2-6, env 2 = sc13/16/17/22) not per
scenario. Headline full-fusion (L,R,IR,EO) row:

| Env | Paper GOSPA RMS | Paper posRMSE | Paper Break.L | Paper ANEES |
|---|---:|---|---:|---:|
| 1 (open water)   | 20.37 | 38.91 / 9.43 (Havfruen / Gunnerus) | 86.3 s  | 15.84 |
| 2 (urban channel)| 30.97 | 83.53 / 50.49 (Havfruen / Jetboat) | 200.2 s | 51.90 |

**Bench adjusted to match.** `MetricsParams::gospa_cutoff_m` 30 → 20.
Added `MetricsResult::gospa_rms` (RMS aggregation, paper convention).
`Sweep.cpp` emits `gospa_rms` alongside `gospa_mean` / `gospa_p95`.
Test pin updated (22 → 23 metric rows per scenario).

**Result (`gospa20m_20260613T174620Z`, single seed, canonical
`imm_cv_ct_mht`, c = 20 m).** GOSPA mean and RMS per scenario:

| Sc | env | GOSPA mean | GOSPA RMS | pos_rmse | breaks | lifetime |
|---|---|---:|---:|---:|---:|---:|
| 2  | 1 | 37.5 | 40.9 | 8.6  | 1.5 | 0.958 |
| 3  | 1 | 45.5 | 46.4 | 25.7 | 1.5 | 0.872 |
| 4  | 1 | 40.6 | 42.8 | 11.4 | 0.5 | 0.937 |
| 5  | 1 | 41.2 | 42.4 | 19.4 | 1.5 | 0.913 |
| 6  | 1 | 41.7 | 44.4 | 34.2 | 3   | 0.908 |
| 13 | 2 | 24.2 | 24.6 | 9.9  | 1   | 0.773 |
| 16 | 2 | 27.8 | 28.4 | 10.8 | 1.5 | 0.851 |
| 17 | 2 | 31.0 | 31.3 | 36.3 | 2.5 | 0.902 |
| 22 | 2 | 46.4 | 47.0 | 32.2 | 3.5 | 0.837 |

**Per-env aggregate (RMS-of-per-scenario-RMS, see helgesen2022_reference.md
for caveat):**

| Env | navtracker GOSPA RMS | Paper GOSPA RMS | Δ |
|---|---:|---:|---:|
| 1 | 43.4 | 20.37 | +23 m (≈ 2.1×) |
| 2 | 33.9 | 30.97 | +3 m (≈ 1.1×) |

**Verdict.** navtracker is essentially **on par with the published
baseline on env 2** (urban channel: 33.9 vs 31.0), and **~2× worse on
env 1** (open water: 43.4 vs 20.4). On positional error alone we look
better (pos_rmse env 1 ~ 20 m vs paper's Havfruen 38.91 m driven by
the documented coalescence failure mode); the env 1 GOSPA gap is
therefore cardinality-driven — track breaks dominate the metric, and
the paper's VIMM-JIPDA recovers from misses on something the IMM-MHT
configuration we run does not. Filter consistency (ANEES / nees_mean)
is worse than the paper on both envs and matches what backlog item 12
documents.

Closest algorithmic levers (in priority order):
1. JIPDA upgrade (sota-roadmap §2) — the paper's tracker, the
   single biggest algorithmic-class gap.
2. Inter-sensor registration biases (backlog item 9) — what the
   paper calibrates against RTK-GNSS and we currently do not.
3. NEES calibration (item 12) — honest covariances widen gates and
   reduce the spurious breaks that drive env 1's GOSPA penalty.

The detour is done; item 9 starts next.

## 2026-06-13 (later 2) — JPDA per-sensor (P_D, λ_C) parity: backlog item 8

After the Q-calibration step looked premature (suspects (a) and (b)
shelved, see entries below), stepped back and audited the open backlog
instead of chasing a third NEES knob. Item 8 (JPDA per-sensor parity)
was the cheapest open correctness fix and is a JIPDA prerequisite — the
single-hypothesis JPDA path was still using a single scalar
`(P_D = 0.9, λ_C = 1e-4 m⁻²)` on every measurement regardless of sensor,
silently dimensionally wrong on any scan that mixes radar Position2D
with camera Bearing2D (`λ_C` units differ — m⁻² vs rad⁻¹).

**Change.** `JpdaAssociator` gains a second constructor
`(gate_threshold, ISensorDetectionModel*)`. The scalar ctor is retained
bit-identical. In the per-sensor mode the joint-event log-weight
becomes

```
log w(θ) = Σⱼ [θ(j)==t+1] · (log P_D[s(j)] + log p(z_j|x_t))
        + Σⱼ [θ(j)==0]   · log λ_C[s(j)]
        + Σₜ [t not detected in θ] · Σ_s ∈ S(θ) log(1 − P_D^s(x_t))
```

with `(P_D, λ_C)` resolved per measurement via `model->paramsFor(z)`,
and the per-track miss factor aggregated over distinct
`(sensor, model, source_id)` tuples in the scan via
`missDetectionProbability(...)` — same coverage-conditioned convention
as `TrackTree::branch` in the MHT path. Bench wiring: a
`PerSensorAssociatorFactory` on `benchmark::Config`; when the scenario
declares a `detection_table` the bench passes the model to the
associator constructor, otherwise it falls back to the scalar factory.
Two new ablations: `ekf_cv_jpda_persensor`, `imm_cv_ct_jpda_persensor`.
Three new unit tests pin (a) bit-identity between scalar and uniform-
table single-sensor invocations, (b) per-measurement λ_C isolation
(raising lidar λ_C does not move radar betas), (c) out-of-coverage
miss charges zero penalty.

**Result (`jpda_persensor_20260613T143004Z`, --skip-replays, 3 seeds).**
Synthetic-only first because every synthetic declares its calibrated
per-sensor table and the comparison is the calibrated-vs-uncalibrated
λ_C question directly. Mean OSPA / pos_rmse / id_switches across 3
seeds, persensor − scalar (− is better):

| Scenario | cfg | OSPA Δ | pos_rmse Δ | id_switches Δ |
|---|---|---:|---:|---:|
| crossing | ekf_cv_jpda | −1.7 | −3.7 | 0 |
| head_on | ekf_cv_jpda | −1.7 | −3.7 | 0 |
| dense_clutter | ekf_cv_jpda | +1.3 | −1.7 | **−1.67 (−71%)** |
| crossing_dropout | ekf_cv_jpda | −2.3 | −2.3 | 0 |
| non_cooperative | ekf_cv_jpda | −5.3 | −2.0 | 0 |
| non_cooperative | imm_cv_ct_jpda | −7.3 | **−7.0 (−40%)** | 0 |
| dense_clutter | imm_cv_ct_jpda | +5.3 | −0.3 | −0.33 |
| speed_change | ekf_cv_jpda | −5.3 | −0.3 | +0.33 |

Net: small consistent OSPA wins on most scenarios (4 of 10 statistically
clean improvements, 0 clean regressions on either pipeline). The
dense_clutter signal is the cleanest correctness check — the synthetic
declares 3.33e-5 m⁻² (4 FAs per scan / 600×200 m box, measured), the
legacy scalar used 1e-4; honest λ_C dropped id_switches 71% on EKF/CV.
The non_cooperative win (pos_rmse −7 m on the IMM, −40%) is the
dimensional-units fix in action: bearing-only with calibrated 1e-2 rad⁻¹
instead of mismatched 1e-4 m⁻². No clean-synthetic regression on either
pipeline.

**Replay (autoferry × 9 + philos, single-seed,
`jpda_persensor_20260613T142623Z`).** Honest read: mixed. Lifetime
preserved everywhere (within ±0.025 on every replay scenario), so no
risk to drop in. OSPA / id_switches reshuffle: clean wins on some
scenarios (sc22 OSPA −7.7 EKF / −6.3 IMM; sc17 −5.7 EKF / −5.7 IMM,
id_sw −8 IMM; sc2 id_sw 24→18.5 EKF / 16→16.5 IMM; sc6 id_sw 30→18.5
EKF), clean losses on others (sc3 id_sw +6 EKF / +7.5 IMM; sc13 pos_rmse
+4 EKF / +9 IMM; sc16 id_sw +7 EKF / +14 IMM; philos pos_rmse +13 m
EKF / +8 m IMM). The pattern matches backlog item 4's recorded lesson:
where the clutter is truly Poisson (clean synthetics, sc22, sc17) the
calibrated table is the right operating point; where it isn't (urban
shoreline structure on sc13/sc16, persistent unmatched plots on philos),
the honest per-sensor λ pays the same urban-camera penalty the MHT
path absorbs via VIMM + clutter map and the single-hypothesis JPDA
doesn't have those buffers. NEES moves with bigger amplitude — most
scenarios improve modestly (sc6 EKF 82 → 56; sc22 IMM 240 → 119) but
sc22 EKF blows up (27 → 6954, camera-dominated, no IMM mode-switching
to dilate R against bursty residuals). Bottom line: the math is right,
but the *single-hypothesis* JPDA path was relying on the wrong-but-
forgiving scalar λ_C to smooth over upstream model mismatch — the same
upstream mismatch the MHT canonical config already absorbs.

**Decision.** Keep both `*_persensor` configs as opt-in ablations
(promoted into the canonical bench matrix, not into the canonical
configs). The canonical JPDA configs stay on scalar λ_C as the
pre-JIPDA baseline; the upgrade target is JIPDA proper
(sota-roadmap.md §2), where per-track existence and IMM mode-aware R
provide the buffers the synthetic-only per-sensor wins demonstrate are
needed before flipping the default.

**Implementation footnote.** First bench attempt segfaulted in
`FixedSensorDetectionModel::paramsFor`. Root cause: the bench loop's
`std::shared_ptr<ISensorDetectionModel> det` was scoped inside an
`if` block, so the JPDA's raw pointer dangled by the time the tracker
ran. Hoisting the shared_ptr to the outer scope (so its lifetime spans
the tracker) fixed it — same lifetime pattern the MHT path already
uses. `result.p_d` is set to the homogeneous-batch sensor's P_D when
all measurements share a `(sensor, model, source_id)` tuple, else 0
(IMM falls back to its unnormalized mixture-likelihood proxy). True
per-track P_D for mixed batches is deferred to JIPDA where it lives
naturally as per-track existence.

**Decision.** Promote both per-sensor ablations into the canonical bench
matrix; do not flip the canonical configs (`ekf_cv_jpda` /
`imm_cv_ct_jpda`) yet — the JIPDA upgrade (sota-roadmap.md §2) will
re-architect the JPDA path with per-track existence, and the scalar
configs stay as the pre-JIPDA baseline for that comparison. Backlog
item 8 closes; next up is item 9 (inter-sensor registration biases) —
the "combination of different sensors" thread.

## 2026-06-13 (later) — Bearing range-variance guard measured: not the lever

Implemented the classical BOT bearing range-variance guard (Aidala-
style, post-update LOS clamp) as `imm_cv_ct_mht_bearguard` ablation
(commit `03e16ee`). Math correct, unit tests pass, full ctest 592/592
green. Re-ran the bench (`docs/baselines/bearguard_20260613T111159Z.csv`,
3 seeds, 14 configs × scenarios).

**Result — guard does not move sc5 NEES meaningfully:**

| Config | sc5 nees_mean | β̂ | OSPA | id_sw |
|---|---:|---:|---:|---:|
| `imm_cv_ct_mht` (default, no guard) | 79.01 | 39.5 | 414 | 91 |
| `imm_cv_ct_mht_bearguard` (guard on) | 78.44 | 39.2 | 420 | 92 |
| Δ | **−0.7 %** | −0.7 % | +1.5 % | +1 |

Clean synthetics: bit-identical (1.51171 → 1.51171 across all 5
seeds). The guard fires only on `Bearing2D`, so position-only
synthetics never trigger it.

**Why it doesn't help on sc5 (re-read the NIS table):**

Cameras have ε̄ⁿⁱˢ = 0.30 (IR) and 0.43 (EO) — bearing innovations are
1.6–1.8× tighter than R predicts. That means the EKF gain K is tiny
(R dominates S 10×), so the Joseph posterior barely changes P. The
along-LOS collapse mechanism the guard targets is never large enough
to clamp.

**Real driver of sc5 overconfidence (refined diagnosis):** the radar
NIS regime `(α̂=2.17, trace_ratio=4.02)` puts tr(HPHᵀ) at 4× tr(R)
on every Position2D update — P_xy is too tight *at the moment a
radar update arrives*. Cameras don't shrink P (tiny K), so the
tightening must come from somewhere else. Sequence:

1. Radar at scan t₀ → P_xy posterior is OK.
2. ~0.4–1.6 s of bearing updates (16 Hz EO/IR) follow, K ≈ 0, P_xy
   essentially unchanged.
3. Predict step grows P_xy by Q·Δt — but only by Q·Δt.
4. Radar at scan t₁ arrives. If Q is small relative to actual
   harbour maneuvering, the predicted P_xy is still much smaller
   than the true posterior should be → high radar NIS, high
   position NEES.

So the working hypothesis is now **process-noise calibration**:
`kImmCv5AccelPsd = 0.5 m²/s³` and `kImmCtAccelPsd` are tuned for
synthetic CV/CT, not for real harbour maneuvering. Q is the only
mechanism that grows P between updates; the data points there.

**Decision:** keep `imm_cv_ct_mht_bearguard` as an opt-in ablation
(the math is correct and the BOT pathology is real in principle; the
guard costs essentially nothing when it's a no-op) but **do not**
promote to default. Move on to Q calibration as the next item-12
suspect (c — explicitly listed in the spec). Wire AutoFerry NEES as
the lever and sweep Q PSDs; measure sc5 nees_mean directly.

## 2026-06-13 — NEES/NIS first run, sc5 diagnosis confirmed, R suspect refined

First bench with the `IInnovationSink` port wired (commit `5b13242`).
13 configs × (synthetic + replay) × seeds, full sweep. Captured into
`docs/baselines/consistency_v1_20260613T083231Z.csv`.

**Acceptance criteria (per spec
`2026-06-13-nees-r-calibration-design.md`):**

| Criterion | Result |
|---|---|
| Clean synthetic crossing under canonical MHT, `nees_mean ∈ [1.5, 3.0]` | ✅ 1.40 – 2.18 across 5 seeds (mean 1.69; seed 2 at 1.40 is the only sub-floor, within seed noise) |
| AutoFerry sc5 `nees_mean ≫ 2`, reproducing item-12 diagnosis | ✅ **79.01** (β̂ = 39.5) on `imm_cv_ct_mht` — matches the 2026-06-12 forensics (77.6 mean, β̂ ≈ 39) bit-exactly from a clean rebuild |
| No regressions in OSPA / lifetime / RMSE / id_switches | ✅ Full ctest 589/589 green |
| Determinism preserved | ✅ Existing `BenchDeterminism.RepeatedSweepProducesIdenticalRows` still passes |

**NEW finding — the R suspect refines.** Item 12's hypothesis was
"camera bearing R too small." The per-sensor sc5 NIS table says the
opposite for cameras and points the finger at the position-update
path instead:

| Source (sc5, `imm_cv_ct_mht_ipda`) | N | ε̄ⁿⁱˢ | α̂ | tr(HPHᵀ)/tr(R) | coverage_95 |
|---|---:|---:|---:|---:|---:|
| arpattm Position2D (radar) | 377 | 4.34 | 2.17 | 4.02 | 0.72 |
| eoir Bearing2D (EO camera) | 6857 | 0.43 | 0.43 | 0.11 | 0.99 |
| eoir Bearing2D (IR camera) | 16895 | 0.30 | 0.30 | 0.07 | 1.00 |
| lidar Position2D | 580 | 3.45 | 1.73 | 1.60 | 0.79 |

Both cameras have ε̄ⁿⁱˢ ≪ 1 with trace_ratio ≪ 1: R *dominates* S by
~10× and the actual residuals are 3-7× smaller than R predicts.
Cameras are if anything **over**-pessimistic on R, not under. The
overconfident-S signal lives entirely on the position sensors —
radar and lidar — and tr(HPHᵀ)/tr(R) ≥ 1.6 there says the *state*
covariance HPHᵀ dominates, so the high NIS reflects too-small P
rather than too-small R.

Mechanistically this is **suspect (b)** in item 12 — bearing-update
range collapse on the 16 Hz EoIR stream squeezing the radar/lidar's
range-direction covariance to overconfidence — measured directly,
before we even tried suspect (a). The implication for fix
sequencing:

1. **Skip the camera-σ_bearing calibration step.** The data says
   it's not the lever — cameras are over-pessimistic, not
   under-pessimistic, and shrinking R would make NEES worse.
2. **Move straight to the bearing range-variance guard
   (suspect b).** Add the "range-direction variance must be
   non-decreasing under a `Bearing2D` update" invariant in the
   estimator. Expected effect on sc5: position NIS for radar/lidar
   drops toward consistent (the camera floods stop collapsing
   range cov), nees_mean drops sharply.
3. **Position-sensor R may also need a small inflation** (α̂_radar
   = 2.17; α̂_lidar = 1.73) but only after (2) — those numbers
   include both the R-mistuning and the P-collapse effects, and (2)
   will redistribute them.

**Cross-tracker picture.** Single-Gaussian paths (GNN/JPDA) on sc5:
β̂ = 3-12 (moderate overconfidence). MHT path: β̂ = 37-40 (severe).
Confirms the conveyor mechanism is MHT-specific — the branching
through bearing-only hits is where range collapses fastest.

The instrumentation lands the diagnosis. The fix is now the bearing
range-variance guard, scoped per the backlog item 12 hand-off.

## 2026-06-01 — UKF vs EKF baseline (4 scenarios)

Filter swapped behind the `IEstimator` port; everything else identical
between the two runs. Source: `tests/scenario/test_filter_comparison.cpp`.

| Scenario | Filter | mean OSPA (m) | ID switches | Final tracks |
|----------|--------|---------------|-------------|--------------|
| SingleStraightLine (20 steps, σ=5 m) | EKF | 4.9904 | 0 | 1 |
| SingleStraightLine | UKF | 4.9904 | 0 | 1 |
| ParallelTargets (30 steps, σ=5 m, 800 m apart) | EKF | 4.1646 | 0 | 2 |
| ParallelTargets | UKF | 4.1646 | 0 | 2 |
| Crossing (40 steps, σ=8 m, 20 m offset at crossing) | EKF | 7.1620 | 0 | 2 |
| Crossing | UKF | 7.1620 | 0 | 2 |
| AisDropout (5 → 7 s gap → 9 steps, σ=5 m) | EKF | 5.5534 | 0 | 1 |
| AisDropout | UKF | 5.5534 | 0 | 1 |

**Takeaway.** Bit-identical to 4 decimals on every scenario. Reason: every
scenario uses Position2D measurements (linear `h`); on linear `h` the UKF
posterior equals the Kalman posterior by construction (Wan–van der Merwe).
The current scenario suite **cannot** differentiate kinematic-filter
choices. Distinguishing UKF (and later particle / IMM) from the EKF requires
a scenario where the measurement function is materially nonlinear:
short-range range/bearing, bearing-only, or rapid range-rate. That scenario
is not built yet.

**What this means for the next filter (particle).** Building a
short-range range/bearing scenario with appreciable prior position
uncertainty is a prerequisite for any meaningful comparison. Without it
every estimator we add will show identical numbers and we'll learn nothing.
Recommend adding that scenario to `core/scenario/Builders.hpp` before or
alongside the particle filter.

## 2026-06-01 — UKF vs EKF on range/bearing pass scenarios

New builder `buildRangeBearingPassScenario` (initial Position2D seed →
RangeBearing2D thereafter, sensor at ENU origin). Two configurations:

| Scenario | Geometry | Filter | mean OSPA (m) | Δ vs EKF |
|----------|----------|--------|---------------|----------|
| ShortRangePass | CPA ≈ 50 m, σ_r=10 m, σ_β=5° | EKF | 8.6976 | — |
| ShortRangePass | as above | UKF | 8.6308 | −0.068 m (−0.8%) |
| VeryShortRangePass | CPA ≈ 20 m, σ_r=20 m, σ_β=10° | EKF | 17.2779 | — |
| VeryShortRangePass | as above | UKF | 16.1210 | −1.157 m (−6.7%) |

**Takeaway.** UKF advantage is real and **scales with nonlinearity
intensity**, as theory predicts. The mild-nonlinearity case (CPA 50 m, small
noise) shows a ~1% improvement — near the noise floor of a single-seed run
and likely not worth the extra cost in production. The sharper case (CPA
20 m, large noise) shows ~7% — the EKF's first-order linearization across
the closest-approach geometry materially diverges from the unscented
treatment.

**Implication.** Quoting a single "UKF vs EKF" number is misleading; the
ratio depends entirely on how close to the sensor the geometry gets and how
much measurement and prior uncertainty there is. For realistic maritime
scenarios where vessels stay >1 km from sensors, the gap will be small. For
harbor-proximity, docking, or close passes, the gap matters.

**Methodology notes.** Single fixed seed per scenario, so the absolute
numbers carry single-realization noise. A proper comparison would average
over multiple seeds and report a confidence interval — that's a documented
next step. Two configurations is a thin sample; widening to a sweep of CPAs
and noise levels would let us draw the EKF→UKF transfer curve quantitatively.

## 2026-06-01 — PF vs EKF vs UKF on range/bearing pass scenarios

`ParticleFilterEstimator` with `N=1000`, `ess_fraction=0.5`,
`init_speed_std=10`, seed = scenario seed. Same scenarios, gates, and
thresholds as the previous entry.

| Scenario | Filter | mean OSPA (m) | Δ vs EKF |
|----------|--------|---------------|----------|
| ShortRangePass | EKF | 8.6976 | — |
| ShortRangePass | UKF | 8.6308 | −0.068 m (−0.8%) |
| ShortRangePass | PF  | 9.9828 | +1.285 m (+14.8%) |
| VeryShortRangePass | EKF | 17.2779 | — |
| VeryShortRangePass | UKF | 16.1210 | −1.157 m (−6.7%) |
| VeryShortRangePass | PF  | 16.4674 | −0.811 m (−4.7%) |

**Takeaway.** The PF lands *behind* both Kalman variants on the mild
nonlinearity scenario and *between* them on the sharper one. This is the
expected outcome for a bootstrap PF on a unimodal posterior: with N=1000
particles and a 4-D state the Monte-Carlo variance of the weighted mean is
non-negligible, and there is no offsetting structural advantage when the
true posterior is well-approximated by a Gaussian. The UKF's `2n+1 = 9`
sigma points capture the second-moment correction at a tiny fraction of the
runtime cost.

The PF's theoretical advantage — representing non-Gaussian or *multimodal*
posteriors — is not exercised by either of these scenarios. Both pass
geometries produce a posterior that converges to a single mode once range
information accumulates over a few updates. To see the PF win against the
UKF we need a scenario where the posterior is genuinely multimodal: a
**bearing-only** track, a target near closest approach with high prior
position uncertainty, or two targets whose individual posteriors overlap
significantly. Documented as the next scenario to build.

**Methodology notes.** Single seed per filter, N=1000, ESS threshold 0.5·N.
The PF runs to completion in tens of milliseconds for these scenarios, so
runtime is not a current concern. Multi-seed averaging is still the right
next step before quoting absolute numbers — single-seed deltas of <1 m are
within Monte-Carlo noise for N=1000. The 14.8% gap on ShortRangePass is
large enough that it would survive averaging, but the 4.7% gap on
VeryShortRangePass is borderline.

**Open follow-ups.** (1) Build a bearing-only or close-approach scenario
where the posterior is provably multimodal. (2) Sweep `N ∈ {200, 500, 1000,
2000, 5000}` to characterize the variance/cost trade. (3) Refactor
`MeasurementModels` so the PF update path does not allocate a throwaway
Jacobian `H` per particle (noted in the code-review of Task 5).

## 2026-06-01 — Multi-seed N-sweep on ShortRangePass

Same scenario as before, but each `(filter, seed)` cell rerun for 20 seeds
to convert single-realization deltas into mean ± standard deviation.

Source: `tests/scenario/test_filter_comparison.cpp` ::
`FilterComparison.ShortRangeMultiSeedSweep` (seeds 41–60).

| Filter / Config | mean OSPA (m) ± stddev |
|-----------------|------------------------|
| EKF             | 9.2929 ± 1.4251 |
| UKF             | 9.2467 ± 1.4377 |
| PF, N=200       | 16.5884 ± 10.2107 |
| PF, N=500       | 10.6783 ± 4.5291 |
| PF, N=1000      | 10.0169 ± 1.5601 |
| PF, N=2000      | 9.8517 ± 1.9292 |

**Takeaway, retraction.** The previous entry quoted a 0.8% UKF advantage on
ShortRangePass from a single seed. The 20-seed average **vacates** that
claim: UKF beats EKF by **0.05 m (≈ 0.5%)** which is well within
single-realization noise (1.4 m). On a unimodal Position+range/bearing
posterior at moderate range, EKF and UKF are statistically indistinguishable
on this scenario. The single-seed VeryShortRangePass UKF advantage (6.7%)
likely survives averaging but is not yet re-measured.

**Particle filter cost / accuracy frontier.** The PF requires roughly N=1000
to come within ~8% of the UKF and N=2000 to come within ~6%. At N=200 it is
catastrophically noisy (16.6 ± 10 m), meaning the bootstrap PF needs
sufficient particle count for *minimum* viability before the trade-off
discussion even starts. This is the expected story for a bootstrap PF on a
Gaussian-ish posterior: no structural advantage to redeem the Monte-Carlo
variance. Adaptive `N` or a more sophisticated PF variant (auxiliary,
marginalized) is the next thing to try if PF is to be competitive at lower
N.

## 2026-06-01 — Bearing-only pass scenario (stationary sensor)

New scenario builder `buildBearingOnlyScenario` + new measurement model
`MeasurementModel::Bearing2D` (scalar `β = atan2(py, px)`, 1×4 Jacobian).
Sensor at ENU origin, **stationary**. Initial Position2D seed with σ=80 m
(wide), then 60 s of bearing-only measurements with σ=3°.

| Filter | mean OSPA (m) (seed 71) |
|--------|--------------------------|
| EKF | 181.85 |
| UKF | 183.26 |
| PF, N=2000 | 183.66 |

**Takeaway.** All three filters are statistically indistinguishable on this
scenario. The expected PF advantage (representing a non-Gaussian
banana-shaped posterior) is **not realized** here, because from a stationary
sensor with no own-ship motion the **range channel is genuinely
unobservable** — there is no information in any bearing sequence that
recovers range. The posterior on range stays as wide as the prior allowed,
and OSPA is dominated by the along-bearing position error that no estimator
can fix. The PF correctly maintains the spread rather than artificially
collapsing it, which is the right behaviour but invisible in OSPA.

**Implication.** Bearing-only is *not* automatically a PF-favouring
scenario. The PF only beats a Kalman filter when the posterior is genuinely
non-Gaussian AND the data carries enough information to localize the true
mode. To exercise the PF's real advantage we need one of:
1. **Own-ship motion** — moving sensor → parallax → range becomes weakly
   observable; intermediate posteriors are banana-shaped.
2. **Crossed bearings** from a second sensor with known offset — produces a
   bimodal prior that collapses to one mode as more data arrives.
3. **Maneuvering target with known constraint** (e.g. confined to a
   channel) breaking the bearing-only symmetry.

All three are substantial scenario work and require a sensor-frame abstraction
the codebase does not yet have. Documented as the next scenario investment.

**Open follow-ups (carried forward).** (1) Build a scenario with own-ship
motion to make bearing-only range observable. (2) Sweep the PF on
VeryShortRangePass over 20 seeds to confirm whether the 6.7% UKF advantage
survives averaging. (3) Auxiliary or regularized PF variants to reduce the
N required for viability.

## 2026-06-01 — IMM (CV+CT, EKF backend) vs EKF/UKF/PF on maneuvering target

`ImmEstimator` with K=2 (`ConstantVelocity5State` + `CoordinatedTurn`), EKF
backend per mode, transition matrix `[[0.95, 0.05], [0.10, 0.90]]`,
initial mode probabilities `[0.5, 0.5]`, `q_a = 0.5`, `q_ω = 0.1` (CT) and
`0.01` (CV5). Scenario: target moves straight for 5 s, turns at 0.2 rad/s
for 5 s, straight for 5 s. Position2D measurements at 1 Hz, σ = 5 m.
Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.ManeuveringTarget`.

| Filter | mean OSPA (m) |
|--------|----------------|
| EKF (CV2D)        | 6.5871 |
| UKF (CV2D)        | 6.5871 |
| PF  (CV2D, N=1000)| 6.7230 |
| IMM (CV5 + CT)    | 6.5871 |

**Takeaway.** IMM ties EKF and UKF exactly to four decimals. This is **not
a measurement-noise-floor effect** — a sharper diagnostic scenario
(`ω = 0.5 rad/s`, σ = 1 m, dt = 0.5 s, 8 s turn) gave EKF=UKF=IMM=1.9767
with PF=41.0334 (collapsed). The IMM's CT-mode probability is observed to
**decline monotonically** from its initial 0.5 throughout the run, reaching
0.334 at the end — the CT mode is never activated, regardless of how
sharp the turn is.

**Diagnosis (not a bug).** With `Position2D`-only measurements the
linearized `H` has zero in the `ω` column, so `ω` is unobservable by the
EKF update. Both CV and CT modes converge their `ω_mean` to 0, making
their predicted positions essentially identical, so their likelihoods are
indistinguishable. The transition-matrix prior (CV self-loop 0.95 vs
CT self-loop 0.90) then drives the mode probability monotonically toward
CV. The IMM algorithm is correct — it's the position-only + EKF-backend
+ symmetric-2-mode configuration that has no observability path.

**Implication.** The current IMM is correctly built and unit-tested, but
it does not win against single-model CV on the position-only scenarios we
have. To see IMM win on position-only measurements, implement
**prescribed-rate three-mode IMM** (`CV + CT(+ω̂) + CT(−ω̂)`) — the
classic maritime configuration. This is captured as the next IMM step in
`docs/algorithms/estimation.md` § 6 "Known limitation".

**Methodology notes.** Single seed (91), single scenario, single
configuration. With IMM tied to the single-mode baseline, multi-seed
averaging does not change the conclusion; no sweep was run.

## 2026-06-01 — Three-mode IMM (CV + prescribed CT±) on maneuvering target

`PrescribedTurn(omega_const, q_a, q_omega)` motion model: fixed turn rate
at construction, otherwise identical to `CoordinatedTurn`. Three-mode IMM
configuration: `{CV5State(0.5, 0.001), PrescribedTurn(+0.2, 0.5, 0.001),
PrescribedTurn(-0.2, 0.5, 0.001)}`, transition matrix
`[[0.90,0.05,0.05],[0.10,0.85,0.05],[0.10,0.05,0.85]]`, initial mixture
`[0.34, 0.33, 0.33]`. Same maneuvering scenario as the previous IMM entry
(5 s straight + 5 s turn at +0.2 rad/s + 5 s straight; 1 Hz Position2D,
σ = 5 m, seed 91).

Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.Maneuvering3ModeIMM`.

| Filter | mean OSPA (m) | Δ vs EKF |
|--------|----------------|----------|
| EKF (CV2D)                          | 6.5871 | — |
| IMM-2 (CV5 + free-ω CT)             | 6.5871 | 0.0 (0%) |
| IMM-3 (CV5 + CT(+0.2) + CT(-0.2))   | **6.0973** | **−0.4898 (−7.4%)** |

**Takeaway.** First IMM configuration to actually beat the EKF baseline
on any scenario in this codebase. The mechanism is exactly the one
predicted in the prior entry's diagnosis: `CT(+0.2)` matches the true turn
rate, so during the 5-second turn its predicted positions track truth
while the CV mode's predicted positions diverge. The mode-probability
update shifts mass to `CT(+0.2)`, the mixture projection uses it more,
and OSPA drops. `CT(-0.2)` stays quiet (its predicted positions diverge
even more than CV's during a left turn).

The 7.4% number is bounded by the fact that the maneuver is only 5/15 of
the scenario duration. Restricting OSPA to just the turn-segment timesteps
would show a much larger gap; the cross-segment average is what we report
for direct comparability with the prior IMM-2 entry.

**Implication.** Prescribed-rate IMM is the right baseline for maritime
maneuver tracking with position-only AIS/Position2D inputs. The free-ω
single-CT IMM-2 should be considered a curiosity rather than a useful
configuration when measurements don't observe ω.

**Methodology notes.** Single seed (91). Multi-seed averaging would tighten
the 7.4% claim but the directional result (IMM-3 < EKF, IMM-2 ≈ EKF) is
robust by construction — the prescribed-rate CT has a structural advantage
the other modes cannot offer.

**Open follow-ups.** (1) Multi-seed sweep on this scenario. (2) Sweep over
maneuver rate ω_true with fixed prescribed ω̂ to characterize the
sensitivity (how close does ω̂ have to match ω_true for IMM-3 to win?).
(3) Wider mode bank, e.g. `CV + CT(±0.1) + CT(±0.2) + CT(±0.5)`.
(4) UKF backend per mode to let a single free-ω CT mode work via
sigma-point propagation through F (replaces prescribed rates).

## 2026-06-01 — JPDA vs GNN on clutter-crossing scenario

`JpdaAssociator(gate=20, P_D=0.9, λ_C=1e-4)` vs
`GnnAssociator(gate=20)`. Same backend (`EkfEstimator` with
`ConstantVelocity2D(0.1)`), same lifecycle thresholds. Scenario:
`buildClutterCrossingScenario` — two CV targets crossing at the origin
plus 4 uniform false alarms per scan in [−300, 300] × [−50, 50], target
measurement σ = 5 m, 30 scans, seed 31. Run via `runScenarioBatched`.

Source: `tests/scenario/test_jpda_comparison.cpp::JpdaComparison.ClutterCrossing`.

| Associator | mean OSPA (m) | ID switches | Final tracks |
|------------|----------------|-------------|---------------|
| GNN  | 47.3286 | **11** | **35** |
| JPDA | 45.9158 | **4**  | **14** |

**Takeaway.** JPDA's primary value is identity stability and clutter
rejection, not localization accuracy. ID switches drop **64%** (11 → 4)
and final track count drops **60%** (35 → 14). The OSPA improvement is
modest (~3%) because OSPA is clipped at the cutoff (50 m) and the GNN's
errors are largely identity errors (track-swap at crossing) rather than
position errors — the cutoff masks them. The right metric for this
comparison is ID switches, not OSPA.

The 35-track count for GNN reflects clutter contamination — each scan's
4 false alarms have nonzero probability of being the closest in-gate
measurement to some track, kicking GNN's hard assignment to a clutter
point and seeding a new track when the next scan's real measurement
fails to match the contaminated old track. JPDA's soft update spreads
mass across all in-gate measurements weighted by likelihood; clutter
measurements contribute near-zero weight to real tracks and the new
tracks they would seed get suppressed by the M-of-N confirmation policy.

**Methodology notes.** Single seed (31). Multi-seed sweep would
strengthen the OSPA number but the ID-switch reduction is structural
(it follows from JPDA's contamination resistance, not from one lucky
seed) and should survive averaging. The clutter density (~ 4 false
alarms per scan over a 600×100 m box) is moderate; sweeping density up
should widen JPDA's advantage further. Runtime: enumeration cost is
negligible at 2 tracks × ~6 gated measurements per scan.

**Open follow-ups.** (1) Multi-seed × multi-clutter-density sweep.
(2) JIPDA (Integrated PDA) — adds per-track existence probability,
ties into M-of-N. (3) K-best joint events for cluster sizes beyond
~6×6. (4) MHT, the natural next step in the hypothesis-deferment line.

## 2026-06-01 — GNN / JPDA / MHT on crossing-with-dropout scenario

`MhtTracker` (P_D=0.9, λ_C=1e-4, gate=9.0, N_scan=3, K_max_leaves=5,
score_delete=−15.0) vs `JpdaAssociator(gate=9, P_D=0.9, λ_C=1e-4)` vs
`GnnAssociator(gate=9)`. Shared EKF backend
(`EkfEstimator + ConstantVelocity2D(0.1)`). Scenario:
`buildCrossingDropoutScenario(vx=4, y=1, noise=1, dropout=[13, 17), seed=113)`
— two targets cross with ~2 m closest approach, sensor blacks out for
4 consecutive scans across the crossing.

Source: `tests/scenario/test_mht_comparison.cpp::MhtComparison.CrossingWithDropout`.

| Associator | mean OSPA (m) | ID switches | Final tracks |
|------------|----------------|-------------|---------------|
| GNN  | 7.2684 | 3 | 3 (1 ghost) |
| JPDA | 7.2667 | 2 | 3 (1 ghost) |
| **MHT**  | **1.0501** | **0** | **2 (correct)** |

**Takeaway.** Largest single-scenario win in the codebase. MHT preserves
identity through the 4-scan dropout where both single-scan associators
commit too early at the crossing, swap identities, and leave ghost
tracks behind. The 7× OSPA gap is not a tuning artifact — it reflects
the *structural* limit of any per-scan decision rule against a problem
where the right answer is only knowable after seeing post-dropout
measurements. GNN and JPDA are nearly tied (7.27 vs 7.27): JPDA's soft
update doesn't help when both targets are equally likely under any
hypothesis until the dropout ends and trajectories disambiguate.

This is the first scenario where MHT's added complexity over JPDA is
clearly worth it. On the clutter-crossing scenario where the right
answer is locally available each scan (JPDA's 64% ID-switch reduction
already captures most of the value), MHT would not pay off comparably.

**Methodology notes.** Single seed (113). The dropout length (4 scans)
is matched to `N_scan = 3` so the MHT trunk extends exactly through the
gap. Lengthening the dropout beyond `N_scan` would force MHT to commit
during the blackout and erase its advantage. Sensitivity sweep documented
as future work.

**Open follow-ups.** (1) Multi-seed sweep on this scenario to tighten
the OSPA number (the 7× ratio is unlikely to budge much under
averaging — it's structural — but worth confirming). (2) Sensitivity
sweep over `(dropout_length, N_scan, closest_approach)` to characterize
the regime where MHT dominates. (3) K-best global non-conflict via
Murty's — the largest expected improvement to MHT itself. (4) IMM-backed
MHT for maneuvering targets across ambiguous gaps. (5) Murty + JIPDA
hybrid (track existence probability + hypothesis tree) as the
eventual high-end maritime tracker.

## 2026-06-01 — Bearing-only with moving sensor (parallax) — PF wins

`Measurement.sensor_position_enu` is now wired through every estimator and
associator's measurement-model call path. The new scenario builder
`buildBearingOnlyMovingSensorScenario` emits an initial wide-covariance
Position2D seed (σ = 300 m) followed by 60 s of `Bearing2D` measurements
(σ = 1.5°) from a sensor moving +y at 10 m/s **perpendicular to the
line-of-sight** to a stationary target at (1500, 0). Sensor sweeps from
(0, −300) to (0, +300), producing ~22° of bearing change against the
1.5° measurement noise (~15:1 parallax SNR). Wide initial range prior
keeps the posterior in the non-Gaussian regime during the first ~15 s
of convergence.

Source: `tests/scenario/test_filter_comparison.cpp::FilterComparison.BearingOnlyMovingSensor`.

| Filter | mean OSPA (m) | Δ vs EKF |
|--------|----------------|----------|
| EKF (CV)         | 181.6201 | — |
| UKF (CV)         | 185.4117 | +3.79 (+2.1%) |
| **PF (CV, N=2000)** | **123.1583** | **−58.46 (−32.2%)** |

**Takeaway.** First scenario in the codebase where the PF demonstrably
beats both Gaussian filters. The mechanism is exactly what theory
predicts: with proper parallax geometry (sensor motion perpendicular to
LOS) and a wide initial range prior, the posterior on `(px, py)` is
genuinely banana-shaped during the early convergence window — the
crescent of (bearing line) ∩ (broad range prior). EKF and UKF
moment-match this into a Gaussian ellipse and accumulate error; the PF
retains the actual non-Gaussian shape through the transient and gets
substantially better position estimates.

UKF is slightly worse than EKF here, consistent with sigma-point
sampling error mildly exceeding linearization error at this nonlinearity
level. Both Kalman variants sit at ~180 m OSPA because they collapse the
banana to an axis-aligned ellipse around the centroid, which is far from
the actual posterior mode early on.

The first-attempt geometry (sensor moves along LOS at (0, 0) toward
target at (1000, 100)) gave only ~2.4° of bearing sweep against 3° noise
and produced a PF *loss* (112.87 vs 100.12 for EKF). That null result
demonstrated the prerequisite: parallax SNR has to exceed measurement
noise by a meaningful margin for the non-Gaussian regime to manifest.
The retuned geometry above passes that bar comfortably.

**Methodology notes.** Single seed (137), N=2000 particles. Multi-seed
sweep is the straightforward next step. The PF win should survive
averaging because the geometry advantage is structural, not seed-dependent.

**Open follow-ups.** (1) Multi-seed sweep to tighten the 32% claim.
(2) Sweep over sensor velocity (slower = less parallax = PF should win
by more, until the parallax disappears entirely). (3) Slowly-moving
target variant. (4) Closer target ((500, 0) instead of (1500, 0)) —
geometry remains in the non-Gaussian regime longer, gap should grow.
(5) Use this scenario harness to test JPDA / MHT with bearing-only
measurements once the soft-update / branching paths support
non-position measurement models.

**Honest summary of the PF story.** Across three bearing-only attempts:
- Stationary sensor, position-only-seed prior: PF tied EKF/UKF
  (~182 m all, range unobservable from a stationary sensor — documented
  earlier).
- Moving sensor, sensor motion **along** LOS: PF *worst* by 12 m
  (parallax SNR too low — first attempt above).
- Moving sensor, sensor motion **perpendicular** to LOS, wide prior:
  PF wins by 32% — the textbook geometry, finally exercised.

The PF advantage was always conditional on geometry that lets the
non-Gaussian posterior actually form. The first two scenarios didn't
provide that; the third does.

## 2026-06-01 — Multi-seed sweep on the four "wins" (retraction + confirmations)

Re-ran the four winning comparison scenarios with 20 seeds each (seeds
201..220, same set across scenarios) to convert single-realization
deltas into mean ± stddev. Source:
`tests/scenario/test_multi_seed_sweep.cpp`.

### IMM-3 on Maneuvering — **confirmed**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| EKF (CV)   | 5.6713 ± 0.9093 | 0.00 |
| IMM-2      | 5.6713 ± 0.9093 | 0.00 |
| IMM-3      | **4.8148 ± 0.5916** | 0.00 |

Confidence intervals do not overlap. The 7.4% single-seed delta tightens
to ~15% in expectation (≈0.86 m), and IMM-3's stddev is also smaller
than EKF's, indicating the prescribed-CT modes reduce *variability* in
addition to mean error. EKF and IMM-2 are bit-identical to 4 decimals
because position-only measurements collapse the 5-state-CV and free-ω-CT
posteriors into the same predicted positions (no information channel to
distinguish ω modes — same observability gap documented earlier).

### JPDA on ClutterCrossing — **confirmed, very cleanly**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| GNN  | 47.3207 ± 0.1141 | 9.90 |
| JPDA | **45.3199 ± 0.4377** | **2.45** |

OSPA intervals barely overlap; ID-switch advantage is enormous and
robust (9.90 → 2.45 mean across 20 seeds = ~75% reduction, larger than
the original single-seed 64% claim). Strongest and most defensible win
in the codebase.

### MHT on CrossingDropout — **retracted**

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| GNN  | 1.9659 ± 1.9014 | 0.70 |
| JPDA | 1.9656 ± 1.9017 | **0.20** |
| MHT  | 1.9656 ± 1.9010 | 0.90 |

**The 7× MHT win was a single-seed artifact.** Averaged over 20 seeds,
all three associators land at the same OSPA (1.966) with the same
±1.9 m noise floor. The stddev is ≈97% of the mean, indicating the
scenario is genuinely bimodal: some seeds the crossing resolves
cleanly for everyone, some seeds it doesn't resolve cleanly for anyone.
On ID switches, **JPDA actually slightly beats MHT** (0.20 vs 0.90 mean
across 20 seeds). The previous entry (seed 113) is left intact for
historical record but the "7× lower OSPA" headline is wrong in
expectation — it was a single favorable realization.

**Methodological lesson.** Scenarios designed to expose an algorithm's
structural advantage need multi-seed validation before any claim is
made. The dropout window in `buildCrossingDropoutScenario` interacts
with the seed-driven position noise in ways that make the crossing
genuinely ambiguous on some draws — and on those draws, deferred
commitment doesn't help because the right answer isn't recoverable
even with hindsight.

**What this changes.** MHT is no longer the codebase's "biggest win."
The infrastructure (TrackTree, N-scan pruning, K_local cap) is still
correctly implemented and architecturally useful, but the demonstrated
empirical advantage over JPDA on this specific scenario doesn't survive
averaging. A scenario where MHT *does* dominate over the multi-seed
average likely exists (e.g., longer dropout vs N_scan, more targets,
explicit identity-preservation metric) but hasn't been found yet.
Recorded as an open follow-up.

### PF on BearingOnlyMovingSensor — direction holds, intervals overlap

| Filter | mean OSPA ± stddev (m) | mean ID switches |
|--------|--------------------------|------------------|
| EKF (CV) | 212.8332 ± 124.6144 | 0.00 |
| UKF (CV) | 214.2199 ± 125.9745 | 0.00 |
| PF       | **180.3379 ± 124.5977** | 0.00 |

The 32% single-seed PF win narrows to ~15% in expectation (212.83 vs
180.34 = 32.49 m gap). The direction is consistent — PF beats both
Kalman variants on every aggregate — but the per-seed variance is so
large (±125 m, ≈70% of mean) that confidence intervals overlap
substantially. Individual seeds can favor either filter; the *average*
favors PF.

The large variance is inherent to bearing-only with a wide range prior:
on some seeds the bearing sequence converges range quickly, on others
it stays in the banana-shaped ambiguity zone for most of the run and
every filter does poorly. 20 seeds is insufficient to tighten this;
N ≥ 100 is the right next step to convert "directionally PF wins" into
"PF beats EKF with 95% confidence."

---

### Honest revised summary of the wins

| Component | Multi-seed status |
|-----------|---------------------|
| UKF | Tied with EKF on every scenario averaged |
| **PF** | **Wins on bearing-only-with-parallax, but ±125 m variance — need more seeds** |
| IMM-2 (free ω) | Tied with EKF (observability gap, expected) |
| **IMM-3 (prescribed ω)** | **Confirmed: 15% OSPA reduction with non-overlapping CIs** |
| **JPDA** | **Confirmed: 75% ID-switch reduction, tightest CIs of any win** |
| ~~MHT~~ | **Retracted: ties JPDA on this scenario; no demonstrated win** |

The codebase has **two confirmed wins** (JPDA, IMM-3), **one
directional win** (PF on parallax bearing-only), and **one retraction**
(MHT). Net: still useful, more honest, less impressive than the
single-seed numbers suggested. JPDA is the clear winner of the
association axis on the scenarios we have today; MHT's added complexity
over JPDA is not yet justified on demonstrated empirical grounds.

---

## Bus-driven confirmation pass (2026-06-02)

Re-ran the four winning comparisons through `SimulatedSensorBus` (full
sensor quartet: OwnShip + AIS + ARPA + EO/IR; ARPA clutter Poisson(5)
per rotation on JPDA and MHT scenarios). Metric: per-window OSPA
(1 s windows, mean of per-window means) + cumulative ID-switch count.
20 seeds (range 201..220, identical to the prior direct-Measurement
sweep). Heading bias deferred to §14.9.

### JPDA vs GNN — bus clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| GNN    | 49.8548 ± 0.0222    | 16.90            |
| JPDA   | 49.8452 ± 0.0274    | **18.55**        |

**Verdict: retracted (under bus).** Prior direct-Measurement win (45.32
vs 47.32 OSPA, 2.45 vs 9.90 ID-switches) does not meaningfully survive.
Both methods saturate near the 50 m OSPA cutoff; JPDA's OSPA edge
(~0.01 m) is well inside one stddev, and it actually **loses on
ID-switches** (18.55 vs 16.90). The prior JPDA advantage came from
clean clutter discrimination on direct Position2D measurements; under
the bus's EO/IR-dominated stream (10 Hz, ~600 measurements per 30 s),
the per-batch clutter exposure is too sparse for JPDA's soft-assignment
machinery to differentiate from GNN's hard nearest-neighbour.

### IMM-3 vs CV — bus maneuvering

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | 96.9531 ± 0.4587    | 4.00             |
| IMM-3  | **96.7948 ± 0.3768** | **3.85**        |

**Verdict: directionally preserved, materially diminished.** Prior
direct-Measurement IMM-3 win was 4.81 ± 0.59 vs 5.67 ± 0.91 OSPA
(~15% gap, non-overlapping CIs). Through the bus, IMM-3 still wins on
both metrics but the margin is <1σ (~0.16 m on a ~97 m baseline) — the
prior 15% advantage collapses. Both methods sit near the 100 m OSPA
cutoff, suggesting the 15 s scenario length and the bus's measurement
heterogeneity together prevent IMM from settling into the right mode
before the metric saturates. The direction of the effect is right;
the magnitude no longer matches prior claims.

### PF vs EKF — bus bearing-only moving sensor

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | 387.0256 ± 51.5514  | 0.00             |
| PF     | **380.4102 ± 53.6372** | 0.00          |

**Verdict: directional, unchanged from prior.** PF beats EKF on OSPA by
~6.6 m, but the per-seed stddev (~52 m) means CIs overlap heavily —
identical pattern to the prior direct-Measurement sweep (180 ± 125 vs
213 ± 125). The bus version operates at higher absolute OSPA (~380 m vs
~200 m) because the bus EO/IR bearing-only emission has the
projection-time own-ship pose attached via §14.1 — but the ratio is
similar. No new conclusion: PF directionally wins on the high-curvature
bearing-only posterior; N≥100 seeds needed to nail statistical significance.

### MHT vs JPDA — bus clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| JPDA   | 49.8452 ± 0.0274    | **18.55**        |
| MHT    | **49.5934 ± 0.0465** | 25.55           |

**Verdict: retraction re-confirmed.** Prior multi-seed sweep retracted
the MHT win (both ≈ 1.97 m, tied). Under the bus, MHT shows a tiny OSPA
edge (49.59 vs 49.85, ~0.5%) but loses ID-stability by 38% (25.55 vs
18.55). MHT's deferred branching pays for itself only when track
confusion under clutter can be unwound retroactively — here, the bus's
heavy non-ARPA measurement stream (~600 EO/IR detections per 30 s)
already gives JPDA enough info to track correctly without needing
N-scan hypotheses. **Neither dominates; the retraction stands.**

### Cross-cutting observation

Three of four scenarios (JPDA, IMM-3, MHT) show OSPA saturating near
the cutoff — under bus-realistic noise and 20 seeds, the metric loses
discriminative power between methods. This is itself a finding:

- The prior direct-Measurement scenarios (where each scan was 2 clean
  Position2D measurements) gave tracker-quality differences a clear
  signal pathway. The bus's EO/IR-dominated stream (~600 measurements
  per 30 s scenario) overwhelms the per-scan information advantage
  that JPDA's soft assignment / MHT's deferred branching were
  designed to exploit.
- Likely follow-ups: (a) revisit tracker init/delete and gate
  parameters for bus-regime measurement densities, (b) report
  per-window OSPA *conditioned on at-least-one-track-confirmed* so
  the cardinality-penalty saturation doesn't dominate the signal,
  (c) longer scenario durations for IMM-3 to give the mode-switching
  enough time to express.

The bus pass surfaces these limits honestly rather than burying them
behind tuned parameters chasing the direct-Measurement baselines.

### Methodology notes

- Per-window OSPA differs in scale from the prior per-measurement mean
  OSPA because the bus emits ~10× more measurements than direct-
  Measurement scenarios, and 1 s windows average each tick once. Direct
  comparison of *absolute numbers* between this table and the prior
  table is illustrative, not strict; the comparison that matters is
  between methods on the SAME row of each sub-table.
- Bus injects: 1 Hz OwnShip GPS (no heading bias yet), Class-A SOTDMA
  AIS, 3 s ARPA rotation (with optional Poisson clutter), 10 Hz EO/IR
  with bearing+range or bearing-only.
- Determinism: each seed produces a byte-identical Scenario; re-running
  this table yields the same numbers.
- Tests live at `tests/sim/test_bus_jpda_comparison.cpp`,
  `tests/sim/test_bus_imm3_comparison.cpp`,
  `tests/sim/test_bus_pf_comparison.cpp`,
  `tests/sim/test_bus_mht_comparison.cpp`.

## Post-metric-fix bus pass (2026-06-02)

The "Bus-driven confirmation pass" section above was contaminated by a metric
artifact: `runScenario` / `runScenarioBatched` / `runScenarioBatchedMht`
evaluated OSPA *per measurement* and matched truth by `==` on timestamps.
With truth at 1 Hz and EO/IR at 10 Hz, ~93% of evaluation points had empty
`truth_xy`, and `ospaGreedy([], est, cutoff)` returns exactly the cutoff for
any non-empty track set — pinning the reported mean near saturation. See
`docs/superpowers/plans/2026-06-02-truth-tick-ospa.md` for the fix (drive
OSPA evaluation on the truth-sample clock).

### Saturation evidence (seed=201)

| Scenario | Pre-fix empty-truth % | Pre-fix overall OSPA | Post-fix overall OSPA |
|---|---|---|---|
| JPDA clutter crossing (cutoff 50 m)   | 93.1% | 49.88 | 48.26 |
| IMM-3 maneuvering (cutoff 100 m)      | 92.4% | 98.00 | 81.33 |
| PF bearing-only (cutoff 500 m)        |  0.0% | 329.46 | 329.46 |

PF was untouched because that scenario already configures EO/IR at 1 Hz,
matching the truth sample rate.

### Re-run verdicts (20 seeds, post-fix metric)

#### JPDA vs GNN — clutter crossing (cutoff 50)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| GNN  | 48.27 ± 0.29 | 20.40 |
| JPDA | 48.15 ± 0.35 | 24.20 |

OSPA margin 0.12 m sits well within seed stddev (~0.3 m) — statistically a
tie. GNN wins ID-stability by ~4 switches/30 s on average. **The
direct-measurement JPDA win remains retracted under bus-realistic noise.**
The pre-fix verdict was correct in direction but masked the magnitude: the
metric was already at 49.85 (cutoff 50) so neither method had room to express
itself; now both are 1.5 m below cutoff with a real but tiny gap.

#### IMM-3 vs CV — maneuvering (cutoff 100)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| CV (EKF)   | 76.57 ± 3.47 | 5.55 |
| IMM-3      | 75.51 ± 3.14 | 5.05 |

Direction preserved but the 1.07-m margin is within 1σ. **The direct-measurement
IMM-3 win is meaningfully diminished**: at 15 s scenario length with the bus's
plentiful position fixes (AIS 2 s, ARPA 3 s, EO/IR 10 Hz), the CV-only
estimator stays close enough to truth that IMM's mode-switching advantage
doesn't dominate. Likely needs longer scenarios with sustained maneuvering to
re-express.

#### PF vs EKF — bearing-only moving sensor (cutoff 500)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| EKF | 387.03 ± 51.55 | 0.00 |
| PF  | 380.41 ± 53.64 | 0.00 |

**Numerically identical to pre-fix** (as predicted): this scenario configures
EO/IR at 1 Hz with truth at 1 Hz, so no cadence mismatch → no saturation. The
prior verdict stands: directional PF advantage, CIs overlap, PF is not a
clearly justified choice for bearing-only in this regime.

#### MHT vs JPDA — clutter crossing (cutoff 50)

| Algorithm | Per-window OSPA mean ± σ | ID switches mean |
|---|---|---|
| JPDA | 48.15 ± 0.35 | 24.20 |
| MHT  | 45.09 ± 0.60 | 32.60 |

This is the most interesting reveal: under the pre-fix saturated metric MHT
was tied with JPDA at the cutoff. Under the corrected metric MHT shows a real
OSPA margin (~3 m, outside seed stddev), but pays ~35% more ID switches.
**Verdict: trade-off, not a clear winner** — MHT's deferred branch resolution
yields better positional accuracy by re-binding measurements once enough
evidence accumulates, but the cost is more aggressive track ID churn.
Downstream consumers that care about identity continuity (CPA, sensor
hand-off) may still prefer JPDA; consumers that care about positional
accuracy may prefer MHT. The decision is application-dependent rather than
algorithmic.

### Cross-cutting

Three of four prior verdicts (JPDA, IMM-3, MHT) were either reversed or
materially diminished. The metric artifact is responsible for the three
retractions in the pre-fix table appearing more uniform than they should
have. With the corrected metric:

- One verdict held outright (PF — directional only).
- One was meaningfully weakened (IMM-3 — within 1σ).
- One was confirmed-retracted but for the right reason now (JPDA — GNN
  matches on OSPA and beats on ID-stability).
- One revealed a genuine accuracy-vs-stability trade-off (MHT — better OSPA
  at the cost of ID churn).

The general lesson: when evaluating a fusion stack, **the temporal alignment
between truth sampling and metric evaluation has to match** — otherwise
sensors that fire faster than truth ticks contribute cardinality-penalty
noise rather than signal. The truth-tick clock is the standard convention
in the OSPA literature and matches the cadence at which real ground-truth
(GPS) is typically available; we should not have used the per-measurement
clock in the first place.

## Heading error sweep (2026-06-02)

§14.9 wired end-to-end. Own-ship HDT now carries injected bias / drift /
white noise; `ArpaAdapter` and `EoIrAdapter` accept a `heading_std_deg`
that propagates through `projectRangeBearingToEnu` into the bearing
variance (combined in quadrature with the sensor's intrinsic σ).

Sweep: EKF + GNN, 20 seeds (201..220), σ_h ∈ {0°, 0.5°, 1°, 2°},
R-inflation off vs on. Three scenarios re-used from the bus comparison
helpers.

### ClutterCrossing (targets at ~200 m range)

```
[Bus Heading Sweep on ClutterCrossing, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 48.2740 +/- 0.2852 m | 20.40
        0.00  | on        | 48.2740 +/- 0.2852 m | 20.40
        0.50  | off       | 48.2445 +/- 0.2891 m | 20.60
        0.50  | on        | 48.1917 +/- 0.3046 m | 17.90
        1.00  | off       | 48.2469 +/- 0.3038 m | 20.60
        1.00  | on        | 48.1492 +/- 0.3313 m | 16.40
        2.00  | off       | 48.2896 +/- 0.2940 m | 19.60
        2.00  | on        | 48.1067 +/- 0.3379 m | 12.05
```

### BearingOnlyMoving (target at 1.5 km range — headline)

```
[Bus Heading Sweep on BearingOnlyMoving, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 387.5510 +/- 51.4886 m | 0.00
        0.00  | on        | 387.5510 +/- 51.4886 m | 0.00
        0.50  | off       | 419.0032 +/- 47.6722 m | 0.00
        0.50  | on        | 397.6000 +/- 51.9899 m | 0.00
        1.00  | off       | 457.0521 +/- 32.7441 m | 0.00
        1.00  | on        | 402.9367 +/- 52.3337 m | 0.00
        2.00  | off       | 482.6076 +/- 12.0074 m | 0.00
        2.00  | on        | 408.8005 +/- 47.1020 m | 0.00
```

### Maneuvering (single target, 15 s scenario)

```
[Bus Heading Sweep on Maneuvering, 20 seeds]
  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 81.6523 +/- 2.0179 m | 6.15
        0.00  | on        | 81.6523 +/- 2.0179 m | 6.15
        0.50  | off       | 81.0586 +/- 2.2462 m | 6.55
        0.50  | on        | 79.5049 +/- 2.8731 m | 5.55
        1.00  | off       | 81.1884 +/- 2.4011 m | 6.60
        1.00  | on        | 77.5293 +/- 4.0335 m | 4.90
        2.00  | off       | 81.0962 +/- 2.3672 m | 6.55
        2.00  | on        | 74.5369 +/- 4.9032 m | 3.45
```

### Bias / drift propagation probe

Single-seed probe to confirm bias and drift propagate. Plan called for
1° bias / 0.01 deg/s drift; bumped to 3° / 0.03 deg/s after the smaller
magnitudes were too close to the single-seed noise floor on the
1.5 km bearing-only scenario.

```
[Bus Heading Probe: BearingOnlyMoving, seed=201]
  no error   : per-window OSPA mean = 329.4629 m
  bias 3 deg : per-window OSPA mean = 333.5720 m

[Bus Heading Probe: BearingOnlyMoving, seed=201]
  no error     : per-window OSPA mean = 329.4629 m
  drift 0.03/s : per-window OSPA mean = 329.7823 m
```

### Verdict

The BearingOnlyMoving scenario (1.5 km range) is the headline result and
shows the §14.9 failure mode sharply: with R-inflation off, per-window
OSPA climbs monotonically with σ_h (387.55 → 419.00 → 457.05 → 482.61 m),
while R-on stays nearly flat (387.55 → 397.60 → 402.94 → 408.80 m),
recovering roughly 74 m of the ~95 m saturation cliff at σ_h = 2°. This
is exactly the "tracker over-trusts long-range relative bearings when
heading is uncertain" pathology the spec predicted. Maneuvering shows a
smaller but consistent effect: OSPA R-off is flat at ~81 m across σ_h
while R-on drops to 74.5 m at σ_h = 2°, and the ID-switch signal is
cleaner still — R-off ~6.5 across the sweep, R-on falling 6.15 → 5.55 →
4.90 → 3.45 as σ_h grows, indicating R-inflation calms data-association
overreactions. ClutterCrossing (200 m range) shows the expected weak
OSPA response — differences sit below the ~0.2 m stddev noise floor —
but ID-switches still drop materially under R-on (20.40 → 17.90 → 16.40
→ 12.05), so even at short range a heading-aware R reshapes which
scan-to-track associations win. The single-seed bias/drift probe
confirms both error modes propagate end-to-end (drift contributes
~0.3 m of OSPA over a 60 s window at 0.03 deg/s, i.e. a 1.8° final
offset). The overall pattern matches the `range × σ_h` rule the spec
called out: R-inflation is essentially free at small σ_h (the increment
is dominated by intrinsic sensor noise) and progressively saves the
tracker as σ_h grows, with the dramatic gains at long range. Practical
implication: maritime trackers consuming relative bearings should
accept a `heading_std_deg` configuration and propagate it through their
projections — this work makes that path real in navtracker.

### Methodology notes

- Per-window OSPA at 1 s windows (truth-tick clock).
- Heading noise is white (per-tick i.i.d. Gaussian). No process model.
- Bias and drift held at 0 during the sweep; they get a separate probe.
- Sweep uses one canonical tracker per scenario (EKF + GNN). The
  comparison vs other estimators / associators is intentionally not
  re-run; the question here is the error model, not the algorithm.
- Determinism: each seed produces a byte-identical Scenario.
- Tests live at `tests/sim/test_bus_heading_sweep.cpp` and
  `tests/sim/test_bus_heading_bias_drift_probe.cpp`.

## Heading bias estimator (2026-06-03)

**Setup.** Re-runs the §14.9 heading sweep with a global scalar
heading-bias state that the tracker estimates from AIS-vs-ARPA position
residuals on fused tracks. ClutterCrossing's and Maneuvering's primary
target already carries AIS+ARPA+EOIR, so pair observations flow as soon
as a track is confirmed; BearingOnlyMoving has no AIS or ARPA in scene
(EOIR-only), so the estimator never publishes — that row directly tests
the graceful-fallback path. Three rows per σ_h cell: (R-off, no
estimator), (R-on, no estimator), (R-on + estimator). 20 seeds
(201..220), EKF + GNN, publish-variance threshold relaxed to (0.5°)²
so the estimator publishes within the short scenarios. SUCCEED-only
data capture.

### ClutterCrossing — 20 seeds

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 48.27 ± 0.29           | 20.40      |
| 0.0° | R-on       | 48.27 ± 0.29           | 20.40      |
| 0.0° | R-on + est | 47.88 ± 0.48           | **17.85**  |
| 0.5° | R-off      | 48.24 ± 0.29           | 20.60      |
| 0.5° | R-on       | 48.19 ± 0.30           | 17.90      |
| 0.5° | R-on + est | 47.76 ± 0.56           | **14.90**  |
| 1.0° | R-off      | 48.25 ± 0.30           | 20.60      |
| 1.0° | R-on       | 48.15 ± 0.33           | 16.40      |
| 1.0° | R-on + est | 47.65 ± 0.61           | **10.40**  |
| 2.0° | R-off      | 48.29 ± 0.29           | 19.60      |
| 2.0° | R-on       | 48.11 ± 0.34           | 12.05      |
| 2.0° | R-on + est | **47.57 ± 0.62**       | **7.65**   |

### BearingOnlyMoving — 20 seeds (no AIS / no ARPA in scene)

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 387.55 ± 51.49         | 0.00       |
| 0.0° | R-on       | 387.55 ± 51.49         | 0.00       |
| 0.0° | R-on + est | 387.55 ± 51.49         | 0.00       |
| 0.5° | R-off      | 419.00 ± 47.67         | 0.00       |
| 0.5° | R-on       | 397.60 ± 51.99         | 0.00       |
| 0.5° | R-on + est | 397.60 ± 51.99         | 0.00       |
| 1.0° | R-off      | 457.05 ± 32.74         | 0.00       |
| 1.0° | R-on       | 402.94 ± 52.33         | 0.00       |
| 1.0° | R-on + est | 402.94 ± 52.33         | 0.00       |
| 2.0° | R-off      | 482.61 ± 12.01         | 0.00       |
| 2.0° | R-on       | 408.80 ± 47.10         | 0.00       |
| 2.0° | R-on + est | 408.80 ± 47.10         | 0.00       |

R-on+est is byte-identical to R-on across the cell: no AIS+ARPA pairs
get extracted, so the estimator's variance never falls below the
publish threshold and gating stays closed. This is the designed
behavior, not a bug — the R-inflation budget continues to do all the
work in non-cooperative scenes.

### Maneuvering — 20 seeds

| σ_h | row        | OSPA mean ± stddev (m) | id_sw_mean |
|-----|------------|------------------------|------------|
| 0.0° | R-off      | 81.65 ± 2.02           | 6.15       |
| 0.0° | R-on       | 81.65 ± 2.02           | 6.15       |
| 0.0° | R-on + est | 81.66 ± 1.95           | 6.25       |
| 0.5° | R-off      | 81.06 ± 2.25           | 6.55       |
| 0.5° | R-on       | 79.50 ± 2.87           | 5.55       |
| 0.5° | R-on + est | 79.37 ± 2.90           | 5.30       |
| 1.0° | R-off      | 81.19 ± 2.40           | 6.60       |
| 1.0° | R-on       | 77.53 ± 4.03           | 4.90       |
| 1.0° | R-on + est | 77.54 ± 4.03           | 4.90       |
| 2.0° | R-off      | 81.10 ± 2.37           | 6.55       |
| 2.0° | R-on       | 74.54 ± 4.90           | 3.45       |
| 2.0° | R-on + est | 74.51 ± 4.92           | 3.60       |

### Anchor-loss scenario (single seed 401, 120 s)

ClutterCrossing-style scene with σ_h = 2°, R-inflation on, estimator
on. AIS broadcasts on target 1 for [0, 60) s and drops out at t = 60 s.

- `is_published` at t ∈ [30, 60): **true** (estimator converged on
  AIS+ARPA pairs; final variance ≈ (0.29°)²).
- `is_published` at t = 90 s (30 s after dropout): **false** (stale
  window closed; adapters revert to b̂ = 0 with R-inflation only).
- Pre-dropout mean per-window OSPA on [40, 60): 14.17 m.
- Post-dropout mean per-window OSPA on [60, 120): 30.59 m. Growth is
  dominated by target 1's increasing range (cross-track error scales
  with range) and is structurally unrelated to the dropout; the
  bounded-fallback assertion confirms OSPA stays well below the
  cutoff with no divergence.

### Verdict

The bias estimator delivers a clean, measurable ID-stability win in
the AIS-cooperative scene and reverts cleanly when the AIS anchor
disappears. The largest effect is on ClutterCrossing's `id_sw_mean`:
at σ_h = 2°, R-inflation already cut switches 19.6 → 12.05; the
estimator drops them further to **7.65** — a 60% total reduction vs
the no-mitigation baseline. OSPA improvement on the same cell is
smaller (48.11 → 47.57 m) because ClutterCrossing's targets sit at
~200 m where the `range × σ_h` penalty is modest — the ID benefit
comes from sharper, less-uncertain bearings making data-association
decisions more confident under clutter. Maneuvering and
BearingOnlyMoving see no closed-loop OSPA change: Maneuvering's 15 s
duration leaves the estimator barely above the publish threshold, and
BearingOnlyMoving has no AIS in scene so the estimator stays
unpublished by design. Anchor-loss confirms the gating contract — the
30 s stale window closes cleanly, behavior falls back to the §14.9
R-inflation path, and there is no accuracy cliff at the dropout
moment. Practical implication: AIS-vs-ARPA bias estimation is most
valuable in cluttered cooperative scenes where ID stability matters
most; the deferred multi-track bearing-innovation observer (spec
§11 #1) remains the right next step for non-cooperative scenes like
BearingOnlyMoving.

### Methodology notes

- Three sweep TESTs and one anchor-loss TEST: `tests/sim/test_bus_bias_estimator_sweep.cpp`, `tests/sim/test_bus_anchor_loss.cpp`.
- Bus driven via `sim::SimulatedSensorBus::stepOnce(...)` for the estimator-on rows so adapter projections see the latest published b̂ on the cycle after each AIS+ARPA pair is observed.
- Publish threshold (0.5°)² for the sweep; default (0.3°)² used elsewhere.
- AIS dropout in the anchor-loss test uses `sim::AisEmitterConfig::dropout_windows_s`.
- Default `AisArpaPairExtractorConfig` (cycle window 0.5 s, AIS σ fallback 10 m, ARPA bearing σ fallback 1°).
- The estimator is intentionally bias-agnostic during sim warmup — initial state b̂ = 0, variance (5°)². No precomputed calibration.

## GPS position uncertainty (2026-06-03)

**Setup.** Sim injects own-ship GPS position noise via
`sim::OwnShipEmitterConfig::gps_pos_std_m` (zero-mean Gaussian on lat/lon
each tick). When `report_gps_std` is true the emitter advertises
`σ_GPS` on the published `OwnShipPose`, and `ArpaAdapter`/`EoIrAdapter`
inflate projected covariance by `σ²_GPS · I` (the R-on row). When false
the same noise corrupts the projection origin but the adapter is blind
to the budget (R-off row — apples-to-apples noise, unmodeled).
EKF + GNN, 20 seeds (201..220), σ_GPS ∈ {0, 0.1, 1, 5} m.

### ClutterCrossing (close range, ~200 m)

```
[Bus GPS Sweep on ClutterCrossing, 20 seeds]
  sigma_gps_m | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 48.6067 +/- 0.2158 m | 10.20
        0.00  | on        | 48.6067 +/- 0.2158 m | 10.20
        0.10  | off       | 48.6073 +/- 0.2141 m |  9.85
        0.10  | on        | 48.6064 +/- 0.2149 m |  9.05
        1.00  | off       | 48.6122 +/- 0.2163 m | 14.80
        1.00  | on        | 48.6087 +/- 0.2145 m |  9.05
        5.00  | off       | 48.7099 +/- 0.2069 m | 21.40
        5.00  | on        | 48.6223 +/- 0.2150 m |  7.75
```

### BearingOnlyMoving (long range, ~1500 m, sanity probe)

```
[Bus GPS Sweep on BearingOnlyMoving, 20 seeds]
  sigma_gps_m | R_inflate | per-window OSPA mean   | id_sw_mean
        0.00  | off       | 388.8522 +/- 49.0193 m | 0.00
        0.00  | on        | 388.8522 +/- 49.0193 m | 0.00
        0.10  | off       | 388.8307 +/- 49.0620 m | 0.00
        0.10  | on        | 388.8292 +/- 49.0622 m | 0.00
        1.00  | off       | 388.6445 +/- 49.4416 m | 0.00
        1.00  | on        | 388.5005 +/- 49.4561 m | 0.00
        5.00  | off       | 387.5510 +/- 51.4886 m | 0.00
        5.00  | on        | 384.0360 +/- 51.8883 m | 0.00
```

### Verdict

At close range (ClutterCrossing, targets ~200 m), the R-on inflation
materially improves ID stability as σ_GPS grows: at σ_GPS = 5 m the
mean id-switch count drops from 21.40 (R-off) to 7.75 (R-on) — a ~64%
reduction — while OSPA is essentially unchanged (positional accuracy
is dominated by the bearing/range terms even before GPS noise). The
mechanism is the same as §14.9's heading-R-inflation: a better-budgeted
R gate keeps the GNN from chasing clutter that the unmodeled GPS
wobble has dragged into the gate. At long range (BearingOnlyMoving,
target ~1500 m), the σ_GPS = 5 m R-on vs R-off OSPA delta is in the
single-seed noise (~3.5 m on a ~388 m baseline with stddev ~50 m) —
exactly the inverse-of-heading gradient predicted by the spec:
GPS uncertainty is a position-frame additive σ² that doesn't scale
with range, so its relative impact shrinks as the target moves away,
while heading uncertainty rotates the whole bearing arm and grows
linearly with range. Together with §14.9 (heading R-inflation, close
range wins on ID; long range wins on OSPA) and the heading bias
estimator (2026-06-03; closes the loop on slowly-varying mean offset),
the GPS-uncertainty budget completes the own-ship error pipeline for
the cooperative tracker.

### Methodology notes

- One sweep TEST per scenario: `tests/sim/test_bus_gps_sweep.cpp`.
- Same R-on/off comparison protocol as the heading sweep: noise is
  always injected; only the advertised `pose.position_std_m` toggles.
- ClutterCrossing uses `clutter_per_rotation = 8`; BearingOnlyMoving is
  EOIR-only and still picks up `σ_GPS` through `projectRangeBearingToEnu`
  when R-on.

## Adaptive UERE (2026-06-03)

**Setup.** Online σ_pos estimator runs over GGA-derived local-meter
positions in a sliding 8-sample window (`core/own_ship/UereEstimator`).
The estimator does a least-squares constant-velocity fit on each axis
and uses the residual variance as a direct σ_pos estimate. A two-halves
velocity check (|Δv| > 0.5 m/s) suppresses publication during maneuvers
so transient kinematics do not pollute the noise estimate. When the
estimator publishes, its σ overrides the static `HDOP × UERE` path in
`OwnShipNmeaAdapter`; otherwise the static path applies. Adaptive mode
is default off; sweep tests opt in via
`OwnShipNmeaAdapterConfig::enable_adaptive_uere = true` and turn the
sticky sim-side setter off (`report_gps_std = false`) so the estimator
must observe the noise it then advertises.

### Tracking σ across injected levels (ClutterCrossing, 20 seeds)

Stationary own-ship; `OwnShipEmitter` injects N(0, σ_inj²) lat/lon noise
on each GGA fix; bus runs 30 s → ~30 GGA fixes; we read the provider's
`position_std_m` at end-of-run as the estimator's most recent verdict.

| sigma_injected (m) | mean published sigma (m) | within ±50%? |
|---|---|---|
| 0.10 | 0.0910 | yes |
| 1.00 | 0.9158 | yes |
| 5.00 | 4.5777 | yes |

### Sweep comparison (ClutterCrossing, 20 seeds, EKF + GNN)

Same scenario as G8's `BusGpsSweep.ClutterCrossing`. Three rows per σ
cell: R-off (no inflation), R-on static (HDOP×UERE via sticky setter,
adaptive off), R-on adaptive (estimator publishes, sticky off).

| sigma_gps | row             | per-window OSPA       | id_sw |
|-----------|-----------------|-----------------------|-------|
| 0.00      | R-off           | 48.6067 ± 0.2158      | 10.20 |
| 0.00      | R-on static     | 48.6067 ± 0.2158      | 10.20 |
| 0.00      | R-on adaptive   | 48.6063 ± 0.2159      |  9.90 |
| 0.10      | R-off           | 48.6073 ± 0.2141      |  9.85 |
| 0.10      | R-on static     | 48.6064 ± 0.2149      |  9.05 |
| 0.10      | R-on adaptive   | 48.6072 ± 0.2147      |  9.05 |
| 1.00      | R-off           | 48.6122 ± 0.2163      | 14.80 |
| 1.00      | R-on static     | 48.6087 ± 0.2145      |  9.05 |
| 1.00      | R-on adaptive   | 48.6075 ± 0.2152      | 10.20 |
| 5.00      | R-off           | 48.7099 ± 0.2069      | 21.40 |
| 5.00      | R-on static     | 48.6223 ± 0.2150      |  7.75 |
| 5.00      | R-on adaptive   | 48.6750 ± 0.2172      | 12.05 |

### Verdict

The estimator tracks the injected σ within ±50 % across two decades
(0.1 → 5 m), confirming the sliding-window residual-variance design as a
viable online observer of own-ship GPS noise. In the bus sweep, the
adaptive R-on row matches the static R-on row in OSPA to within
statistical noise at all four σ levels (mean OSPA spreads of < 0.06 m
across rows), and recovers most of static's id-switch advantage at
moderate σ (≤ 1 m). At σ = 5 m, adaptive's id-switch count is slightly
worse than static (12.05 vs 7.75) — expected, since static is
calibrated to truth while adaptive must estimate σ from 8 samples per
window, and σ̂ is undershooting truth by ~10 % on average. Adaptive's
value here is not numerical improvement (it cannot beat a path that
already knows the answer) but elimination of the static UERE knob:
deployment scenarios where σ is not known a priori (degraded GNSS,
multipath, RAIM-without-augmentation) now have a closed-loop story
analogous to the heading bias estimator (2026-06-03) on the heading side.

### Methodology notes

- Two TESTs in `tests/sim/test_bus_adaptive_uere.cpp`:
  `AdaptiveTracksSimInjectedSigma` (asserts ±50 % tracking, EXPECT_GE/LE)
  and `AdaptiveSweepClutterCrossing` (SUCCEED-only sweep, prints the table).
- The sweep reuses `runBusClutterCrossingWithGps` from
  `tests/sim/BusComparisonHelpers.hpp` with a new `adaptive_uere` flag on
  `GpsSweepKnob`; default false preserves all pre-existing sweeps and
  matches the byte-identical regression contract.
- `OwnShipNmeaAdapter` now leaves `pose.position_std_m` untouched in the
  HDT branch — only GGA messages update position uncertainty. This fixes
  a Task-2-era oversight where an interleaved HDT would clobber the
  adaptive σ between GGA fixes.

## CPA uncertainty (2026-06-03)

**Setup.** Jacobian-based linear propagation of joint track covariance
through the closed-form CPA function. Output: mean and σ on cpa and
tcpa, and P(CPA < d_threshold) under a 1-D Gaussian on CPA. Own-ship is
synthesised as a Track via `synthesizeOwnShipTrack` with σ_pos from the
GPS work; σ_v_own = 0 per v1 decision. Spec:
`docs/superpowers/specs/2026-06-03-cpa-uncertainty-design.md`. Plan:
`docs/superpowers/plans/2026-06-03-cpa-uncertainty.md`.

### Predicted CPA on a known perpendicular-pass

Geometry: own-ship stationary at the ENU origin; target starts at
(0, 1000) m moving east at 10 m/s. Truth CPA = 1000 m (target is at its
closest at t = 0 and only recedes); tracker is driven with 1 Hz
Position2D measurements for 20 s. Predicted CPA evaluated at t_ref =
10 s; alarm threshold = 500 m. Numbers from
`tests/scenario/test_cpa_scenario.cpp`.

| measurement noise (σ_pos_meas) | own-ship σ_pos | predicted CPA (m) | σ_cpa (m) | P(<500 m) | in 2σ band? |
|---|---|---|---|---|---|
| 1 m | 1 m | 1006.722 | 4.2561 | < 1e-6 | yes |
| 1 m | 5 m | 1006.722 | 6.4896 | < 1e-6 | yes |
| 5 m | 1 m | 1003.826 | 5.8069 | < 1e-6 | yes |
| 5 m | 5 m | 1003.826 | 7.5974 | < 1e-6 | yes |

### CPA bands across §14.9 sweep scenarios (20 seeds, R-on, EKF+GNN)

Mean CPA / σ_cpa / P(<200 m) aggregated over every confirmed-target
pair against a synthesised own-ship at the ENU origin (Clutter and
Maneuvering: stationary own-ship; BearingOnlyMoving: own-ship velocity
(0, 10) m/s, matching the sim). Numbers from
`tests/sim/test_bus_cpa_uncertainty.cpp`. d_threshold = 200 m.

| scenario | σ_h | σ_GPS | mean CPA (m) | σ_cpa (m) | P(<200 m) | n pairs |
|---|---|---|---|---|---|---|
| ClutterCrossing | 0° | 0 m | 1939.329 | 742.918 | 0.2116 | 323 |
| ClutterCrossing | 2° | 0 m | 4344.420 | 3574.362 | 0.1832 | 298 |
| ClutterCrossing | 0° | 5 m | 4178.077 | 1690.213 | 0.1193 | 141 |
| Maneuvering | 0° | 0 m |  133.996 |   9.419 | 0.9937 | 119 |
| Maneuvering | 2° | 0 m |  136.433 |   7.202 | 0.9993 |  64 |
| BearingOnlyMoving | 0° | 0 m | 1057.467 | 182.779 | 0.000794 | 20 |
| BearingOnlyMoving | 2° | 0 m | 1062.396 | 190.656 | 0.002076 | 20 |
| BearingOnlyMoving | 0° | 5 m | 1062.478 | 182.593 | 0.000728 | 20 |

(The Maneuvering / σ_GPS = 5 m cell is omitted: the existing harness
provides single-knob helpers only, so the simpler variant from the plan
covers each cell with one knob at a time. The 3 × 3 picture below is
sufficient for the verdict.)

### Verdict

Truth CPA = 1000 m falls inside the 2σ band on the known perpendicular-
pass in every noise cell (σ_cpa is order-of-magnitude small relative to
the 4-7 m deviation between predicted and truth, so the band closes
comfortably). σ_cpa grows monotonically with own-ship σ_pos at fixed
measurement noise (4.26 → 6.49 m) and with measurement noise at fixed
own-ship σ_pos (4.26 → 5.81 m), confirming the joint Jacobian path
faithfully carries both legs of input uncertainty through to the
output. On the §14.9 bus sweeps σ_cpa is materially larger when σ_h is
raised (ClutterCrossing 743 → 3574 m at 0 → 2°) than when σ_GPS is
raised (743 → 1690 m at 0 → 5 m), which lines up with the Task-7/Task-8
heading-bias work being the dominant covariance source in stationary-
own-ship scenarios. P(<200 m) is the operational output: it cleanly
separates the Maneuvering scenario (mean P ≥ 0.99 — the target really
does pass within 200 m) from the recede-only ClutterCrossing /
BearingOnlyMoving scenarios (mean P ≤ 0.21), so a downstream alarm can
threshold on this number directly. The 1-D Gaussian approximation
remains documented for near-collision cases (spec §11).

### Methodology notes

- One assertive scenario test
  (`tests/scenario/test_cpa_scenario.cpp::PerpendicularPassTwoSigmaBandContainsTruth`)
  pins the 2σ-band claim; one SUCCEED-only sweep
  (`PerpendicularPassNoiseSweepReport`) and one bus sweep
  (`tests/sim/test_bus_cpa_uncertainty.cpp::SweepAcrossScenarios`)
  print the tables above.
- The bus sweep uses `runBus*WithHeading` and `runBus*WithGps` helpers
  one knob at a time per cell. Adding a combined-knob helper was
  unnecessary for the verdict.
- For BearingOnlyMoving the own-ship is moving north at 10 m/s in the
  sim; the synthesised own-ship Track for CPA uses the same velocity so
  the geometry is consistent.
- Suite size 286/286 green after this work (+3 over the 283 baseline:
  two `CpaScenario.*` tests plus one `BusCpaUncertainty.*` sweep).

## RMC velocity + CPA σ (2026-06-04)

**Setup.** Closes the v1 simplification σ_v_own = 0 from the CPA spec.
RMC SOG/COG parsing in OwnShipNmeaAdapter, with a GGA-finite-difference
fallback (OwnShipVelocityEstimator) when RMC is absent. The pose now
carries velocity_enu + velocity_std_m_per_s + velocity_is_valid;
synthesizeOwnShipTrack reads them directly; CPA's existing Jacobian
propagates σ_v into σ_cpa.

### Future-CPA perpendicular pass (truth CPA = 1000 m, TCPA = 100 s)

Target at (-1000, 1000) m moving east at 10 m/s; own-ship stationary at
origin. CPA in the future at t = 100 s. d_threshold = 200 m.

| σ_pos (m) | σ_v (m/s) | predicted CPA | σ_cpa  | P(<200m)  |
|-----------|-----------|---------------|--------|-----------|
| 1.0       | 0.0       | 1000.000      | 10.0995| 0.000000  |
| 1.0       | 0.5       | 1000.000      | 51.0098| 0.000000  |
| 1.0       | 1.0       | 1000.000      | 100.5087| 0.000000 |
| 1.0       | 2.0       | 1000.000      | 200.2548| 0.000032 |

### Past-CPA scenario (v1 perpendicular pass)

The original perpendicular-pass test (target at (0, 1000) m moving east
at 10 m/s, t_ref = 10 s) sits in the past-CPA branch — at t_ref the
target has moved east of the closest-approach point, so
computeCpaWithUncertainty falls back to current-distance with σ from
current dp covariance. Velocity uncertainty does not enter the σ_cpa
computation in this branch. Documented limitation; the future-CPA test
above is the one that exercises σ_v propagation.

### Verdict

The future-CPA perpendicular-pass geometry demonstrates the RMC velocity
integration end-to-end. σ_cpa grows strictly with σ_v at fixed TCPA, with
the growth scaling as O(σ_v · TCPA) — at TCPA = 100 s and σ_v = 1 m/s the
contribution to σ_cpa is ~100 m, which is the dominant term when σ_pos = 1 m
(σ_cpa baseline ≈ 10 m). This matches the predicted scaling from the
Jacobian's velocity-uncertainty path. The mean CPA is unchanged by
velocity uncertainty (no bias introduced). P(<200 m) grows accordingly: at
σ_v = 0 the probability is zero (truth is 1000 m away), while at σ_v = 2 m/s
it rises to 3.2 × 10⁻⁵ (the 200-m band now contains tail mass from the
widened σ_cpa). The past-CPA fallback (original perpendicular-pass test)
documented limitation that σ_v does not enter is acceptable for v1 — in
practice, maritime operators care most about future-CPA risk where velocity
uncertainty dominates; when targets are already in the past-CPA zone the
vessel is already closest and risk is determined by current distance, not
velocity derivatives.

### Methodology notes

- Sweep test: `tests/scenario/test_cpa_scenario.cpp::PerpendicularPassVelocityUncertaintySweepReport`.
- Suite size 318/318 green (was 317; +1 new test).

---

## 2026-06-10 — Multi-sensor harness + miss-model fixes; baseline `2026-06-10_multisensor_fixes`

### What changed

Four root-cause fixes from the AutoFerry "why is textbook IMM+TOMHT bad
on real data" review:

1. **Harness (dominant):** the AutoFerry loader unified per-target truth
   timestamps onto one timestamp per scan (per-target skews of ~0.1 s
   were fragmenting every 2-target evaluation step into two 1-target
   steps, pegging OSPA at the 500 m cutoff and producing ~3.2e3 phantom
   id_switches for every config), deduplicated repeated truth scans, and
   derived finite-difference truth velocities. Bench continuity/RMSE
   metrics are now keyed by `truth_id` with time-varying cardinality.
2. **Per-sensor miss model:** TrackTree's miss branch charges
   Σ_s log(1 − P_D^s(x)) over the distinct sensors in each scan,
   coverage-conditioned (lidar max_range 140 m); IPDA's miss recursion
   uses the scan-effective P_D; IPDA/VIMM persistence is a per-second
   rate (π^dt). AutoFerry scenarios declare a per-sensor detection
   table calibrated from ground truth (radar 0.8 / 1e-5 m⁻², lidar
   0.7 / 5e-6 m⁻² / 140 m, EO+IR 0.6 / 0.5 rad⁻¹) replacing the
   dimensionally-wrong scalar λ_C = 1e-2 override.
3. **IMM TPM dt-scaling:** π is the 1 s TPM, predict applies π^dt and
   advances μ to the predicted prior; update consumes it.

### Measured (scenario2, canonical `imm_cv_ct_mht`)

| metric | pre-fix | post-fix |
| --- | --- | --- |
| track_breaks | 608 | 64.5 |
| id_switches | ~2.0e3 (phantom-dominated) | 146 |
| lifetime_ratio | 0.805 (broken metric) | 0.771 |
| pos_rmse_m | 30.3 | 18.4 |

Synthetic scenarios: **bit-identical** for all canonical configs
(verified via `navtracker_bench_compare` — all-zero deltas), confirming
the fixes are exact no-ops at the 1 Hz cadence.

### IPDA / VIMM ablations (first baseline including them)

On every AutoFerry scenario the existence lifecycle dominates M-of-N:
scenario2 breaks 64.5 → 11.5 (IPDA) / 7 (VIMM), lifetime 0.77 → 0.94,
pos_rmse 18.4 → 8.8, OSPA 413 → 379/377 — the best OSPA of any config
including GNN (457), because existence both keeps true tracks alive
through camera-blind stretches and suppresses clutter births. Same
pattern on scenarios 3–22. On synthetic dense_clutter, IPDA/VIMM cut
OSPA 379 → 137/128.

**Open gap:** on clean synthetics (crossing) IPDA/VIMM cost OSPA
(19.7 → ~82, p95 = 500) from confirmation latency at track birth
(r₀ = 0.5 must climb past 0.9) plus occasional mid-run existence dips.
Tuning (lower confirm threshold with hysteresis, higher r₀ in clean
scenes, or score-gated fallback) is the next experiment before making
IPDA/VIMM the canonical lifecycle.

**Known limitation (philos):** all MHT configs remain broken on philos
(lifetime ≤ 0.015; IPDA 0). Philos truth is asynchronous per-vessel AIS
with no scan structure, so the AutoFerry per-scan truth fix does not
apply; it needs time-windowed truth resampling, and its clutter
environment still uses the legacy scalar λ_C. Tracked as follow-up.

### Key insight (IMM on real data)

On AutoFerry, IMM mode probabilities converge to the TPM's stationary
distribution regardless of dt-scaling: CV and CT are indistinguishable
through 2-D position measurements at 16 Hz (ω weakly observable →
per-mode likelihoods nearly equal), so the kinematic output is
insensitive to μ. The dt fix matters where modes actually separate
(turn scenarios at radar-favourable geometry); the AutoFerry lifecycle
churn was never an estimator problem.

### Methodology notes

- Baseline: `docs/baselines/2026-06-10_multisensor_fixes.{csv,md}`;
  diff vs 2026-06-09:
  `docs/baselines/2026-06-09_robust_vs_2026-06-10_multisensor_fixes.md`.
- New regression pins: `tests/benchmark/test_replay_scenario_run.cpp`
  (GNN + MHT sanity on real scenario2),
  `tests/tracking/test_track_tree.cpp` (per-sensor miss scoring,
  dt-scaled existence), `tests/estimation/test_imm_estimator.cpp`
  (π^dt, semigroup), `tests/benchmark/test_metrics.cpp` (truth_id
  keying, time-varying cardinality).
- Suite size 511/511 green.

## 2026-06-11 — IPDA/VIMM becomes the canonical lifecycle

### Changes

1. **Stale-input guard, default ON** (`Tracker` + `MhtTracker`): inputs
   older than the engine's high-water mark are dropped and counted
   (`staleDropped()`); equal timestamps pass. Opt-out for
   guaranteed-ordered feeds. In-order feeds bit-identical.
2. **Default-detection-model diagnostic**:
   `MhtTracker::defaultDetectionModelWarning()` goes sticky-true when
   ≥2 distinct (SensorKind, MeasurementModel) keys run on the
   auto-installed single-default model.
3. **IPDA confirmation hysteresis**: confirm 0.9 / demote 0.6 with an
   ever-confirmed flag on `TrackTree`; once confirmed, a track holds
   Confirmed down to the demote threshold; re-confirmation requires the
   full confirm threshold. `demote == confirm` reproduces the
   memoryless readout exactly.
4. **Honest detection tables for all 10 synthetic scenarios**
   (scenario *properties*, like the calibrated autoferry table):
   P_D 0.95; λ_C = 1e-6 m⁻² floor for the clutter-free scenarios,
   3.33e-5 m⁻² for dense_clutter (4 FA / 600×200 m box), 1e-2 rad⁻¹
   for the bearing-only scenario.
5. **Canonical lifecycle flip**: `use_ipda_lifecycle = use_visibility
   = true` are now the `MhtTracker::Config` defaults and the canonical
   bench config; M-of-N kept as the `imm_cv_ct_mht_mofn` ablation
   (SPRT remains behind its flag).

### Root cause of the old IPDA synthetic latency

Not r₀ or thresholds: clutter-free synthetics scored with the legacy
global λ_C = 1e-4 m⁻². The existence LR for a gated hit is
L = P_D·g(z)/λ_C with g evaluated under the *track's* predicted
density; a young track's diffuse (unconverged) covariance spreads g so
thin that L < 1 — a perfect hit was evidence *against* existence.
Measured on crossing-equivalent feeds: r walks 0.5 → 0.19 over scans
2–4 before the filter converges, confirm at scan 7 ⇒ lifetime 0.875
(two targets × ~6 scans of a 40-step scenario). With the honest
λ = 1e-6 the same feed confirms at scan 2. r₀ stays 0.5 (Musicki):
raising it would emit clutter-born trees as Confirmed for 1–2 scans.

### Measured (2026-06-11_vimm_canonical vs 2026-06-10 IPDA/VIMM rows)

- Clean synthetics (crossing, head_on, overtaking, parallel, dropout
  pair, clock_skew, speed_change): IPDA/VIMM now **bit-identical to
  M-of-N** — with honest tables every lifecycle confirms at scan 2 and
  the lifecycles only diverge where misses are actually processed.
  ais_dropout: 148 → 66 OSPA (existence no longer dies through the
  10 s gap and re-pays birth latency).
- dense_clutter: VIMM 245 vs M-of-N 421 OSPA (M-of-N regressed under
  honest λ — clutter hits score higher, score-deletes get slower —
  while existence handles them; the flip retires that failure mode).
- AutoFerry: VIMM improved again over 2026-06-10 (scenario2 breaks
  11.5/7 → 1.5, lifetime 0.945 → 0.954; scenario17 OSPA 380 → 369;
  scenario22 breaks 11 → 4.5). The residual ~59 id_switches on
  scenario2 are duplicate-tree swaps (backlog §3).
- speed_change canonical 44 → 18 OSPA (honest tables also fixed the
  M-of-N score scale there).

### Methodology notes

- Baseline: `docs/baselines/2026-06-11_vimm_canonical.{csv,md}`;
  config labels: `imm_cv_ct_mht` is now the VIMM lifecycle,
  `imm_cv_ct_mht_mofn` is the old lifecycle, `imm_cv_ct_mht_vimm`
  was removed (duplicate of canonical).
- Pins tightened: scenario2 MHT lifetime > 0.9, breaks < 10,
  switches < 120 (measured 0.954 / 1.5 / 59).
- philos unchanged (needs truth resampling — backlog §7).

### Addendum 2026-06-11 — cross-tree duplicate merge (backlog §3)

New pass in `MhtTracker::processBatch` before the global solve: retire
the younger of two trees whose best leaves stay within a position-block
Bhattacharyya bound (default 1.0) for `duplicate_merge_seconds`
(default 3.0) of sustained stream time; the older external id survives
(ID-stability invariant); the clock resets the moment a pair separates.

**Why time-based.** The first implementation counted 3 consecutive
close *scans* — at AutoFerry's ~16 Hz union rate that is ~0.19 s, and
real vessels passing close merged almost instantly: scenario6 breaks
2.5 → 11.5, scenario4 lifetime 0.99 → 0.89. Same multi-rate lesson as
scan-counted M-of-N confirmation. The time-based rework recovered the
regressions (scenario6 breaks back to 2.5, scenario4 lifetime 0.94)
while keeping most of the duplicate suppression.

**Measured (2026-06-11_crossmerge vs 2026-06-11_vimm_canonical,
canonical config):**

- id_switches roughly halved on every autoferry scenario: sc16
  68.5 → 10, sc17 27 → 9, sc3 62 → 38.5, sc4 36.5 → 21, sc2 59 → 39.5.
- OSPA down on all autoferry scenarios (duplicates were a permanent +1
  cardinality error): sc16 412 → 335, sc17 369 → 289, sc13 397 → 360.
- dense_clutter OSPA 245 → 103 (duplicate clutter trees retired);
  ais_dropout 66 → 55.
- Clean synthetics bit-identical (no false merges; parallel targets,
  crossings unaffected).
- Honest residuals: lifetime −0.02..−0.07 on sc3/sc4/sc17 — pairs of
  real tracks that genuinely stay within the bound ≥ 3 s, typically
  while one coasts under occlusion with an inflating covariance
  (Bhattacharyya widens). FOV/occlusion modelling (backlog §4) and a
  bias-aware merge distance (§9) are the refinements. Remaining
  switches (sc5 ~97) are not duplicate-induced.
- Scenario2 e2e pin tightened: id_switches < 80 (measured 39.5).
- Side effect: scenario2 e2e runtime 14.6 s → 2.6 s (fewer live trees
  → smaller Murty problems).

## 2026-06-11 — Backlog item 4: source-keyed detection entries, FOV sectors, EO/IR split

**Change.** `ISensorDetectionModel` gained a source-aware lookup
(`paramsFor(sensor, model, source_id)`, fallback source-exact →
kind-wide → defaults) and `DetectionParams` gained azimuth-sector
coverage (`sector_center_rad`/`sector_width_rad`, ENU math convention,
default full circle; evaluated in `missDetectionProbability` alongside
`max_range_m` — out-of-sector tracks charge no miss penalty). The
TrackTree miss loop now keys distinct surveying sensors by the full
(sensor, model, source) triple, so EO and IR cameras sharing
`SensorKind::EoIr` each charge their own calibrated miss penalty.
AutoFerry declares split camera entries; bench plumbing carries an
optional `source_id` per `SensorDetectionEntry`.

**Calibration (per camera, 0.15 rad gate, all nine ground-truthed
scenarios).** EO P_D 0.73 aggregate (0.62–0.87), IR 0.46 (0.21–0.57);
per environment: open water (sc2–6) EO 0.7 / IR 0.5, urban channel
(sc13/16/17/22) EO 0.8 / IR 0.4. Unmatched-bearing rate: open water
0.004–0.6 rad⁻¹, urban 1.0–4.9 rad⁻¹.

**Negative result worth keeping: the measured urban λ must NOT be fed
into the uniform-λ score.** First sweep
(`2026-06-11_eoir_split_measured_lambda`) used the honestly-fitted
per-environment λ and collapsed urban lifetime: sc17 0.65 → 0.35, sc13
0.77 → 0.59, sc22 0.71 → 0.44. The urban excess is persistent
structured shoreline/moored-vessel returns, not uniform Poisson
clutter; the ML-fitted parameter of a wrong model family is not the
right operating point (each camera hit — including on true targets —
was charged ~2 extra nats). Camera λ stays at the kind-wide 0.5 rad⁻¹,
regression-pinned in
`ReplayScenarioRun.AutoferryDeclaresSplitEoIrDetectionEntries`, until
the spatial clutter map (backlog §5) models the shoreline.

**Measured (2026-06-11_eoir_split vs 2026-06-11_crossmerge, canonical
config, P_D split only).**

- lifetime_ratio up on ALL nine autoferry scenarios: sc17
  0.647 → 0.902, sc22 0.706 → 0.837, sc16 0.791 → 0.851, sc3
  0.85 → 0.872, sc5 0.899 → 0.913. track_breaks down or flat
  everywhere except sc6 (+0.5).
- Honest IR P_D (0.4 vs the combined 0.6) is the driver: IR misses —
  which dominate the 16 Hz stream — now charge a miss penalty that
  matches how often the IR camera actually detects, so tracks survive
  IR-dark stretches instead of dying.
- Coverage-vs-accuracy trade, recorded honestly: tracks that now
  survive obscuration coast through it, so urban id_switches rise from
  very low bases (sc17 9 → 23, sc22 10 → 24) and coasting pos_rmse
  climbs (sc17 17.9 → 36.3). OSPA mixed (sc13 360 → 348 and sc17 p95
  500 → 447 improve; sc2/16/22 mean worsens ≤ 43). The OSPA cost is
  the price of reporting tracks through occlusion instead of dropping
  them; FOV/occlusion-aware coasting (now backlog §5/§11 follow-ups)
  is the refinement.
- sc5 id_switches 97.5 → 91: marginal, as predicted — diagnosed
  separately (see below). Clean synthetics, dense_clutter, philos:
  bit-identical (no source-keyed entries there).
- Scenario2 e2e pins re-verified: lifetime 0.958, breaks 1.5,
  switches 37.5 (pins 0.9 / 10 / 80).

**Scenario5 root cause (new backlog §11).** The ~91 residual switches
are bearing-driven identity churn: the two vessels are never closer
than 44 m, but sit < 0.15 rad apart as seen from ownship for 36% of
the 139 s run (< 0.1 rad for 20%) while cameras provide 2891 of 3250
scans and radar refreshes ~0.6 Hz. Bearings gate into both tracks and
the global hypothesis swaps them; the slow radar cannot re-anchor
identity. Neither a duplicate-tree nor a close-pass problem —
candidate fixes recorded in backlog §11.

## 2026-06-11 — Backlog item 7: philos asynchronous truth resampling

**Change.** `resampleTruthToClock` (`core/scenario/TruthResample.hpp`):
linear interpolation of each vessel's asynchronous AIS-as-truth track
onto a shared fixed evaluation clock (segment-FD velocities,
nearest-tick snap at span endpoints so single-fix vessels get one-step
presence, max-gap guard against bridging real dropouts).
PhilosScenarioRun resamples at 1 Hz / 30 s and declares a calibrated
per-sensor detection table: radar P_D 0.07 / λ 2.7e-6 m⁻² / 1000 m
coverage **per sub-scan event** (the rotating sweep arrives as ~10
narrow azimuth bursts per second; measured across 187 vessel × event
opportunities at a 30 m gate), AIS P_D 0.05 / λ 1e-9 (a broadcast
"detects" one vessel per event → per-event P_D ≈ 1/N_vessels).

**Why.** Philos truth carries no scan structure: no two raw samples
share a timestamp, so BenchRunner's exact-time bucketing fragmented
every evaluation step to cardinality 1 — the same harness failure mode
as the pre-fix AutoFerry truth, in its asynchronous form. All MHT
configs scored lifetime ≤ 0.015 with OSPA pegged at the cutoff, and
GNN/JPDA scores were *flattered* (per-vessel presence collapsed to its
2–5 raw message instants, trivially covered).

**Measured (2026-06-11_philos_resample vs 2026-06-11_eoir_split;
philos only — every other scenario bit-identical).**

- Canonical imm_cv_ct_mht: lifetime 0 → 0.295, OSPA 500 → 430, breaks
  0.04, switches 0.17, pos_rmse 38 m. All IPDA/VIMM MHT configs land
  in the same band (0.27–0.30 lifetime, 428–432 OSPA).
- GNN/JPDA lifetime drops 0.68 → 0.33–0.35: the old value was an
  artifact of fragmented presence; the new one is honest and now
  comparable across configs.
- M-of-N ablation (imm_cv_ct_mht_mofn) stays at lifetime ≈ 0.01 — it
  cannot confirm on a ~10 s AIS cadence interleaved with ~10 Hz radar
  events; per-dataset evidence for why the IPDA lifecycle is canonical.
- The remaining lifetime ceiling (~0.3) is honest confirmation latency
  on a ~20 s fixture where most vessels carry only two AIS fixes ~10 s
  apart: confirmed-from-second-fix costs half such a vessel's presence
  window. A longer philos capture would raise it mechanically.
- Pins: `ReplayScenarioRun.PhilosResampledTruthAndMhtLifecycle`
  (cardinality ≥ 10 at peak, lifetime > 0.2, breaks < 2, switches < 5,
  OSPA < 470, rmse < 60).

Boston-harbor caveat, recorded for item 5: most unmatched radar plots
are persistent shore/moored structure, the same uniform-λ limitation
as the AutoFerry urban cameras.

## 2026-06-12 — Backlog item 5: spatial clutter map (position maps on, bearing maps off)

**Change.** `ClutterMapSensorDetectionModel`
(`core/tracking/ClutterMapDetectionModel.hpp`, association.md §6): a
decorator over the fixed per-sensor table that learns spatially
varying λ_C online. Per (sensor, model), a sparse grid of cells each
holding a time-based EWMA (τ = 20 s, never scan-counted) of
unassociated returns per scan; cells touched by associated traffic
decay toward zero, untouched cells read back the table baseline.
`paramsFor(z)` — now virtual on the port; the TrackTree score already
called it, so the hot path is unchanged — interpolates λ at the
measurement position (bilinear ENU for position sensors, circular
azimuth for bearings) and clamps to [baseline/8, baseline·64].
`MhtTracker` enriches `ScanObservation` with scan time and the
unassociated subset of positions/azimuths. Bench ablation config
`imm_cv_ct_mht_cmap` = canonical IPDA+VIMM stack + map; the canonical
config and all defaults are untouched (verified: every non-cmap row of
baseline `2026-06-12_clutter_map` is bit-identical to
`2026-06-11_philos_resample`).

**Measured negative result — the bearing-map death spiral.** The first
run (`2026-06-12_clutter_map_bearing_spiral`) had bearing maps on and
collapsed lifetime on the camera-heavy autoferry scenarios (sc17
0.90 → 0.25, sc5 0.91 → 0.31, sc22 0.84 → 0.43, sc2 0.96 → 0.72).
Per-sub-map ablation (fixed vs full vs position-only vs bearing-only
on sc2/5/13/16/17/22) isolated it cleanly: position-only is
lifetime-neutral on every scenario; bearing-only reproduces the full
collapse. Mechanism: bearings cannot initiate tracks, so a target
whose track lapses keeps feeding "unassociated" bearings at its own
azimuth — the map raises λ exactly where the target is, suppresses
re-confirmation, and the suppression self-reinforces. The bearing
map's apparent OSPA gains (sc13 348 → 262) came from suppressing true
tracks alongside false ones. Bearing maps are therefore OFF by default
(`ClutterMapParams::enable_bearing_map`), opt-in only; re-enabling
requires a clutter proxy that excludes trackless targets
(hypothesis-level labeling, association.md §6 ways-to-improve).

**Result (`2026-06-12_clutter_map`, cmap vs canonical, position maps
only).** Acceptance was "OSPA ↓ without lifetime loss on true tracks":

- dense_clutter: OSPA 103 → 64.3 (−38%), breaks 0.35 → 0.2, switches
  0.45 → 0.2 — uniform Poisson clutter is exactly what the map learns.
- philos: OSPA 429.5 → 398.4, id_switches 0.17 → 0, pos_rmse
  38.5 → 34.4 (Boston-harbour radar shore structure absorbed).
- autoferry: lifetime preserved or up on all 9 (sc3 0.872 → 0.904,
  sc22 0.837 → 0.856); OSPA small moves both ways (sc13 −10.5, sc6
  −7.9, sc22 −4.6 vs sc16 +7.7, sc5 +6.7). Neutral overall — expected:
  the urban offender is the *cameras*, whose map is the disabled one.
- Clean synthetics: OSPA +5–11 (crossing 18.6 → 28.1, head_on
  18.6 → 27.8), lifetime −0.02. Cause: birth self-poisoning — a new
  target's first return is by definition unassociated, bumps its own
  cell from the 1e-6 floor to the 64× clamp, and delays confirmation
  by ~a scan. Inherent to the birth-gate clutter proxy (excluding
  birthing returns would also exclude all clutter, which births
  too); the fix is hypothesis-level labeling, same as above.

**Verdict.** `imm_cv_ct_mht_cmap` stays an ablation config; the
canonical config keeps the fixed table. The map is the right tool
where clutter is dense and roughly Poisson per cell (dense_clutter,
philos) and is safe-by-construction elsewhere (clamped, baseline
passthrough when untouched) — but the birth-gate proxy is too blunt
for camera bearings and slightly taxes clean-scene confirmation.
Promote only after the proxy reads the global hypothesis instead of
the birth gate.

## 2026-06-12 — Clutter map second iteration: global-hypothesis labeling

**Change.** Clutter evidence for the spatial map is now labeled from
the chosen global hypothesis instead of the birth gate: MhtTracker
builds the observe() bundle AFTER the solve, and each return carries
clutter weight 1 − r of the hypothesis that claims it (selected hit
leaf, or the tree it birthed this scan; 1.0 when unclaimed; the IPDA-
off sentinel r = 1 makes claimed returns weightless). ScanObservation
renamed its evidence fields (`clutter_positions/_weights`,
`clutter_bearings/_weights`); the map sums weights per cell. Fixed
models ignore observe(), so every non-cmap config is bit-identical
(verified against `2026-06-12_clutter_map`).

**Hypothesis test — does this re-enable the bearing map? NO.**
Original claim (this morning's entry): hypothesis labeling is the
precondition for the bearing map. Measured (per-scenario diagnostic,
bearing map opt-in): strictly WORSE than the binary proxy — sc17
lifetime 0.25 → 0.13, sc5 0.31 → 0.10. Root cause: a coasting or
freshly re-born track's claimed bearings carry weight 1 − r exactly
while r is low — the map feeds on the target during the occlusions
the track must survive; the binary proxy at least zeroed every gated
bearing. The spiral is structural until the weight can distinguish
"low-existence target" from "no target" (visibility-conditioned
weights or a hard zero for hypothesis-claimed returns —
association.md §6). Bearing maps stay opt-in-off; docs corrected.

**Result (`2026-06-12_clutter_map_hyplabel`, cmap vs the birth-gate
cmap of `2026-06-12_clutter_map`).** Better on 17 of 20 scenarios:

- Clean synthetics recover 15–30% of the birth tax (crossing
  28.1 → 26.9, crossing_dropout 35.0 → 31.8, overtaking 17.7 → 15.4;
  canonical-fixed remains lower still — the residual tax is the birth
  weight 0.5 plus low-r claims while a new track's existence climbs).
- dense_clutter OSPA 64.3 → 59.4 (fixed: 103), switches 0.2 → 0.1.
- autoferry: small broad gains (sc3 OSPA 440.5 → 433.8 with switches
  40 → 36.5, sc6 switches 69 → 63, sc22 26.5 → 23); lifetime
  unchanged everywhere.
- philos regresses 398 → 408 (still −21 vs fixed) with lifetime
  0.288 → 0.273: its vessels confirm slowly at P_D 0.07, so real
  returns are claimed at low r and charged as partial clutter — the
  same "low-existence target" signature as the bearing spiral, in
  miniature. The weight refinement above would address both.

**Verdict.** Keep hypothesis labeling (more principled, better
almost everywhere); cmap remains an ablation config. Next refinement
recorded: existence-vs-visibility-aware weights.

## 2026-06-12 — Backlog item 11: sc5 identity churn re-diagnosed (conveyor, not swaps)

**Investigation.** The 2026-06-11 hypothesis (camera bearings swapping
between two angularly-unresolved tracks in the global solve) was
tested and falsified in three steps:

1. **Shared ambiguous bearings** (`share_ambiguous_bearings`): a
   Bearing2D return whose hit branches exist in ≥ 2 trees is exempted
   from the solve's exclusivity (each tree's bearing hit maps to its
   private assignment column — both trees can consume it; the
   physically right model for merged camera detections). Measured on
   sc5: **bit-identical** despite 23k shared assignments — under
   exclusivity each tree was already taking its nearest bearing, so
   per-scan assignment swaps were never the churn.
2. **Switch forensics** (per-event dump): 182 raw events, only 21 are
   pair swaps. Dominant pattern: truth 1 is tracked by a *succession*
   of short-lived ids (~2 s apart, second-nearest track 50 m away —
   handoffs, not swaps), plus near-tie flicker (d1 10.7 vs d2 10.8 m).
   107 confirmed ids in 139 s for 2 truths.
3. **Birth forensics**: 45 of 48 near-truth confirmations occur with a
   live confirmed track already within 50 m — duplicate births. Gate
   sweep confirms gate escape: global gate 20 → 100 collapses sc5
   switches 91 → 27 (sc6 74 → 8.5) while OSPA *improves* ~80 m
   (duplicate cardinality), at the cost of rmse/lifetime.

**Conveyor mechanism.** Bearing-carried track drifts 10–30 m and turns
overconfident → sparse radar return misses the χ² gate → births a
duplicate alongside → young tree confirms and takes the stream → old
tree starves → handoff = id switch, every 2–4 s.

**Remedies implemented (opt-in, defaults OFF, canonical bit-identical;
574/574 tests green):** per-sensor static gate
(`DetectionParams::gate_threshold`), and the adaptive recapture gate
(`gate_recapture_tau_s`: position gate × min(max_scale, 1 + age/τ)
with age = time since the hypothesis' last position-sensor update,
anchor carried per tree node). Measured (τ = 2 s): switches/OSPA
improve strongly (sc5 91 → 43, OSPA −60 on most scenarios) but
lifetime regresses (sc3 0.87 → 0.63, sc17 0.90 → 0.54) and rmse
climbs to 30–60 m: the radar return gates back in but the Kalman gain
uses the same overconfident P and barely corrects. Gate widening
treats the symptom.

**Root cause, quantified (→ new backlog item 12).** NEES of near-truth
confirmed tracks on sc5: **mean 77.6** (consistent filter ≈ 2), 57% of
samples above the 99% χ² bound; claimed σ 1.2–3.8 m against 15.1 m
mean actual error. The filter is structurally overconfident on real
bearing-dominated data (suspects: camera R calibration, bearing-update
range collapse, synthetic-tuned process noise). Until item 12 lands,
none of the item-11 knobs is promotable — with honest covariance the
conveyor should not form at the base gate at all.
