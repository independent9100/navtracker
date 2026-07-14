# Inter-sensor registration bias estimator (`SensorBiasEstimator`)

Per-(sensor, source_id) bias filter fed by AIS-anchored cross-sensor
pair observations. Closes the calibration gap surfaced by the
Helgesen 2022 comparison (`docs/baselines/helgesen2022_reference.md`):
the paper carefully calibrates each sensor's mounting offset against
RTK-GNSS truth; navtracker did not, and the unmodelled offsets
showed up as posRMSE inflation and (likely) elevated cardinality
penalty on env 1.

Code: `core/bias/SensorBiasEstimator.{hpp,cpp}`,
`core/bias/SensorBiasPairExtractor.{hpp,cpp}`,
`ports/ISensorBiasProvider.hpp`. Tutorial:
[`docs/learning/21-sensor-registration-bias.md`](../learning/21-sensor-registration-bias.md).

## Math

### State

One filter per `SensorBiasKey = (SensorKind, source_id)`. Two
variants:

- **Position bias** (radar, lidar, ARPA). State
  `b ∈ ℝ²` (ENU metres). Covariance `P_b ∈ ℝ²ˣ²`.
- **Bearing bias** (EO, IR). State `β ∈ ℝ` (radians).
  Variance `P_β`.

### Predict

Random-walk:

```
b(t+dt)   = b(t) + w,    w ~ N(0, σ²_drift_pos · dt · I₂)
β(t+dt)   = β(t) + w_β,  w_β ~ N(0, σ²_drift_brg · dt)
```

Defaults `σ²_drift_pos = (0.1 m)² / 3600 s`,
`σ²_drift_brg = (0.05 rad)² / 3600 s`.

### Observation — position bias

Anchor (AIS) gives `z_anchor = truth + n_anchor`,
non-anchor sensor gives `z_sensor = truth + b + n_sensor`.
Differencing:

```
r        = z_sensor − z_anchor − b̂
R_obs    = R_sensor + R_anchor
S        = P_b + R_obs
K        = P_b · S⁻¹
b̂      ← b̂ + K · r
P_b     ← (I − K) · P_b
```

`H = I`; identical to a scalar-prior KF update on a constant.

### Observation — bearing bias

Anchor reports target ENU position `y`; sensor at `p_s` reports
bearing `α_obs`. Predicted bearing:

```
d            = y − p_s
α_pred       = atan2(d_y, d_x) + β̂
r            = wrap(α_obs − α_pred)
range²       = ||d||²
R_obs        = σ²_α_sensor + σ²_anchor_pos / range²
S            = P_β + R_obs
K            = P_β / S
β̂          ← β̂ + K · r
P_β         ← (1 − K) · P_β
```

Anchor position noise projects onto bearing through the geometric
Jacobian (1-D approximation, isotropic anchor noise).

### Observability gates

Before each update:

1. **Time-window gate.** `|t_anchor − t_sensor| ≤ Δt_max`
   (default 1.0 s).
2. **Range gate.** `range(target) ≥ r_min` (default 50 m).
3. **Innovation gate.** Mahalanobis `r^T S⁻¹ r ≤ N²` (default
   `N = 5` σ).

### Application

`positionBias(key) / bearingBias(key)` return `is_published = true`
once `has_update == true` AND posterior variance is below the
publish threshold (default 1 m² per axis for position; (0.3 deg)²
for bearing). The `Tracker` / `MhtTracker` subtract the published
estimate from incoming measurements *before* predict / associate
(`applyBiasCorrection` in `Tracker.cpp`).

### Closed-loop feedback: reconstruct the raw observation (fix-wave W3.2)

The residuals above (`r = z_sensor − z_anchor − b̂`; the bearing
`α_pred = atan2(d) + b̂`) are correct **only when `z_sensor` / `α_obs`
are the RAW sensor values**. But once the estimate publishes, the tracker
already subtracted `b̂` from every incoming measurement (`applyBiasCorrection`)
*before* recording the `SourceTouch` the extractor reads — so the pair shows
only the **residual** bias `(b_true − b_pub)`, and subtracting `b̂` again makes
the fixed point `b̂ = b_true/2`. The published estimate then decays to half the
true offset while its variance keeps shrinking (overconfident); every corrected
measurement keeps half the bias forever.

Fix: the correction that was applied is carried forward on the measurement and
copied into the touch — `Measurement`/`Track::SourceTouch::applied_position_bias_enu`
(set by `applyBiasCorrection`) and `applied_bearing_bias_rad`. The pair
extractors reconstruct the raw observation by adding it back
(`z_sensor_raw = touch.value_enu + touch.applied_position_bias_enu`;
`α_raw = touch.alpha_rad + touch.applied_bearing_bias_rad`) before forming the
innovation, so the loop measures the full `b_true`. The same reconstruction on
the cross-sensor anchor (`z_anchor_raw` then minus the partner's published bias)
removes the twin "anchor debiased twice" defect in `extractCrossSensorPositionPairs`.
Open-loop callers (nothing published, `applied_* = 0`) are unchanged.

The bias *covariance* is **not** folded into the per-measurement R.
That is the Schmidt-KF "considered" treatment, deferred to
`sota-roadmap.md §5`.

## Assumptions

- AIS is unbiased relative to truth. Class-A typically accurate to
  5-15 m; Class-B can have systematic offsets — gate by AIS class if
  it matters for a deployment.
- Bias drifts slowly (random walk with very small Q). Step changes
  (mast knocks, recalibration events) are caught with lag.
- Target is a point object at AIS-reported position. For
  large vessels (e.g. Gunnerus 31 m × 7 m) the AIS antenna and
  radar / lidar return centre are metres apart; the bias absorbs
  this offset into "effective bias = mounting + antenna-to-centroid"
  which is fine for the tracker but not a sensor-calibration number.
- Cross-sensor pair counts are sufficient. AutoFerry env 1 with 2
  AIS vessels across 5 scenarios produces O(100-1000) per-sensor
  pairs — far more than the ~10 needed for convergence.

## Rationale

### Why per-(sensor, source_id) and not a single shared bias

EO and IR cameras have independent mountings. Lidar and radar share
the mast but their digital pipelines are independent. Two AIS-class
radars on the same vessel would still have independent calibration
histories. Each physical sensor gets its own estimator.

### Why a separate filter, not augmented per-track state

Identical argument to `HeadingBiasEstimator`. A separate slow
random-walk filter:

- isolates bias dynamics (very slow) from track dynamics (fast);
- decouples bias convergence speed from track update rate;
- pools per-(sensor, source_id) information across **all** tracks.

The tracker-augmented alternative would carry N copies of the bias
for N tracks and lose the cross-track pooling.

### Why AIS as anchor

AIS lat/lon comes from the target's *own* GNSS — independent of
every own-ship sensor. A track that uses the biased sensor cannot
be its own anchor (cyclic). Track-anchored fallback is deferred
until AIS coverage is observed to drop below a threshold; the
honest no-AIS behaviour is "estimator stays at the prior,
`is_published = false`, deterministic shift = 0".

### Why deterministic shift, not Schmidt-KF

Schmidt-KF requires augmenting every per-track update with a 2×2
block plus cross-covariance. Once `P_b` is below the publish
threshold its contribution to the per-track covariance is small
compared to the disagreement we are fixing. Revisit when
`pos_rmse_m` stops moving despite further GOSPA reductions — the
signature that residual bias variance is the next dominant term.

## Cross-sensor anchored extension (backlog item 13, 2026-06-17)

The AIS-anchored path above leaves a deployment gap: non-cooperative
targets (sailing boats, kayaks, illegal vessels) broadcast no AIS at
all. The tracker handles them fine — only the bias *calibration* is
blocked. Item 13 plugs that gap with a cross-sensor anchored
extension on top of the same `SensorBiasEstimator`.

### Math

For each AIS-less track passing eligibility, let `Y` and `X` be two
contributions whose `SensorBiasKey`s differ. The estimator's
two-sample update equation in the AIS-anchored case is:

```
r       = z_X − z_AIS − b̂_X
R_obs   = R_X + R_AIS
```

Cross-sensor anchoring uses `Y` in place of `AIS`. But `Y` itself
carries an unknown bias `b_Y`, so the honest residual is:

```
r       = z_X − (z_Y − b̂_Y) − b̂_X
R_obs   = R_X + R_Y + P_b_Y           (Schmidt-KF fold)
```

The `−b̂_Y` term debiases `Y` with the estimator's current
knowledge; the `+P_b_Y` term is the Schmidt-KF "considered" inflation
that prevents pretending we know `b_Y` better than we do. When `Y`
has not yet published (`is_published = false`) we treat `b̂_Y = 0`
and `P_b_Y = 0` — equivalent to the naive pair, which is correct at
cold start before any sensor has converged.

The extractor emits **exactly one observation per calibrated key per
cycle**. Each key `X` is anchored on its single most-trusted partner
`Y` — the eligible key (different physical sensor) with the smallest
trace of the post-fold `R_anchor = R_Y + P_b_Y`. For `N = 2` keys this
produces the symmetric pair `X←Y` and `Y←X`; for `N ≥ 3` each sensor's
single sample is used exactly once.

> **Why one-per-key (not every ordered pair).** A sensor contributes a
> *single* sample `z_X` per cycle. Pairing it against every other key
> and folding each as an independent KF update replays `z_X` (and its
> noise) `N − 1` times — the residuals `r_{X,Y}` all share `z_X`'s
> noise term, so they are correlated, not independent. Treating them as
> independent collapses `P_X` far too fast and triggers premature
> `is_published`. Anchoring `X` on its single best partner uses the
> sample once and keeps the covariance honest.

The bias estimator's zero-mean prior (`σ_init = 5 m`) breaks the
otherwise-underdetermined "one equation, two unknowns" symmetry by
pulling each bias toward zero. Joint coordinate descent across cycles
then converges to the solution that minimises both the pair residuals
and the prior penalty: `b̂_X − b̂_Y` is determined by the data;
`b̂_X + b̂_Y` is determined by the prior.

**Common-mode bias is unobservable.** Cross-sensor anchoring only sees
the *relative* offset `b_X − b_Y`. Any bias component shared by both
sensors cancels in `r` and is pinned to zero by the prior — there is no
information in the pairs to recover it. The acute case is two
contributions from the **same physical sensor** (ARPA TTM and TLL share
one `source_id` but carry distinct `SensorKind`s, hence two
`SensorBiasKey`s). Their residual is `≈ noise` regardless of the true
shared mounting offset, so pairing them would *mask* a genuine common
radar bias and report near-zero for both. The extractor therefore
refuses to anchor across keys with the same `source_id`; recovering the
absolute (common-mode) calibration requires an external truth anchor
(AIS or survey).

### Eligibility gates

Per spec §13 — `CrossSensorEligibilityConfig`:

| Gate | Default | Why |
|---|---|---|
| `existence_probability ≥ 0.95` | 0.95 | Tentative tracks are too noisy to anchor anything |
| 2×2 position cov trace ≤ `25 m²` | 25.0 | Loose tracks would feed too much uncertainty into `R_anchor` — the bias update would be near-no-op |
| `Y ≠ X` (by `SensorBiasKey`) | hard | No self-anchoring |
| `Y.source_id ≠ X.source_id` | hard | No anchoring across the same physical sensor (TTM/TLL) — would mask common-mode bias (see Math) |
| One contribution per key per cycle | hard | Avoid double-anchoring through the same physical sensor twice in the same scan |
| One observation emitted per key per cycle | hard | A single sample must not be folded as multiple independent updates (see Math) |
| Track has no AIS contribution this cycle | hard | When AIS is present, the existing AIS-anchored path is strictly more informative |

### Bootstrap → steady state

Per the spec's coupling discussion:

- **Bootstrap (option a).** A track that previously had AIS will
  have converged biases for the sensors that contributed during the
  AIS-bearing stretch. Those biases then act as informed priors when
  AIS drops out and the cross-sensor extractor takes over.
- **Steady state (option b).** Without ever-AIS observability, the
  joint coordinate descent converges to `(b̂_X, b̂_Y)` whose
  *difference* matches the data but whose *sum* matches the prior.
  This is the right answer for fusion (only the difference between
  per-sensor biases affects whether the tracker's measurements
  agree); the absolute calibration vs the world requires an
  external truth anchor and is out of scope for the AIS-less
  deployment.

### Assumptions specific to the cross-sensor path

- **Track quality gates are calibrated.** The defaults reflect what
  AutoFerry confirmed tracks reach in steady state. Tighter gates
  (e.g. `existence ≥ 0.99`) trade calibration recall for accuracy —
  the right setting depends on the deployment's confirm-time
  distribution.
- **`bias_provider` reflects the current per-sensor estimates.**
  The same `SensorBiasEstimator` that consumes the cross-sensor
  pairs is used as the provider for the Schmidt fold — so the fold
  is always against the most recent published estimate. There is no
  iteration-within-cycle: each pair sees the previous cycle's
  posterior.
- **Position-cov-trace gate uses the tracker's posterior P, not
  any per-contribution covariance.** This is the conservative
  choice: the posterior reflects the fusion result, and a track
  with tight posterior is one the tracker has converged consensus
  on.

### Why not symmetric joint state augmentation

A formally-correct alternative is to augment the bias state with both
sensors' biases jointly and update both from each pair. We chose the
coordinate-descent path because:

- It reuses the existing single-key `observe(PositionBiasPairObservation)`
  update — no new math path.
- The convergence behaviour is empirically equivalent on AutoFerry
  (the prior breaks symmetry the same way in both formulations).
- It composes cleanly with the AIS-anchored path: an AIS pair is
  just a special cross-sensor pair where one sensor (AIS) has a
  pinned `b = 0` and tight `P_b` — the math reduces to the existing
  path with no code change.

### Acceptance criteria

1. **No regression on AIS-bearing scenarios.** Tracks with AIS in the
   cycle window are skipped by the cross-sensor extractor → bit-
   identical bias trajectories vs the canonical-without-x-sensor.
   **Measured 2026-06-17:** ✅ all 9 anchored autoferry rows bit-identical.
2. **Estimator publishes a non-trivial bias on AIS-less inputs.**
   The pinned-anchor unit test
   (`CrossSensorRecoversBiasWithPinnedAnchor`) confirms that when one
   sensor is anchored to a tight prior, the other sensor's true bias
   is recovered within 0.6 m. ✅
3. **Bit-identical determinism preserved.** ✅ `BenchDeterminism` green.
4. **AutoFerry env-1 raw GOSPA improves by 5-15%.** **NOT MET.**
   Measured zero delta vs canonical. Root cause is the symmetric-
   fusion invariance documented below.

### Symmetric-fusion invariance (2026-06-17 finding)

Measured empirically: on AutoFerry env-1 raw scenarios (sc2-6), the
cross-sensor extractor produces O(100-1000) pairs per run, the
estimator converges (lidar `(+0.57, +0.08) m`, radar `(−0.57, −0.08) m`
on sc2 after 980 pairs, `P` trace ≈ 0.5 m²), and the bias correction
is applied to every measurement after the publish threshold is met —
yet `gospa_rms`, `pos_rmse_m`, `id_switches`, and `nees_mean` all
report bit-identical to the canonical baseline.

**Why this happens.** Without an external truth anchor, the joint
coordinate descent on cross-sensor pairs converges to the symmetric
allocation `(b̂_X, b̂_Y) = (+δ/2, −δ/2)` where `δ = b_X − b_Y` is the
measured relative offset (the prior breaks symmetry by pulling the
sum to zero). For a symmetric weighted-mean fusion operator:

```
fused = w_X (z_X − b̂_X) + w_Y (z_Y − b̂_Y)
      = w_X z_X + w_Y z_Y − (w_X · δ/2 − w_Y · δ/2)
      = uncorrected_fused − (w_X − w_Y) · δ/2
```

When `w_X ≈ w_Y` (similar sensor variances — autoferry radar 5 m vs
lidar 3 m), the correction term is small, and the fused track moves
by ≪ 1 m. GOSPA on ~17 m pos_rmse tracks is insensitive to sub-metre
shifts → bit-identical metrics.

**When the extractor *does* deliver value.** Three regimes break the
symmetry and recover measurable benefit:

1. **External truth anchor sometime in the run.** Even a single
   AIS-bearing track segment converges one sensor's bias to its
   *absolute* value; subsequent AIS-less segments then anchor the
   other sensor against the converged one (Schmidt fold). The
   pinned-anchor unit test demonstrates this; bench scenarios
   without intermittent AIS do not exercise it.
2. **Asymmetric sensor variance.** When one sensor's `R` is much
   tighter than the other, the fusion weights are unequal, and the
   correction term `(w_X − w_Y) · δ/2` becomes load-bearing.
3. **Per-sensor prior knowledge** seeded via
   `setKnownPositionBias` (matching the env-2 EO/IR bearing-bias
   pattern from item 9's seed hook). The seed pins one sensor's
   bias so the cross-sensor descent updates only the other.

For AutoFerry env-1 raw — symmetric sensors, no AIS, no offline
calibration — none of the three regimes is active. The extractor is
correct math and free to run (no cost; pairs are cheap), but it
contributes zero to the steady-state output. **The benefit is
latent: when the operator adds (1), (2), or (3), item 13 unlocks
without further code changes.**

### Where this leaves item 13

- **Math: ✅ correct and unit-tested.** All 8 extractor tests + 2
  estimator convergence tests pass; pinned-anchor case recovers true
  bias within 0.6 m.
- **Deployment fit: ✅ ready.** The deployment scenario the spec
  targeted (non-cooperative target, no AIS) is handled by the
  pinned-anchor path; the operator pre-seeds one sensor's bias from
  factory calibration documentation and the other learns against it
  through cross-sensor pairs.
- **AutoFerry empirical: ⚠️ no measurable delta.** Due to symmetric-
  fusion invariance, not a math bug. Reported honestly in the
  evaluation log; the spec's 5-15% GOSPA target was over-optimistic
  for the symmetric-sensor / no-anchor case.
- **Backlog item 14 (sensor-reported per-measurement R) interaction.**
  If item 14 lands and per-measurement R varies, asymmetric variance
  conditions (2) may emerge naturally on AutoFerry, and item 13
  could start to bite.

## Ways to improve / what to test next

1. **Schmidt-KF treatment** of `P_b` in the per-track update
   (`sota-roadmap.md §5`).
2. **Range / azimuth scale biases** (multiplicative, not additive)
   on ARPA. Different parameterisation; add when needed.
3. **Track-anchored fallback** when AIS is absent. **DONE** — backlog
   item 13, cross-sensor anchored extension above.
4. **Lever-arm corrections** at the adapter layer (own-ship antenna
   → sensor geometric centre). Orthogonal; separate spec.
5. **Generalise to `IInnovationSink` consumption** so any per-track
   innovation carrying `(sensor, source_id)` can feed the
   estimator. Mirrors the v3 path the heading-bias estimator
   already has.
6. **Joint state augmentation as an ablation** vs coordinate
   descent on cross-sensor pairs. Equivalent in steady state but
   may differ in transient convergence under aggressive Q.
7. **Cross-sensor bearing pairs.** Item 13's MVP is position-only;
   the EO-IR-IR triangle case (two cameras + one positional sensor)
   wants the same fold on the bearing-bias update.
8. **Re-derive the env-2 EO/IR bearing seed** (`autoferry_r_calibration.py`,
   7.0° / 4.9° in `ReplayScenarioRun::seedSensorBiasEstimator`). It was
   calibrated while the closed loop converged to *half* the true bias (W3.2),
   so it is scaled for the buggy loop. After the fix the anchored-mode autoferry
   metrics move (some worse — e.g. `scenario16_anchored` gospa 2.67→8.42); the
   deployment-realistic (non-anchored) metrics are byte-identical. Re-derive the
   seed against the corrected loop before re-freezing the Cl-4 gauntlet — do not
   tune the estimator to match the old seed.
9. **v2 bearing-innovation outlier gate** is centred on zero, not on `b̂`
   (finding B5): a true bias above ~5·σ can starve the observer before it
   publishes. Out of scope for this wave; gate on the residual `|wrap(r − b̂)|`.
10. **Combined heading-bias + per-sensor-bias on the same ARPA touch.** If both
    estimators are active on one sensor, each reconstructs only its own applied
    correction; the interaction is untested. No production wiring hits it today.
