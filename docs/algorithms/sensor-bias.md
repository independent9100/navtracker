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

## Ways to improve / what to test next

1. **Schmidt-KF treatment** of `P_b` in the per-track update
   (`sota-roadmap.md §5`).
2. **Range / azimuth scale biases** (multiplicative, not additive)
   on ARPA. Different parameterisation; add when needed.
3. **Track-anchored fallback** when AIS is absent. Requires solving
   the cyclic-anchor problem (lock bias once converged on AIS,
   only then permit track-anchored observations).
4. **Lever-arm corrections** at the adapter layer (own-ship antenna
   → sensor geometric centre). Orthogonal; separate spec.
5. **Generalise to `IInnovationSink` consumption** so any per-track
   innovation carrying `(sensor, source_id)` can feed the
   estimator. Mirrors the v3 path the heading-bias estimator
   already has.
