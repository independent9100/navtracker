# Inter-sensor registration bias estimation — design spec

Backlog item 9. Closes the calibration gap the Helgesen 2022
comparison surfaced: navtracker uses lidar / radar / EO / IR but
treats their measurements as unbiased in the common ENU frame, while
the paper carefully calibrates each sensor's mounting offset against
RTK-GNSS truth. On AutoFerry these biases fold silently into pos_rmse
and gate sizes; once the heading-bias estimator converges, *residual*
sensor-mounting bias becomes the dominant unmodelled term.

## Scope

**In scope.** Per-non-anchor-sensor position bias (radar, lidar) and
per-camera bearing bias (EO, IR). Estimated as separate filters,
fed by cross-sensor observations against an *anchor* (AIS), exactly
analogous to the v1 AIS↔ARPA pair flow that feeds
`HeadingBiasEstimator`.

**Anchor.** AIS is the cleanest choice when the target is AIS-broadcasting:
its absolute lat/lon comes from the target's own GNSS, independent of
own-ship sensors. When AIS is absent, the high-confidence anchor falls
back to a *converged track's* position. The converged-track fallback
is deferred until the AIS-anchored path is measured — anchoring on a
track that itself depends on the biased sensor is a cycle.

**Not in scope (this spec).**
- Range scale / azimuth scale errors on ARPA (a different parameter
  class — multiplicative, not additive).
- Schmidt-KF / "considered" treatment of the converged bias in the
  per-track update — that is `sota-roadmap §5` and stays separate.
- Online recalibration during mission. The bias is treated as a
  slowly-drifting random walk; full recalibration belongs upstream.
- Lever-arm corrections (own-ship antenna geometry). Those are
  static and live in the adapter layer if they matter.

## Math

### State

One independent estimator per non-anchor (sensor, source_id) tuple.

**Position-bias estimator (Position2D sensors, lidar / radar).**
State `b ∈ ℝ²`, the mounting/translation offset of that sensor's
measurements in the common ENU frame at the time the measurement is
taken (so it absorbs both static mounting offset and any slowly-
varying calibration drift).

```
x_b  = b ∈ ℝ²          (state)
P_b  ∈ ℝ²ˣ²           (covariance)
```

Random-walk dynamics with isotropic process noise:

```
b(t+dt) = b(t) + w,    w ~ N(0, Q_b · dt)
Q_b = σ²_drift · I₂   (m² per second)
```

Default `σ²_drift = (0.1 m)² / 3600 s` → drift std ~ 0.1 m over an
hour. Justification: thermal / mechanical mounting biases vary
*slowly*; the heading-bias precedent uses ~2 deg/hr for a gyro, this
is the position analogue.

**Bearing-bias estimator (Bearing2D sensors, EO/IR).** State scalar
`β ∈ ℝ`, the bearing offset added to that camera's reported bearings.
Random-walk dynamics, defaults `σ²_drift_β = (0.05 rad)² / 3600 s`.

### Observation

At time t, when an anchor (AIS) and a non-anchor sensor produce a
detection of the *same* target within a tight time window and gate:

**Position bias.** Anchor gives `z_anchor = y(t) + n_anchor`,
sensor gives `z_sensor = y(t) + b + n_sensor`. The disagreement is
a direct measurement of b:

```
r = z_sensor − z_anchor = b + (n_sensor − n_anchor)
R_obs = R_sensor + R_anchor
```

Sequential measurement of b. Standard scalar/2D KF update.

**Bearing bias.** Anchor gives target ENU position y(t); sensor gives
bearing α from sensor position p_s:

```
α_predicted = atan2(y_y − p_y, y_x − p_x) + β
r           = wrap(α_observed − α_predicted)
H_β         = 1   (scalar derivative w.r.t. β)
R_obs       = σ²_α_meas + R_anchor·∇ + R_own_pos·∇
```

The anchor-position uncertainty projects onto bearing via the
geometric Jacobian; this is the same projection
`HeadingBiasEstimator` already implements for the AIS↔ARPA case
(see `AisArpaPairObservation`).

### Observability gates

Three gates, modelled directly on the heading-bias estimator's
G1-G2-G3:

1. **Time-window gate.** `|t_anchor − t_sensor| ≤ Δt_max` (default
   1.0 s). Beyond that, target may have moved relative to its truth.
2. **Range gate.** `range(target) ≥ r_min` (default 50 m). At very
   short range the geometric Jacobians become ill-conditioned for
   bearing-bias (small position error → large bearing error → high
   observation variance).
3. **Innovation gate.** `||r|| ≤ N · √(diag(R_obs + P_b))` (default
   N = 5). Rejects pair mismatches (wrong target paired).

### Application

A converged bias is applied during measurement processing — when the
estimator's posterior variance falls below a publish threshold (same
mechanism `HeadingBiasEstimator::publishVarianceThreshold` uses):

- Position2D sensor S, before any tracker association: subtract
  `b̂_S` from `z_sensor.value`.
- Bearing2D camera C: subtract `β̂_C` from the reported bearing.

The bias *uncertainty* `P_b` is NOT inflated into the per-track
covariance in this spec (that is `sota-roadmap §5`, Schmidt-KF). The
correction is applied as a deterministic shift; the residual bias
uncertainty is small once the gate triggers.

## Assumptions

- The anchor (AIS) is unbiased relative to the truth. Reasonable for
  modern Class-A AIS (~5-15 m position accuracy, no systematic
  offset). Class-B can be biased — gate by AIS class if it matters.
- Mounting bias is **slowly-varying** (random walk with very small
  Q). If a sensor's bias jumps (e.g. recalibration / mast knock) the
  estimator catches up but with lag.
- Targets are **point objects** at the AIS-reported position. For
  small leisure craft this is fine. For Gunnerus (31 m × 7 m) the
  AIS antenna may be tens of metres from where radar / lidar centre
  their return — this is a *bounded* offset that the estimator will
  absorb into the bias. The actual mounting bias is recoverable only
  for small targets; for large targets we estimate "effective bias =
  mounting + antenna-to-radar-centroid offset" which is fine for the
  tracker but is *not* a sensor calibration number.
- Cross-sensor pair counts are sufficient. Each estimator needs O(10)
  independent observations to converge. AutoFerry env 1 with 2 AIS
  vessels across 5 scenarios gives O(100-1000) per-sensor pairs.

## Rationale

### Why per-(sensor, source_id) and not a single shared bias

EO and IR cameras have different mountings, different drift
characteristics. Lidar and radar share the same mast on milliAmpere
but their digital pipelines are independent. Each physical sensor
gets its own bias.

### Why separate filters instead of augmenting the tracker state

Per the heading-bias precedent. A separate slow random-walk filter:

- isolates bias dynamics (very slow) from track dynamics (fast);
- decouples bias convergence speed from track update rate;
- lets one estimator share information across tracks — the
  millimetre-accurate posterior built from 200 pair observations
  benefits a track that only had 2 detections.

The tracker-augmented alternative (carrying the bias as part of each
track's state vector) would require N copies for N tracks and lose
the cross-track information.

### Why AIS, not a converged-track anchor

A track that uses the biased sensor cannot be its own bias anchor —
that would close the loop and converge to whatever bias minimises
*self-consistency*, not to the true offset. AIS is the only signal in
our pipeline that is independent of the sensors being calibrated.

In scenarios with no AIS the bias estimator just doesn't converge for
that scenario. That is the honest behaviour.

### Why apply as deterministic shift, not Schmidt-KF

Schmidt-KF requires augmenting every track update with a 2×2 block
plus cross-covariance for every biased sensor. That is a big change
in the per-track update path. Once the bias is converged
(`P_b` < publish threshold), the variance contribution to the
per-track covariance is small — much smaller than the existing
mismatch we are trying to fix. We will revisit Schmidt-KF if the
deterministic shift improves GOSPA but pos_rmse stops moving (the
signature that residual bias variance is the next dominant term).

## Wiring

New port: `ISensorBiasProvider` — analogous to
`IHeadingBiasProvider`. Returns `(b̂_S, P_b_S)` per (sensor,
source_id) on demand. Implementations:

- `SensorBiasEstimator` — the actual estimator, KF on each bias,
  feeds via `AnchorPairObservation` (a new struct).
- `NullBiasProvider` — returns zeros, the default for callers that do
  not wire one.

Adapters apply the bias correction when reading from the provider:

- `adapters/ais` and `adapters/radar` / `adapters/lidar` are the
  pair-feed sources. They emit `AnchorPairObservation` whenever a
  pair is gated.
- `Tracker::process(Measurement&)` applies `b̂_S` before predict /
  associate when `provider != nullptr`.

The pair-feed lives outside the core (it knows about AIS specifically)
— mirrors the `AisArpaPairExtractor` that already feeds
`HeadingBiasEstimator`.

## Acceptance criteria

1. **Unit tests pass.** Constant true-bias scenario: estimator
   converges within 20 paired observations to within 0.1 m / 0.01 rad
   of truth, posterior variance below publish threshold.
2. **Observability gates work.** Range-gated pairs at < r_min do not
   move the bearing-bias state.
3. **Existing replays bit-identical** when `bias_provider == nullptr`
   (the default).
4. **GOSPA env 1 improvement.** When wired into the canonical
   AutoFerry config, env 1 GOSPA RMS drops measurably (target:
   43 m → 35-40 m, a ~15-25% reduction). The full gap to the paper
   (43 → 20) needs JIPDA on top, but item 9 alone should close a
   chunk.
5. **No regression on philos / synthetics.** Synthetics have no AIS
   anchor → bias provider stays at the initial (zero, broad-var)
   prior and the deterministic shift is zero. Bit-identical to today.

## Ways to improve / what to test next

1. **Schmidt-KF treatment of `P_b` in the per-track update**
   (`sota-roadmap §5`). The natural follow-on if GOSPA improves but
   the residual gap is large.
2. **Range / scale biases.** Multiplicative biases (radar range
   scale, gain on lidar return strength) are not in this spec. They
   need a different parameterisation (one scalar per sensor) and a
   different observation projection. Add when needed.
3. **Track-anchored fallback** when AIS is absent. Requires solving
   the cyclic-anchor problem — likely "lock the bias once converged
   on AIS, only then permit track-anchored observations". Spec out
   when AIS coverage drops below a measured threshold.
4. **Lever-arm corrections** at the adapter layer (own-ship antenna →
   sensor geometric centre). Separate spec; orthogonal to this work.
5. **Generalise to `IInnovationSink` consumption** so any per-track
   innovation that already carries the (sensor, source_id) tuple can
   feed the estimator (cleaner than a separate pair-feed for every
   sensor). Mirrors the v3 path the heading-bias estimator already
   has.

## What we did NOT pick, and why

- **Augmented per-track state.** N copies for N tracks; loses
  cross-track information; pollutes the per-track filter with
  slow-dynamics state. Rejected.
- **Online sensor R recalibration** (VB-AKF, `sota-roadmap §5`).
  Different problem — R is the *unknown* noise variance, b is the
  *unknown* offset. Both are real issues but conflating them in one
  recursion makes the math harder than necessary.
- **Static calibration only.** Mounting biases drift (thermal,
  mechanical settling, recalibration events). A slow random walk is
  the right model.
