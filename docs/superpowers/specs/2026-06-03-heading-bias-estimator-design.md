# Heading Bias Estimator — Design

**Date:** 2026-06-03
**Status:** Approved, ready for plan

## 1. Motivation

The §14.9 work (commits `3c32554`..`af00c00`) added passive σ_heading budgeting: heading uncertainty inflates measurement covariance R so the filter trusts long-range bearings less. The eval-log shows this recovers ~74 m of the ~95 m loss at σ_h = 2° on BearingOnlyMoving — leaving ~21 m on the table because inflation absorbs the error rather than removing it.

This spec adds the **active** side: estimate the own-ship heading bias `b` as a tracker state and feed the correction back into adapters. Where σ_heading R-inflation buys robustness against unknown bias, the estimator buys *accuracy* once enough information is available to pin b down.

The two mechanisms compose: as `var(b̂)` shrinks, the residual heading uncertainty fed into R-inflation shrinks with it. The filter stays correctly skeptical when b is unobservable and becomes correctly confident once it isn't.

## 2. Scope

In scope:
- Scalar global heading bias `b` modeled as a random walk.
- Observation channel: AIS-vs-ARPA position residuals on tracks the tracker has already fused.
- Application point: ARPA and EO/IR adapters subtract `b̂` from raw heading before projecting bearings.
- Observability gating with graceful fallback (AIS drops → estimator stops publishing → adapters revert to `b̂ = 0`).
- Validation via extended §14.9 sweep scenarios with an added AIS anchor.

Out of scope (deferred):
- **Multi-track bearing-innovation observer** (Option 2 from brainstorming). The `IHeadingBiasProvider` interface is forward-compatible so a future estimator can plug in without adapter changes.
- **Per-sensor / per-gyro bias**. Single global `b` for now.
- **AIS-vs-EOIR** as an independent anchor. EOIR shares `b`, so it isn't independent.

## 3. Architecture

```
Sensor messages → adapters (apply ψ̂ = ψ_raw − b̂) → tracker → fused tracks
                       ↑                                          │
                       │                                          ▼
                       └─── b̂  ←── HeadingBiasEstimator ◄── AIS+ARPA pairs
```

**New module:** `core/bias/`
- `HeadingBiasEstimator.hpp / .cpp` — owns the scalar state `(b̂, P_b)`, consumes per-cycle AIS+ARPA pair observations, implements predict + sequential KF updates, exposes the current published estimate.

**New port:** `ports/IHeadingBiasProvider.hpp`
```cpp
struct HeadingBiasEstimate {
  double bias_rad{0.0};
  double variance_rad2{0.0};
  bool is_published{false};  // gating flag
};

class IHeadingBiasProvider {
 public:
  virtual ~IHeadingBiasProvider() = default;
  virtual HeadingBiasEstimate current() const = 0;
};
```
`HeadingBiasEstimator` implements `IHeadingBiasProvider`. Adapters take a `const IHeadingBiasProvider*` (nullable).

**Tracker:** unchanged. The estimator is a sibling component wired in `app/`, not a tracker stage.

**Composition flow per cycle** (composition root pseudo-code):
1. Adapters consume sensor messages, querying the bias provider for the current `b̂` to apply.
2. Tracker processes the measurements; produces fused track snapshot.
3. Composition root inspects the snapshot for tracks fused from both an AIS and an ARPA source at the current cycle's truth-tick, extracts the (track_id → AIS-source position, ARPA-source bearing-projected position) pairs, and feeds them to `HeadingBiasEstimator::update(...)`.
4. Estimator runs predict + updates; updates internal `(b̂, P_b)` and the published value (subject to gating).

Step 3 is the only new piece of composition logic. Identifying AIS+ARPA fused tracks requires the tracker to expose the source-type provenance of recent measurements per track — see §6.

## 4. Math

### 4.1 State and process model

Scalar state `b` (rad). Random walk:

```
b_{k+1} = b_k + w_k,    w_k ~ N(0, Q_b · Δt)
```

`Q_b` is config (units: rad²/s). Default chosen so a 2°/hr drift stays within ±1σ:

```
Q_b_default ≈ (2° · π/180 / 3600 s)² ≈ 9.4e-11 rad²/s
```

(Implementation may take it as `q_b_deg_per_hr` config and convert; see §6.)

Predict step at time interval Δt:
```
b̂_k|k−1 = b̂_{k−1|k−1}
P_b_k|k−1 = P_b_{k−1|k−1} + Q_b · Δt
```

### 4.2 Measurement model

For an AIS+ARPA pair fused into the same track at cycle k with own-ship position `(x_o, y_o)` (ENU):

- AIS gives target position `(x_a, y_a)` directly.
- ARPA gives relative bearing β_rel and range r; current code computes `β_enu_arpa = ψ_raw + β_rel`.

Define:
```
β_truth = atan2(y_a − y_o, x_a − x_o)
z_k     = wrap_to_pi(β_enu_arpa − β_truth)
```

Then to first order, `z_k = b + v_k` with
```
σ²_v = σ²_β_arpa + σ²_β_ais
σ²_β_ais ≈ σ²_ais_pos / r²   (small-angle approximation of AIS position noise projected to bearing)
```

`σ_β_arpa` is the per-measurement ARPA bearing std (already known per measurement). `σ_ais_pos` is taken from AIS-source measurement covariance; the conversion to angular variance uses target range.

### 4.3 Update step

Standard scalar Kalman, applied once per AIS+ARPA pair (sequential update if multiple in one cycle):

```
y = wrap_to_pi(z − b̂)
S = P_b + σ²_v
K = P_b / S
b̂ ← b̂ + K · y
P_b ← (1 − K) · P_b
```

`wrap_to_pi` keeps innovations on `(-π, π]`.

### 4.4 Coordinate / sign conventions

`b` is defined such that `ψ_corrected = ψ_raw − b`. A positive `b` means the reported gyro reads higher than truth (rotated CCW in ENU). Adapters apply the correction by subtracting `b̂` from `pose.heading_true_deg * kDeg2Rad` before adding the relative bearing.

## 5. Adapter integration

### 5.1 Interface change

`ArpaAdapter` and `EoIrAdapter` constructors gain one parameter:
```cpp
explicit ArpaAdapter(ArpaAdapterConfig cfg = {},
                    const IHeadingBiasProvider* bias_provider = nullptr);
```
Backward compatible — nullable, defaulted last.

### 5.2 Effective heading and effective σ_heading

Inside the adapter, when handling each bearing-bearing measurement:

```cpp
HeadingBiasEstimate est = bias_provider_ ? bias_provider_->current()
                                         : HeadingBiasEstimate{};
const double b_hat = est.is_published ? est.bias_rad : 0.0;
const double var_b_hat = est.is_published ? est.variance_rad2 : 0.0;

const double heading_rad = pose.heading_true_deg * kDeg2Rad - b_hat;

const double sigma_heading_eff =
    std::sqrt(cfg_.heading_std_deg * cfg_.heading_std_deg * kDeg2Rad * kDeg2Rad
              + var_b_hat);
```

`sigma_heading_eff` is the value passed to `projectRangeBearingToEnu(...)`. As `var(b̂) → 0`, R-inflation collapses to the configured residual heading std (representing remaining un-modeled jitter). When the estimator hasn't published yet (`is_published == false`), behavior is identical to today.

### 5.3 No tracker changes

The tracker continues to consume `Measurement` objects with the projected ENU position and covariance from the adapter. It does not need to know `b` exists.

## 6. Identifying AIS+ARPA fused pairs

`HeadingBiasEstimator::update(...)` needs (per cycle): the set of tracks for which both an AIS and an ARPA measurement contributed.

Two options, listed with the chosen one first:

**Chosen:** annotate `Track` with a small `last_source_contributions` ring of `{source_type, measurement_index, timestamp}` per cycle. The composition root walks tracks in the current snapshot and emits `(track_id, ais_pos, arpa_proj_pos, arpa_range, σ_β_arpa, σ_ais_pos)` tuples when both source types appear in the ring inside the current cycle window. This keeps the tracker stateless about bias but exposes provenance.

**Alternative considered:** pass observations to the estimator directly from inside `TrackManager`. Rejected — couples the tracker to bias logic, violating the architecture invariant "core has zero sensor-format knowledge."

Concrete `Track` extension: a fixed-size `std::array<SourceTouch, 4>` (overwrite-oldest) suffices; cycle window means anything older than the current cycle's measurement set is ignored.

## 7. Observability gating

`HeadingBiasEstimator` publishes `(b̂, P_b, is_published=true)` only when:

1. `P_b ≤ P_b_publish_threshold` (config, default `(0.3°)² ≈ 2.74e-5 rad²`)
2. At least one update applied within the last `T_stale_seconds` (config, default 30 s)

Outside those conditions, `is_published = false` and adapters revert to `b̂ = 0`. After a long AIS dropout the variance grows (random-walk Q accumulates during predict-only); once it exceeds the publish threshold, gating closes naturally. No special "reset" logic required.

Initial conditions: `b̂_0 = 0`, `P_b_0 = (5°)²` (config). Wide initial prior means the first valid AIS+ARPA pair drives `b̂` strongly toward the true value.

## 8. Assumptions

- One own-ship compass/gyro — bias is global, not per-sensor.
- Bias varies slowly relative to the AIS update rate (~10 s typical). Random-walk Q chosen accordingly.
- AIS position accuracy is significantly better than ARPA bearing-projected position at long range — i.e. AIS is a reasonable "truth anchor" for bearing residuals.
- The tracker's data association correctly fuses an AIS report with the corresponding ARPA report. Mis-association would inject false observations into the bias estimator; this is the same risk the tracker itself runs and we don't double-mitigate.
- Small-angle approximation for converting AIS position noise to angular noise: valid when `σ_ais_pos << r`. At r > 100 m and `σ_ais_pos ~ 10 m`, the error is < 1%.

## 9. Rationale

**Why a global state and not per-track?** The bias is a physical property of the own-ship compass, not the targets. Per-track would split a globally observable parameter into N weakly-observable copies, wasting information. Global also matches how operators think about gyro calibration.

**Why random walk and not static?** §14.9 modeled both static bias and drift; the eval-log probe at 0.03°/s shows drift is the dominant mode in some scenarios. Random walk subsumes both — Q controls how aggressively the estimator forgets old data.

**Why adapter-side correction and not tracker-side?** The tracker's job is fusion under uncertainty. The adapter's job is rotating relative measurements into the world frame using the best available ψ. Bias correction is a refinement to that rotation, so the adapter is the right home. Keeps the tracker bias-agnostic and preserves the per-track independence the codebase is built around.

**Why AIS-vs-ARPA only (not also bearing-innovation) in v1?** Two reasons. (1) Clean signal — motion-independent, high SNR, mathematically a direct measurement of b. We see fast convergence and can validate behavior cleanly. (2) Forward-compatible interface — adding a multi-track bearing-innovation observer later doesn't touch adapters or the gated provider; it adds a second `update(...)` overload or a sibling estimator that contributes to the same published estimate.

**Why R-inflation stays after the estimator lands?** Until `b̂` is published the σ_h budget is still doing real work, and once published the residual heading jitter (truncation, NMEA quantization, sub-cycle drift) is still real. The two combine in quadrature; one doesn't replace the other.

## 10. Test plan

### Unit (`tests/bias/test_heading_bias_estimator.cpp`)

1. **Construction** — initial published state has `is_published = false`, `b̂ = 0`, `P_b = (5°)²`.
2. **Single-pair update** — inject one AIS+ARPA pair with known bias 1°; verify `b̂` moves toward 1°, `P_b` shrinks.
3. **Convergence** — stream 30 pairs at 1 Hz with bias = 2°; verify `|b̂ − 2°| < 0.1°` and `is_published == true` within 30 s.
4. **Random-walk drift** — true bias slides from 1° to 3° over 60 s; verify estimator tracks within 0.3°.
5. **Anchor loss** — converge for 60 s, then no pairs for 60 s; verify `is_published` flips to false after `T_stale_seconds`.
6. **Observability gating** — single noisy update; verify `is_published = false` until enough updates drop `P_b` below threshold.
7. **Wrap-around safety** — true bias near ±π; verify innovation wraps correctly and estimator converges.

### Adapter integration (`tests/adapters/...`)

8. **ARPA adapter with provider** — adapter with mock `IHeadingBiasProvider` returning published `(b̂ = 2°, var = 0.01°²)` applies the correction to ENU bearings; verify projected position rotates by exactly −2°.
9. **ARPA adapter unpublished** — provider returns `is_published = false`; adapter behavior identical to no-provider path.
10. **R-inflation composes** — `var(b̂) > 0` adds in quadrature to `cfg_.heading_std_deg`; verify against `projectRangeBearingToEnu` output.

### Scenario sweep (`tests/sim/test_bus_bias_estimator_sweep.cpp`)

Extend §14.9 sweep scenarios (`ClutterCrossing`, `BearingOnlyMoving`, `Maneuvering`) with one AIS-equipped target added to the scene. 20 seeds (201..220), σ_h ∈ {0°, 0.5°, 1°, 2°}, three rows per σ_h:
- R-off, no estimator (baseline from §14.9)
- R-on, no estimator (current best from §14.9)
- R-on, estimator (new)

`SUCCEED()`-only; data capture for eval-log.

### Anchor-loss scenario (`tests/sim/test_bus_anchor_loss.cpp`)

Single scenario: AIS active 0–60 s, drops 60–120 s. Verify:
- Tracker accuracy on the ARPA target tracks the converged b̂ during 0–60 s.
- After 60 s, gating closes and behavior reverts to R-inflation-only.
- No accuracy cliff at the dropout moment.

### Eval-log

Append "Heading bias estimator (2026-06-03)" section with sweep tables and verdict paragraph quantifying recovered accuracy beyond R-inflation-only.

## 11. Ways to improve / what to test next

1. **Multi-track bearing-innovation observer** (Option 2 from brainstorming). Sits in the same `HeadingBiasEstimator` or alongside; consumes ARPA bearing innovations from all tracks, gated by an observability check on track-bearing geometry. Earns its keep in zero-AIS scenes.
2. **First-order Gauss-Markov dynamics** instead of random walk — bounded excursions, one extra hyperparameter α. Probably small win unless drift is large and mean-reverting in practice.
3. **Per-sensor bias** — separate b for the navigation gyro versus a stabilized camera pedestal. Likely only matters if EO/IR is mounted independently with its own IMU.
4. **Cross-validate against GPS-derived heading-over-ground** when own-ship is moving — independent check on `b̂` without needing target AIS at all.
5. **Sensitivity sweep** on `Q_b` and `P_b_publish_threshold` — verify default doesn't over-trust or over-discount real-world drift rates.

## 12. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| State scope | Global scalar | One compass; multi-track info should fuse into one estimate. |
| Dynamics | Random walk | Subsumes both static-bias and drift error modes from §14.9. |
| Observer (v1) | AIS-vs-ARPA position residual | Motion-independent direct measurement of b. |
| Observer (v2) | Multi-track bearing innovations | Deferred; covers zero-AIS scenes. |
| Application point | Adapter-side (subtract from ψ_raw) | Keeps tracker bias-agnostic; matches per-track architecture. |
| R-inflation interaction | `σ_heading_eff² = σ²_heading + var(b̂)` | Composes; R-inflation shrinks naturally as confidence grows. |
| Gating | Publish iff `P_b ≤ thresh` AND fresh update within T_stale | Graceful fallback when AIS drops. |
| Validation | Extend §14.9 sweep + anchor-loss scenario | Directly comparable to existing baselines. |
