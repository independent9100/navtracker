# 17 — Multi-sensor fusion and the heading bias estimator

> Prerequisites: [03 — Bayes' rule](03-bayes-and-recursion.md),
> [10 — Measurement models](10-measurements-frames-time.md),
> [04 — KF](04-kalman-filter.md).
> Next: [18 — CPA + collision risk](18-cpa-and-collision-risk.md).

navtracker fuses four sensor streams: **AIS**, **ARPA** (radar),
**EO/IR** (camera), and **own-ship navigation** (GPS + heading).
Each has different timing, different noise, different frame, and
different reliability. This chapter explains how they combine
cleanly into a single track — and what extra estimation we run on
the side to keep them honest.

## 1. Why "fusion" is mostly just Bayes

If you accept that:

- each sensor's measurement model is `z = h(x) + v` with known
  `R`,
- measurements arrive at known times (with `Measurement.time`),
- measurement noises are independent across sensors,

then fusion is *automatic*: every measurement applies the same
Bayes update to the single shared posterior `(x̂, P)`. No special
"fusion layer" is needed. The Tracker just calls
`estimator.update(track, z)` regardless of which sensor `z` came
from.

This is the deep payoff of the **single-state, multi-sensor**
architecture: there is exactly one belief about the world, and
every sensor sharpens it.

```mermaid
flowchart LR
    AIS([AIS Measurement<br/>h=I, R_AIS])-->T[Track posterior<br/>(x̂, P)]
    ARPA([ARPA Measurement<br/>h=RangeBearing, R_ARPA])-->T
    EOIR([EO/IR Measurement<br/>h=Bearing, R_EO])-->T
    OWN([own-ship pose<br/>updates the datum])-->T
    T-->Out[fused state<br/>fused covariance<br/>fused identity]
```

## 2. The four sensors at a glance

### 2.1 AIS

- **What it gives**: vessel position (lat/lon), SOG, COG, MMSI,
  vessel type, name.
- **Noise**: GPS-grade, typically a few metres. Very low
  clutter (`λ_C ≈ 0` once parsed correctly).
- **Frequency**: irregular, 2–10 s between reports for moving
  vessels, much longer when stationary.
- **Gotchas**: vessels can turn AIS off, lie about MMSI, send
  stale positions. Treat MMSI as a **hint**, not as identity.
- **Measurement model**: `PositionVelocity2D` (chapter 10).

### 2.2 ARPA radar

- **What it gives**: range and bearing per tracked plot.
- **Noise**: a few metres of range, a few mrad of bearing.
- **Frequency**: per scan, typically 1–5 Hz.
- **Gotchas**: wave clutter in sea state, blob merging for
  closely-spaced targets, range ambiguity at close range.
- **Measurement model**: `RangeBearing2D`. Bearing is from
  own-ship's bow; we transform to the ENU frame in the adapter.

### 2.3 EO/IR camera

- **What it gives**: pixel position of detections, sometimes
  classifications, sometimes range estimates.
- **Noise**: small angular noise, large range noise (or no
  range at all).
- **Frequency**: 5–30 Hz nominally, lower under bad seeing.
- **Gotchas**: sun glare, fog/rain, occlusion, large angular
  field of view requiring careful pose handling.
- **Measurement model**: `Bearing2D` until range converges
  (chapter 05); then `RangeBearing2D`.

### 2.4 Own-ship navigation

- **What it gives**: own-ship lat/lon (GGA/RMC), heading (HDT
  from gyrocompass, optionally GPS-derived heading from
  multi-antenna systems, optionally magnetic).
- **Noise**: GPS metres; heading 0.1–2° depending on source.
- **Frequency**: 1–10 Hz.
- **Gotchas**: heading bias (sec. 4 below); GPS dropout in
  covered docks.

Own-ship is not a *target* measurement; it is what the tracker
uses to know **where own-ship is at the moment of every target
measurement**. That is what feeds the sensor pose into
range/bearing measurement models (chapter 10).

## 3. Why we treat AIS as just another sensor

A common temptation: *"AIS gives us the true MMSI; just use it
to label tracks!"*. We do not. Reasons:

- MMSI can be spoofed or wrong.
- A vessel can turn off AIS (an MMSI suddenly disappearing
  must not delete a track that ARPA is still tracking).
- A new MMSI mid-stream must not re-key an existing track
  (would break ID stability — invariant D1).

So MMSI is stored as a **track attribute** but is not the
fusion key. The fusion key is `track_id`. AIS measurements are
associated by **kinematic gating** (Mahalanobis), and the MMSI
is only a tiebreaker hint.

This is a deliberate architecture decision. See CLAUDE.md
"Stable track identity".

## 4. The heading bias problem

The own-ship gyrocompass drifts. Over hours, the reported
heading can be 1–3° wrong even on a good system. Every
range/bearing measurement from ARPA or EO/IR is *referenced to
this heading*. A heading bias of 1° at 5 km range translates
into ~87 m of cross-track error. Big enough to:

- Push ARPA measurements out of the gate of correctly-tracked
  AIS targets, breaking fusion.
- Bias the fused position over time, biasing CPA.

We need to estimate and subtract this bias. That is the job of
the **`HeadingBiasEstimator`**.

### The bias model

A scalar Kalman filter on a single state `b` (radians) with
random-walk dynamics:

```
b_t = b_{t-1} + w_b,    w_b ~ N(0, q_b · dt)
```

`b` drifts slowly. The filter is cheap; the design is in
`core/bias/HeadingBiasEstimator.{hpp,cpp}`.

### Observations of the bias

We can observe `b` from several sources:

| Kind                          | What it is                                                  | Math                                                |
|-------------------------------|-------------------------------------------------------------|-----------------------------------------------------|
| `AisArpaPairObservation`      | AIS-target bearing vs ARPA-measured bearing pair            | direct `b` measurement at the pair's range          |
| `BearingInnovation`           | Tracker's own bearing innovation, fed via `IBearingInnovationSink` | `r = wrap(β_obs − β_pred)`; needs an anchor track   |
| `GyroVsGpsHeadingObservation` | Multi-antenna GPS heading vs gyro                           | direct `b = gyro − gps_hdg`                         |
| `GyroVsGpsCogObservation`     | GPS course-over-ground vs gyro (with crab-angle uncertainty)| `b = gyro − cog`, gated by SOG/turn-rate            |
| `GyroVsMagneticObservation`   | Magnetic compass vs gyro (with declination)                 | `b = gyro − (mag + variation)`                      |

Any combination can be wired in. Some sources go quiet for hours
(e.g. AIS-target pairs); others fire continuously (GPS COG).
The KF handles arbitrary observation gaps because it is
*time-driven*, not scan-driven.

### Why a separate estimator and not bake it into the state?

Two reasons:

1. **Bias is per-platform**, not per-target. A single shared
   bias serves *all* targets. Baking `b` into every track state
   would replicate it 30 times.
2. **Observability**: target tracks rarely observe `b`
   directly. The dedicated observation kinds above are the
   only way to nail `b` down quickly.

The bias is then applied as a per-measurement correction *in
the adapter* before the measurement enters the Tracker. This
keeps the core estimator unaware of platform-level effects.

## 5. Combining distributions across sensors — three patterns

We have already covered the three Gaussian operations
(chapter 02 §8) but it is worth saying which navtracker uses
them for:

- **Product (sharpening from independent observations).** The
  Kalman update step combines the predicted prior with the
  measurement likelihood. Used every time a measurement
  arrives, regardless of sensor.
- **Mixture (per-mode in IMM, per-leaf in MHT).** The IMM and
  MHT both maintain *multiple* Gaussians per target, mixed
  at the output. Collapsing back to a single Gaussian uses
  moment matching.
- **Sum (uncertainty stacking).** Process noise adds to state
  covariance during predict. Sensor noise adds to predicted
  measurement covariance during update. Sensor-pose
  uncertainty adds to range/bearing `R` via the
  composition step (own-ship covariance composed into
  `R` for moving-sensor measurements).

You will see all three throughout the code. They are not
separate techniques; they are special cases of one Bayesian
recipe.

## 6. Composing own-ship covariance into `R`

The `makeMeasurementFromRelativeBearing(...)` helper
(`CLAUDE.md` library-use section) does an under-appreciated
thing: it **composes own-ship heading and GPS uncertainty
into the target measurement's `R`**.

Why? Because a 1° heading error and a 5 m GPS error are both
*real noise* in the resulting ENU position estimate of the
target. If we ignored them, the radar measurement would look
artificially precise, NIS would spike, and gating would falsely
reject good fusions.

The helper composes:

```
R_final = R_sensor + J_pose · P_pose · J_poseᵀ
```

where `J_pose` is the Jacobian of the ENU target position with
respect to the own-ship pose. The result is a slightly inflated
`R` that honestly reflects the full uncertainty chain.

## 7. Assumptions

| Assumption                                           | When it pinches                                       |
|------------------------------------------------------|-------------------------------------------------------|
| Sensor noises independent across sensors             | Mostly true. Shared platform vibration may correlate. |
| Heading bias is a single scalar that drifts slowly   | Step changes at gyro restart need re-initialisation.  |
| Own-ship covariance is honest                        | Bad GPS → cascade of bad fusions; fall back to inertial. |
| Measurement timestamps are accurate                  | NTP drift → ordering bugs.                            |
| MMSI is a hint, not identity                         | Built into the architecture.                          |

## 8. Why we can use this fusion architecture here

The hexagonal core + per-sensor adapter pattern is the cleanest
way to fuse heterogeneous sensors. The bias estimator handles
the only systematic error that crosses sensor boundaries
(heading). Everything else is sensor-local and the Kalman math
takes care of it automatically.

The library boundary is `Measurement` and `OwnShipPose`. Any new
sensor that can produce a `Measurement` with the right
`(model, z, R, sensor_position_enu)` plugs in without core
changes. That is the test of a good fusion architecture.

## 9. Where this lives in code

- `core/types/Measurement.hpp` — the library contract.
- `adapters/own_ship/OwnShipProvider.hpp` — auto-datum, GPS,
  heading, `IDatumChangeSink`.
- `core/bias/HeadingBiasEstimator.{hpp,cpp}` — the scalar KF.
- `adapters/own_ship/OwnShipNmeaAdapter` — dispatches the v3
  bias observation kinds automatically.
- `core/projection/` — helpers that compose own-ship covariance
  into per-measurement `R`.
- `tests/integration/test_full_stack_pipeline.cpp` — the
  canonical end-to-end example with all sources wired.

## 10. What we did not pick, and why

- **One huge augmented state** with own-ship + every target
  jointly estimated. Theoretically optimal, in practice an
  `O(N²)` covariance to maintain. Not worth it for ~30 tracks.
- **Hard MMSI fusion**: use MMSI as the fusion key. Discussed
  and rejected for identity-stability reasons.
- **Per-sensor batch processing** (collect a scan, fuse at
  once). The current single-measurement orchestration matches
  the asynchronous, multi-rate reality better. See
  `docs/algorithms/pipeline.md`.

---

Previous: [16 — NEES / NIS](16-nees-nis.md)
Next: [18 — CPA and collision risk](18-cpa-and-collision-risk.md) →
