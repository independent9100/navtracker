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

### 3.1 The opposite case: pseudo-measurements

AIS is a **good** witness because it is *independent*: the
target's own GPS, a fresh source of information we don't already
have. But some inputs only *look* like new witnesses. The clearest
example is **another tracker's output** — a shore radar or VTS
station that sends you finished *tracks*, not raw blips.

Why is that different from a real sensor? Because a track is
already the *answer* to a tracking problem someone else solved. It
has been smoothed, associated, and filtered. If you feed it to your
tracker as if it were a raw measurement, two things go wrong:

- **It is not fresh evidence.** A shore feed's *velocity* is just
  its own smoothed guess from the positions it already sent you.
  Feeding both the positions and the velocity is like asking the
  same person twice and counting it as two opinions. The filter
  then thinks two witnesses agreed, and becomes **overconfident** —
  it shrinks its uncertainty too far, its gates get too small, and
  it starts missing real associations.
- **It is correlated with itself over time.** Consecutive outputs
  of a filter are not independent samples; each one carries the
  memory of the last. Ten updates a second is not ten independent
  looks.

So we call another tracker's output a **pseudo-measurement** and
treat it with suspicion. Two cheap, honest defences (no fancy
maths):

1. **Inflate the noise (`R`).** Multiply the stated covariance by a
   factor (default ×3) — tell our filter "trust this less than it
   claims". Being *too* careful only wastes a little precision;
   being *not careful enough* is the dangerous direction, so we err
   pessimistic.
2. **Thin the rate.** Keep at most one update every couple of
   seconds per remote track, so we don't mistake a filter repeating
   itself for many independent looks.

The proper cure (called *track-to-track fusion* / *covariance
intersection*) is more work and we have deliberately **not** built
it yet — inflate-and-thin is enough until a real feed proves
otherwise. The tell that it is no longer enough is a **NEES** check
(chapter 18): if the fused track's error is consistently larger
than the filter's claimed uncertainty, the pseudo-measurement is
making us overconfident and it is time for the real cure.

This is the same wariness we apply to ARPA's own speed/course
fields (chapter 2.2) — the radar computing a derivative of blips it
already handed us. The rule is one sentence: **before feeding any
value, ask whether it is new information or your own data coming
back smoothed.** Consumer-side wiring: integration-guide §3
("another tracker's output").

> The heading bias is **one** type of bias the heterogeneous
> sensor stack must deal with. There is a second layer:
> per-sensor mounting offsets that remain after heading bias is
> removed. Chapter 21 covers
> [inter-sensor registration biases](21-sensor-registration-bias.md)
> — same KF pattern, one estimator per (sensor, source_id), AIS
> as the anchor.

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

### Which way does the bias point? (one convention, watch the sign)

There are two ways to measure an angle, and they turn in *opposite*
directions:

- **Compass** (what sailors use): 0° is north, and the number grows
  as you turn **clockwise** (east = 90°, south = 180°).
- **Map-math** (what the tracker uses inside): 0° is east, and the
  number grows as you turn **counter-clockwise** (north = 90°).

A gyro that reads a little **too high** on the compass (bias `b`) pushes
a radar blip **clockwise** — which, on the map-math side, is the
**opposite** (counter-clockwise-negative) direction. So the same
physical error looks like `+b` in one frame and `−b` in the other.

We pick **one** convention for the stored bias: `b` is the **compass**
error the gyro adds to the true heading (`gyro = true + b`), and the
adapter removes it with `corrected = measured − b`. Three of the five
observation kinds already speak this language (they compare the gyro to
another compass heading). The two that work in map-math coordinates —
the AIS↔ARPA pair and the bearing-innovation — are **flipped to the
compass convention as they enter the filter**. If they were not, they
would fight the other three and pull the estimate toward `−b`, and the
adapter would then *double* the error instead of removing it. (This was
a real bug, fixed in fix-wave wave 3.)

### Observations of the bias

We can observe `b` from several sources. "Frame" is the convention the
raw value arrives in; the filter converts everything to the one compass
convention above:

| Kind                          | What it is                                                  | Frame → math                                             |
|-------------------------------|-------------------------------------------------------------|----------------------------------------------------------|
| `AisArpaPairObservation`      | AIS-target bearing vs ARPA-measured bearing pair            | map-math → `z = wrap(β_ais − β_arpa)` (flipped to compass) |
| `BearingInnovation`           | Tracker's own bearing innovation, fed via `IBearingInnovationSink` | map-math → `b = −wrap(β_obs − β_pred)` (negated); needs an anchor track |
| `GyroVsGpsHeadingObservation` | Multi-antenna GPS heading vs gyro                           | compass → `b = gyro − gps_hdg`                            |
| `GyroVsGpsCogObservation`     | GPS course-over-ground vs gyro (with crab-angle uncertainty)| compass → `b = gyro − cog`, gated by SOG/turn-rate       |
| `GyroVsMagneticObservation`   | Magnetic compass vs gyro (with declination)                 | compass → `b = gyro − (mag + variation)`                 |

Any combination can be wired in. Some sources go quiet for hours
(e.g. AIS-target pairs); others fire continuously (GPS COG).
The KF handles arbitrary observation gaps because it is
*time-driven*, not scan-driven.

### The feedback trap: measure the whole bias, not half of it

Once the estimate is good enough, the adapter starts subtracting it from
every ARPA/EO-IR bearing. But the AIS↔ARPA pairs are built *from those
already-corrected measurements*, so a pair only shows what is **left
over** after the correction, not the whole bias. If the filter then
subtracts its estimate a second time, it settles at **half** the real
bias — and every corrected bearing keeps the other half forever, while
the filter reports (wrongly) that it is confident.

The fix: each measurement remembers how much correction was already
applied to it (carried on `Measurement`/`SourceTouch`), and the pair
builder **adds that back** to rebuild the original, uncorrected bearing
before the filter looks at it. Now the filter sees the whole bias and
converges to the true value. (Fix-wave wave 3; the same "rebuild the raw
value" idea fixes the per-sensor registration bias in chapter 21.)

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
