# navtracker

A maritime multi-sensor, multi-target fusion tracker. It takes heterogeneous,
asynchronous output from the sensors a vessel actually carries — AIS transponder
reports, navigation-radar ARPA tracks, EO/IR camera detections, own-ship
GPS/IMU — and produces one fused, continuously maintained set of vessel
**tracks** in a common geographic frame.

## The problem

At sea, no single sensor sees everything. AIS gives identity (MMSI) and a
position, but only for cooperative, transponder-equipped vessels. Navigation
radar sees non-cooperative targets, including in fog and rain, but loses small
objects and confuses close encounters. EO/IR classifies well, but visible
cameras fail at night and IR loses resolution in clutter. Each sensor reports
at its own rate, in its own units, in its own frame, with its own error
characteristics — and the rates differ by orders of magnitude (camera ~10 Hz
vs. AIS static every 6 minutes).

navtracker resolves this into a single authoritative picture. It answers:
*which detection belongs to which target, where is each target, how confident
are we, what do we know about its identity, and which sensors contributed?* —
the multi-target tracking problem with data association.

A core requirement is that **every track carries a stable internal ID
independent of external identity**. Non-cooperative vessels without AIS still
get a unique, persistent track. MMSI, ARPA track-IDs, and camera tracker-IDs
are *hints* and *attributes*, never the fusion key.

## What it offers

- A C++17 fusion engine with a clean hexagonal architecture: a domain core with
  zero I/O, swappable strategies behind well-defined ports.
- Concrete adapters for the four canonical maritime sensor classes (AIS, ARPA
  TTM/TLL, EO/IR with range, own-ship GGA/HDT).
- A **time-driven engine** that advances on message timestamps, not wall-clock.
  The same code runs live on a vessel and replays recorded logs deterministically
  — same input ordering, bit-identical output.
- Pluggable estimator and data-association strategies. The baseline is a
  constant-velocity EKF with greedy global-nearest-neighbor; an IMM
  estimator, an MHT tracker, and a PMBM tracker are also implemented and
  ship behind the same interfaces (PMBM/IMM are the current defaults per
  ADR 0002). Further alternatives (UKF, particle filter, JPDA, Hungarian)
  drop in behind the same interfaces.
- A scenario harness with an OSPA metric so algorithm choices are *evaluable*
  with a quantitative score, not opinion.
- Documented math, assumptions, decision rationale, and improvement paths for
  every algorithmic component (`docs/algorithms/`).

## How it is used

The fusion engine is a library, not an application. A host program composes
the pieces it wants and feeds measurements through.

A typical live setup:

1. Construct a motion model (`ConstantVelocity2D`), an estimator
   (`EkfEstimator`), an associator (`GnnAssociator`), an `OwnShipProvider`, a
   `TrackManager`, and a `Tracker` that wires them together. The `OwnShipProvider`
   automatically manages the working datum (local tangent plane origin), initializing
   it from the first own-ship pose and auto-recentering when the vessel moves > 30 km.
2. Wire an `IDatumChangeSink` that calls `shiftTracksOnDatumChange(...)` to
   keep your track state consistent if the datum recenters.
3. As raw input arrives, parse it and push an `OwnShipPose` to the provider,
   then construct `Measurement` objects (via builders like
   `makeMeasurementFromRelativeBearing` or direct `makeMeasurementFromEnuPosition`)
   and feed them into `tracker.process(z)`.
4. Read the current track set from `TrackManager::tracks()` whenever a
   downstream consumer needs it — display, collision avoidance, logging.
   For push-based consumption, register an `ITrackSink` on the
   `TrackManager` for lifecycle events and an `ICollisionRiskSink` on a
   `CpaEvaluator` for CPA-derived collision alarms.
5. Optional: wire a `HeadingBiasEstimator` to absorb heading-source
   disagreement (gyro vs. GPS multi-antenna vs. GPS COG vs. magnetic
   compass) into a continuously-published bias estimate; the NMEA
   adapter dispatches the relevant observation kinds automatically when
   `setHeadingBiasEstimator(&est)` is wired.

Replay is the same code with a log reader in place of the live feeds and the
original message timestamps preserved.

The engine deliberately stops at the track API. Wire-level concerns below
the adapter (AIVDM 6-bit decoding, raw radar protocols, image-frame
transport) and decisions above it (COLREGS, maneuver planning, display
rendering) live in the host program.

### Library use

Pre-parsed `Measurement` and `OwnShipPose` are the canonical contract. The
NMEA adapters in `adapters/` are one optional implementation — if your
pipeline produces parsed sensor data, skip them.

See `app/example.cpp` for the canonical drain pattern and
`tests/integration/test_full_stack_pipeline.cpp` for the assembled
library exercised end-to-end (NMEA in, lifecycle events + collision
alerts out, multi-source heading-bias estimation). For output format,
see `docs/output-contract.md`. CMake targets:

- `navtracker_core` — domain + ports + helpers. No I/O. Link this alone.
- `navtracker_nmea` — NMEA-format adapters. Link when you consume NMEA.
- `navtracker_t2t` — track-to-track fusion (tracker-of-trackers): fuse other trackers' tracks with covariance intersection. Link when your inputs are *tracks*, not detections.
- `navtracker_sim` — synthetic generators. Tests only.

For more details, see `CLAUDE.md`.

### Adding a sensor — what to pull from the spec

Integrating a new sensor means building `Measurement` objects plus (for
the MHT path) one detection-model entry. This is the checklist of what
to look up in the sensor's datasheet / ICD, and where each number goes.

**1. Measurement model — what does one detection observe?**

| The sensor reports | `MeasurementModel` | `value` layout (SI, rad) |
| --- | --- | --- |
| absolute position (GPS-equipped, AIS) | `Position2D` | `(E, N)` metres in the working ENU frame |
| position + velocity | `PositionVelocity2D` | `(E, N, vE, vN)` |
| range + bearing relative to the platform | `RangeBearing2D` | `(r, θ)` — use `makeMeasurementFromRelativeBearing(...)` to fold in heading + GPS uncertainty |
| bearing only (camera, ESM) | `Bearing2D` | `(θ)` |

Bearings are ENU `atan2(dy, dx)` — measured from **east,
counter-clockwise**, in radians. If the spec gives compass bearings
(from north, clockwise), convert: `θ_enu = π/2 − θ_compass`.

A bearing-only sensor **never initiates tracks** (range is unobservable
from one look — see `canInitiateTrack`); it only refines tracks that
active sensors created. If your new sensor is bearing-only, you need at
least one active sensor in the system for its data to matter.

**2. Timestamps.** `Measurement.time` is the *time of observation* in
UTC, not the arrival time. From the spec you need: (a) whether the
sensor timestamps at the sensor head (good) or at the output interface
(then add the documented internal latency); (b) the worst-case delivery
latency and jitter — that sizes the `ReorderBuffer` window if this
sensor can arrive out of order relative to others. The engine requires
non-decreasing timestamps and enforces it: both trackers drop and count
(`staleDropped()`) any measurement older than what they have already
processed. A `ReorderBuffer` in front *recovers* late data within its
window; without one, late data is rejected, never rewound into the
state.

**3. Uncertainty (R).** `Measurement.covariance` lives in the
measurement space of the chosen model (m² for positions, rad² for
bearings, mixed for range/bearing). Prefer, in order: per-detection
covariance from the sensor; the datasheet's 1σ accuracy (σ² on the
diagonal); nothing — then leave the covariance empty and call
`applyDefaultsIfEmpty(m, pessimisticSensorDefaults())` so the gap is
explicit and flagged (`covariance_is_default`). Beware datasheets
quoting 95% / 2σ figures — convert to 1σ before squaring. For polar
sensors the cross-range error grows with range; the
`makeMeasurementFrom...` builders handle that projection.

**4. Sensor position.** Set `sensor_position_enu` to the platform's ENU
position at observation time. Mandatory for `Bearing2D` /
`RangeBearing2D` (the math is relative to the sensor), and required for
any sensor that declares a finite coverage range (next item) — range
conditioning measures track distance *from the sensor*.

**5. Detection model entry (MHT path) — the numbers that drive scoring.**
For each (SensorKind, MeasurementModel) pair the tracker wants:

- **P_D** — probability of detecting a target *inside coverage* per
  scan. Datasheet value if given; otherwise calibrate from data
  (fraction of scans with a return near a ground-truth/AIS target).
- **λ_C** — clutter intensity in the *measurement space's* units:
  false alarms per m² per scan (positions), per radian per scan
  (bearings), per (m·rad) (range/bearing). Estimate: unassociated
  returns per scan ÷ surveyed area (or 2π for a full-circle bearing
  sensor). A single global λ cannot be correct across mixed sensors —
  that is why this is per-sensor.
- **max_range_m** — coverage radius. With it set, scans from this
  sensor charge no missed-detection penalty against tracks it could not
  have seen. Leave at the default (infinite) only if the sensor truly
  has no practical range limit for target sizes of interest.

Pass these via `FixedSensorDetectionModel::set(...)` into `MhtTracker`
(or `ScenarioDescriptor::detection_table` in the bench). **If you skip
this, every sensor runs at the defaults (P_D 0.9, λ_C 1e-4 m⁻²,
infinite range)** — tolerable for a single radar, badly wrong for
multi-sensor fusion. It is exactly the misconfiguration that tanked the
AutoFerry baseline (see `docs/algorithms/evaluation-log.md`,
2026-06-10 entry). `MhtTracker::defaultDetectionModelWarning()` goes
true if it sees two or more sensor kinds while running on the
un-injected default model — poll it once after warm-up.

**6. Rates.** Note the scan/frame rate. It needs no configuration —
scoring, IMM mixing and the IPDA lifecycle are dt-scaled — but it sizes
the reorder window and tells you how much weight the sensor will carry
relative to slower ones.

**7. Identity hints.** External identifiers (MMSI, ARPA target numbers)
go into `Measurement.hints` as *attributes*. They are never the fusion
key — the tracker's own `track_id` is.

**8. Validate at the edge.** Your adapter owns plausibility: reject
NaN/Inf, out-of-range values, impossible speeds, and unit mistakes
(deg vs rad, knots vs m/s) *before* constructing the `Measurement`.
Internal stages trust their inputs by design.

Finally: if the modality is genuinely new (not AIS / radar / EO-IR /
lidar / own-ship), add a `SensorKind` enumerator — the detection table
and per-sensor adaptive statistics key on it, so reusing the wrong kind
silently merges your sensor's calibration with another's.

## The processing chain

```
   sensor feeds                                                          fused tracks
        │                                                                      ▲
        ▼                                                                      │
 ┌──────────────┐   ┌─────────────┐   ┌──────────────┐   ┌──────────────┐   ┌────────────┐
 │ Sensor       │──►│ Normalize + │──►│ Time-ordered │──►│ Predict →    │──►│ Track      │
 │ adapters     │   │ ENU project │   │ reorder buf  │   │ associate →  │   │ manager    │
 │ (AIS, ARPA,  │   │ (per-sensor │   │ (drops late) │   │ update or    │   │ (existence │
 │ EO/IR,       │   │  R, hints)  │   │              │   │ initiate     │   │ lifecycle, │
 │ own-ship)    │   │             │   │              │   │              │   │ stable IDs)│
 └──────────────┘   └─────────────┘   └──────────────┘   └──────────────┘   └────────────┘
                                                                │
                                                                └─ + miss-timeout maintenance
```

Each stage does one thing:

1. **Sensor adapter** — parse the native input (NMEA sentence, decoded record),
   tag with sensor identity, and emit a normalized `Measurement`.
2. **Normalize + project** — convert geodetic positions (lat/lon) into a local
   ENU tangent plane via a configurable `Datum`. Relative measurements (radar
   range/bearing, camera bearing+range) are combined with the latest own-ship
   pose and projected into the same ENU frame with an anisotropic covariance
   derived from the polar measurement uncertainty.
3. **Time-ordered buffer** — releases measurements in timestamp order within a
   configurable reorder window. Anything older than the window is dropped and
   counted, so late/out-of-order arrivals can't corrupt the fusion sequence.
4. **Predict → associate → update / initiate** — for each released measurement,
   predict every active track forward to the measurement's time, gate candidate
   tracks by Mahalanobis distance, assign greedy global-nearest-neighbor, then
   EKF-update the matched track or initiate a new tentative one.
5. **Track manager** — owns the track set, allocates stable monotonic IDs that
   are never reused, runs the Tentative → Confirmed → Coasting → Deleted
   lifecycle, and applies a miss-timeout so unobserved tracks coast and
   eventually drop.

The whole chain runs on message timestamps. Two replays of the same log
produce bit-identical output.

## How it works, briefly

Everything kinematic happens in a local **East-North-Up tangent plane** about
a configurable origin (`Datum`). Conversion to/from geodetic WGS-84 happens
only at the boundary, so the filter sees flat-Earth coordinates and the linear
algebra stays clean.

Per-track state is `[east, north, vₑ, vₙ]` under a constant-velocity model.
Prediction is closed-form `F(dt)·x` with a continuous-white-noise-acceleration
process covariance. Updates use an EKF: linear `H` for position measurements;
Jacobian linearization for range/bearing; bearing residuals wrapped to
(−π, π] so the filter behaves across the ±180° meridian. The innovation
covariance `S = H P Hᵀ + R` and the gating statistic `d² = yᵀ S⁻¹ y` double
as the data-association cost.

Association is greedy global-nearest-neighbor over gated `d²`. Tracks not in
the assignment continue predict-only; measurements not assigned spawn new
Tentative tracks. The track manager promotes Tentative → Confirmed after
enough consecutive hits, demotes to Coasting on a miss, and deletes after a
configurable miss timeout. The track ID is stable across all of this and is
never reused after deletion.

Sensor identity (MMSI, ARPA track-ID, camera tracker-ID) is folded onto the
matched track as an attribute and is available as an association hint, but
never as the fusion key. A vessel without AIS gets the same stable
track-ID treatment as one with it.

The CV-EKF + GNN path above is the baseline, not the whole tracker: an IMM
estimator (`core/estimation/ImmEstimator.hpp`), an MHT tracker
(`core/pipeline/MhtTracker.hpp`), and a PMBM tracker
(`core/pmbm/PmbmTracker.hpp`) are implemented and selectable, with PMBM/IMM
as the current defaults per ADR 0002.

Every non-trivial algorithm is documented in `docs/algorithms/` with its
math, assumptions, why-this-over-the-alternatives rationale, and a concrete
"ways to improve / what to test next" list. The scenario harness in
`core/scenario/` lets you swap any port-level strategy and put a number on
the change via OSPA — that's how the deferred alternatives (UKF, particle
filter, JPDA, MMSI-hint locking, OOSM retrodiction, …) become *evaluable*
rather than just hypothesized.

## Known limitations

- **Land-clutter prior has a near-shore no-birth zone (opt-in `…_coverage_land`
  config).** The optional land/coastline clutter prior suppresses new-target
  births near the shore to kill stationary radar shore-clutter (it collapses the
  philos Boston-Harbor over-count: `card_err +108 → +7`). A measured consequence:
  because the phantom-birth gate equals the birth target in that config, the
  entire offshore soft band — within `offshore_halfwidth_m` (50 m) of shore, and
  around piers — is a **no-birth zone**: a genuine vessel that *starts* within
  50 m of shore will not initiate a track under this config. Vessels outside the
  band, and tracks already confirmed before approaching shore, are unaffected
  (the prior gates *births*, not maintenance). This is an accepted trade — the
  prior is opt-in and near-land operation is rare, and lowering the gate to fix it
  throws away ~⅓ of the shore-clutter win on real data. Full rationale,
  measurements, and the open principled fix (sensor-aware suppression, which
  needs camera/AIS coverage): **ADR 0001**
  (`docs/adr/0001-land-clutter-prior-offshore-no-birth-zone.md`) and
  `docs/algorithms/synthetic-clutter-bench.md`.

## Build

Dependencies are resolved by Conan (`conanfile.txt`): **Eigen 3.4**,
**mcap 1.4**, **nlohmann_json 3.11**, and **GoogleTest 1.14** (tests).

```bash
conan install . --build=missing --output-folder=build/ -s build_type=Release
cmake --preset conan-release
cmake --build build --target navtracker_tests
ctest --test-dir build --output-on-failure
```
