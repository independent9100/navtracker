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
  constant-velocity EKF with greedy global-nearest-neighbor; alternatives
  (UKF, IMM, particle, JPDA, MHT, Hungarian) drop in behind the same
  interfaces.
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

See `app/example.cpp` for a complete end-to-end example. CMake targets:

- `navtracker_core` — domain + ports + helpers. No I/O. Link this alone.
- `navtracker_nmea` — NMEA-format adapters. Link when you consume NMEA.
- `navtracker_sim` — synthetic generators. Tests only.

For more details, see `CLAUDE.md`.

## The processing chain

```
   sensor feeds                                                          fused tracks
        │                                                                      ▲
        ▼                                                                      │
 ┌──────────────┐   ┌─────────────┐   ┌──────────────┐   ┌──────────────┐   ┌────────────┐
 │ Sensor       │──►│ Normalize + │──►│ Time-ordered │──►│ Predict →    │──►│ Track      │
 │ adapters     │   │ ENU project │   │ reorder buf  │   │ associate →  │   │ manager    │
 │ (AIS, ARPA,  │   │ (per-sensor │   │ (drops late) │   │ update or    │   │ (M-of-N    │
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

Every non-trivial algorithm is documented in `docs/algorithms/` with its
math, assumptions, why-this-over-the-alternatives rationale, and a concrete
"ways to improve / what to test next" list. The scenario harness in
`core/scenario/` lets you swap any port-level strategy and put a number on
the change via OSPA — that's how the deferred alternatives (UKF, IMM,
particle filter, JPDA, MHT, MMSI-hint locking, OOSM retrodiction, …) become
*evaluable* rather than just hypothesized.

## Build

```bash
conan install . --build=missing --output-folder=build/ -s build_type=Release
cmake --preset conan-release
cmake --build build/Release --target navtracker_tests
ctest --test-dir build/Release --output-on-failure
```
