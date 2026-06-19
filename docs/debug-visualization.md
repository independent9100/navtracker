# Debug Visualization — MCAP / Foxglove Recorder

The `FoxgloveDebugRecorder` is an offline debug adapter that writes the
navtracker fusion pipeline to a Foxglove-compatible `.mcap` file. Open
the file in [Lichtblick](https://lichtblick.dev) or
[Foxglove Studio](https://foxglove.dev) to scrub through the full run
and inspect tracks, detections, gates, associations, NIS plots, and CPA
events on a shared timeline.

The recorder is a **pure observer**. With it unwired, tracking behavior
is byte-identical. It implements the existing driven ports
(`ITrackSnapshotSink`, `ITrackSink`, `IInnovationSink`,
`ICollisionRiskSink`, `IDatumChangeSink`) plus two input-side taps
(`recordMeasurement`, `recordOwnShip`) that you call before
`tracker.process(m)`.

**Spec:** `docs/superpowers/specs/2026-06-13-debug-visualization-foxglove-design.md`

---

## How to record

### Via the example app (quickest)

Build with the Foxglove option (it is ON by default):

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
      -DCMAKE_BUILD_TYPE=Release -DNAVTRACKER_BUILD_FOXGLOVE=ON
cmake --build build --target navtracker_example_foxglove
```

Set the output path with an environment variable and run:

```bash
NAVTRACKER_MCAP=/tmp/run.mcap ./build/navtracker_example_foxglove
```

The file is written and closed when the program exits.

### Wiring it yourself

```cpp
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "adapters/sinks/TrackSnapshotFanout.hpp"

// Construct — path, datum, optional bias provider, optional config.
foxglove::RecorderConfig cfg;
cfg.ellipse_k   = 2.0;   // covariance ellipses shown at 2σ (default)
cfg.gate_gamma  = 9.21;  // χ² 2-DOF 99 % threshold; 0 disables /gates

foxglove::FoxgloveDebugRecorder recorder{"/tmp/run.mcap", datum, &bias_provider, cfg};

// Register on the tracker/manager sinks.
mgr.setTrackSink(&recorder);                // lifecycle → /log
tracker.setInnovationSink(&recorder);       // → /diag/innovation
cpa_evaluator.setCollisionRiskSink(&recorder);  // → /cpa, /log
own_ship_provider.registerDatumSink(&recorder); // → /tf, /log

// If you already have an ITrackSnapshotSink registered, fan-out.
TrackSnapshotFanout fanout;
fanout.add(&existing_sink);
fanout.add(&recorder);
// tracker uses &fanout instead of &existing_sink

// Per measurement: tee BEFORE tracker.process(m).
recorder.recordMeasurement(m);
tracker.process(m);

// Per own-ship fix:
recorder.recordOwnShip(pose);
```

Call `recorder.close()` (or let the destructor run) when the session ends.

---

## Channels (topics) and Foxglove panels

| Topic | Foxglove schema | Source | What it shows |
|---|---|---|---|
| `/tracks` | `foxglove.SceneUpdate` | `ITrackSnapshotSink` | Track position, `ellipse_k`-sigma covariance ellipse (from `P`), velocity arrow, ID + status label. Green = Confirmed, yellow = Tentative. |
| `/detections/<source_id>` | `foxglove.SceneUpdate` | `recordMeasurement` | **One topic per sensor source** (e.g. `/detections/autoferry_radar`, `/detections/autoferry_lidar`) so each sensor is an independent, toggleable layer. `Position2D`/`RangeBearing2D` → point + ellipse; `Bearing2D` → bearing wedge from `sensor_position_enu`. With a bias provider, also draws the bias-corrected marker at reduced opacity. Color is fixed per `SensorKind` (radar=red, lidar=orange, EO/IR=cyan, AIS=magenta), nudged per `source_id`. |
| `/ownship` | `foxglove.SceneUpdate` | `recordOwnShip` | White diamond + "own-ship" label at the own-ship ENU position (stable id → moves in place). |
| `/associations` | `foxglove.SceneUpdate` | `ITrackSnapshotSink` | Line from each contributing detection to the track it updated this scan. Bearing-only touches anchor at the sensor position. |
| `/gates` | `foxglove.SceneUpdate` | `ITrackSnapshotSink` + cached `S` | Gate ellipse per track = `√γ · ellipse(S)`. Populated only when `gate_gamma > 0` and a cached innovation covariance `S` exists for the track. See [gate caveat](#gate-caveat). |
| `/map/tracks` | `foxglove.LocationFix` | `ITrackSnapshotSink` | One `LocationFix` per track per scan, for the Map panel. Lat/lon via `toGeodeticWithCov`. **Accumulates** over the whole log (the Map panel never clears points). |
| `/map/detections/<source_id>` | `foxglove.LocationFix` | `recordMeasurement` | Per-sensor lat/lon for position-type detections (Map panel). |
| `/map/ownship` | `foxglove.LocationFix` | `recordOwnShip` | Own-ship lat/lon track for the Map panel. |
| `/tf` | `foxglove.FrameTransform` | recorder + `recordOwnShip` + datum events | A one-time identity `map→enu` root transform (so the 3D panel has a frame), plus an `enu→own_ship` transform per pose. Datum recenters emit a `/log` note. |
| `/log` | `foxglove.Log` | `ITrackSink`, CPA, datum | Track lifecycle transitions (Initiated / Confirmed / Deleted), CPA Entered/Exited with distance and TCPA, datum-recenter notes. |
| `/cpa` | `foxglove.SceneUpdate` | `ICollisionRiskSink` | CPA marker per (own-ship × track) pair with distance and TCPA label. |
| `/diag/innovation` | custom JSON scalars | `IInnovationSink` | Per-update NIS `nis = νᵀ S⁻¹ ν` and measurement dimension `dim`, keyed by `(track_id, sensor, source_id)`. Feed a Plot panel; add a horizontal reference line at `dim` to see the chi-squared expectation. |
| `/diag/track_count` | custom JSON scalars | `ITrackSnapshotSink` | Confirmed + tentative + total count. Plot over time to see lifecycle health. |
| `/diag/bias` | custom JSON scalars | `ISensorBiasProvider` | Per-`SensorBiasKey` position bias (ENU m) and bearing bias (rad) plus `is_published`. Plot to watch registration bias converge after startup. |

Foxglove recognizes well-known schemas **by name** regardless of
encoding, so JSON-encoded `SceneUpdate`, `LocationFix`, etc.
auto-render in Lichtblick with no protobuf dependency. Custom
`/diag/*` channels carry flat JSON objects that Plot panels read
by field path.

---

## Recommended Lichtblick panel layout

1. **3D panel** — subscribe to all `foxglove.SceneUpdate` topics
   (`/tracks`, `/detections`, `/associations`, `/gates`, `/cpa`).
   Set the display frame to `enu`. This gives the full spatial picture:
   covariance ellipses, bearing wedges, association arrows, gate
   boundaries, CPA markers.

2. **Map panel** — subscribe to `/map/tracks` and `/map/detections`
   for a georeferenced view. The recorder uses `toGeodeticWithCov`
   so the lat/lon position covariance also appears on the map.

3. **Plot panels** (one or more):
   - `/diag/innovation.nis` vs update index — add a fixed reference
     line at the measurement dimension `m` to compare against the
     chi-squared expectation.
   - `/diag/track_count.confirmed` — track population over time.
   - `/diag/bias.bearing_bias_rad` per source — sensor registration
     bias converging after startup.

4. **Raw Messages / Log panel** — subscribe to `/log` to read
   lifecycle events and CPA alerts in order.

5. **TF panel or 3D overlay** — subscribe to `/tf` to visualize
   own-ship position and heading.

For the `/gates` layer: enable it in the 3D panel's topic list only
when you need it. It is generated when `gate_gamma > 0` and visibility
is off in the default layout to avoid clutter during normal review.

---

## Covariance ellipses and the `ellipse_k` parameter

The ellipse drawn for each track and detection is:

```
semi-axis_i = ellipse_k · sqrt(λ_i(P₂))
```

where `P₂` is the top-left 2×2 of the covariance and `λ_i(P₂)` is the
`i`-th eigenvalue. The default `ellipse_k = 2` corresponds to the
2-sigma boundary (≈ 86 % probability mass in 2D). Set it to `3` for the
3-sigma boundary (≈ 99 %).

---

## Gate caveat

The `/gates` layer draws the **validation gate** centered on the track
position with semi-axes `√(γ · λ_i(S))`, where `S` is the innovation
covariance from the most recent `InnovationEvent` for that track and
`γ` is the chi-squared gate threshold (`gate_gamma`).

**Limitation:** `S` is only available when an `InnovationEvent` has
fired for the track in this run. This happens when the track had a
successful hard-match association that scan (GNN / Hungarian). Tracks
that were not matched — coasting tracks, tentative tracks with no
measurement — do not produce an `InnovationEvent`, so their gate
ellipse is not drawn. A per-track gate emission covering coasting
tracks is a documented follow-up (see the spec's *Ways to improve*).

In practice this means:
- Active, confirmed tracks show their gate.
- Coasting tracks show only their covariance ellipse, not the gate.

---

## Timestamp alignment with upstream MCAPs

Every message is stamped with the **source event's `Timestamp`** — the
`Measurement.time` or event time — not the wall clock. Replaying a
recorded scenario produces byte-identical message payloads for the same
input (the determinism test asserts this).

This design means you can open a navtracker debug `.mcap` **alongside**
the upstream raw-sensor MCAPs in the same Foxglove session and the
timelines align without any offset correction.

---

## Disabling the adapter

Pass `-DNAVTRACKER_BUILD_FOXGLOVE=OFF` to cmake. This drops the `mcap`
and `nlohmann_json` dependencies entirely. Consumers linking only
`navtracker_core` are unaffected regardless of this option.

---

## Learning reference

For an explanation of how covariance ellipses and gate ellipses differ
visually, how to read association lines and bearing wedges, and what
NIS plots reveal about filter confidence, see:

**Chapter 11 — Gating, GNN, Hungarian** →
[Section 9: Seeing the tracker in Foxglove](learning/11-gating-gnn-hungarian.md#9-seeing-the-tracker-in-foxglove)
