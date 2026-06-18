# Debug visualization — MCAP / Foxglove recorder

**Predecessor baseline:** track + collision-risk sinks design
(2026-06-04), output-interface design (2026-06-04), NEES/NIS
instrumentation (2026-06-13). This consumes those ports; it adds no
new core ports.

## Why now

The fusion internals — which detections associated to which track,
how gates accepted or rejected them, how a covariance ellipse shrinks
as evidence accrues, where innovations spike — are currently only
visible through CSV/Markdown bench reports and log lines. Debugging
"why did track 7 absorb that clutter point at t=412" from a flat table
is slow. We already record everything needed (`ITrackSnapshotSink`,
`ITrackSink`, `IInnovationSink`, `ICollisionRiskSink`, `Measurement`);
this spec turns those streams into a scrubbable spatial view.

The team already uses **MCAP + Foxglove/Lichtblick** on other
projects. Recording navtracker's debug stream as `.mcap` reuses that
muscle memory, gets a scrubbable timeline / map / plot panels for
free, and — because the raw sensor streams (images, raw radar, raw
range/bearing) are logged to their **own** MCAPs by upstream
processes — a navtracker debug `.mcap` opened alongside them in one
Foxglove session is **time-aligned** by construction.

## Scope (and what's explicitly out)

In:
- A new **output adapter**, `adapters/foxglove/`, that writes a
  Foxglove-compatible `.mcap` from the existing driven ports plus an
  input-side measurement tee.
- Foxglove **well-known schemas**, **JSON-encoded** (no protobuf, no
  codegen): `SceneUpdate`, `LocationFix` / `GeoJSON`,
  `FrameTransform`, `Log`; plus a few custom JSON scalar channels for
  Plot panels.
- Composition-root wiring in `app/` that tees each `Measurement` to
  the recorder before `tracker.process(m)`.
- A small **read-only accessor on the associator** exposing the gate
  threshold `γ` (per measurement dimension) and the predicted
  innovation covariance `S` it used, so the `/gates` ellipse is
  faithful to the actual association decision. This is the one piece
  outside `adapters/foxglove/` that the recorder depends on; covariance
  (`P`) and innovation (`ν, S, R`) data is already emitted.
- **Sensor-bias visualization** (new since first draft — item 9/13).
  The `Tracker`/`MhtTracker` now bias-correct each measurement before
  fusion (`applyBiasCorrection(z, bias_provider_)`,
  `core/pipeline/Tracker.cpp`). The recorder holds a nullable
  `const ISensorBiasProvider*` and (a) draws both the **raw** and
  **bias-corrected** detection so the applied registration offset is
  visible, and (b) emits a `/diag/bias` channel tracking each
  per-`SensorBiasKey` position/bearing bias estimate + `is_published`
  as it converges. Null provider = no bias layer, no overhead.
- **Per-model detection rendering.** `Measurement.model` may be
  `Position2D` / `RangeBearing2D` (point + ellipse) **or** `Bearing2D`
  (a ray with angular σ from `sensor_position_enu`, no position
  ellipse). `/detections` branches on `model`; bearing-only sensors
  (EO/IR) render as a wedge, not a point.
- Unit tests (serializers), an MCAP round-trip test, a determinism
  test, and a smoke test driving an existing replay fixture.
- `docs/debug-visualization.md` (channel reference + how to open) and
  a short `docs/learning/` "seeing the tracker" section + one figure.

Out:
- **Raw sensor data** (images, raw radar scans, raw range/bearing).
  These are recorded to separate MCAPs by upstream processes. We
  record detections *as ingested* (the `Measurement` stream) and the
  fusion internals — nothing below the detection layer.
- **Live streaming.** Deferred. The same serializers can later feed a
  Foxglove WebSocket server adapter; this spec only writes files. No
  schema change is needed to add live later.
- **Any change to tracking behavior.** The recorder is a pure
  observer; with it unwired, behavior is byte-identical to today.
- **A new core port for measurements.** The tee is composition-root
  wiring (see Architecture), not a domain abstraction.

## Architecture

```
adapters/foxglove/
  FoxgloveDebugRecorder.{hpp,cpp}   implements ITrackSnapshotSink, ITrackSink,
                                    IInnovationSink, ICollisionRiskSink,
                                    IDatumChangeSink (adapters/own_ship);
                                    holds nullable const ISensorBiasProvider*;
                                    + recordMeasurement(const Measurement&),
                                      recordOwnShip(const OwnShipPose&)
  serialize/                        pure navtracker-type -> Foxglove JSON
                                    mappers (no I/O, unit-testable)
  McapWriter.{hpp,cpp}              thin RAII wrapper over the mcap C++ lib:
                                    channel registration + timestamped write,
                                    zstd chunk compression
```

Invariants preserved:
- **#1 / #2 (hexagonal, inward deps):** the recorder lives in
  `adapters/`, depends on `core/` + `ports/`, never the reverse.
  `navtracker_core` gains no dependency. The `mcap` dep is confined to
  this adapter behind a CMake option (see Dependency).
- **#3 (pluggable hot path):** the recorder only *reads* the existing
  sink events; it cannot influence association or the estimator.
- **#4 (time-driven, deterministic):** every message is stamped with
  the **source event's `Timestamp`** (the `Measurement.time` /
  `*_Event.time`), never wall-clock. Same replay input → identical
  message payloads (asserted by the determinism test). This also
  guarantees time-alignment with the externally-recorded raw-sensor
  MCAPs.

### Measurement tee (input-side tap)

`ITrackSnapshotSink` / `ITrackSink` / `IInnovationSink` /
`ICollisionRiskSink` all sit *downstream* of fusion. To show
detections as ingested we need the input stream. The composition root
in `app/` already calls `tracker.process(m)`; it tees:

```cpp
recorder.recordMeasurement(m);   // RAW detection (per-sensor, with R)
tracker.process(m);              // unchanged; bias-corrects internally
```

No core change, no new port — the tap is wiring in `app/`, consistent
with refinement that the sensor stream "lives in main."

The tee captures the **raw** measurement. Because the tracker now
bias-corrects internally (and `Track` does *not* store the applied
bias), the recorder reproduces the correction itself by querying its
`ISensorBiasProvider` with the same `applyBiasCorrection` helper, so
the `/detections` "corrected" marker is bit-faithful to what fusion
saw. The provider snapshot is read at record time per measurement.

### Wiring summary (app composition root)

```cpp
FoxgloveDebugRecorder recorder{"/path/run.mcap", &bias_provider};  // provider nullable
mgr.setTrackSink(&recorder);                 // lifecycle -> /log
sink_fanout.add(&recorder);                  // ITrackSnapshotSink -> /tracks,/map
tracker.setInnovationSink(&recorder);        // -> /diag/innovation
cpa.setCollisionRiskSink(&recorder);         // -> /cpa, /log
provider.registerDatumSink(&recorder);       // onDatumRecentered -> /tf, /log
// per measurement: recorder.recordMeasurement(m); recorder.recordOwnShip(pose);
// (same &bias_provider the tracker uses via setSensorBiasProvider)
```

(If a snapshot fan-out does not yet exist where a real sink is already
registered, add a small `TrackSnapshotFanout : ITrackSnapshotSink`
multiplexer in `adapters/` — it is generally useful and keeps the
single-sink setter intact.)

## Channels (topics) → Foxglove panels

| Topic | Schema (well-known unless noted) | Source port | Content |
|---|---|---|---|
| `/detections` | `foxglove.SceneUpdate` | `recordMeasurement` (+ bias provider) | per-`Measurement`, rendered by `model`: `Position2D`/`RangeBearing2D` → marker + 2σ ellipse; `Bearing2D` → ray/wedge with angular σ from `sensor_position_enu`. Draws **raw** and **bias-corrected** marker when a provider is present. Color by `SensorKind`/`source_id`; label with model + default-cov flag |
| `/tracks` | `foxglove.SceneUpdate` | `ITrackSnapshotSink` | track position, 2σ covariance ellipse, velocity arrow, id + status label, color by `TrackStatus` |
| `/associations` | `foxglove.SceneUpdate` | snapshot + measurement step | line from each detection to the track it updated this step |
| `/gates` | `foxglove.SceneUpdate` | snapshot + associator | gating ellipse per track. **Always recorded**; the shipped Lichtblick layout ships with this topic's visibility **off**, so enabling it is a single checkbox with no re-run. Requires the associator accessor below. |
| `/map/tracks`, `/map/detections` | `foxglove.LocationFix` / GeoJSON | snapshot + measurement | lat/lon for the Map panel (via `toGeodeticWithCov` / datum) |
| `/tf` | `foxglove.FrameTransform` | `recordOwnShip` + datum events | own-ship pose; datum recenter emits an updated transform |
| `/log` | `foxglove.Log` | `ITrackSink`, CPA, datum | lifecycle transitions, CPA Entered/Exited, datum-recenter notes |
| `/cpa` | `foxglove.SceneUpdate` | `ICollisionRiskSink` | CPA geometry per (own-ship × track) pair |
| `/diag/innovation` | **custom JSON scalars** | `IInnovationSink` | per-track NIS = νᵀS⁻¹ν, residual norm, keyed by `(sensor, source_id)` |
| `/diag/track_count`, `/diag/gate_ratio` | **custom JSON scalars** | snapshot | confirmed/tentative counts, accepted/total gate ratio |
| `/diag/bias` | **custom JSON scalars** | `ISensorBiasProvider` | per-`SensorBiasKey` position bias (ENU m) + bearing bias (rad) + `is_published`, sampled each cycle — Plot panel shows registration bias converging |

Foxglove recognizes well-known schemas **by name** regardless of
encoding, so JSON-encoded `SceneUpdate` etc. auto-render in Lichtblick
with no protobuf dependency. Custom `/diag/*` channels carry a flat
JSON schema (`{time, track_id, sensor, source_id, nis, ...}`) that
Plot panels read by field path.

## Frames & math (four-part doc standard)

### Math
- **Root frame** = ENU about the current datum (x=east, y=north,
  z=up). All `SceneUpdate` geometry lives here; the Map panel is fed
  lat/lon via the existing `toGeodeticWithCov(enu, cov, datum)` so
  positions and covariance appear in both the 3D scene and the map.
- **Covariance ellipse** from the symmetric 2×2 position covariance
  `P₂`: eigen-decomposition `P₂ = V diag(λ₁, λ₂) Vᵀ`; semi-axes
  `aᵢ = k·√λᵢ`, orientation `θ = atan2(V[1,0], V[0,0])`. Default
  `k = 2` (≈ 2σ). Rendered as an oriented Foxglove primitive (scaled
  flat cylinder / ellipse line-loop) at the ENU position, z≈0.
- **Gate ellipse** (the `/gates` layer): the validation region
  `{ z : (z − ẑ)ᵀ S⁻¹ (z − ẑ) ≤ γ }` centered on the predicted
  measurement `ẑ`, with semi-axes `√(γ·λᵢ(S))` and orientation from
  the eigenvectors of `S`. `γ` and `S` come from the associator
  accessor (see Scope); it is strictly larger than the `P`-based
  covariance ellipse because `S` adds `R` and `γ` inflates by the χ²
  bound.
- **Bearing-only detection** (`Bearing2D`): no position to plot — a ray
  from `sensor_position_enu` along `α` with a half-angle wedge of
  `k·σ_α` (`σ_α²` from the 1×1 `R`). `SourceTouch.alpha_rad` /
  `alpha_var_rad2` carry the same convention (ENU, `α=atan2(dy,dx)`
  from east CCW) for the `/associations` line back to the contributing
  bearing.
- **Bias correction** (detection layer): corrected = raw shifted by the
  provider's `PositionBiasEstimate.bias_enu_m` (Position2D/RangeBearing2D)
  or `BearingBiasEstimate.bias_rad` added to `α` (Bearing2D), exactly as
  `applyBiasCorrection` does. Only applied/shown when `is_published`.
- **NIS** per innovation: `ε = νᵀ S⁻¹ ν`, computed by the recorder
  from `InnovationEvent.{residual, S}` (already carried — see
  `ports/IInnovationSink.hpp`). Emitted on `/diag/innovation` for
  Plot panels; the χ² band per `dim` is a Foxglove reference line.

### Assumptions
- `Track` exposes a 2D position state + covariance reachable the same
  way `toTrackOutput` reaches it; the velocity arrow is drawn only when
  `VelocityGeodeticWithSigma.is_valid`, which is now gated by
  `Track.velocity_observed` (review #13) — a prior-only COG on a
  freshly-initiated track is not drawn as a heading.
- `Track` does **not** store the bias applied to its contributing
  measurements, so the recorder re-derives the correction from the
  `ISensorBiasProvider` snapshot (see Measurement tee). The provider is
  read-only; querying it cannot perturb fusion.
- The datum-recenter event is `IDatumChangeSink::onDatumRecentered`
  (declared in `adapters/own_ship/OwnShipProvider.hpp`, not `ports/`);
  the recorder registers via `OwnShipProvider::registerDatumSink`.
- Datum shifts are infrequent (>30 km auto-recenter). On a shift,
  geometry already emitted stays in the *old* frame's coordinates; we
  emit a new `FrameTransform` and a `/log` note marking the
  discontinuity rather than re-projecting history.
- Message timestamps equal source event timestamps (monotonic per the
  reorder-buffer guarantees upstream of the tracker).

### Rationale
- **MCAP + Foxglove well-known schemas (JSON):** reuses the team's
  existing tooling; auto-renders with zero panel code; single new dep.
  Chosen over a bespoke matplotlib animation (no scrub/interactivity,
  no map) and over a custom web front-end (maintenance cost) — see the
  approaches weighed in brainstorming.
- **JSON encoding over protobuf:** avoids a protobuf dependency +
  codegen step; MCAP zstd chunking absorbs the size cost; offline
  debugging is size-insensitive.
- **Observer adapter, not core change:** keeps the hot path pluggable
  and the recorder unable to perturb fusion — debug output must never
  change what it observes.

### Ways to improve / what to test next
- **Live mode:** add a `FoxgloveWsServer` adapter reusing the same
  serializers (Foxglove WebSocket protocol). Evaluate when a live
  picture is actually needed.
- **MHT hypothesis view:** render competing leaf hypotheses / their
  weights as a layered `SceneUpdate` to debug `MhtTracker` directly.
- **Truth overlay:** when a scenario fixture has ground truth, add a
  `/truth` channel + per-track position-error plot (NEES band), tying
  the view to the bench consistency metrics.
- **Confidence-level toggle:** expose `k` (1σ/2σ/3σ) as a recorded
  parameter so the operator can compare; test that ellipse area scales
  as expected.

## Testing

- **Serializer unit tests** (`tests/adapters/foxglove/`): each mapper
  (measurement→SceneUpdate, track→SceneUpdate, innovation→scalar JSON,
  covariance→ellipse, enu→LocationFix) produces expected JSON fields
  for hand-built inputs. No I/O. Includes the eigen-decomposition
  ellipse math (axis lengths + orientation for a known `P₂`).
- **MCAP round-trip test:** write a short synthetic run to a temp
  `.mcap`, read it back with the mcap reader, assert channel set,
  per-channel message counts, schema names, and a few decoded field
  values.
- **Determinism test:** drive the same replay fixture twice; assert
  the ordered (timestamp, topic, payload) sequence is identical. This
  extends invariant #4 to the debug output.
- **Smoke test:** generate a sample `.mcap` from an existing
  scenario/replay fixture (e.g. a crossing scenario) as a build-time
  artifact the test asserts is non-empty and openable.

## Docs

- `docs/debug-visualization.md`: channel/topic reference, the panel
  layout to recreate in Lichtblick, how timestamps align with the
  upstream raw-sensor MCAPs, and the `k`/confidence convention.
- `docs/learning/`: a short "seeing the tracker" section (reading
  covariance ellipses, gates, association lines, NIS plots), with one
  generated figure (annotated screenshot-style diagram via the
  matplotlib `generate.py`), cross-referenced from the KF, gating, and
  NEES/NIS chapters. This is tooling, not a new algorithm, so no new
  numbered chapter is added.
- Cross-reference from `adapters/foxglove/` header docs back to the
  learning section.

## Dependency

Add to `conanfile.txt`:

```
mcap/<version>     # + transitive lz4, zstd for chunk compression
```

Confined to the `adapters/foxglove/` build target behind a CMake
option `NAVTRACKER_BUILD_FOXGLOVE` (default ON). New CMake target
`navtracker_foxglove` links `navtracker_core` + `mcap`. Consumers
linking only `navtracker_core` are unaffected; turning the option OFF
drops the dep entirely. Verify `mcap` availability in Conan Center
during the first plan step; if unavailable, vendor the single-header
writer + lz4/zstd (already common transitive deps) — fallback noted in
the plan, not chosen here.
