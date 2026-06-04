# CLAUDE.md — navtracker

Guidance for working in this repository. Read before making changes.

## What this is

navtracker fuses outputs from multiple maritime sensors (AIS, navigation radar/ARPA, EO/IR camera, own-ship nav) into a single authoritative set of vessel **tracks**. It is a *fusion/tracking* system, not a raw detector. Full design: `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md`. Sensor data details: `docs/sensors/sensor-reference.md`.

## Tech stack

- **C++17** (no later standard without discussion).
- **CMake** build, **Conan** dependency management.
- **GoogleTest (gtest/gmock)** for unit and scenario tests, pulled via Conan.

## Architecture invariants (do not violate)

1. **Clean / hexagonal (ports & adapters).** The domain core (`core/`) contains tracking logic and has **zero I/O and zero sensor-format knowledge**. Source dependencies point *inward*: adapters depend on the core, never the reverse.
2. **Ports are interfaces.** Strategies and edges are abstractions: `IDataAssociator`, `IEstimator`, `IMotionModel`, `ISensorAdapter`, `ITrackSink`, time-source. Concrete sensor parsers and I/O live in `adapters/`; wiring lives in `app/` (composition root).
3. **Pluggable hot path.** Data association and the per-track estimator must remain swappable without touching the pipeline — these will be evaluated and changed.
4. **Time-driven, not wall-clock-driven.** The engine advances on message timestamps. Live and replay use the *same* core; replay of a log must be **deterministic** (same input ordering → identical output).
5. **Stable track identity.** Every track has a unique internal `track_id` that is its primary key, independent of external identity (MMSI/name). IDs are never reused after deletion. External identifiers are *attributes/hints*, never the fusion key.
6. **Validate at the edges.** Adapters validate parsing/units/NaN/plausibility. Internal stages trust their inputs.

## Module layout

```
core/      domain types, tracker, estimators, association, track mgmt   (no I/O)
ports/     interfaces (IEstimator, IDataAssociator, ISensorAdapter, ...)
adapters/  ais/ arpa/ eoir/ ownship/ sinks/                              (I/O, formats)
app/       composition root: wires adapters + strategies; run/replay modes
tests/     gtest unit + scenario/replay tests, metrics harness
docs/      specs/, sensors/, algorithm docs
```

## Library use

navtracker is designed as a hexagonal-architecture library. The contract a consumer plugs against is two concrete types and two methods:

- `core/types/Measurement.hpp` — what you feed in (per sensor reading).
- `adapters/own_ship/OwnShipProvider.hpp` — `OwnShipPose` (per GPS fix).
- `Tracker::process(Measurement)` / `OwnShipProvider::update(pose)` — the entry points.

You construct Measurements directly from your parsed sensor data. The NMEA adapters in `adapters/` are one optional implementation for consumers whose input is NMEA strings — they're not the canonical path. Skip them if you have your own pipeline.

### Auto-datum pattern

The `OwnShipProvider` owns and manages the working datum (local tangent plane origin). Construct it with no arguments; it auto-initializes from the first `update(pose)` call and auto-recenters (replaces the datum) when own-ship moves > 30 km. This eliminates the need to pass a `Datum` object through the measurement builders.

When the datum shifts, the provider fires an `IDatumChangeSink` event. Wire a sink that calls `shiftTracksOnDatumChange(TrackManager&, old_datum, new_datum)` to keep your track state consistent with the new ENU frame:

```cpp
struct TrackShifterSink : IDatumChangeSink {
  TrackManager* mgr;
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    shiftTracksOnDatumChange(*mgr, o, n);
  }
};
TrackShifterSink mgr_sink{&mgr};
provider.registerDatumSink(&mgr_sink);
```

Push at least one `OwnShipPose` via `provider.update()` before constructing measurements — this initializes the datum.

### Common patterns

- Range/bearing sensor (radar, EO-IR, sonar): `makeMeasurementFromRelativeBearing(...)` — adds heading, projects to ENU, composes GPS and heading covariance.
- Absolute-position sensor (AIS, GPS-equipped target): `makeMeasurementFromEnuPosition(...)` — direct ENU input.
- No uncertainty info from the sensor: leave covariance empty, call `applyDefaultsIfEmpty(m, pessimisticSensorDefaults())`.

See `app/example.cpp` for a complete end-to-end example.

### Output contract

The canonical drain is `toTrackOutput(track, datum)` (in
`core/output/TrackOutput.hpp`). It returns a `TrackOutput` with:

- position in lat/lon (WGS84 degrees);
- position covariance in m² in the target's local NED frame;
- velocity in SOG (m/s) / COG (degrees true) with derived σ
  values and an `is_valid` flag;
- track metadata (stable id, lifecycle status, last_update,
  attributes, contributing_sources);
- a `covariance_is_default` diagnostic flag.

See `docs/output-contract.md` for unit semantics, validity rules,
and a worked example. See `app/example.cpp` for the canonical
drain pattern.

### CMake targets for library consumers

- `navtracker_core` — pure domain + ports + helpers. No I/O. Link this alone if you supply pre-parsed Measurements.
- `navtracker_nmea` — NMEA-format adapters. Link this in addition when your input is NMEA strings.
- `navtracker_sim` — synthetic measurement emitters. Tests only.

## Documentation standard (REQUIRED for every non-trivial algorithm)

Document each algorithmic component with these four parts — no exceptions:

1. **Math** — state/measurement models, equations, covariances, coordinate conventions.
2. **Assumptions** — what must hold for the math to be valid.
3. **Rationale** — why this approach over the alternatives considered.
4. **Ways to improve / what to test next** — concrete candidate alternatives and the experiment to evaluate them.

Decisions are recorded with rationale (see the decision table in the design spec). When changing an algorithm, update its doc's four sections.

## Testing expectations

- Unit-test every stage and strategy against the port interfaces (use fakes/mocks for ports).
- Maintain scenario/replay tests with synthetic ground truth (crossing, overtaking, head-on, AIS dropout, non-cooperative target, clock skew); assert track continuity, **ID stability**, and accuracy vs. truth.
- A determinism test (same log replayed twice → identical output) must stay green.
- Use the metrics harness (track-accuracy/OSPA) when comparing estimator/association choices.

## Conventions

- Coordinates: common frame is **ENU local tangent plane** about a configurable datum; geodetic conversion only at boundaries. Units SI (m, m/s, rad internally; document any deg/knots at edges).
- Prefer small, single-purpose units with clear interfaces. If a file grows large, it's probably doing too much.
- No new third-party dependency without it being added through Conan and noted.
