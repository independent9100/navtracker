# CLAUDE.md — navtracker

Guidance for working in this repository. Read before making changes.

## What this is

navtracker fuses outputs from multiple maritime sensors (AIS, navigation radar/ARPA, EO/IR camera, own-ship nav) into a single authoritative set of vessel **tracks**. It is a *fusion/tracking* system, not a raw detector. Full design: `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md`. Sensor data details: `docs/sensors/sensor-reference.md`.

**Static-vs-dynamic goal (do not re-litigate; ADR 0002 + its 2026-07-03 amendment):**
every persistent object must appear in the output as a vessel track OR a static
hazard — never neither ("presence over classification"). When sensors cannot
tell an anchored vessel from fixed structure, static-hazard-with-keep-clear is
an acceptable degraded mode; suppressing an object into *nothing* is the
forbidden failure. Anything represented as static must be promoted to a moving
track within bounded latency once it starts moving.

## Parallel work convention (implementer sessions)

Implementer/agent sessions do their work in a **git worktree on their own
branch** (`git worktree add ../navtracker-<task> -b <task-branch>`, own build
dir) — never by switching the main tree's checkout. The main working tree
belongs to the user/arbiter; a branch switch under a concurrent session makes
whoever commits next land on the wrong branch (it happened twice on
2026-07-06). Hand your branch to the arbiter for merge; don't merge or push
master yourself.

**Fixture trap in worktrees:** `tests/fixtures/` and `data/` are gitignored —
the data exists only in the MAIN tree. In a worktree, fixture-gated tests
silently SKIP, so "full suite green" there does NOT cover them. Symlink the
fixture dirs into the worktree (or point the fixture env at the main tree)
and say IN THE HANDOFF which fixture-gated tests actually RAN vs skipped.

**Never build or verify in the shared main checkout while other sessions are
active.** A temporary source edit + incremental rebuild in the shared tree
races anyone else's test run there (2026-07-07: exactly this produced a false
red that reverted a good merge). Verification builds — including the
arbiter's merge verification — happen in isolated worktrees with fixtures
symlinked.

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
adapters/  ais/ arpa/ eoir/ own_ship/ land/ static/ …                    (I/O, formats)
app/       composition root: wires adapters + strategies; run/replay modes
tests/     gtest unit + scenario/replay tests, metrics harness
docs/      specs/, sensors/, algorithm docs
```

## Library use

navtracker is designed as a hexagonal-architecture library. **If you are wiring
navtracker into your own system, read `docs/integration-guide.md`** — it is the
consumer-facing guide organized by "what you have / what you want" and indexes
every config, port, builder, adapter, and output type. This section is the quick
orientation; the guide is the full map.

The contract a consumer plugs against is two concrete types and two methods:

- `core/types/Measurement.hpp` — what you feed in (per sensor reading).
- `core/own_ship/OwnShipProvider.hpp` — `OwnShipPose` (per GPS fix). (The
  `adapters/own_ship/OwnShipProvider.hpp` path is a back-compat shim that just
  includes this one; new code should include the `core/` header directly.)
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

`StaticObstacleModel`, `CoastlineModel`, and `LiveOccupancyModel` also implement
`IDatumChangeSink` (they cache obstacle / shore / occupancy positions in the ENU
frame). If you use auto-recenter, register each one you wire as a datum sink too,
or its cache goes silently stale after a recenter — for the occupancy grid that
means a consumer using it with auto-recenter gets a silently wrong suppression map:

```cpp
provider.registerDatumSink(&obstacle_model);   // rebuilds the ENU cache on recenter
provider.registerDatumSink(&coastline_model);
provider.registerDatumSink(&occupancy_model);  // re-anchors the occupancy grid
```

(The bench `Sweep` skips this on purpose — it uses a single fixed datum per run, so
no recenter ever fires.)

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

### Push-based events (lifecycle and collision risk)

Two optional sinks turn the library from pull to push for the two
operator-facing concerns:

- `ITrackSink` (`ports/ITrackSink.hpp`) — `onTrackInitiated`,
  `onTrackConfirmed`, `onTrackUpdated`, `onTrackDeleted`. Registered
  via `TrackManager::setTrackSink(...)`. Lifecycle events fire from
  the manager; updates flow through `recordUpdated`, which the Tracker
  calls after each successful `estimator.update`.
- `ICollisionRiskSink` (`ports/ICollisionRiskSink.hpp`) — receives
  `CollisionRiskEvent {Entered/Exited/Updated}` per (own-ship × track)
  pair, with the full `CpaPrediction` payload. Emitted by
  `CpaEvaluator` (`core/collision/CpaEvaluator.hpp`) walking pairs
  each `evaluate(t)` call with hysteresis (`enter_probability`,
  `exit_probability`).

Both sinks are nullable; null = today's behavior, no overhead.
Pull-based access via `mgr.tracks()` / `toTrackOutput(...)` stays
fully supported alongside.

### Heading-bias estimator (multi-source)

`HeadingBiasEstimator` (`core/bias/HeadingBiasEstimator.hpp`) is a
scalar KF on a single gyro-bias state `b` with random-walk dynamics,
fed by any subset of five observation kinds. The five-kinds table and
the wiring for each source now live in the consumer guide,
`docs/integration-guide.md` §5 ("Heading and bias"). The concepts are
in `docs/learning/` (heading-bias chapter).

### End-to-end example

`tests/integration/test_full_stack_pipeline.cpp` is the canonical
assembled example: synthetic NMEA stream + AIS-style target
measurements → adapter → bias estimator → Tracker → CPA evaluator
→ recording sinks. Asserts lifecycle, bias convergence, CPA
Entered/Exited transitions across a head-on passing scenario.

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

## Learning / foundations docs (REQUIRED to keep in sync)

`docs/learning/` is the team's plain-English introduction to every
mathematical concept in this repository (Bayes, KF, EKF, UKF, PF,
IMM, gating, GNN, Hungarian, JPDA, clutter modelling, MHT,
NEES/NIS, multi-sensor fusion, heading-bias estimator, CPA, etc.).
It is what non-experts (including non-native English speakers)
use to onboard and to argue about design changes.

**Whenever a new concept, algorithm, or technique is introduced
into the codebase, you must extend `docs/learning/`:**

- If it fits an existing chapter, expand that chapter with how it
  works / why it works / assumptions / why-here, plus a diagram.
- If it is a fundamentally new concept, add a new numbered chapter
  and update `docs/learning/00-index.md` and the glossary
  (`docs/learning/19-glossary.md`).
- Cross-reference back from `docs/algorithms/` (precise reference)
  to the relevant learning chapter (intuitive introduction).

Tone rules for the learning series:

- Easy English. Short sentences. Define jargon on first use.
- Math is always followed by a plain-words explanation.
- Use diagrams wherever the concept is geometric, temporal, or
  has shape. Do not drown the reader in text — show, don't only
  tell.

Diagrams: **mermaid** for flowcharts/state machines/sequence
diagrams (renders inline in GitHub and VS Code), **real PNG
plots** for everything quantitative (Gaussians, ellipses,
particle clouds, mode probabilities, NIS histograms, CPA
geometry, …). The PNG plots are generated by
`docs/learning/figures/generate.py` (matplotlib in a venv — see
`docs/learning/figures/README.md`). **Never hand-edit the PNGs**;
change the script and re-run. Add new figures by adding a
`fig_*()` function and calling it from `main()`.

A PR that introduces a non-trivial new algorithm without an
accompanying `docs/learning/` update (text + figure) is
incomplete.

## Integration guide (REQUIRED to keep in sync)

`docs/integration-guide.md` is the consumer-facing map of the library,
organized by *integration surface* — "what you have / what you want".
It indexes every knob and wiring option a consumer of `navtracker_core`
+ `navtracker_nmea` touches.

**Whenever a change touches the consumer surface, the same PR must
update `docs/integration-guide.md`.** The consumer surface is: a new or
renamed config field or changed default; a new port, sink, builder, or
adapter; a new output field; a new named strategy config. The update is
a new entry or a corrected entry, *including the config-reference table
in the appendix*. A PR that changes the consumer surface without a guide
update is **incomplete** — the same standard as the learning-docs rule.

This is mechanically enforced for config structs by
`tests/docs/test_integration_guide_config_coverage.cpp`: every
`struct <Name>Config` under `core/`, `ports/`, `adapters/` (excluding
tests and `core/benchmark/`) must be mentioned in the guide, or be
listed with a reason in that test's explicit exclusion set. "Added a
Config, forgot the guide" is a red test, not a review hope.

**Three docs, three personas — keep material where it belongs.**
`docs/learning/` teaches concepts to team members; `docs/integration-guide.md`
serves a consumer wiring the library; `docs/algorithms/` justifies design
decisions. New material goes to whichever persona it serves — usually one,
not all three.

## Testing expectations

- Unit-test every stage and strategy against the port interfaces (use fakes/mocks for ports).
- Maintain scenario/replay tests with synthetic ground truth (crossing, overtaking, head-on, AIS dropout, non-cooperative target, clock skew); assert track continuity, **ID stability**, and accuracy vs. truth.
- A determinism test (same log replayed twice → identical output) must stay green.
- Use the metrics harness (track-accuracy/OSPA) when comparing estimator/association choices.

## Conventions

- Coordinates: common frame is **ENU local tangent plane** about a configurable datum; geodetic conversion only at boundaries. Units SI (m, m/s, rad internally; document any deg/knots at edges).
- Prefer small, single-purpose units with clear interfaces. If a file grows large, it's probably doing too much.
- No new third-party dependency without it being added through Conan and noted.
