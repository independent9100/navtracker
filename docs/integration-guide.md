# navtracker integration guide

**Who this is for.** You are a competent C++ engineer who knows ships but not
this codebase, and you want to wire navtracker into your own system. This guide
is organized by *what you have and what you want* — pick your situation, copy the
snippet, follow the link into the deep docs for the details.

**What this guide is (and is not).** It is an *index* to the consumer surface, not
a re-explanation of the algorithms. Each entry is a few sentences of plain
English, the config with the defaults worth changing, a short wiring snippet, and
links into the authoritative docs. The single source of truth stays in those deep
docs:

- `docs/algorithms/` — precise algorithm references (the *why* of each design).
- `docs/learning/` — plain-English introductions to every math concept.
- `docs/sensors/sensor-reference.md` — what each real sensor gives you.
- `docs/output-contract.md` — exact unit/validity semantics of the output.
- `docs/adr/` — architecture decisions.

**Scope — the consumer surface only.** The boundary is the CMake targets. What a
consumer of `navtracker_core` + `navtracker_nmea` touches is *in*: Measurement
builders, `OwnShipProvider`, `Tracker`/`TrackManager` wiring, ports and sinks, the
sensor adapters and their configs, and the output types. *Out of scope:*
benchmark/simulation internals (`navtracker_sim`, `core/benchmark/` sweep knobs,
scenario builders, replay harnesses) and debug tooling (`adapters/foxglove/`).
Those exist to test and measure the library, not to be wired into a product.

> **Thread-safety — the core is NOT internally synchronized.** There is no
> locking anywhere in the core. `OwnShipProvider::update(pose)`,
> `Tracker::process(m)`, `CpaEvaluator::evaluate(t)`, and every sink callback
> must all be serialized onto a **single thread** (or guarded by an external
> lock you own). `OwnShipProvider` mutates an unlocked pose deque and stores raw
> sink pointers; calling it concurrently with `process` — or from two threads —
> corrupts state. If your sensor feeds arrive on multiple threads, funnel them
> through one queue and drain that queue on one thread.

> **Keeping this guide honest.** Whenever you change the consumer surface — a
> config field, a default, a port/sink/builder/adapter, an output field, or a
> named strategy config — update this guide in the same PR. See CLAUDE.md,
> *"Integration guide (REQUIRED to keep in sync)"*. A drift-guard test
> (`tests/docs/test_integration_guide_config_coverage.cpp`) fails if a
> `…Config` struct exists in the code but is never mentioned here.

---

## Contents

1. [Minimum viable integration](#1-minimum-viable-integration)
2. [Own-ship, datum, and moving far](#2-own-ship-datum-and-moving-far)
3. [Feeding sensors, by what your sensor gives you](#3-feeding-sensors-by-what-your-sensor-gives-you)
4. [The NMEA path](#4-the-nmea-path)
5. [Heading and bias](#5-heading-and-bias)
6. [Getting results out](#6-getting-results-out)
7. [Static environment inputs](#7-static-environment-inputs)
8. [Choosing strategies](#8-choosing-strategies)
9. [Pitfall checklist](#9-pitfall-checklist)
10. [Config reference (appendix)](#10-config-reference-appendix)

---

## 1. Minimum viable integration

The shortest path: build an `OwnShipProvider`, push one pose, build a
`Measurement`, call `Tracker::process`, drain with `toTrackOutput`.

**Which CMake target?**

- `navtracker_core` — pure domain + ports + helpers, no I/O. Link this alone if
  you already have parsed sensor data and build `Measurement`s yourself.
- `navtracker_nmea` — the NMEA-0183 adapters. Link this *in addition* only if your
  input is raw NMEA strings.
- `navtracker_sim` is tests-only; ignore it.

The following is the real assembled pipeline from `app/example.cpp` (the canonical
end-to-end example — read it in full for context).

Wire the pieces (`app/example.cpp`):

```cpp
OwnShipProvider provider;  // library handles datum automatically
auto motion = std::make_shared<ConstantVelocity2D>(/*q=*/0.1);
EkfEstimator ekf(motion, /*init_pos_std_m=*/5.0);
GnnAssociator gnn(/*chi2_gate=*/20.0);
TrackManager mgr(/*confirm_hits=*/2, /*delete_misses=*/3);
Tracker tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/30.0);
```

Push at least one own-ship pose *before* any measurement — this initialises the
datum (`app/example.cpp`):

```cpp
OwnShipPose pose;
pose.time = Timestamp::fromSeconds(123.0);
pose.lat_deg = 53.500;
pose.lon_deg = 8.000;
pose.heading_true_deg = 45.0;
pose.position_std_m = 5.0;    // from your GPS receiver, or 0 + defaults
provider.update(pose);
```

Build a measurement and process it (`app/example.cpp`):

```cpp
const double lat = 53.55, lon = 8.05;
const auto enu = provider.datum().toEnu({lat, lon, 0.0});
Measurement m = makeMeasurementFromEnuPosition(
    SensorKind::Ais, "my_ais_feed",
    Timestamp::fromSeconds(123.0),
    Eigen::Vector2d(enu.x(), enu.y()),
    Eigen::Matrix2d::Zero(),  // empty -> defaults will fill in
    AssociationHints{/*mmsi=*/200000001u, std::nullopt});
applyDefaultsIfEmpty(m, defaults);
tracker.process(m);
```

Drain the results (`app/example.cpp`):

```cpp
for (const Track& t : mgr.tracks()) {
  if (t.status != TrackStatus::Confirmed) continue;
  const TrackOutput out = toTrackOutput(t, provider.datum());
  // out.position.lat_deg / lon_deg (WGS84 deg)
  // out.position.position_covariance_m2 (m^2, target-local NED)
  // out.velocity.sog_m_per_s / .cog_deg / .sigma_* / .is_valid
  // out.id, out.status, out.attributes, out.contributing_sources
}
```

That is the whole loop: `provider.update(pose)` as fixes arrive,
`tracker.process(m)` as measurements arrive, `toTrackOutput` whenever you want the
current picture. Everything below is optional refinement on top of this.

**Feed measurements in non-decreasing timestamp order.** The engine advances on
message timestamps (invariant 4), not the wall clock. A measurement stamped
*before* the current high-water mark is **silently dropped** (and counted as
`stale_dropped_` in `core/pipeline/Tracker.cpp`), not reordered. If your sensor
feeds can arrive out of order relative to each other, put a small reorder buffer
in front of `process`. See §9.

---

## 2. Own-ship, datum, and moving far

The **datum** is the origin of the local East-North-Up (ENU) tangent plane that
the whole engine works in. `OwnShipProvider`
(`core/own_ship/OwnShipProvider.hpp`) owns it for you.

- Construct with **no arguments** — it lazily initialises the datum from the first
  `update(pose)` call (`core/own_ship/OwnShipProvider.hpp`). A second constructor
  pins an explicit `geo::Datum` if you need one
  (`core/own_ship/OwnShipProvider.hpp`).
- It **auto-recenters** when own-ship moves more than the threshold in
  `DatumRecenterPolicy` — default **`recenter_threshold_km{30.0}`**
  (`core/own_ship/OwnShipProvider.hpp`). Set `enable_auto_recenter{false}` to pin
  the datum for a whole run.

> The datum is **bookkeeping, not a detection window.** Nothing is ever dropped
> for being far from the datum. Recentering only re-anchors the tangent plane so
> ENU coordinates stay numerically well-conditioned.

**The recenter gotcha.** When the datum moves, every component that caches
positions in ENU must be told, or its cache goes silently stale. The provider
fires `IDatumChangeSink::onDatumRecentered(old, new)`
(`core/own_ship/OwnShipProvider.hpp`) to everyone you register via
`registerDatumSink(...)` (`core/own_ship/OwnShipProvider.hpp`).

Wire a sink that reprojects your tracks (`app/example.cpp`):

```cpp
struct TrackShifterSink : IDatumChangeSink {
  TrackManager* mgr;
  explicit TrackShifterSink(TrackManager* m) : mgr(m) {}
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    shiftTracksOnDatumChange(*mgr, o, n);  // core/tracking/DatumShift.hpp
  }
};
TrackShifterSink mgr_sink{&mgr};
provider.registerDatumSink(&mgr_sink);
```

**Register every ENU-caching component as a datum sink.** These three concrete
models cache positions in ENU and must be registered if you use auto-recenter:

- `StaticObstacleModel` (`core/static/StaticObstacleModel.hpp`) — rebuilds its
  obstacle ENU cache.
- `CoastlineModel` (`core/land/CoastlineModel.hpp`) — swaps its datum.
- `LiveOccupancyModel` (`core/static/LiveOccupancyModel.hpp`) — re-anchors its
  occupancy grid.

```cpp
provider.registerDatumSink(&obstacle_model);
provider.registerDatumSink(&coastline_model);
provider.registerDatumSink(&occupancy_model);
```

> **Note.** If you use the MHT tracker, its hidden per-node kinematic state is not
> reachable through `shiftTracksOnDatumChange`; wire an `IDatumChangeSink` for it
> too (`core/pipeline/MhtTracker.hpp`).

**Facts worth knowing:** `datum()` **throws** `std::runtime_error` if no datum has
been established yet — always push a pose (or pin a datum) first
(`core/own_ship/OwnShipProvider.hpp`). Use `hasDatum()` to check
(`core/own_ship/OwnShipProvider.hpp`). `poseAtOrBefore(t)` gives the pose the
engine will use for a measurement stamped `t` (`core/own_ship/OwnShipProvider.hpp`).

---

## 3. Feeding sensors, by what your sensor gives you

You build a `Measurement` (`core/types/Measurement.hpp`) from your parsed data.
The `model` field picks how the engine reads `value` and `covariance`. There are
four kinds (`core/types/Ids.hpp`):

| `MeasurementModel` | `value` | `covariance` (R) | can start a track? |
|---|---|---|---|
| `Position2D` | `[east, north]` (m, ENU) | 2×2 (m²) | yes |
| `PositionVelocity2D` | `[px, py, vx, vy]` | 4×4 | yes |
| `RangeBearing2D` | `[range_m, bearing_rad]` (relative to `sensor_position_enu`) | 2×2 diag | yes |
| `Bearing2D` | `[bearing_rad]` (absolute ENU azimuth) | 1×1 | **no** |

Whether a kind can start a track is decided by the free function
`canInitiateTrack(model)` (`core/estimation/MeasurementModels.hpp`) — it is
*not* a field on `Measurement`. Only `Bearing2D` returns false.

**`R` is per measurement.** The covariance travels with each `Measurement`, so a
sensor whose accuracy grows with range just sets a larger `R` on the far reading.
That is the adapter's job and is fully supported — no global sensor-noise knob is
needed or wanted.

### You have absolute position (AIS-style)

Use `makeMeasurementFromEnuPosition` (`core/types/MeasurementBuilders.hpp`):

```cpp
Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor, std::string source_id, Timestamp t,
    Eigen::Vector2d enu_xy, Eigen::Matrix2d covariance,
    AssociationHints hints = {});
```

It produces a `Position2D` measurement directly — no pose lookup. If you pass an
all-zero covariance it is treated as "unknown" and left empty, so
`applyDefaultsIfEmpty` (below) can fill it (`core/types/MeasurementBuilders.cpp`).

### You have range + bearing (radar / EO-IR / sonar)

Two builders, depending on whether your bearing is relative to own-ship's bow or
already true (world-frame):

- `makeMeasurementFromRelativeBearing(...)`
  (`core/types/MeasurementBuilders.hpp`) — adds own-ship heading, then projects.
- `makeMeasurementFromTrueBearing(...)`
  (`core/types/MeasurementBuilders.hpp`) — bearing is already true.

Real call site (`app/example.cpp`):

```cpp
Measurement m = makeMeasurementFromRelativeBearing(
    SensorKind::ArpaTtm, "my_radar",
    Timestamp::fromSeconds(123.0),
    range_m, rel_bearing_rad,
    range_std_m, bearing_std_rad,
    provider);
```

**Important:** both bearing builders *project* range+bearing into an absolute
`Position2D` measurement, composing the range/bearing σ with own-ship's GPS
*position* uncertainty. They do **not** fold in own-ship *heading* uncertainty —
that σ must already be baked into your `bearing_std_rad` upstream
(`core/types/MeasurementBuilders.cpp`). The NMEA adapters do this for you from
their `heading_std_deg` config; if you call the builders directly, inflate
`bearing_std_rad` yourself. See §5.

If there is no own-ship pose at or before `t` (or no datum), the builder returns a
`Measurement` with empty `value`/`covariance` — check and buffer/drop it
(`core/types/MeasurementBuilders.hpp`).

### You have bearing only (passive camera, direction finder)

A `Bearing2D` measurement is a **wedge, not an ellipse**: one look fixes the
direction but not the range. So it can **refine or corroborate an existing track,
never start one** — `canInitiateTrack(Bearing2D) == false`, enforced at every
track-birth site in all three trackers (e.g. `core/pipeline/Tracker.cpp`,
`core/pmbm/PmbmTracker.cpp`). A bearing-only reading that gates to no existing
track is dropped rather than used to seed a guessed position.

There is **no public builder for `Bearing2D`** — construct it by hand: set
`m.model = MeasurementModel::Bearing2D`, `m.value = [bearing_rad]` (absolute ENU
azimuth), a 1×1 `R`, and `m.sensor_position_enu` if the sensor is not at the
datum.

For a worked reference producer, see `adapters/replay/CameraBearingCsvReader`
(a YOLO-detector-derived EO/IR camera-bearing channel: `Bearing2D`,
corroboration-only, with *absence* treated as evidence). It is a replay/fixture
loader — **out of scope** for live wiring (see the scope note in the intro) — but
it is the canonical example of building bearing-only measurements by hand.

### Your sensor gives no uncertainty at all

Leave `covariance` empty and apply per-(sensor, model) defaults
(`core/types/SensorDefaults.hpp`):

```cpp
auto defaults = pessimisticSensorDefaults();
applyDefaultsIfEmpty(m, defaults);
```

`applyDefaultsIfEmpty` fills the covariance only if it is empty, and sets
`covariance_is_default = true` as a diagnostic when it does
(`core/types/SensorDefaults.cpp`). The `pessimisticSensorDefaults()` values
(`core/types/SensorDefaults.cpp`) are deliberately conservative and cover six
(sensor, model) pairs:

| sensor + model | default 1-σ |
|---|---|
| `Ais` + `Position2D` | 30 m position |
| `Cooperative` + `Position2D` | 10 m position |
| `ArpaTll` + `Position2D` | 50 m position |
| `ArpaTtm` + `RangeBearing2D` | 75 m range, 1.5° bearing |
| `EoIr` + `RangeBearing2D` | 50 m range, 1.0° bearing |
| `EoIr` + `Bearing2D` | 1.5° bearing |

Any other (sensor, model) pair is left empty. A measurement whose covariance is
still empty when it reaches the tracker is **silently skipped** — never used to
initiate or update a track — so you *must* supply your own `R`, or call
`applyDefaultsIfEmpty`, for those pairs. Do not rely on the tracker to fill it in.
Operators with real sensor specs override the relevant
`pessimisticSensorDefaults()` fields before use.

**`covariance_is_default`** is diagnostic only — the tracker behaves identically
whether it is true or false (`core/types/Measurement.hpp`). It flows through to the
per-track `TrackOutput.covariance_is_default` (see §6) so downstream displays can
flag "this track has only default-noise measurements". Note these are two
different flags at two layers (per-measurement vs per-track output); see
`docs/output-contract.md`.

---

## 4. The NMEA path

The adapters in `adapters/` are **one optional implementation** for consumers
whose input is raw NMEA-0183 strings. If you already parse your sensors, skip this
section and build `Measurement`s directly (§3). Link `navtracker_nmea` to use
them.

- **`ArpaAdapter`** (`adapters/arpa/ArpaAdapter.hpp`) parses radar **TTM**
  (range/bearing) and **TLL** (target lat/lon) sentences into `Position2D`
  measurements. TTM needs the latest own-ship pose to project.
- **`EoIrAdapter`** (`adapters/eoir/EoIrAdapter.hpp`) takes a `CameraDetection`
  struct (relative bearing + optional range). Each `CameraDetection` carries its
  own per-detection uncertainties; if you leave them at their struct defaults they
  fall back to **`bearing_std_deg = 0.5`** and **`range_std_m = 10.0`** (defined on
  the struct in `adapters/eoir/EoIrAdapter.hpp`) — set them to your camera's real
  1-σ values.
- **`AisAdapter`** (`adapters/ais/AisAdapter.hpp`) takes an `AisDynamicReport`
  struct (MMSI + lat/lon + accuracy flag). It has no config struct; construct with
  just a datum.

**The TTM-vs-TLL rule.** A radar may report the *same target* two ways. Prefer
**TTM** when you want own-ship pose error modelled explicitly in `R`; **TLL** has
own-ship position error already folded into its lat/lon and cannot be separated —
inflate its position `R` to compensate. Do **not** double-count a target reported
both ways; associate by target number within one radar. The full TTM-vs-TLL
comparison (measurement, covariance, and heading-bias-correction differences) and
the wire formats live in `docs/sensors/sensor-reference.md` §2 ("Navigation radar
/ ARPA") — read it there; this guide does not duplicate it.

**Own-ship NMEA.** `OwnShipNmeaAdapter`
(`adapters/own_ship/OwnShipNmeaAdapter.hpp`) turns GGA/RMC/HDT/HDG sentences into
`OwnShipPose` updates on your `OwnShipProvider`. Its `OwnShipNmeaAdapterConfig`
(`adapters/own_ship/OwnShipNmeaAdapter.hpp`) carries GPS-noise, velocity-source,
and multi-heading-source knobs. The key one is **`gps_heading_talkers`** (empty by
default): list the NMEA talker IDs whose `$--HDT` sentences are true-heading from a
multi-antenna GPS. Any HDT talker *not* in that set is treated as gyro heading
(the backward-compatible `$GPHDT`-as-gyro path). See §5 for what the adapter does
with those.

---

## 5. Heading and bias

**Where heading comes from.** Own-ship heading is a field on `OwnShipPose`:
`heading_true_deg` (`core/own_ship/OwnShipProvider.hpp`). The v3 multi-source
fields (`gps_true_heading_deg`, `magnetic_heading_deg`, `magnetic_variation_deg`)
default to `NaN` = "not present" (`core/own_ship/OwnShipProvider.hpp`).

> **The "perfect gyro" pitfall.** The bearing-projection builders and the NMEA
> adapters read a heading-noise σ from the adapter config field
> **`heading_std_deg`, which defaults to `0.0`** — i.e. *zero* heading
> uncertainty, a perfect gyro (`adapters/arpa/ArpaAdapter.hpp`,
> `adapters/eoir/EoIrAdapter.hpp`). If your heading comes from a real compass,
> set this to its real 1-σ (degrees), or your range/bearing measurements will be
> over-confident in cross-range. (Note: this field lives on the *adapter configs*,
> not on `OwnShipPose`, which has no σ for its primary `heading_true_deg`.)

**Estimating heading bias.** `HeadingBiasEstimator`
(`core/bias/HeadingBiasEstimator.hpp`) is a scalar Kalman filter on a single
gyro-bias state `b` with random-walk dynamics. Wire any subset of five observation
kinds; sources can come and go mid-mission:

| Kind | Source | Math |
|---|---|---|
| `AisArpaPairObservation` | v1 AIS↔ARPA bearing pair | direct `b` measurement at the pair's range |
| `BearingInnovation` | v2 Tracker emission via `IBearingInnovationSink` | r = wrap(β_obs − β_pred); R = HᵀPH + R_meas; needs an anchor |
| `GyroVsGpsHeadingObservation` | v3 multi-antenna GPS | r = gyro − gps_hdg; R = σ_gps² |
| `GyroVsGpsCogObservation` | v3 GPS COG | r = gyro − cog; R = σ_cog² + σ_crab²; SOG and turn-rate gates |
| `GyroVsMagneticObservation` | v3 magnetic compass | r = gyro − (mag + variation); R = σ_mag² + σ_deviation² |

How each source is fed:

- **v3** (the three gyro-vs-X kinds) — `OwnShipNmeaAdapter` dispatches these
  automatically when you call `setHeadingBiasEstimator(&est)` and configure
  `gps_heading_talkers` (`adapters/own_ship/OwnShipNmeaAdapter.hpp`). RMC → COG
  obs, tagged HDT → GPS-heading obs, HDG → magnetic obs.
- **v1** — extract AIS↔ARPA pairs with the free function `extractPairs(...)`
  (`core/bias/AisArpaPairExtractor.hpp`) and feed them to `observe(...)`.
- **v2** — `Tracker::setBearingInnovationSink(&est)`
  (`core/pipeline/Tracker.hpp`); the tracker emits a `BearingInnovation` after
  each bearing update.

Wiring (`tests/integration/test_full_stack_pipeline.cpp`):

```cpp
HeadingBiasEstimator bias({});
adapter.setHeadingBiasEstimator(&bias);
// ...
Tracker tracker(estimator, associator, manager, 30.0);
tracker.setBearingInnovationSink(&bias);
```

The estimator implements `IHeadingBiasProvider`
(`ports/IHeadingBiasProvider.hpp`); pass it to the sensor adapters
(`ArpaAdapter`/`EoIrAdapter` take a `const IHeadingBiasProvider*`) so the estimated
bias is removed from their bearings.

**Per-sensor mounting bias** is a *separate* concern. `SensorBiasEstimator`
(`core/bias/SensorBiasEstimator.hpp`) estimates a per-sensor 2D position offset and
scalar bearing offset (one KF per sensor), independent of the platform-wide gyro
bias above. Its `SensorBiasEstimatorConfig`
(`core/bias/SensorBiasEstimator.hpp`) applies two observation gates — a **minimum
range** (`min_range_m`) and an **outlier σ** (`outlier_sigma`) reject. Wire it only
if you have mounting misalignment to calibrate; details in `docs/algorithms/`.

For the concepts behind all of this, see `docs/learning/` (heading-bias chapter).

---

## 6. Getting results out

Two ways, usable together.

### Pull

Walk the tracks and convert each with `toTrackOutput(track, datum)`
(`core/output/TrackOutput.hpp`). The `TrackOutput` (`core/output/TrackOutput.hpp`)
carries:

- `position` — `lat_deg`/`lon_deg` (WGS84 deg) and a 2×2 covariance in **m², in
  the target's local NED frame** (not ENU).
- `velocity` — `sog_m_per_s`, `cog_deg` (true, [0,360)), their σ, and `is_valid`.
  Velocity is `is_valid == false` for a 2D-only state or an unobserved/degenerate
  velocity covariance (`core/output/TrackOutput.cpp`).
- metadata — `id`, `status`, `last_update`, `attributes`, `contributing_sources`.
- `covariance_is_default` — OR of the contributing measurements' flags (§3).

Exact unit and validity rules, plus a worked example, are in
`docs/output-contract.md` — the authority for the output semantics.

`TrackStatus` (`core/types/Ids.hpp`) has **four** values: `Tentative`,
`Confirmed`, `Coasting`, `Deleted`. (`Coasting` is a track being propagated
without a fresh detection.)

### Push

Register sinks and get called on events instead of polling. All sinks are
nullable; null = today's behaviour, no overhead.

- **`ITrackSink`** (`ports/ITrackSink.hpp`) — `onTrackInitiated`,
  `onTrackConfirmed`, `onTrackUpdated`, `onTrackDeleted`. Register with
  `TrackManager::setTrackSink(...)` (`core/tracking/TrackManager.hpp`). (PMBM
  has its own `setTrackSink` in `core/pmbm/PmbmTracker.hpp`.)
- **`ICollisionRiskSink`** (`ports/ICollisionRiskSink.hpp`) — one
  `onCollisionRisk(event)` with a `CollisionRiskEvent {Entered/Exited/Updated,
  other_track_id, time, CpaPrediction}`.

Lifecycle + risk wiring (`tests/integration/test_full_stack_pipeline.cpp`):

```cpp
manager.setTrackSink(&lifecycle);        // ITrackSink
// ...
CpaEvaluatorConfig cpa_cfg;
cpa_cfg.d_threshold_m = 75.0;
cpa_cfg.enter_probability = 0.5;
cpa_cfg.exit_probability = 0.3;
CpaEvaluator cpa(manager, provider, cpa_cfg);
cpa.setSink(&risk);                              // ICollisionRiskSink
// each cycle:
cpa.evaluate(t);
```

### Collision risk (CPA)

`CpaEvaluator` (`core/collision/CpaEvaluator.hpp`) walks own-ship × each
Confirmed/Coasting track every `evaluate(t)` and emits risk transitions with
hysteresis. `CpaEvaluatorConfig` (`core/collision/CpaEvaluator.hpp`) defaults:
`d_threshold_m{500.0}`, `enter_probability{0.5}`, `exit_probability{0.3}`,
`evaluate_tentative{false}`, `emit_updates{false}`. The payload `CpaPrediction`
(`core/collision/Cpa.hpp`) carries CPA distance/σ, TCPA/σ, and
`probability_below_threshold`.

### Static hazards

`StaticHazardEvaluator` (`core/collision/StaticHazardEvaluator.hpp`) does the same
for own-ship vs charted obstacles: it fires `IStaticHazardSink`
(`ports/IStaticHazardSink.hpp`) `onStaticHazard(event)` on keep-clear-ring
crossings, using a plain range check (not CPA). `StaticHazardEvaluatorConfig`
(`core/collision/StaticHazardEvaluator.hpp`) defaults: `exit_hysteresis = 1.1`,
`emit_updates = false`. Convert a `StaticObstacle` to an operator-facing
`StaticHazardOutput` with `toStaticHazardOutput(obs)`
(`core/output/StaticHazardOutput.hpp`). See §7 for the obstacle input.

---

## 7. Static environment inputs

navtracker fuses moving vessels; the static-environment inputs tell it what parts
of the world are *fixed* so it does not spawn phantom tracks on structures, and so
it can warn about charted hazards. These are the "presence over classification"
mechanism of ADR 0002 — see `docs/adr/`.

### Charted hazards — `StaticObstacle`

Build a `std::vector<StaticObstacle>` (`core/types/StaticObstacle.hpp`) from
your chart / AtoN data — each has a geodetic `position`, a `footprint_radius_m`
(hard core), a `keep_clear_radius_m` (soft buffer), a `position_uncertainty_m`,
and category/water-level/lit metadata. Feed them to `StaticObstacleModel`
(`core/static/StaticObstacleModel.hpp`):

```cpp
StaticObstacleModel obstacles(std::move(vec), datum);  // params default soft_max=0.9
pmbm.setStaticObstacleModel(&obstacles);   // suppresses births under obstacles
```

The model implements `IStaticObstacleModel` (`ports/IStaticObstacleModel.hpp`) —
nullable, so unwired behaviour is exactly today's. Register it as a datum sink
(§2). Drain hazards to operators via `toStaticHazardOutput` /
`StaticHazardEvaluator` (§6).

### Land / coastline

`CoastlineModel` (`core/land/CoastlineModel.hpp`) implements `ILandModel`
(`ports/ILandModel.hpp`): it returns a clutter prior that ramps from open water to
inland, so the tracker expects more clutter near shore. Load shoreline from GeoJSON
with `loadCoastlineGeoJson(path, params)`
(`adapters/land/GeoJsonCoastline.hpp`) — this returns a `CoastlineGeometry`; then
construct the model with `CoastlineModel(geom, datum)`
(`core/land/CoastlineModel.hpp`) to get an `ILandModel`. Tune the ramp with
`CoastlinePriorParams` (`core/land/CoastlineGeometry.hpp`,
`inland_halfwidth_m`/`offshore_halfwidth_m`, both 50 m). Register as a datum sink
(§2).

```cpp
CoastlineGeometry geom = loadCoastlineGeoJson(path, params);
CoastlineModel coastline_model(geom, datum);   // ILandModel
provider.registerDatumSink(&coastline_model);  // §2
```

### Live occupancy detector — *in active development*

`LiveOccupancyModel` (`core/static/LiveOccupancyModel.hpp`) learns a persistence
grid from repeated detections and suppresses births on cells that behave like fixed
structure. It wears three faces: `IStaticObstacleModel` (birth suppression),
`ILiveOccupancyFeed` (`ports/ILiveOccupancyFeed.hpp`, fed scan observations), and
`IDatumChangeSink`. Its `LiveOccupancyParams` (`core/static/LiveOccupancyModel.hpp`)
has grid/EWMA/persistence knobs.

Three optional **corroboration inputs** discriminate real structure from a departed
vessel that pinned a cell:

- `setChartedStructure(points)` — a charted structure point cloud (piers/wharves
  densified to points). An emitted hazard within `chart_corroboration_radius_m`
  (default 100 m) of a charted point is labelled *corroborated* and is held
  regardless of the camera (evidence precedence).
- `observeVesselFix(fix)` — one AIS/cooperative vessel fix (current-datum ENU +
  timestamp). A birth is NEVER suppressed within `veto_radius_m` (default 100 m) of
  a fix seen within `veto_window_s` (default 60 s): an AIS/cooperative-known vessel
  must stay track-eligible (the strongest vessel discriminator). The veto is local
  (the rest of a structure still suppresses) and only lowers suppression to 0
  (conservation preserved); it lapses when the feed goes quiet. Feed only positional
  anchors (`isNonScanningSource` — AIS/Cooperative); the model stays kind-agnostic.
- `observeCamera(frame)` — one live camera frame (own-ship ENU, absolute-bearing
  detections, FOV, match tolerance). A structure cell the camera watches (in FOV,
  live frame) with no detection at its bearing for `camera_empty_sustain_s`
  (default 2 s) is *camera-observed-empty*. This is label-only by default. Setting
  `evict_camera_empty = true` promotes it to **behaviour**: a camera-observed-empty,
  chart-UNconfirmed cell whose streak is still recent (`camera_empty_recency_window_s`,
  default 5 s) is EVICTED — its persistence is spent so the frozen pin cannot
  re-emit. Eviction is conservation-safe (suppression is re-derived from the
  post-eviction hazard set) and inert until `observeCamera` is fed.

> **Maturity.** This detector is under active development — its design of record
> and remaining increments are in
> `docs/superpowers/plans/2026-07-03-stage1b-ii-structure-detector.md` (and the
> Stage 1b-i plan). **Its config may change**; check the plan before depending on
> it, and expect field names/defaults to move.

---

## 8. Choosing strategies

The two hot-path stages are **swappable ports** (architecture invariant 3):

- `IEstimator` (`ports/IEstimator.hpp`) — the per-track filter
  (predict/update/initiate/gate/likelihood).
- `IDataAssociator` (`ports/IDataAssociator.hpp`) — assigns a measurement batch to
  tracks (hard matches or soft betas).
- `IMotionModel` (`ports/IMotionModel.hpp`) — the state-transition model.

**Shipped defaults.** There is no single composition-root binary that picks a
canonical strategy — the choice is yours at wiring time. The simplest assembled
example (`app/example.cpp`) uses the single-hypothesis `Tracker` with
`EkfEstimator(ConstantVelocity2D)` + `GnnAssociator(chi2_gate=20.0)`. A fuller
example (`app/mht_fusion_example.cpp`) wires `MhtTracker` with an IMM estimator.

**The multi-hypothesis trackers** carry big nested `Config` structs — review the
few fields that matter most and leave the rest at header defaults:

- `MhtTracker::Config` (`core/pipeline/MhtTracker.hpp`) — e.g.
  `probability_of_detection = 0.9`, `clutter_density = 1e-4`,
  `gate_threshold = 9.0`, `n_scan = 3`, `k_best = 3`; header defaults enable the
  IPDA lifecycle + VIMM visibility.
- `PmbmTracker::Config` (`core/pmbm/PmbmTracker.hpp`) — e.g.
  `survival_probability = 0.99`, `probability_of_detection = 0.9`,
  `clutter_intensity = 1e-4`, `gate_threshold = 9.0`,
  `max_global_hypotheses = 30`, `confirm_threshold = 0.5`.

**Named tuned presets** (`makeMhtConfig()`, `makePmbmConfig()`, …) live in
`core/benchmark/Config.cpp`. That file is bench-side (out of the consumer scope
boundary), but it is the reference for known-good parameter sets — copy the values,
do not link the target. Deep comparison of estimator/associator choices lives in
`docs/algorithms/` (see the comparison-baselines doc).

---

## 9. Pitfall checklist

One-liners, each linking to where it is explained above.

- **No pose before first measurement** — `datum()` throws; push an `OwnShipPose`
  (or pin a datum) first. → §1, §2.
- **Calling the core from multiple threads** — nothing in the core is locked.
  `OwnShipProvider::update`, `Tracker::process`, `CpaEvaluator::evaluate`, and
  sink callbacks must be serialized onto one thread (or externally locked);
  concurrent calls corrupt `OwnShipProvider`'s unlocked deque and raw sink
  pointers. → intro.
- **Out-of-order timestamps** — measurements must arrive in non-decreasing
  timestamp order; one stamped before the high-water mark is silently dropped
  (counted as `stale_dropped_` in `core/pipeline/Tracker.cpp`), not reordered.
  Put a reorder buffer in front of `process` if your feeds can interleave. → §1.
- **Datum sink not registered** — after a 30 km auto-recenter, obstacle/coastline/
  occupancy caches and your tracks go stale. Register every ENU-caching component.
  → §2.
- **`heading_std_deg` left at 0** — a "perfect gyro"; range/bearing measurements
  become over-confident. Set the adapter config to your compass's real σ. → §5.
- **Empty covariance is dropped, not defaulted** — a measurement that reaches the
  tracker with an empty `R` is silently skipped (never initiates or updates a
  track). The tracker does *not* fill it in for you. Supply your own `R` or call
  `applyDefaultsIfEmpty(m, pessimisticSensorDefaults())` first. → §3.
- **Expecting bearing-only to create tracks** — `Bearing2D` can only refine an
  existing track, never start one. → §3.
- **TLL when TTM exists** — TLL has own-ship error baked in; prefer TTM, and never
  double-count a target reported both ways. → §4.
- **Treating MMSI / ARPA target id as the fusion key** — external identifiers are
  *hints* (`AssociationHints`), never the primary key. The stable `track_id` is
  the identity (invariant 5). → §3.

---

## 10. Config reference (appendix)

Every consumer-facing `…Config` struct, its header, what it controls, and the two
or three fields most worth reviewing. (`core/benchmark/` sweep configs and the
`adapters/foxglove/` debug recorder are out of scope and omitted.)

| Struct | Header | Controls | Fields worth reviewing |
|---|---|---|---|
| `OwnShipNmeaAdapterConfig` | `adapters/own_ship/OwnShipNmeaAdapter.hpp` | Own-ship NMEA parsing: GPS noise, velocity source, heading sources | `uere_m{5.0}`, `gps_heading_talkers{}`, `prefer_rmc_velocity{true}` |
| `ArpaAdapterConfig` | `adapters/arpa/ArpaAdapter.hpp` | Radar TTM/TLL → measurements | `heading_std_deg{0.0}`, `position_std_m{50.0}`, `bearing_std_deg{1.0}` |
| `EoIrAdapterConfig` | `adapters/eoir/EoIrAdapter.hpp` | EO/IR camera heading σ | `heading_std_deg{0.0}` |
| `CpaEvaluatorConfig` | `core/collision/CpaEvaluator.hpp` | Collision-risk thresholds + hysteresis | `d_threshold_m{500.0}`, `enter_probability{0.5}`, `exit_probability{0.3}` |
| `StaticHazardEvaluatorConfig` | `core/collision/StaticHazardEvaluator.hpp` | Keep-clear-ring alerts | `exit_hysteresis{1.1}`, `emit_updates{false}` |
| `HeadingBiasEstimatorConfig` | `core/bias/HeadingBiasEstimator.hpp` | Gyro-bias KF tuning + per-kind gates | `initial_variance_rad2` (5°)², `cog_min_sog_mps{3.0}`, `bi_min_range_m{50.0}` |
| `AisArpaPairExtractorConfig` | `core/bias/AisArpaPairExtractor.hpp` | AIS↔ARPA v1 pairing window | `cycle_window_seconds{0.5}`, `ais_position_std_fallback_m{10.0}` |
| `SensorBiasEstimatorConfig` | `core/bias/SensorBiasEstimator.hpp` | Per-sensor mounting-bias KF (two gates: min range, outlier σ) | `min_range_m{50.0}`, `outlier_sigma{5.0}`, `initial_pos_std_m{5.0}` |
| `SensorBiasPairExtractorConfig` | `core/bias/SensorBiasPairExtractor.hpp` | Per-sensor bias pairing window | `cycle_window_seconds{0.5}`, `sensor_position_std_fallback_m{10.0}` |
| `CrossSensorEligibilityConfig` | `core/bias/SensorBiasPairExtractor.hpp` | Which cross-sensor pairs are trustworthy | `min_existence_probability{0.95}`, `max_position_cov_trace_m2{25.0}` |
| `OwnShipVelocityEstimatorConfig` | `core/own_ship/OwnShipVelocityEstimator.hpp` | Own-ship velocity smoothing (nested in NMEA config) | `window_size{8}`, `maneuver_dv_threshold_mps{0.5}` |
| `UereEstimatorConfig` | `core/own_ship/UereEstimator.hpp` | Adaptive GPS-σ estimation (nested in NMEA config) | `window_size{8}`, `min_sigma_m{0.05}` |

Two big tracker `Config` structs are covered in §8 rather than tabulated here
because of their size: `MhtTracker::Config` (`core/pipeline/MhtTracker.hpp`) and
`pmbm::PmbmTracker::Config` (`core/pmbm/PmbmTracker.hpp`).

Non-`Config` tuning structs a consumer also touches: `DatumRecenterPolicy`
(`core/own_ship/OwnShipProvider.hpp`, §2), `StaticObstacleParams`
(`core/static/StaticObstacleModel.hpp`, §7), `CoastlinePriorParams`
(`core/land/CoastlineGeometry.hpp`, §7), `LiveOccupancyParams`
(`core/static/LiveOccupancyModel.hpp`, §7).
