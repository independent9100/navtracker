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
- `BearingWedgeModel` (`core/static/BearingWedgeModel.hpp`) — re-anchors its wedge
  apexes (§7). Wire it as a datum sink too if you use auto-recenter, or every
  emitted wedge points the wrong way after a recenter.

```cpp
provider.registerDatumSink(&obstacle_model);
provider.registerDatumSink(&coastline_model);
provider.registerDatumSink(&occupancy_model);
provider.registerDatumSink(&bearing_wedge_model);
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

**Your AIS also reports SOG/COG (speed and course)? Use `PolarVelocity.hpp` —
do not hand-roll the conversion.** `core/estimation/PolarVelocity.hpp` is the
single source of truth both our own AIS paths use (NMEA adapter and replay
loader — pinned equal by a no-drift test). It packages four decisions that were
each learned from a measured failure:

```cpp
#include "core/estimation/PolarVelocity.hpp"
// 1) Anchored/moored (AIS nav-status 1 or 5)? Then NO velocity content —
//    watch-circle drift is not a velocity (measured: it crosses the SOG
//    threshold and pollutes the track).
if (!aisNavStatusSuppressesVelocity(nav_status) &&
    sog_mps >= kAisSogVelocityMinMps) {           // 2) low-SOG: COG is jitter
  auto v = sogCogToEnuVelocity(sog_mps, cog_rad,  // 3) compass→ENU + units
                               sog_std, cog_std); // 4) speed-dependent cov +
  // → PositionVelocity2D [e, n, ve, vn], cov block   isotropic floor (kills
  //   from v.velocity / v.covariance                 the low-SOG rank-1
} else { /* emit Position2D as above */ }         //   overconfidence)
```

Default 1-σ / threshold constants (`kAisSogStdMps`, `kAisCogStdDeg`,
`kAisVelocityIsoFloorMps`, `kAisSogVelocityMinMps`) live in the same header —
AIS carries no uncertainty of its own, so use these unless your feed documents
better. Also populate `hints.nav_status` regardless (the anchored-vessel veto
keys on it — see the pitfall checklist) and `hints.mmsi` for identity.

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
    provider,
    /*hints=*/{}, /*heading_std_floor_deg=*/1.0);  // optional, see §5
```

**Important:** both bearing builders *project* range+bearing into an absolute
`Position2D` measurement, composing the range/bearing σ with own-ship's GPS
*position* uncertainty. For own-ship *heading* uncertainty they compose
`max(pose.heading_std_deg, heading_std_floor_deg)` in quadrature (#16, see §5):
set `OwnShipPose::heading_std_deg` per fix if your nav source reports varying
heading quality, and pass a `heading_std_floor_deg` as the static trust bound.
Leave both at their defaults (pose σ absent, floor `0.0`) and the builders add
*zero* heading σ — then you must bake heading σ into `bearing_std_rad` yourself
(the NMEA adapters do this for you from their `heading_std_deg` config, which
also acts as the floor over any per-pose σ).

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

### You have another tracker's output (shore / VTS remote tracks)

Your input is a **remote surveillance station's own track list** — a shore radar
or VTS feed that ships *filtered tracks*, not raw detections. Treat every such
track as a **pseudo-measurement** (design spec §13): someone else's filtered,
correlated, lifecycle-managed estimate — **never an independent observation**.
The same stance the library takes for ARPA TTM applies here, made explicit.

Use `RemoteTrackAdapter` (`adapters/remote_track/RemoteTrackAdapter.hpp`): feed it
a `RemoteTrackReport` per remote update, `poll()` for `SensorKind::RemoteTrack`
`Position2D` measurements in your datum frame.

```cpp
RemoteTrackAdapter remote(datum);               // defaults below
RemoteTrackReport r;
r.time = t; r.remote_track_id = 42;             // the remote station's own id
r.lat_deg = ...; r.lon_deg = ...;
r.position_covariance = stated_R;               // m², or leave zero if none stated
r.mmsi = 211000123u;                            // when the feed carries identity
r.source_id = "vts_hamburg";                    // ONE per remote STATION
remote.ingest(r);
tracker.processBatch(remote.poll());
```

Three rules the adapter enforces, because a filtered track is not a raw fix:

1. **R-inflation.** The remote system's stated covariance is multiplied by
   `r_inflation_factor` (a scalar on the covariance *matrix*/variance; default
   **×3**) to price the correlation/overconfidence of a filtered output. When the
   feed states **no** covariance, a pessimistic absolute default is used instead
   (`default_position_std_m`, default **50 m** — looser than AIS's 30 m; also in
   `pessimisticSensorDefaults().remote_track_position`). Never both.
2. **Rate thinning.** Consecutive filtered outputs for one remote track are
   correlated, not independent, so at most one update per `min_update_interval_s`
   (default **2 s**, per `(source_id, remote_track_id)`) is emitted; the rest are
   dropped (`thinnedCount()`).
3. **Identity as a hint, never the key.** `remote_track_id` → `hints.sensor_track_id`
   (scoped to the station's `source_id`; the remote may reuse/swap it), and `mmsi`
   is passed through. The fusion key stays the library's own `track_id` — a remote
   id-swap does **not** split the fused track.

**Defaults are blind.** ×3 / 50 m / 2 s were chosen with no deployment numbers,
biased to the safe direction (over-inflation only wastes a little of the feed's
precision; under-inflation is the dangerous direction). Replace them with the
shore feed's real numbers when known — all config, no code. The consistency
tripwire is a **NEES check** on a fusion scenario: if the fused estimate goes
overconfident, R-inflation has stopped being enough and you need proper
track-to-track fusion (covariance intersection) — deliberately **out of scope**
today (spec §13), to be revisited only when a real feed shows it.

**Velocity is opt-in and extra-suspicious.** By default velocity fields are
ignored (`accept_velocity{false}`) and every measurement is `Position2D` — a
shore feed's velocity is its smoothed derivative of positions it also sends, the
classic double-counting trap (see below). Enable `accept_velocity` only if you
have vetted the feed's velocity as genuinely independent.

**Circular-AIS guard.** If you wire raw AIS *and* an AIS-fusing shore feed, the
same transmission arrives twice. The adapter cannot pick a path silently, so it
surfaces the overlap: `remote.circularAisMmsis(raw_ais_mmsis)` returns the MMSIs
carried on both channels. Non-empty → dedupe (one path per vessel) or inflate for
the correlation; this is a deployment decision.

**A camera with a rangefinder is NOT a remote track.** A camera plus a distance
sensor gives you range + true/relative bearing — use the range+bearing path
(`makeMeasurementFromRelativeBearing` with `SensorKind::EoIr`), not this adapter.
RemoteTrack is only for another *tracker's* output.

RemoteTrack is a **non-scanning source** (`isNonScanningSource`): its point
reports are excluded from occupancy coverage-sector self-estimation (a filtered
track is not a swept arc), and it is strong vessel-evidence for the corroboration
suppression veto (§7). A remote *station* may still carry a **declared**
surveillance coverage area via `DeclaredSensorActivity` (§7) — that is you
telling the tracker where the feed sees, which is different from the tracker
inferring a wedge from point reports (the thing excluded).

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
(`core/types/SensorDefaults.cpp`) are deliberately conservative and cover seven
(sensor, model) pairs:

| sensor + model | default 1-σ |
|---|---|
| `Ais` + `Position2D` | 30 m position |
| `Cooperative` + `Position2D` | 10 m position |
| `RemoteTrack` + `Position2D` | 50 m position |
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

### Derived data and double-counting (the recurring trap)

Before you feed *any* value a sensor reports, ask one question: **is this new
information, or my own data coming back smoothed?** Feeding a sensor's *derived*
output alongside the raw data it was derived from counts the same evidence
twice. The filter's covariance shrinks as if two independent witnesses agreed —
when one witness merely repeated himself. Overconfident tracks → smaller gates →
missed associations. The trap has one shape and many disguises:

- **ARPA speed/course (TTM fields)** — computed by the radar *from the same
  range/bearing detections you already feed*. Never feed them as recurring
  measurements. Legitimate uses, **both now wired in `ArpaAdapter` (#20)**: a
  **one-shot** velocity prior at track birth (`hints.birth_velocity_enu`, seeded
  from TTM speed/course, consumed only in the estimator's `initiate` then
  discarded — a prior can't double-count; toggle `seed_birth_velocity_from_ttm`),
  and a target-swap diagnostic (`hints.sensor_track_id_suspect` set when the
  radar's own course for a TTM target number jumps > `swap_course_jump_deg`
  between reports while moving — the number-reuse signature; diagnostic only, as
  association does not consume `sensor_track_id`). Mind the radar's stabilisation
  mode: TTM course may be ground- or water-referenced — confirm per deployment.
- **TLL positions** — the radar's echo already fused with *its* own-ship GNSS
  and heading (see §4).
- **A shore/VTS feed's velocity** — another tracker's smoothed derivative of
  positions it also sends you. Same rule; see the remote-track ticket (R10).
- **Raw AIS + an AIS-fusing shore feed** — the same transmission arriving twice
  dressed as two sensors.

The contrast that makes the rule easy to remember: **AIS SOG/COG/heading are
fine as measurement content** — they come from the *target's own* GPS and gyro,
a genuinely independent witness, not a derivative of positions you already
consume.

---

## 4. The NMEA path

The adapters in `adapters/` are **one optional implementation** for consumers
whose input is raw NMEA-0183 strings. If you already parse your sensors, skip this
section and build `Measurement`s directly (§3). Link `navtracker_nmea` to use
them.

- **`ArpaAdapter`** (`adapters/arpa/ArpaAdapter.hpp`) parses radar **TTM**
  (range/bearing) and **TLL** (target lat/lon) sentences into `Position2D`
  measurements. TTM needs the latest own-ship pose to project. TTM speed/course
  are NOT fed as recurring content (double-counting; §3) — instead they seed a
  one-shot `hints.birth_velocity_enu` prior and drive the swap diagnostic
  (`hints.sensor_track_id_suspect`); see the `ArpaAdapterConfig` knobs.
- **`EoIrAdapter`** (`adapters/eoir/EoIrAdapter.hpp`) takes a `CameraDetection`
  struct (relative bearing + optional range). Each `CameraDetection` carries its
  own per-detection uncertainties; if you leave them at their struct defaults they
  fall back to **`bearing_std_deg = 0.5`** and **`range_std_m = 10.0`** (defined on
  the struct in `adapters/eoir/EoIrAdapter.hpp`) — set them to your camera's real
  1-σ values.
- **`AisAdapter`** (`adapters/ais/AisAdapter.hpp`) takes an `AisDynamicReport`
  struct (MMSI + lat/lon + accuracy flag, plus optional self-reported
  `sog_knots` / `cog_deg` / `heading_deg` / `nav_status` — backlog #20) and an
  optional **`AisAdapterConfig`** (`adapters/ais/AisAdapter.hpp`). Heading →
  `hints.heading_deg` (attribute), nav-status → `hints.nav_status` (the
  anchored/moored veto cue, §6/§7); AIS "not available" sentinels (heading 511,
  nav-status 15, SOG 1023) are dropped at the edge. When the report carries a
  usable SOG (≥ `sog_velocity_min_mps`, default 0.5 m/s) **and** COG, the adapter
  emits a **`PositionVelocity2D`** measurement with the target-reported velocity
  as content (COG is true, clockwise from north → ENU); below the threshold COG
  is meaningless and it stays `Position2D` ("COG down-weighted at low SOG"). The
  velocity covariance is the polar Jacobian of SOG/COG (σ from `sog_std_mps` /
  `cog_std_deg`, AIS carries none) plus an isotropic `velocity_iso_floor_mps`
  floor so a noisy low-SOG COG can't make the velocity *direction* overconfident.
  This is legitimate content, not double-counting: AIS SOG/COG come from the
  target's *own* GPS (§3). Set `emit_velocity_from_sog_cog = false` to force
  `Position2D` if you distrust a feed's velocity. Position σ is
  `position_std_high_accuracy_m` / `position_std_standard_m` (10 / 30 m).

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

**Per-fix heading σ (#16).** `OwnShipPose` also carries an optional
`std::optional<double> heading_std_deg` — the per-fix 1-σ of `heading_true_deg`,
in degrees. Set it when your nav source reports heading quality that varies
fix-to-fix (gyro settling, sensor switchover). Every relative/true-bearing
measurement composed through that pose then folds `max(pose.heading_std_deg,
floor)` into its angular covariance in quadrature, where `floor` is a static
trust bound: the builders take it as the `heading_std_floor_deg` argument
(default `0.0`), and the NMEA adapters use their config `heading_std_deg` as the
floor. **The floor can only widen, never tighten** — a pose claiming an
implausibly tight σ cannot make measurements overconfident. Absent per-pose σ
(the default) ⇒ heading σ comes only from the floor / adapter config, exactly as
before, bit-identical.

> **The "perfect gyro" pitfall.** The bearing-projection builders and the NMEA
> adapters read a heading-noise σ from the adapter config field
> **`heading_std_deg`, which defaults to `0.0`** — i.e. *zero* heading
> uncertainty, a perfect gyro (`adapters/arpa/ArpaAdapter.hpp`,
> `adapters/eoir/EoIrAdapter.hpp`). If your heading comes from a real compass,
> set this to its real 1-σ (degrees), or your range/bearing measurements will be
> over-confident in cross-range. With #16, this config σ is now the *floor* in
> `max(pose.heading_std_deg, config)`, so it protects you even when a per-fix
> pose σ is present but bogus. If you call the builders directly instead of the
> adapters, pass `heading_std_floor_deg` for the same protection.

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
  `attributes` carries the vessel identity: `mmsi` (from AIS) and `platform_id`
  (numeric cooperative/fleet identity, also carried by a remote/VTS feed when it
  has one) — **parallel keys, either or both may be set**. Both are surfaced
  last-write-wins from the hints of the measurements a track claims, under **both
  the PMBM and MHT trackers** (before R11 the PMBM path filled no attributes, so
  the operator display could not name which fleet member a track was). Identity
  is an attribute, never the fusion key — that is always the internal `id`
  (invariant 5). Feed it in via `Measurement.hints.mmsi` / `hints.platform_id`.
  `attributes` also carries **target-reported kinematics** (backlog #20, from an
  AIS self-report): `heading_deg` (true heading, deg [0,360)) and `nav_status`
  (AIS navigational-status code; 1 = at anchor, 5 = moored). Same last-write-wins
  surfacing. `heading_deg` fills the "stationary, direction undefined" gap — an
  anchored vessel still *points* somewhere when SOG≈0 makes COG meaningless — and
  `nav_status` is both an operator cue and the corroboration veto's data path
  (§7: a self-declared anchored/moored vessel is never suppressed). Feed them via
  `Measurement.hints.heading_deg` / `hints.nav_status` (the `AisAdapter` fills
  both from an `AisDynamicReport`, dropping the AIS "not available" sentinels).
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
- **`IPmbmDiagnosticSink`** (`core/pmbm/PmbmDiagnostics.hpp`) — **forensics only,
  not a production integration point.** `PmbmTracker::setDiagnosticSink(...)`
  emits a per-scan `PmbmScanDiag` exposing MBM-internal state the aggregated
  `tracks()` output collapses away (per-identity existence mass, dominant-hyp
  `r`, claimed measurement, state divergence, prune/cap events). It lives in
  `core/pmbm/` (not `ports/`) because a normal consumer never wires it; it exists
  for close-pass / track-death diagnosis (backlog #25, reproducer
  `tools/pmbm_closepass_trace.py`). Null (default) = zero overhead,
  byte-identical tracking.

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
and category/water-level/lit metadata. Either build the vector yourself or load
Point features from GeoJSON with `loadStaticObstaclesGeoJson(path)` /
`parseStaticObstaclesGeoJson(json_text)`
(`adapters/static/GeoJsonStaticObstacles.hpp`) — symmetric with the coastline
loader below. Feed the vector to `StaticObstacleModel`
(`core/static/StaticObstacleModel.hpp`):

```cpp
std::vector<StaticObstacle> vec = loadStaticObstaclesGeoJson(path);  // or build by hand
StaticObstacleModel obstacles(std::move(vec), datum);  // params default soft_max=0.9
pmbm.setStaticObstacleModel(&obstacles);   // suppresses births under obstacles
```

The model implements `IStaticObstacleModel` (`ports/IStaticObstacleModel.hpp`) —
nullable, so unwired behaviour is exactly today's. Register it as a datum sink
(§2). Drain hazards to operators via `toStaticHazardOutput` /
`StaticHazardEvaluator` (§6).

> **Wiring the pointer is a silent no-op without the flag.**
> `setStaticObstacleModel(&obstacles)` is inert unless you also set
> **`PmbmTracker::Config::use_static_obstacle_model = true`** (default
> **`false`**, `core/pmbm/PmbmTracker.hpp`). With the flag on, adaptive-birth
> intensity is scaled by `(1 − birthSuppression(pos))`; a birth whose
> suppression exceeds **`static_obstacle_hard_gate`** (default **`0.95`** — only
> the footprint interior reaches it, so the keep-clear buffer stays soft) is
> dropped entirely.

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
pmbm.setLandModel(&coastline_model);           // birth-suppression prior near shore
provider.registerDatumSink(&coastline_model);  // §2
```

> **Wiring the pointer is a silent no-op without the flag.**
> `setLandModel(&coastline_model)` is inert unless you also set
> **`PmbmTracker::Config::use_land_model = true`** (default **`false`**,
> `core/pmbm/PmbmTracker.hpp`). With the flag on, adaptive-birth intensity is
> scaled by `(1 − clutterPrior(birth_pos))`, and a birth whose prior exceeds
> **`land_birth_hard_gate`** (default **`0.95`** — confidently inland only, never
> the waterline) is dropped, protecting anchored near-shore vessels.

### Live occupancy detector — *in active development*

`LiveOccupancyModel` (`core/static/LiveOccupancyModel.hpp`) learns a persistence
grid from repeated detections and suppresses births on cells that behave like fixed
structure. It wears three faces: `IStaticObstacleModel` (birth suppression),
`ILiveOccupancyFeed` (`ports/ILiveOccupancyFeed.hpp`, fed scan observations), and
`IDatumChangeSink`. Its `LiveOccupancyParams` (`core/static/LiveOccupancyModel.hpp`)
has grid/EWMA/persistence knobs.

Because it is both a feed sink and an obstacle model, wire it on **both** PMBM
setters — and each has its own enabling flag:

```cpp
pmbm.setLiveOccupancyFeed(&occupancy_model);      // per-scan (position, 1−r) feed IN
pmbm.setStaticObstacleModel(&occupancy_model);    // learned structure suppresses births OUT
provider.registerDatumSink(&occupancy_model);     // §2 — re-anchors the grid on recenter
```

> **Two pointers, two silent-no-op flags.** The feed only flows when
> **`PmbmTracker::Config::feed_clutter_map = true`** (default **`false`**,
> `core/pmbm/PmbmTracker.hpp`) — that flag is what *produces* the per-scan
> `(position, 1 − r)` clutter bundle the model consumes; with it off,
> `setLiveOccupancyFeed` receives nothing and the grid never learns. The
> suppression face is gated by **`use_static_obstacle_model = true`** (§ static
> obstacle above), the same flag the charted-obstacle model uses.

Three optional **corroboration inputs** discriminate real structure from a departed
vessel that pinned a cell:

- `setChartedStructure(points)` — a charted structure point cloud (piers/wharves
  densified to points). An emitted hazard within `chart_corroboration_radius_m`
  (default 100 m) of a charted point is labelled *corroborated* and is held
  regardless of the camera (evidence precedence).
- `observeVesselFix(t, pos, anchored=false)` — one AIS/cooperative vessel fix
  (current-datum ENU + timestamp). A birth is NEVER suppressed within
  `veto_radius_m` (default 100 m) of a fix seen within `veto_window_s` (default
  60 s): an AIS/cooperative-known vessel must stay track-eligible (the strongest
  vessel discriminator). The veto is local (the rest of a structure still
  suppresses) and only lowers suppression to 0 (conservation preserved); it lapses
  when the feed goes quiet. Feed only positional anchors (`isNonScanningSource` —
  AIS/Cooperative/RemoteTrack); the model stays kind-agnostic. **#20:** pass
  `anchored=true` for a self-declared stationary vessel (the `PmbmTracker` producer
  sets it from AIS nav-status 1 = anchor / 5 = moored) — its veto is held for the
  longer **`anchored_veto_window_s`** (default 600 s) instead, because an anchored
  vessel reports infrequently (~3 min) yet (being stationary, structure-like) is
  exactly the object that must never be suppressed into nothing (ADR 0002 / R3).
  The port speaks `anchored`, not "nav-status": the sensor-format translation
  stays in the producer.
- `observeCamera(frame)` — one live camera frame (own-ship ENU, absolute-bearing
  detections, FOV, match tolerance). A structure cell the camera watches (in FOV,
  live frame) with no detection at its bearing for `camera_empty_sustain_s`
  (default 2 s) is *camera-observed-empty*. This is label-only by default. Setting
  `evict_camera_empty = true` promotes it to **behaviour**: a camera-observed-empty,
  chart-UNconfirmed cell whose streak is still recent (`camera_empty_recency_window_s`,
  default 5 s) is EVICTED — its persistence is spent so the frozen pin cannot
  re-emit. Eviction is conservation-safe (suppression is re-derived from the
  post-eviction hazard set) and inert until `observeCamera` is fed.

**LOS/shadow guard (`LiveOccupancyParams::shadow_guard`, `ShadowGuardParams`).**
Coverage-aware decay (when `estimate_coverage_sector` is on, below) forgets a cell
the sensor swept but got no return from. A cell swept in azimuth can be physically
blocked by a closer vessel — the return truncates at the occluder — so "no return"
is a *shadow*, not vacancy. The guard, from each scan's own returns, casts a
shadow wedge behind every closer occluder cluster and SKIPS decay of cells inside
it (holding a moored vessel's occupancy through a passing ship's crossing).
Per-instance, **disabled by default** (so a default-constructed model is
byte-identical); **ON in `imm_cv_ct_pmbm_occupancy_detector_coverage`**. Knobs:
`enabled`, `min_occluder_returns` (1 for CFAR-plot feeds — each return is a real
detection; raise for raw sub-threshold-cell feeds), `cluster_gap_rad`,
`wedge_pad_rad`, `range_margin_m` (a cell must be this far beyond the occluder —
occluder radial extent + range noise). Safe direction: an over-detected occluder
only holds a cell longer, never causes a spurious decay. Geometry + rationale:
`core/static/ShadowMask.hpp` and `docs/algorithms/live-static-occupancy.md` §1.2.1.

> **Maturity.** This detector is under active development — its design of record
> and remaining increments are in
> `docs/superpowers/plans/2026-07-03-stage1b-ii-structure-detector.md` (and the
> Stage 1b-i plan). **Its config may change**; check the plan before depending on
> it, and expect field names/defaults to move.

### Camera-only contact — the bearing-wedge hazard

**What you have:** a camera (or any bearing-only sensor) seeing a radar-silent
target — a kayak, a small wooden boat — as a `Bearing2D` stream. **What you
want:** it not to vanish. A bearing-only measurement cannot initiate a track (no
range → no position), but ADR 0002 forbids it becoming nothing. The clean fix is a
range sensor (camera + range → the existing range/bearing path, §3). Absent that,
`BearingWedgeModel` (`core/static/BearingWedgeModel.hpp`) is the safety net: it
surfaces the **direction** as a wedge from own-ship. Learning intro: chapter 28;
reference: `docs/algorithms/bearing-wedge-hazard.md`.

It is **standalone** — not on the PMBM hot path. You feed it each cycle:

```cpp
BearingWedgeModel wedges(datum);                 // BearingWedgeParams optional
provider.registerDatumSink(&wedges);             // MANDATORY with auto-recenter (§2)
// ... per cycle:
wedges.observeBearing(t, own_enu, bearing_math_rad, sigma_composed_rad,
                      "cam0", contact_id);        // a camera-only detection
wedges.observeConfirmedTracks(confirmed_track_enu);  // for handover (below)
wedges.pruneStale(now_s);                         // drop contacts the camera lost
for (const auto& h : wedges.hazardOutputs()) { /* draw the sector */ }
```

Four things to get right:

1. **`bearing_math_rad` is the `Bearing2D` value verbatim** — the math angle
   `atan2(dN, dE)` (CCW from east), the same convention the tracker uses
   internally. The output (`BearingWedgeOutput`, §6-style drain) converts it to a
   **true** bearing for display.
2. **`sigma_composed_rad` must be the COMPOSED σ** — `σ_camera ⊕ σ_heading` in
   quadrature — because the bearing you feed is *relative bearing + own-ship
   heading*, so heading error is part of it. This is the same composition the
   bearing builders now do internally (#16, §5): the heading term is
   `max(pose.heading_std_deg, floor)`. Compute σ_heading the same way here — the
   per-fix `OwnShipPose::heading_std_deg` floored by your compass's config σ —
   then quadrature-add σ_camera. The wedge half-width is `max(2σ,
   min_half_width_rad)`; the floor (default ≈ 1.5°) stops an optimistic σ from
   drawing an implausibly thin wedge (calibration p90 was 1.32°).
3. **Handover is by suppression, not deletion.** `observeConfirmedTracks(...)`
   supplies the current confirmed-track positions; a wedge is *hidden from the
   drain* while a track sits in its angular span, and *reappears* the instant no
   track does — recomputed every drain, never latched. This is deliberate: a near
   vessel crossing the bearing of a far, still-seen camera contact must not
   permanently erase it (the ADR-0002 forbidden failure). Only `pruneStale`
   (camera went quiet) actually removes a wedge.
4. **Identity survives, reuse does not.** Key each detection by
   `(source_id, contact_id)` — the camera's own contact/detector track number. A
   number that goes quiet and returns (sensor number-reuse) mints a **new**
   `wedge_id`, never resurrecting the dead one; pass `suspect=true` (e.g. from the
   #20 `sensor_track_id_suspect` cue) to force a fresh id immediately.

`max_range` is `std::optional<double>` — **absent = unbounded** (range genuinely
unknown, the defining case). CPA is **not** computable for a wedge; the operator
reading is "keep clear along that bearing".

### Sensor coverage / visibility — `ISensorActivity`

The models above tell the tracker where the *world* is fixed. This channel tells
it where each *sensor sees* — so a track's silence can be read correctly. The
existence recursion after a missed detection needs the right `p_D` charged at the
right time: a radar sweep that **covered** a track's predicted position and
returned nothing is strong evidence, but a track simply between sweeps, out of
range, or outside the azimuth sector is **no** evidence and must not bleed
existence. Without this channel existence decays on wall-clock time
(`idle_halflife_sec`); with it, existence moves only on a genuine per-duty-cycle
surveillance miss.

You supply the coverage picture through the **`ISensorActivity`** port
(`ports/ISensorActivity.hpp`). The shipped implementation is
**`DeclaredSensorActivity`** (`core/sensor_activity/DeclaredSensorActivity.hpp`):
you hand it a `std::vector<ChannelProfile>`, one per sensor channel. Each
`ChannelProfile` declares a `ChannelKind` (`Surveillance` or `Cooperative`), the
`SensorKind` it applies to, and — for surveillance — `duty_cycle_sec`,
`max_range_m`, `sector_center_rad`, `sector_width_rad` (default full circle),
`p_D`; for cooperative — `expected_report_interval_sec`. The tracker treats the
sensor as mounted at the ENU origin, so mount off-centre needs a translated
profile or an adapter.

```cpp
DeclaredSensorActivity activity({
    {ChannelKind::Surveillance, SensorKind::ArpaTtm,
     /*duty_cycle_sec=*/2.5, /*max_range_m=*/8000.0,
     /*sector_center_rad=*/0.0, /*sector_width_rad=*/6.2831853, /*p_D=*/0.9,
     /*expected_report_interval_sec=*/0.0},
    {ChannelKind::Cooperative, SensorKind::Ais,
     0.0, 0.0, 0.0, 6.2831853, 0.0, /*expected_report_interval_sec=*/12.0},
});
pmbm.setSensorActivity(&activity);   // ports/ISensorActivity.hpp — nullable
```

> **Opt-in, and a silent no-op without the flag.** `setSensorActivity(...)` is
> nullable (null = today's behaviour) *and* inert unless you also set
> **`PmbmTracker::Config::use_sensor_activity = true`** (default **`false`**,
> `core/pmbm/PmbmTracker.hpp`). With both in place, the misdetection step charges
> at most one miss per surveillance duty cycle instead of one per scan, and
> `idle_halflife_sec` / the per-blip miss path are bypassed. A cooperative-only
> track is **never** penalised in existence; it is retired only after
> **`cooperative_stale_timeout_sec`** (default **`0.0`** = never by this rule)
> seconds without an own-identity report.

#### Cooperative + radar: the deployment recipe (R9)

The first real deployment is a cooperative fleet partner (`SensorKind::Cooperative`
GNSS fixes carrying `platform_id`) fused with radar. The correct "silence" and
retirement behaviour for that mix depends on getting three things right together:

1. **Miss model = `use_sensor_activity` ALONE.** Do **not** also set
   `source_aware_misdetection` — the two are **alternative** miss models and the
   `PmbmTracker` constructor **throws** on the pair (a fail-loud guard). With both
   on, the identity gate short-circuits an empty scan before the activity model
   runs, silently blocking the cooperative retirement in (3). The named benchmark
   `imm_cv_ct_pmbm` configs use `source_aware_misdetection`; the deployment
   configs (`imm_cv_ct_pmbm_land`, coverage variants) use `use_sensor_activity`
   alone. Pick one.

2. **Declare BOTH channels a profile.** Give the radar a
   `ChannelKind::Surveillance` profile (its `duty_cycle_sec`, `max_range_m`,
   sector, `p_D`) **and** the cooperative feed a `ChannelKind::Cooperative`
   profile (its `expected_report_interval_sec`). Without profiles the tracker
   falls back to a legacy `== SensorKind::Cooperative` check that behaves
   differently — the cooperative timer resets and retirement timing are only
   correct when the profiles are present. (AIS declared as `Cooperative` counts
   as a cooperative announce too: its silence is weak identity-keyed evidence,
   never a surveillance miss.)

3. **Radar coverage suppresses cooperative-timeout retirement.** The
   cooperative-stale retirement sits inside the *no-surveillance-opportunity*
   branch: while radar keeps covering the track's predicted position, a
   cooperative dropout does **not** retire it (the platform is still observed,
   just not announcing). Retirement fires only when **both** channels go silent —
   no covering radar sweep *and* no cooperative report — past
   `cooperative_stale_timeout_sec`. This is the behaviour the fusion scenario
   `PmbmRemoteTrackFusion` and `test_pmbm_sensor_activity.cpp` pin.

```cpp
DeclaredSensorActivity activity({
    {ChannelKind::Surveillance, SensorKind::ArpaTtm,
     /*duty_cycle_sec=*/1.0, /*max_range_m=*/20000.0,
     0.0, 6.2831853, /*p_D=*/0.9, /*expected_report_interval_sec=*/0.0},
    {ChannelKind::Cooperative, SensorKind::Cooperative,
     0.0, 0.0, 0.0, 6.2831853, 0.0, /*expected_report_interval_sec=*/5.0},
});
PmbmTracker::Config c;
c.use_sensor_activity = true;            // NOT source_aware_misdetection
c.cooperative_stale_timeout_sec = 20.0;  // retire only after BOTH go silent this long
pmbm.setSensorActivity(&activity);
```

The cooperative platform's `platform_id` is surfaced to the operator on the fused
`TrackOutput.attributes.platform_id` (see §6, R11) — this recipe delivers the
lifecycle half, that field the identity half.

Two optional surveillance-sector self-estimation knobs feed the occupancy grid's
decay (see the occupancy detector above): **`estimate_coverage_sector`** (default
**`false`**) makes each per-scan occupancy bundle carry the swept arc of that
scan's returns so the grid decays only cells it actually observed, padded by
**`coverage_az_pad_rad`** (default **`0.087`** rad ≈ 5° each side) and
**`coverage_range_pad_frac`** (default **`0.1`** = 10 % beyond the farthest
return); **`coverage_cluster_gap_rad`** (default **`0.349`** rad ≈ 20°) splits a
scan's returns into separate echo clusters so an inter-cluster gap is not
over-claimed as swept.

A comms-loss notification is registerable: **`setStaleSignalSink(...)`**
(`ports/IStaleSignalSink.hpp`) fires `onTrackStale(id, now)` once per scan per
track whose cooperative report is overdue. It is **pure notification** — it MUST
NOT be wired to anything that lowers existence (decision spec §9c).

> **Why it stays opt-in.** The coverage channel is deliberately behind the
> `use_sensor_activity` flag pending broader real-data A/B validation — it needs
> per-sensor duty-cycle/coverage numbers that not every deployment has, and its
> effect is A/B-gated per scenario (win depends on the feed carrying cooperative
> identity; the zero-AIS `philos` real replay, for example, exercises none of the
> cooperative path). Full reference: `docs/algorithms/pmbm-design.md` §8
> ("Coverage / Visibility Channel"); plain-English intro:
> `docs/learning/24-coverage-visibility-channel.md`.

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

`PmbmTracker::Config` also carries the opt-in enabling flags for the static-
environment models (`use_land_model`, `use_static_obstacle_model`,
`feed_clutter_map`, §7) and the coverage channel (`use_sensor_activity`,
`estimate_coverage_sector`, §7) — a wired model pointer does nothing until its
flag is on.

**PDA soft detected-branch (data-association softening).** Under the default
K=1 GNN, a detected Bernoulli hard-commits to its single lowest-cost gated
measurement, so a gate-closer clutter return can pull the state off a real
target. **`use_pda_soft_detected_branch`** (default **`false`**) replaces that
hard update with a likelihood-weighted (β) combination over the Bernoulli's
gated, unclaimed measurements — reducing to today's behaviour when only one
measurement gates. When a land model is also wired, **`pda_pool_excludes_land`**
(default **`false`**) drops any non-winner return whose land clutter prior
exceeds **`pda_pool_land_clutter_gate`** (default **`0.5`**, the waterline) from
that pool, so it softens against water clutter only, not shore/dock structure.
Detail in `docs/algorithms/pmbm-design.md`.

**Named tuned presets** (`makeMhtConfig()`, `makePmbmConfig()`, …) live in
`core/benchmark/Config.cpp`. That file is bench-side (out of the consumer scope
boundary), but it is the reference for known-good parameter sets — copy the values,
do not link the target. Ready-made recipes that turn on the §7 static-environment
and coverage features (copy their field values into your own `Config`):
`imm_cv_ct_pmbm_land`, `imm_cv_ct_pmbm_coverage`, `imm_cv_ct_pmbm_coverage_land`,
`imm_cv_ct_pmbm_bundle_land`, and `imm_cv_ct_pmbm_occupancy`. Deep comparison of
estimator/associator choices lives in `docs/algorithms/` (see the
comparison-baselines doc).

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
  become over-confident. Set the adapter config (or builder
  `heading_std_floor_deg`) to your compass's real σ; it is the *floor* over any
  per-fix `OwnShipPose::heading_std_deg` (#16). → §5.
- **Empty covariance is dropped, not defaulted** — a measurement that reaches the
  tracker with an empty `R` is silently skipped (never initiates or updates a
  track). The tracker does *not* fill it in for you. Supply your own `R` or call
  `applyDefaultsIfEmpty(m, pessimisticSensorDefaults())` first. → §3.
- **Expecting bearing-only to create tracks** — `Bearing2D` can only refine an
  existing track, never start one. → §3.
- **TLL when TTM exists** — TLL has own-ship error baked in; prefer TTM, and never
  double-count a target reported both ways. → §4.
- **Feeding derived data as measurements** — ARPA speed/course, a shore feed's
  velocity, any value a sensor computed from positions you already consume:
  feeding it back double-counts the same evidence and makes tracks
  over-confident. Ask "new information, or my own data smoothed?" One-shot
  birth priors and diagnostics are fine; recurring measurements are not.
  (AIS SOG/COG is the exception — the target's own GPS is an independent
  witness.) → §3.
- **COG wired as heading on a slow or stationary ship** — GPS course-over-ground
  is *derived from motion*; at SOG ≈ 0 it is driven by meter-level GPS jitter
  and can point anywhere, jumping scan to scan. If `heading_true_deg` comes
  from COG, every relative-bearing sensor (radar TTM, camera) rotates with the
  jumps and targets smear in arcs around own-ship — worst exactly when moored
  or at anchor. Use a real heading source (gyro, dual-antenna GPS heading,
  magnetic); only fall back to COG above a firm SOG threshold (the bias
  estimator's own COG observation is SOG- and turn-rate-gated for this exact
  reason — copy that discipline). The **nav-input guard** below now *flags* this
  (and a stale feed, and position/heading glitches) at the edge. → §5, guard below.
- **`hints.nav_status` not populated for anchored/moored AIS targets** — nothing
  errors, but two protections silently never fire: the anchored-vessel
  suppression veto (ADR 0002 — the "never suppress a self-declared vessel"
  path holds its veto for the long anchored window only when it knows the
  vessel is anchored) and the velocity gate in `PolarVelocity.hpp` (an
  anchored ship's watch-circle drift would otherwise enter as velocity).
  If your middleware parses AIS itself, carry nav-status through to the hint.
- **Treating MMSI / ARPA target id as the fusion key** — external identifiers are
  *hints* (`AssociationHints`), never the primary key. The stable `track_id` is
  the identity (invariant 5). → §3.

### Nav-input guard — flag bad own-ship input at the edge

The three own-ship pitfalls above (COG-as-heading at low SOG, a stale nav feed,
and position/heading step-jumps) are checked by a **fact-free guard** at the
`OwnShipProvider` edge (backlog #18). Wire a sink and each `update(pose)` that
trips a sanity flag fires it:

```cpp
provider.setNavHealthSink(&nav_health_sink);   // INavHealthSink; nullable
// optionally: provider.setNavHealthSink(&sink, NavInputGuardConfig{...});
```

`INavHealthSink::onNavHealth(NavHealth)` reports four flags with their measured
quantities: `heading_unreliable_low_sog` (SOG below `heading_min_sog_mps`),
`stale_gap` (gap since the previous pose > `stale_after_s`), `position_jump`
(step speed > `max_position_speed_mps`), `heading_jump` (yaw rate >
`max_heading_rate_dps`). The `NavInputGuardConfig` defaults (0.5 m/s / 3 s /
50 m/s / 60°/s) are **generic plausibility bounds, not tuned per sensor** — the
calibrated thresholds (your heading source, GPS quality) are a deployment fact,
still on the shopping list. Two rules:

- **Degrade visibly, never silently.** The guard is *pure notification*: it does
  NOT rewrite the pose or reduce trust inside the core (validate at the edge,
  invariant #6 — the tracker keeps using the pose it is given). Its value is
  shrinking the *blind window* — turning a silent smear into an operator event.
- **Nullable = inert.** No sink ⇒ the guard does not run; behaviour is
  bit-identical to today.

---

## 10. Config reference (appendix)

Every consumer-facing `…Config` struct, its header, what it controls, and the two
or three fields most worth reviewing. (`core/benchmark/` sweep configs and the
`adapters/foxglove/` debug recorder are out of scope and omitted.)

| Struct | Header | Controls | Fields worth reviewing |
|---|---|---|---|
| `OwnShipNmeaAdapterConfig` | `adapters/own_ship/OwnShipNmeaAdapter.hpp` | Own-ship NMEA parsing: GPS noise, velocity source, heading sources | `uere_m{5.0}`, `gps_heading_talkers{}`, `prefer_rmc_velocity{true}` |
| `ArpaAdapterConfig` | `adapters/arpa/ArpaAdapter.hpp` | Radar TTM/TLL → measurements; TTM speed/course → one-shot birth prior + swap diagnostic (#20) | `heading_std_deg{0.0}` (floor over per-pose σ, #16), `position_std_m{50.0}`, `bearing_std_deg{1.0}`, `seed_birth_velocity_from_ttm{true}`, `swap_course_jump_deg{90.0}`, `swap_min_speed_mps{1.0}` |
| `EoIrAdapterConfig` | `adapters/eoir/EoIrAdapter.hpp` | EO/IR camera heading σ (floor over per-pose σ, #16) | `heading_std_deg{0.0}` |
| `RemoteTrackAdapterConfig` | `adapters/remote_track/RemoteTrackAdapter.hpp` | Shore/VTS remote track → pseudo-measurement (R-inflation, rate thinning, velocity opt-in) | `r_inflation_factor{3.0}`, `min_update_interval_s{2.0}`, `default_position_std_m{50.0}`, `accept_velocity{false}` |
| `AisAdapterConfig` | `adapters/ais/AisAdapter.hpp` | AIS SOG/COG → PositionVelocity2D content (#20), COG down-weighted at low SOG | `emit_velocity_from_sog_cog{true}`, `sog_velocity_min_mps{0.5}`, `sog_std_mps{0.5}`, `cog_std_deg{5.0}`, `velocity_iso_floor_mps{0.3}`, `position_std_high_accuracy_m{10.0}`, `position_std_standard_m{30.0}` |
| `NavInputGuardConfig` | `core/own_ship/NavInputGuard.hpp` | Fact-free own-ship nav-input sanity flags at the OwnShipProvider edge (#18); flags, never rewrites | `heading_min_sog_mps{0.5}`, `stale_after_s{3.0}`, `max_position_speed_mps{50.0}`, `max_heading_rate_dps{60.0}` |
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

**`PmbmTracker::Config` opt-in flags most likely to matter to a consumer** (all
on `core/pmbm/PmbmTracker.hpp`; a flag off = the wired model/port does nothing):

| Flag | Default | Effect (see) |
|---|---|---|
| `use_land_model` | `false` | enables the `setLandModel` shore birth-suppression prior (§7) |
| `land_birth_hard_gate` | `0.95` | clutter-prior above which a near-shore birth is dropped (§7) |
| `use_static_obstacle_model` | `false` | enables the `setStaticObstacleModel` / occupancy birth suppression (§7) |
| `static_obstacle_hard_gate` | `0.95` | suppression above which a birth in an obstacle footprint is dropped (§7) |
| `feed_clutter_map` | `false` | produces the per-scan `(position, 1−r)` feed for `setLiveOccupancyFeed` (§7) |
| `use_sensor_activity` | `false` | enables the `setSensorActivity` coverage/visibility miss model (§7) |
| `estimate_coverage_sector` | `false` | self-estimates the swept sector for occupancy-grid decay (§7) |
| `use_pda_soft_detected_branch` | `false` | soft (β-weighted) detected-branch update instead of hard commit (§8) |
| `pda_pool_excludes_land` | `false` | drops shore/structure returns from the PDA soft pool (§8) |
| `pda_pool_land_clutter_gate` | `0.5` | land clutter-prior above which a non-winner is excluded from the pool (§8) |

Non-`Config` tuning structs a consumer also touches: `DatumRecenterPolicy`
(`core/own_ship/OwnShipProvider.hpp`, §2), `StaticObstacleParams`
(`core/static/StaticObstacleModel.hpp`, §7), `CoastlinePriorParams`
(`core/land/CoastlineGeometry.hpp`, §7), `LiveOccupancyParams` (incl. its nested
`ShadowGuardParams shadow_guard` LOS/shadow-guard knobs,
`core/static/ShadowMask.hpp`) (`core/static/LiveOccupancyModel.hpp`, §7),
`BearingWedgeParams`
(`core/static/BearingWedgeModel.hpp`, §7),
`DeclaredSensorActivity::ChannelProfile`
(`core/sensor_activity/DeclaredSensorActivity.hpp`, §7).
