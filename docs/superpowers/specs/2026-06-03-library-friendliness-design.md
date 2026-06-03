# Library-Friendliness Pass — Design

**Date:** 2026-06-03
**Status:** Approved, ready for plan

## 1. Motivation

The project ships with NMEA-parsing adapters that obscure a simpler truth: the tracker's actual contract is `Measurement` (a parsed-and-normalised struct) and `OwnShipPose` (likewise). A library consumer with their own sensor pipeline does not need NMEA parsing — they construct these structs directly and call `tracker.process(...)` / `provider.update(...)`.

Today this works but is not documented or ergonomic. Three frictions:

1. **NMEA-coupled CMake.** `navtracker_core` includes the adapter sources, so a library consumer linking core gets NMEA parsing whether they want it or not.
2. **No canonical library example.** Tests serve as composition examples but they're test-shaped (multi-seed loops, ad-hoc fixtures). A library user has to reverse-engineer the canonical use.
3. **Bare construction is verbose.** Building a `Measurement` from a range/bearing reading requires the user to either reimplement what `ArpaAdapter::ingest` does internally (heading combination, ENU projection, covariance composition) or import the projection helper and wire it themselves. Same problem for missing covariances — `Measurement.covariance` is `Eigen::MatrixXd` with no default; if a sensor doesn't report uncertainty, the user must invent a value.

This spec adds the small set of helpers, examples, and CMake structure that make the library use case a first-class path while keeping every existing test byte-identical.

## 2. Scope

In scope:
- `SensorDefaults` struct with pessimistic per-`(SensorKind, MeasurementModel)` defaults, plus `applyDefaultsIfEmpty(Measurement&, const SensorDefaults&)` helper.
- New `Measurement` field `bool covariance_is_default{false}` (diagnostic only; default `false` preserves byte compat).
- `MeasurementBuilders.{hpp,cpp}` exposing three constructors (relative-bearing, true-bearing, ENU-position) returning Position2D measurements with full covariance composition through the existing `projectRangeBearingToEnu`.
- CMake split: `navtracker_core` (core + ports + helpers, no I/O) vs. `navtracker_nmea` (NMEA-parsing adapters) vs. `navtracker_sim` (existing sim emitters). `navtracker_tests` links all three.
- `app/example.cpp` showing the canonical end-to-end library use case.
- `CLAUDE.md` "Library use" section + cross-references.

Out of scope (explicitly deferred):
- **RMC parsing → own-ship velocity**. Slots in cleanly after this pass.
- **First-class `RangeBearing2D` documentation.** Decision: keep Position2D as the recommended path; advanced users with long-range geometry can still construct RB2D measurements directly (it's already in the type system), but it's not the documented contract.
- **Type-safe bearing units (`Radians`/`Degrees` newtypes).** Decision: radians throughout, documented; conversions stay at the call site.
- **Header-only or installable-package distribution.** Stays a CMake project. Library consumers add it as a submodule or fetch-content; this spec doesn't change distribution mechanics.
- **C API / language bindings.** Future work; not in scope.

## 3. Architecture

```
                            ┌──────────────────────────────────────┐
   YOUR application         │ navtracker_core (library)            │
                            │                                      │
   parsed AIS / radar /     │   core/   ports/                     │
   GPS data                 │   core/types/SensorDefaults          │
        │                   │   core/types/MeasurementBuilders     │
        │ build Measurement │   core/pipeline/Tracker              │
        ▼                   │   core/tracking/TrackManager         │
   makeMeasurementFromXxx() │   adapters/own_ship/OwnShipProvider  │
   applyDefaultsIfEmpty()   │                                      │
        │                   │   No I/O; pure domain + data types.  │
        ▼                   └──────────────────────────────────────┘
   tracker.process(m)
   provider.update(pose)
        │
        ▼
   Drain mgr.tracks() → your sink
```

`navtracker_nmea` (optional target) carries `OwnShipNmeaAdapter`, `AisAdapter`, `ArpaAdapter`, `EoIrAdapter`. Library consumers who don't need NMEA parsing simply don't link it.

`navtracker_sim` (optional target) carries the existing sim emitters. Library consumers don't link it unless they want simulation-driven testing.

No new ports. No new abstract interfaces. The contract is concrete types: `Measurement`, `OwnShipPose`, and the static `tracker.process(...)` / `provider.update(...)` methods.

## 4. `SensorDefaults` and the diagnostic flag

### 4.1 Type

```cpp
// core/types/SensorDefaults.hpp
struct PerSensorCov {
  double sigma_pos_m{0.0};         // Position2D
  double sigma_range_m{0.0};       // RangeBearing2D
  double sigma_bearing_rad{0.0};   // RangeBearing2D, BearingOnly2D
};

struct SensorDefaults {
  PerSensorCov ais_position;
  PerSensorCov arpa_tll_position;
  PerSensorCov arpa_ttm_range_bearing;
  PerSensorCov eoir_range_bearing;
  PerSensorCov eoir_bearing_only;

  // Produce a sized covariance for (sensor, model). Returns empty
  // matrix when the lookup misses (unknown sensor/model combination).
  Eigen::MatrixXd covarianceFor(SensorKind sensor,
                                MeasurementModel model) const;
};

// Pessimistic, literature-based defaults. Operators with real specs
// override the relevant fields after constructing this.
SensorDefaults pessimisticSensorDefaults();
```

Default values:

| Sensor | Model | Default σ |
|---|---|---|
| AIS | Position2D | σ_pos = 30 m |
| ARPA TLL | Position2D | σ_pos = 50 m |
| ARPA TTM | RangeBearing2D | σ_r = 75 m, σ_β = 1.5° (in rad) |
| EO/IR | RangeBearing2D | σ_r = 50 m, σ_β = 1.0° |
| EO/IR | BearingOnly2D | σ_β = 1.5° |

Pessimistic vs. the typical sensor spec — better to under-trust than over-trust. Operators override.

### 4.2 Helper

```cpp
// core/types/SensorDefaults.hpp
void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d);
```

If `m.covariance.size() == 0`, fill it from `d.covarianceFor(m.sensor, m.model)` and set `m.covariance_is_default = true`. No-op when `m.covariance.size() > 0`.

### 4.3 Diagnostic flag

```cpp
// In Measurement struct (additive):
bool covariance_is_default{false};
```

Default `false` so existing constructors and tests are byte-compatible. Downstream sinks may inspect it for logging or "low-confidence" UI flags. Tracker behavior is identical regardless of the flag's value — it's purely diagnostic.

## 5. `MeasurementBuilders`

### 5.1 Three constructors

```cpp
// core/types/MeasurementBuilders.hpp
namespace navtracker {

// Range + RELATIVE bearing (the radar / EO-IR / sonar common case).
// Internally adds own-ship heading and projects to ENU Position2D via
// projectRangeBearingToEnu. Composes sigma_heading and sigma_gps from
// the pose; the resulting covariance reflects the full GPS-and-heading
// budget the rest of the system uses. Returns Position2D measurement
// with sensor_position_enu set to own-ship's ENU position.
//
// All angles in radians. Range in meters.
Measurement makeMeasurementFromRelativeBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double relative_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipPose& own_ship_pose,
    const geo::Datum& datum,
    AssociationHints hints = {});

// Range + TRUE bearing (already-projected, world-frame). Useful when
// the sensor pipeline pre-computes true bearings outside this library.
// Otherwise identical to the relative-bearing variant.
Measurement makeMeasurementFromTrueBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double true_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipPose& own_ship_pose,
    const geo::Datum& datum,
    AssociationHints hints = {});

// Absolute ENU position (AIS-style). Mostly trivial — fills value and
// covariance, sets time/sensor/source/model/hints. Exposes a uniform
// construction surface so SensorDefaults composition is consistent.
Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    Eigen::Vector2d enu_xy,
    Eigen::Matrix2d covariance,
    AssociationHints hints = {});

}  // namespace navtracker
```

### 5.2 Implementation notes

All three builders set:
- `m.time = t`
- `m.sensor = sensor`
- `m.source_id = std::move(source_id)`
- `m.model = MeasurementModel::Position2D`
- `m.hints = hints`
- `m.sensor_position_enu = own-ship's ENU position` (from datum conversion)
- `m.sensor_position_std_m = own_ship_pose.position_std_m`
- `m.covariance_is_default = false`

The relative-bearing builder also:
- `bearing_true_rad = relative_bearing_rad + own_ship_pose.heading_true_deg * kDeg2Rad`
- `cov, value = projectRangeBearingToEnu(range_m, bearing_true_rad, range_std_m, bearing_std_rad, sigma_heading_from_pose, sigma_gps_from_pose, own_xy)`

The true-bearing builder is identical minus the heading addition.

The ENU-position builder skips projection entirely — it's the AIS case.

### 5.3 Interaction with SensorDefaults

Builders take explicit `range_std_m`, `bearing_std_rad`, or `covariance` parameters. If the user has no uncertainty info, they pass `0.0` (or empty 2×2) and call `applyDefaultsIfEmpty(m, defaults)` immediately after to fill in. The two helpers compose; one doesn't subsume the other. Rationale: the builder is the construction path, the defaults are the missing-data fallback — keeping them orthogonal lets users mix freely.

For the ENU-position builder, passing an empty 2×2 covariance produces a measurement that `applyDefaultsIfEmpty` will fill from defaults. For the bearing builders, passing 0 in the std params produces a covariance that's still meaningful (the GPS and heading contributions are non-zero), but doesn't represent the sensor's own noise. Callers should typically supply the sensor noise or call defaults afterward.

## 6. CMake split

### 6.1 Three targets

```cmake
# CMakeLists.txt — new structure

# Core: no I/O, no parsing, no sim. Pure domain + data types.
add_library(navtracker_core
  core/geo/Wgs84.cpp
  core/geo/Datum.cpp
  core/estimation/...
  core/collision/...
  core/association/...
  core/tracking/...
  core/pipeline/...
  core/bias/...
  core/own_ship/UereEstimator.cpp
  core/types/SensorDefaults.cpp        # NEW
  core/types/MeasurementBuilders.cpp   # NEW
  core/scenario/...
  adapters/own_ship/OwnShipProvider.cpp   # passive data store, stays in core
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)

# NMEA: parses NMEA strings into Measurements. Optional for library users.
add_library(navtracker_nmea
  adapters/util/Nmea.cpp
  adapters/util/Projection.cpp     # NMEA-adapter-shared helpers; can stay here
  adapters/ais/AisAdapter.cpp
  adapters/arpa/ArpaAdapter.cpp
  adapters/eoir/EoIrAdapter.cpp
  adapters/own_ship/OwnShipNmeaAdapter.cpp
)
target_link_libraries(navtracker_nmea PUBLIC navtracker_core)

# Sim: synthetic measurement generators. Used by tests.
add_library(navtracker_sim
  sim/TruthTrajectory.cpp
  sim/NmeaEncode.cpp
  sim/OwnShipEmitter.cpp
  sim/AisEmitter.cpp
  sim/ArpaEmitter.cpp
  sim/EoIrEmitter.cpp
  sim/SimulatedSensorBus.cpp
)
target_link_libraries(navtracker_sim PUBLIC navtracker_nmea)

# Tests: link all three.
add_executable(navtracker_tests
  ...all existing test sources...
)
target_link_libraries(navtracker_tests PRIVATE
  navtracker_core navtracker_nmea navtracker_sim GTest::gtest_main)
```

### 6.2 `Projection.cpp` placement

`adapters/util/Projection.cpp` is currently used by both the NMEA adapters and the new `MeasurementBuilders`. Two choices:

- **(A) Keep in `adapters/util/` but make it a `navtracker_core` source.** Slightly violates the "adapters depend on core, not the reverse" rule but the file is pure math.
- **(B) Move to `core/types/` or `core/projection/`.** Architecturally cleaner.

Chosen: **(B)** — move `Projection.{hpp,cpp}` to `core/projection/`. Updates are mechanical (include-path search-and-replace). Adapters that include the new path keep working.

### 6.3 Backward compat

All existing tests pass byte-identical because `navtracker_tests` links all three targets. No code moves source files; only `add_library` boundaries change. Builds that currently link `navtracker_core` get the same behavior because the test executable already includes everything.

A library consumer who *wants* the NMEA path links `navtracker_nmea` (which transitively pulls `navtracker_core`).

## 7. `app/example.cpp`

```cpp
// app/example.cpp — Library use example for navtracker.
//
// Demonstrates the canonical path for plugging navtracker into your
// stack: build the tracker, build the own-ship provider, feed parsed
// Measurements through tracker.process(), and drain track snapshots
// from the TrackManager whenever your sink wants them.
//
// This file is for documentation; it builds but the main() does not
// run any external I/O.

#include <iostream>
#include <memory>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/MeasurementBuilders.hpp"
#include "core/types/SensorDefaults.hpp"
#include "core/types/Track.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/geo/Datum.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"

int main() {
  using namespace navtracker;
  using navtracker::geo::Datum;

  // ---- Composition root (build once at startup) -----------------------

  Datum datum({53.5, 8.0, 0.0});                // your operating datum
  OwnShipProvider provider;
  auto motion = std::make_shared<ConstantVelocity2D>(/*q=*/0.1);
  EkfEstimator ekf(motion, /*init_pos_std_m=*/5.0);
  GnnAssociator gnn(/*chi2_gate=*/20.0);
  TrackManager mgr(/*confirm_hits=*/2, /*delete_misses=*/3);
  Tracker tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/30.0);

  const SensorDefaults defaults = pessimisticSensorDefaults();

  // ---- Each time YOUR pipeline emits a parsed AIS report --------------
  //
  // AIS gives the target's absolute lat/lon directly. Convert to ENU
  // once at the boundary, then construct the Measurement.

  {
    const double lat = 53.55, lon = 8.05;
    const auto enu = datum.toEnu({lat, lon, 0.0});

    Measurement m = makeMeasurementFromEnuPosition(
        SensorKind::Ais, "my_ais_feed",
        Timestamp::fromSeconds(123.0),
        Eigen::Vector2d(enu.x(), enu.y()),
        Eigen::Matrix2d::Zero(),  // empty -> defaults will fill in
        AssociationHints{/*mmsi=*/200000001u, std::nullopt});
    applyDefaultsIfEmpty(m, defaults);
    tracker.process(m);
  }

  // ---- Each time YOUR pipeline emits a parsed radar return ------------
  //
  // Radar reports (range, relative_bearing). The library adds own-ship
  // heading and projects to ENU, including GPS and heading covariance.

  {
    const double range_m = 1500.0;
    const double rel_bearing_rad = 0.5;   // 28.6° to port of bow
    const double range_std_m = 50.0;
    const double bearing_std_rad = 1.0 * 3.14159265358979 / 180.0;

    const auto own_opt = provider.latest();
    if (own_opt) {
      Measurement m = makeMeasurementFromRelativeBearing(
          SensorKind::ArpaTtm, "my_radar",
          Timestamp::fromSeconds(123.0),
          range_m, rel_bearing_rad,
          range_std_m, bearing_std_rad,
          *own_opt, datum);
      // If your radar didn't report std values, leave them 0 and call:
      // applyDefaultsIfEmpty(m, defaults);
      tracker.process(m);
    }
  }

  // ---- Each time YOUR pipeline gets an own-ship pose ------------------

  {
    OwnShipPose pose;
    pose.time = Timestamp::fromSeconds(123.0);
    pose.lat_deg = 53.500;
    pose.lon_deg = 8.000;
    pose.heading_true_deg = 45.0;
    pose.position_std_m = 5.0;    // from your GPS receiver, or 0 + defaults
    provider.update(pose);
  }

  // ---- Drain the current track snapshot whenever your sink wants it ---

  for (const Track& t : mgr.tracks()) {
    if (t.status != TrackStatus::Confirmed) continue;
    std::cout << "Track id=" << t.id.value
              << " pos=" << t.state(0) << ", " << t.state(1)
              << " status=" << static_cast<int>(t.status) << "\n";
  }

  return 0;
}
```

The example builds as part of the test target (linked into `navtracker_tests` or as a stand-alone `navtracker_example` executable) so it stays compileable.

## 8. Documentation pass

### 8.1 `CLAUDE.md` addition

Append a new section after "Module layout":

```markdown
## Library use

navtracker is designed as a hexagonal-architecture library. The
contract a consumer plugs against is two concrete types and two
methods:

- `core/types/Measurement.hpp` — what you feed in (per sensor reading).
- `adapters/own_ship/OwnShipProvider.hpp` — `OwnShipPose` (per GPS fix).
- `Tracker::process(Measurement)` / `OwnShipProvider::update(pose)` —
  the entry points.

You construct Measurements directly from your parsed sensor data. The
NMEA adapters in `adapters/` are one optional implementation for
consumers whose input is NMEA strings — they're not the canonical
path. Skip them if you have your own pipeline.

Common patterns:

- Range/bearing sensor (radar, EO-IR, sonar):
  `makeMeasurementFromRelativeBearing(...)` — adds heading, projects
  to ENU, composes GPS and heading covariance.
- Absolute-position sensor (AIS, GPS-equipped target):
  `makeMeasurementFromEnuPosition(...)` — direct ENU input.
- No uncertainty info from the sensor: leave covariance empty,
  call `applyDefaultsIfEmpty(m, pessimisticSensorDefaults())`.

See `app/example.cpp` for a complete end-to-end example.

### CMake targets for library consumers

- `navtracker_core` — pure domain + ports + helpers. No I/O. Link
  this alone if you supply pre-parsed Measurements.
- `navtracker_nmea` — NMEA-format adapters. Link this in addition
  when your input is NMEA strings.
- `navtracker_sim` — synthetic measurement emitters. Tests only.
```

### 8.2 `README.md`

Project doesn't have one today. This pass adds a minimal README that points to `CLAUDE.md` and the example. ~20 lines.

## 9. Assumptions

- Library consumers run navtracker single-threaded per tracker instance. Multi-threaded use (one tracker, many parser threads) is not supported in this pass; the existing tracker isn't thread-safe and we're not changing that.
- Library consumers' coordinate frame is ENU about a configured datum, matching the existing architectural invariant. We don't expose a way to use a different frame.
- Pessimistic defaults are acceptable as the missing-covariance fallback. Operators with real sensor specs override.
- Time is `Timestamp` (nanoseconds since some epoch chosen by the consumer). Defaults to `Timestamp::fromSeconds(...)`. We don't add an epoch concept.

## 10. Rationale

**Why fold defaults and builders together?** Both serve the same goal — let a library consumer plug parsed data in without reinventing the math. Defaults handle missing covariance; builders handle the construction surface. Splitting them into two specs would create artificial scope boundaries and double the documentation overhead.

**Why pessimistic defaults?** The library can't know whether the operator under-budgeting σ is the worse failure mode (over-confident filter, spurious associations) or over-budgeting σ (lazy filter, sluggish tracking). Pessimistic is the safer default for a system whose downstream consumer is collision-avoidance: better to alarm slightly more than miss a real risk.

**Why Position2D as the recommended contract?** One concept, one builder pattern, one composition with covariance. RangeBearing2D preserves more accuracy at long range but adds complexity that most consumers don't need; advanced users can still construct RB2D measurements directly. Keeps the library docs focused on the common case.

**Why radians and not degrees?** Matches the rest of the C++ math API (`std::atan2`, Eigen, Eigen rotations). Library consumers thinking in degrees do `bearing_rad = bearing_deg * M_PI / 180.0` at the call site — one line, explicit, unit-safe by convention.

**Why split CMake into three targets and not just two (core, adapters)?** Sim is conceptually separate from real-world I/O — it's truth-driven generation, not parsing. A library consumer with their own simulation infrastructure doesn't want our sim either. Three targets cleanly separates "what's the contract", "what parses NMEA", and "what generates synthetic data".

**Why is `OwnShipProvider` in core, not nmea?** It's a passive data store, not a parser. A library consumer with no NMEA needs to push poses through it; if it lived in `nmea`, they'd have to link the parser library just to get a `provider.update(pose)` method.

**Why is `Projection` moved to `core/`?** It's pure math (Jacobian-based ENU projection) used by `MeasurementBuilders`. Keeping it under `adapters/` would force `core` to depend on `adapters` — exactly the inversion we're trying to avoid.

**Why `app/example.cpp` and not `examples/`?** CLAUDE.md already names `app/` as the composition-root home. The example becomes the seed of that directory rather than a parallel structure.

## 11. Test plan

### Unit (new test files)

1. **`tests/types/test_sensor_defaults.cpp`**
   - `PessimisticFactoryReturnsExpectedValues`: confirm σ values match spec §4.1.
   - `CovarianceForReturnsCorrectShape`: AIS+Position2D → 2×2 with σ²_pos on diagonal; ARPA TTM+RangeBearing2D → 2×2 with σ²_r, σ²_β on diagonal.
   - `ApplyDefaultsFillsEmptyAndFlagsIt`: empty covariance → filled, `covariance_is_default == true`.
   - `ApplyDefaultsNoOpWhenSet`: pre-filled covariance → unchanged, flag stays false.
   - `ApplyDefaultsUnknownSensorLeavesEmpty`: unknown (SensorKind, Model) combination → covariance stays empty, flag stays false.

2. **`tests/types/test_measurement_builders.cpp`**
   - `RelativeBearingProducesEnuConsistentWithDirectProjection`: build via helper, compare against direct `projectRangeBearingToEnu` with the same heading combination. Equal to 1e-9.
   - `RelativeBearingComposesGpsAndHeadingSigma`: pose with position_std_m=5 and sigma_heading from somewhere → output covariance ≥ baseline by `25·I + (range·σ_h)²·cross`.
   - `TrueBearingSkipsHeadingCombo`: same setup but skip heading add; output bearing direction matches input.
   - `EnuPositionPassesThroughCovariance`: explicit covariance argument → exact match in output.
   - `EnuPositionEmptyCovariancePlaysWithDefaults`: construct with empty cov, call applyDefaultsIfEmpty → result has populated cov and flag true.

3. **Existing test impact**
   - Add a `covariance_is_default{false}` field to `Measurement` — confirm no existing test that explicitly initializes a Measurement with `{}` is affected (default value matches).

### Integration (existing tests cover this implicitly)

- The `app/example.cpp` builds against `navtracker_core` only (or `+ nmea` for the AisAdapter line, depending on the example's exact composition). Confirms the link-only-what-you-need claim.
- The full test suite (273+ tests) links all three CMake targets and remains green.

### Eval-log

Append "Library-friendliness pass (2026-06-03)" — brief section noting the CMake split, the new helpers, and the example. No quantitative numbers to record; the value here is structural, not measured.

## 12. Files touched

### Created
- `core/types/SensorDefaults.hpp`, `core/types/SensorDefaults.cpp`
- `core/types/MeasurementBuilders.hpp`, `core/types/MeasurementBuilders.cpp`
- `app/example.cpp`
- `README.md` (~20 lines)
- `tests/types/test_sensor_defaults.cpp`
- `tests/types/test_measurement_builders.cpp`

### Moved
- `adapters/util/Projection.hpp`, `adapters/util/Projection.cpp` → `core/projection/Projection.{hpp,cpp}`. Include paths search-and-replaced across the codebase.

### Modified
- `core/types/Measurement.hpp` — add `bool covariance_is_default{false};` field.
- `CMakeLists.txt` — three internal libraries; test target links all three; `app/example.cpp` builds as a test-suite executable or stand-alone.
- `CLAUDE.md` — "Library use" section per §8.1.

## 13. Ways to improve / what to test next

1. **C API / language bindings.** A thin extern "C" layer over `tracker.process(...)` and the builders enables Python / Rust / Go bindings. Useful when the consumer's pipeline is in a non-C++ language.
2. **Type-safe units.** `Radians` / `Degrees` newtypes catch unit confusion at compile time. Adds new types to the surface; defer until we see a confusion bug.
3. **First-class RangeBearing2D path.** Document the trade-off and provide a `makeMeasurementFromRangeBearing(...)` variant that returns `MeasurementModel::RangeBearing2D` directly (no projection). For long-range tracking where the projection covariance approximation degrades.
4. **`OwnShipPose` builders.** Symmetrically with `MeasurementBuilders`, helpers like `makeOwnShipPoseFromGps(...)` that fill in derived fields (velocity from successive calls if appropriate). Slots in with the RMC work.
5. **Header-only / installable distribution.** CMake `install()` rules + version macros so consumers can `find_package(Navtracker)`. Tedious but unblocks easy adoption.
6. **Multi-tracker support.** Today `Tracker` and `TrackManager` are not thread-safe. A documented pattern for one-tracker-per-thread (with shared SensorDefaults / OwnShipProvider) would help library consumers with parallel sensor pipelines.

## 14. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Missing-covariance handling | `SensorDefaults` + `applyDefaultsIfEmpty` helper | Defaults are the sensible fallback; explicit opt-in to fill. |
| Defaults shape | Single library-wide struct | Simple, deterministic, easy to log. |
| Defaults values | Pessimistic vs. typical sensor spec | Better to under-trust than over-trust for collision-avoidance use. |
| Diagnostic flag | `Measurement.covariance_is_default` | Sinks can flag low-confidence tracks. |
| Measurement-construction API | Three helpers (relative-bearing, true-bearing, ENU) | Common patterns, one shape (Position2D) for the recommended contract. |
| Bearing units | Radians | Matches Eigen / std math API. |
| Recommended measurement model | Position2D | One concept, one builder; RB2D stays available for advanced use. |
| CMake structure | Three internal libraries (core / nmea / sim) | Clear separation of contract, parsing, simulation. |
| `Projection` location | Move to `core/projection/` | Pure math, no I/O; needed by both core and adapters. |
| `OwnShipProvider` location | Stays in core | Passive data store, not a parser. |
| Library example | `app/example.cpp` | Realises the directory CLAUDE.md already names as the composition root. |
