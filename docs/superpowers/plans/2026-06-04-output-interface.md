# Output Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a canonical drainable output shape — `TrackOutput` — built from three structs (position in lat/lon + cov in m² local NED, velocity in SOG/COG with σ, metadata bundle) and three pure-conversion helpers. Position in lat/lon degrees; covariance via Option A (rotated into target's local NED frame). Velocity in SOG/COG with polar-Jacobian σ derivation. Document the contract clearly.

**Architecture:** New `core/output/` directory (parallel to `core/bias/`, `core/projection/`). Lift `datumAxisRotation` from `core/tracking/DatumShift.hpp` to `core/geo/AxisRotation.hpp` so output and tracking share the rotation helper. No new ports. No tracker changes.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-04-output-interface-design.md`. Section references refer to that spec.

---

## Task 1: Lift `datumAxisRotation` to `core/geo/AxisRotation`

**Files:**
- Create: `core/geo/AxisRotation.hpp`, `core/geo/AxisRotation.cpp`
- Modify: `core/tracking/DatumShift.hpp`, `core/tracking/DatumShift.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §7: `datumAxisRotation` is pure geo math. Output needs it; tracking already uses it. Lift to the layer both depend on.

### Steps

- [ ] **Step 1: Create the lifted header**

Create `core/geo/AxisRotation.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"

namespace navtracker::geo {

/// 2x2 rotation matrix between the ENU axes of two datums.
/// Implements the convergence angle
///   gamma = delta_lon_rad * sin(mean_lat_rad)
/// At equator: gamma = 0. At 60°N over 1° of longitude: gamma ≈ 0.0151 rad.
/// Used by:
///   - core/tracking/DatumShift: shift tracks across a datum recenter.
///   - core/output/TrackOutput: rotate position covariance into the
///     target's local NED frame (Option A of the output-interface spec).
Eigen::Matrix2d datumAxisRotation(const Datum& old_datum,
                                  const Datum& new_datum);

}  // namespace navtracker::geo
```

- [ ] **Step 2: Move the implementation**

Create `core/geo/AxisRotation.cpp`:

```cpp
#include "core/geo/AxisRotation.hpp"

#include <cmath>

namespace navtracker::geo {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

Eigen::Matrix2d datumAxisRotation(const Datum& old_datum,
                                  const Datum& new_datum) {
  const auto& o = old_datum.origin();
  const auto& n = new_datum.origin();
  const double delta_lon_rad = (n.lon_deg - o.lon_deg) * kDeg2Rad;
  const double mean_lat_rad = 0.5 * (o.lat_deg + n.lat_deg) * kDeg2Rad;
  const double gamma = delta_lon_rad * std::sin(mean_lat_rad);
  const double c = std::cos(gamma), s = std::sin(gamma);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

}  // namespace navtracker::geo
```

- [ ] **Step 3: Update `DatumShift` to use the new location**

In `core/tracking/DatumShift.hpp`:
- Remove the `datumAxisRotation` declaration (it's now in `core/geo/AxisRotation.hpp`).
- Add `#include "core/geo/AxisRotation.hpp"`.

In `core/tracking/DatumShift.cpp`:
- Remove the local `datumAxisRotation` definition.
- Use `geo::datumAxisRotation(...)` at the call site.

- [ ] **Step 4: Update CMake**

Add `core/geo/AxisRotation.cpp` to the `navtracker_core` source list in `CMakeLists.txt` (near the existing `core/geo/Datum.cpp` and `core/geo/Wgs84.cpp`).

- [ ] **Step 5: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```

Expected: 332/332 still green. Pure-mechanical move; the lifted function has identical math.

If the existing `DatumShiftTest.RotatesVelocityByConvergenceAngle` test asserts against `datumAxisRotation(...)` directly, update its include if needed.

- [ ] **Step 6: Commit**

```
git add core/geo/AxisRotation.hpp core/geo/AxisRotation.cpp \
        core/tracking/DatumShift.hpp core/tracking/DatumShift.cpp \
        CMakeLists.txt
git commit -m "geo: lift datumAxisRotation from tracking/ to geo/AxisRotation"
```

---

## Task 2: `core/output/TrackOutput` — structs and helpers

**Files:**
- Create: `core/output/TrackOutput.hpp`, `core/output/TrackOutput.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §3, §4, §6: the three structs and three conversion helpers. Full Doxygen comments on every field and function — the operator-facing surface needs to be self-explanatory.

### Steps

- [ ] **Step 1: Write the header**

Create `core/output/TrackOutput.hpp` per spec §6 verbatim. Include full Doxygen comments on each struct, field, and function per the docstrings in spec §6.

- [ ] **Step 2: Write `toGeodeticWithCov`**

In `core/output/TrackOutput.cpp`:

```cpp
#include "core/output/TrackOutput.hpp"

#include <cmath>

#include "core/geo/AxisRotation.hpp"

namespace navtracker {

PositionGeodeticWithCov toGeodeticWithCov(
    const Eigen::Vector2d& enu_xy,
    const Eigen::Matrix2d& cov_enu_m2,
    const geo::Datum& datum) {
  const auto geo_target = datum.toGeodetic(
      Eigen::Vector3d(enu_xy.x(), enu_xy.y(), 0.0));
  const geo::Datum target_datum(
      geo::Geodetic{geo_target.lat_deg, geo_target.lon_deg, 0.0});
  const Eigen::Matrix2d R = geo::datumAxisRotation(datum, target_datum);
  PositionGeodeticWithCov out;
  out.lat_deg = geo_target.lat_deg;
  out.lon_deg = geo_target.lon_deg;
  out.position_covariance_m2 = R * cov_enu_m2 * R.transpose();
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 3: Write `toVelocityOutput`**

Append to `core/output/TrackOutput.cpp`:

```cpp
namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kSogEpsilon = 0.01;   // m/s — below this, COG is undefined
}

VelocityGeodeticWithSigma toVelocityOutput(
    const Eigen::Vector2d& v_enu,
    const Eigen::Matrix2d& v_cov_m2_per_s2,
    bool is_valid) {
  VelocityGeodeticWithSigma out;
  out.is_valid = is_valid;
  if (!is_valid) return out;
  const double sog = v_enu.norm();
  const double cog_rad = std::atan2(v_enu.x(), v_enu.y());
  double cog_deg = cog_rad * kRad2Deg;
  if (cog_deg < 0.0) cog_deg += 360.0;
  if (cog_deg >= 360.0) cog_deg -= 360.0;
  out.sog_m_per_s = sog;
  out.cog_deg = (sog < kSogEpsilon) ? 0.0 : cog_deg;

  // Polar Jacobian on (v_east, v_north) -> (sog, cog).
  if (sog < kSogEpsilon) {
    // Direction undefined; σ_sog still well-defined.
    Eigen::RowVector2d j_sog;
    // Limit: as sog -> 0, ∂sog/∂v_east, ∂sog/∂v_north depend on direction;
    // safest practical choice is the isotropic trace bound.
    const double sigma_sog2 = 0.5 * v_cov_m2_per_s2.trace();
    out.sigma_sog_m_per_s = std::sqrt(std::max(sigma_sog2, 0.0));
    out.sigma_cog_deg = 0.0;
    return out;
  }
  const double s = std::sin(cog_rad);
  const double c = std::cos(cog_rad);
  Eigen::Matrix2d J;
  J << s,         c,
       c / sog,  -s / sog;
  const Eigen::Matrix2d cov_polar = J * v_cov_m2_per_s2 * J.transpose();
  out.sigma_sog_m_per_s = std::sqrt(std::max(cov_polar(0, 0), 0.0));
  out.sigma_cog_deg     = std::sqrt(std::max(cov_polar(1, 1), 0.0)) * kRad2Deg;
  return out;
}
```

- [ ] **Step 4: Write `toTrackOutput`**

```cpp
TrackOutput toTrackOutput(const Track& track,
                          const geo::Datum& datum) {
  TrackOutput out;
  out.id = track.id;
  out.status = track.status;
  out.last_update = track.last_update;
  out.attributes = track.attributes;
  out.contributing_sources = track.contributing_sources;
  out.covariance_is_default = false;  // v1: forwarding from recent_contributions deferred

  Eigen::Vector2d pos_enu = Eigen::Vector2d::Zero();
  Eigen::Matrix2d pos_cov = Eigen::Matrix2d::Zero();
  if (track.state.size() >= 2) {
    pos_enu = Eigen::Vector2d(track.state(0), track.state(1));
  }
  if (track.covariance.rows() >= 2 && track.covariance.cols() >= 2) {
    pos_cov = track.covariance.topLeftCorner<2, 2>();
  }
  out.position = toGeodeticWithCov(pos_enu, pos_cov, datum);

  if (track.state.size() >= 4 && track.covariance.rows() >= 4 &&
      track.covariance.cols() >= 4) {
    const Eigen::Vector2d v_enu(track.state(2), track.state(3));
    const Eigen::Matrix2d v_cov = track.covariance.block<2, 2>(2, 2);
    const bool v_valid = v_cov.trace() > 0.0;
    out.velocity = toVelocityOutput(v_enu, v_cov, v_valid);
  } else {
    out.velocity = VelocityGeodeticWithSigma{};  // default: is_valid=false, zeros
  }
  return out;
}
```

- [ ] **Step 5: Wire into CMake**

Add `core/output/TrackOutput.cpp` to the `navtracker_core` source list.

- [ ] **Step 6: Build (no tests yet)**

```
cmake --build build --target navtracker_tests
```

Expected: compile success. Tests in Task 3.

- [ ] **Step 7: Commit**

```
git add core/output/TrackOutput.hpp core/output/TrackOutput.cpp CMakeLists.txt
git commit -m "output: TrackOutput + toGeodeticWithCov/toVelocityOutput/toTrackOutput"
```

---

## Task 3: Unit tests

**Files:**
- Create: `tests/output/test_track_output.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §11.1 plus the specific math from §4: cover position round-trip, covariance rotation, SOG/COG conversion, σ derivation, validity semantics, edge cases (zero-velocity, near-datum, distant-from-datum within recenter horizon).

### Steps

- [ ] **Step 1: Write tests**

Create `tests/output/test_track_output.cpp`. Test list:

1. **`GeodeticPositionRoundTripsAtDatum`** — track at (0, 0) in ENU; `toGeodeticWithCov` returns datum's lat/lon. Covariance unchanged (R = identity at datum).
2. **`GeodeticPositionRoundTripsOffDatum`** — track at known ENU; `toGeodeticWithCov` returns the right lat/lon; back through `datum.toEnu` should round-trip.
3. **`CovarianceRotatedAtHighLatitude`** — track 30 km east at 60°N; covariance σ_east=10, σ_north=1 in datum-ENU. After rotation, σ_east' and σ_north' differ from raw by the expected angle.
4. **`SogCogFromEastVelocity`** — `v_enu = (10, 0)` → sog=10, cog=90°. σ from identity cov → σ_sog=1, σ_cog from Jacobian.
5. **`SogCogFromNorthVelocity`** — `v_enu = (0, 10)` → sog=10, cog=0°.
6. **`StationaryTrackHandlesSingularity`** — `v_enu = (0, 0)`, cov = identity → cog=0, σ_cog=0, sog=0, σ_sog from isotropic trace bound.
7. **`InvalidVelocityZeroesAllNumeric`** — `is_valid=false` → sog/cog/σ all 0; is_valid stays false.
8. **`TrackOutputFor4DTrack`** — full integration: 4D track with state and covariance → returns valid position + velocity, attributes propagated.
9. **`TrackOutputFor2DTrackHasInvalidVelocity`** — 2D-state track → velocity.is_valid = false.
10. **`TrackOutputForZeroVelocityCovarianceHasInvalidVelocity`** — 4D state but velocity covariance trace = 0 → velocity.is_valid = false.

For tests involving Track construction, use the helper pattern from `tests/collision/test_cpa_uncertainty.cpp` (the `makeTrack` lambda).

- [ ] **Step 2: Wire test into CMake**

Add `tests/output/test_track_output.cpp` to the `navtracker_tests` source list.

- [ ] **Step 3: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R TrackOutputTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 10 new PASS; full suite 342/342 green.

- [ ] **Step 4: Commit**

```
git add tests/output/test_track_output.cpp CMakeLists.txt
git commit -m "test: TrackOutput conversion helpers (round-trip, rotation, SOG/COG, validity)"
```

---

## Task 4: `docs/output-contract.md`

**Files:**
- Create: `docs/output-contract.md`

### Why

Per spec §10: focused operator-facing doc covering units, semantics, validity, worked example.

### Steps

- [ ] **Step 1: Write the doc**

Create `docs/output-contract.md` with the 6 sections from spec §10. Length: ~120 lines. Tone: matches the rest of `docs/` — terse, technically precise, with concrete examples.

Sketch outline:

```markdown
# Output Contract

How to interpret `TrackOutput` and the related output structs.

## Position

- `lat_deg`, `lon_deg`: WGS84 geodetic coordinates, in degrees. Range
  lat ∈ [-90, 90], lon ∈ (-180, 180].
- `position_covariance_m2`: 2×2 matrix in m². Frame: target's local
  NED. Row/column 0 = north, row/column 1 = east. The covariance is
  rotated from datum-ENU into target-NED via the convergence angle
  (always small for tracks within auto-datum's 30 km recenter horizon
  — typically < 0.5° rotation).
- Read σ_north = sqrt(position_covariance_m2(0,0)) and similarly for
  σ_east. These are physical meters at the target's location.

## Velocity

- `sog_m_per_s`: speed over ground, m/s, ≥ 0.
- `cog_deg`: course over ground, degrees true (clockwise from north),
  ∈ [0, 360).
- `sigma_sog_m_per_s`, `sigma_cog_deg`: 1σ uncertainty derived from
  the velocity covariance via the polar Jacobian.
- `is_valid`: true when the track carries meaningful velocity info.
  Set false when the underlying track has 2D state (position only),
  or when the velocity covariance is zero (e.g., own-ship pose
  before RMC arrived).

### Stationary tracks

When sog < 0.01 m/s, the COG direction is not meaningful and both
cog_deg and sigma_cog_deg are set to 0. is_valid stays true (the
"track is confirmed stationary" signal vs. "no velocity info at
all" signal).

## Track metadata

- `id`: stable, monotonic integer. Never reused after a track is
  deleted. Two reads on the same cycle return the same id.
- `status`: Tentative (initial), Confirmed (M-of-N satisfied),
  Deleted (will be removed from drain on the next cycle).
- `last_update`: timestamp of the most recent measurement that
  contributed to this track.
- `attributes`: optional fields (mmsi, name, vessel_type, length_m,
  beam_m) sourced from AIS reports or other identity-carrying
  sensors.
- `contributing_sources`: vector of source_id strings ("ais",
  "arpa", "eoir") that have ever contributed measurements.

## Diagnostic flag

- `covariance_is_default`: true when any contributing measurement
  populated its covariance from SensorDefaults rather than a real
  sensor uncertainty. Downstream sinks may flag low-confidence
  tracks; the tracker behaves identically regardless. v1 forwarding
  from recent_contributions is incomplete; default value is false.

## Worked example

(track at 53.6°N, 8.2°E, moving east at 5 m/s with 5 m position σ and
0.5 m/s velocity σ; full dump of TrackOutput fields and how each
should display in a UI.)

## Singularities at a glance

- Stationary track: sog ≈ 0 → cog = 0, σ_cog = 0; is_valid based on
  caller's signal.
- Track at datum: covariance rotation is identity; magnitudes
  unchanged.
- Distant track (< 30 km): rotation < 0.5°; small effect on
  off-diagonal terms.
```

Fill in the worked example with realistic numbers; tone-match other docs.

- [ ] **Step 2: Commit**

```
git add docs/output-contract.md
git commit -m "docs: output contract — units, semantics, validity, worked example"
```

---

## Task 5: CLAUDE.md + `app/example.cpp` + README

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`
- Modify: `app/example.cpp`

### Why

Per spec §10: connect the dots. CLAUDE.md points operators to the contract; README has the one-line. Example demonstrates the drain pattern.

### Steps

- [ ] **Step 1: Update `app/example.cpp`**

Add a final block to the existing example showing the drain:

```cpp
#include "core/output/TrackOutput.hpp"

// ... existing example code ...

// ---- Drain track snapshot in operator-friendly form ----
for (const Track& t : mgr.tracks()) {
  if (t.status != TrackStatus::Confirmed) continue;
  const TrackOutput out = toTrackOutput(t, provider.datum());
  // out.position.lat_deg / lon_deg in WGS84 degrees
  // out.position.position_covariance_m2 in m^2 (north, east)
  // out.velocity.sog_m_per_s, .cog_deg, .sigma_*, .is_valid
  // out.id, out.status, out.attributes, out.contributing_sources
  std::cout << "Track id=" << out.id.value
            << "  lat=" << out.position.lat_deg
            << "  lon=" << out.position.lon_deg;
  if (out.velocity.is_valid) {
    std::cout << "  sog=" << out.velocity.sog_m_per_s
              << "  cog=" << out.velocity.cog_deg;
  }
  std::cout << "\n";
}
```

The example now demonstrates: composition root → builders → tracker.process → poseAtOrBefore → drain via toTrackOutput. The full library use case in one file.

- [ ] **Step 2: Update CLAUDE.md**

Append a new subsection to the existing "Library use" section:

```markdown
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
```

- [ ] **Step 3: Update `README.md`**

Add one line to the "Library use" section pointing to `docs/output-contract.md`.

- [ ] **Step 4: Build + run**

```
cmake --build build --target navtracker_example && \
  ctest --test-dir build --output-on-failure
```

Expected: example builds; full suite stays green at 342/342.

- [ ] **Step 5: Commit**

```
git add CLAUDE.md README.md app/example.cpp
git commit -m "docs: CLAUDE.md output-contract subsection; example shows drain"
```

---

## Done criteria

- All 5 tasks committed.
- Full suite green at the end (≥ 342/342).
- `app/example.cpp` ends with a `toTrackOutput` drain showing the canonical library use end-to-end.
- `docs/output-contract.md` exists and is referenced from CLAUDE.md and README.
- `datumAxisRotation` lives in `core/geo/`; both `core/tracking/` and `core/output/` consume it.
- No tracker code changes; no port changes.
