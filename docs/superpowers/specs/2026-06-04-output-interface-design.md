# Output Interface — Design

**Date:** 2026-06-04
**Status:** Approved, ready for plan

## 1. Motivation

The library's input contract is now operator-friendly end-to-end (auto-datum, builders, σ_pos/σ_heading/σ_v composition, CPA σ via Jacobian). The output contract is still raw: consumers must read `TrackManager::tracks()`, project ENU back to lat/lon themselves, derive SOG/COG from the velocity vector, and decide what to do with the 4×4 state covariance.

This spec adds a canonical drainable output shape — `TrackOutput` — built from three structs (`PositionGeodeticWithCov`, `VelocityGeodeticWithSigma`, `TrackOutput`) and three pure-conversion helpers. Position is in lat/lon degrees; covariance stays in m² in the target's local NED frame (Option A from brainstorming). Velocity is in SOG/COG with derived σ values via the standard polar Jacobian.

The output contract is the *pull-based* foundation. Event-based push interfaces (`ITrackSink::onTrackCreated/...`) reuse this shape and are out of scope for this spec.

## 2. Scope

In scope:
- Three structs in `core/output/`:
  - `PositionGeodeticWithCov` — `(lat_deg, lon_deg, position_covariance_m2)`.
  - `VelocityGeodeticWithSigma` — `(sog_m_per_s, cog_deg, sigma_sog_m_per_s, sigma_cog_deg, is_valid)`.
  - `TrackOutput` — bundle of position + velocity + metadata (id, status, last_update, attributes, contributing_sources, covariance_is_default).
- Three conversion helpers:
  - `toGeodeticWithCov(enu_xy, cov_enu_m2, datum)` — Option A: rotates covariance into target's local NED frame, returns PositionGeodeticWithCov.
  - `toVelocityOutput(v_enu, v_cov_m2, is_valid)` — SOG/COG with polar-Jacobian σ derivation.
  - `toTrackOutput(track, datum)` — top-level helper that drives the other two.
- One internal lift: `datumAxisRotation` moves from `core/tracking/DatumShift.hpp` to `core/geo/AxisRotation.hpp` so output and tracking share it without one depending on the other.
- Unit tests covering position round-trip, covariance rotation, SOG/COG conversion, σ derivation at low speed, validity semantics, IMM-mixture handling.
- Operator-facing `docs/output-contract.md` covering units, semantics, worked example.
- CLAUDE.md "Library use" section gains "Output contract" subsection.
- `app/example.cpp` ends with a drain showing `toTrackOutput` in use.

Out of scope (deferred):
- **Event sinks** (`ITrackSink::onTrackCreated/Updated/Deleted`). Separate spec.
- **Collision-risk sink** (`ICollisionRiskSink`). Separate spec.
- **Serialization** (toJson, toCbor, ROS messages). Each is a one-line wrapper consumers can add.
- **Per-axis covariance** in geodetic (deg²). Operator-unfriendly; covariance stays in m².
- **Re-projection across recenter** of cached output snapshots. Consumers re-drain on next cycle.
- **CPA output struct** redesign. `CpaPrediction` is already operator-friendly.

## 3. Architecture

```
TrackManager::tracks()  ───► toTrackOutput(track, datum)  ───► TrackOutput
                                       │
                          ┌────────────┴────────────┐
                          ▼                         ▼
              toGeodeticWithCov(...)      toVelocityOutput(...)
                          │                         │
              ┌───────────┴───────┐        ┌────────┴───────┐
              ▼                   ▼        ▼                ▼
       datum.toGeodetic    datumAxisRotation    polar Jacobian for σ_sog/σ_cog
                          (from core/geo)
```

`core/output/` depends on `core/geo/`, `core/types/`. No new ports. No tracker changes.

## 4. Math

### 4.1 `PositionGeodeticWithCov` — `toGeodeticWithCov`

Inputs:
- `enu_xy ∈ ℝ²` — position in datum-ENU meters.
- `cov_enu_m2 ∈ ℝ²ˣ²` — position covariance, datum-ENU, m².
- `datum` — the current `geo::Datum`.

Computation:
1. `geo_target = datum.toGeodetic({enu_xy.x, enu_xy.y, 0})`.
2. `target_datum = Datum({geo_target.lat_deg, geo_target.lon_deg, 0})`.
3. `R = datumAxisRotation(datum, target_datum)` — 2×2 convergence rotation from spec §4.3 of `2026-06-04-auto-datum-design.md`.
4. `cov_local = R · cov_enu_m2 · Rᵀ`.

Output:
```
PositionGeodeticWithCov {
  lat_deg = geo_target.lat_deg,
  lon_deg = geo_target.lon_deg,
  position_covariance_m2 = cov_local,
}
```

For tracks at the datum origin (or close), `R = I` and `cov_local = cov_enu`. Magnitudes of σ are exact; the rotation matters only off-diagonals at small angles (< 0.5° for tracks within 30 km of datum — guaranteed by auto-datum policy).

### 4.2 `VelocityGeodeticWithSigma` — `toVelocityOutput`

Inputs:
- `v_enu ∈ ℝ²` — `(v_east, v_north)` in m/s.
- `v_cov_m2 ∈ ℝ²ˣ²` — velocity covariance, m²/s².
- `is_valid ∈ {true, false}` — caller's signal whether the velocity is populated at all (e.g., `pose.velocity_is_valid` or track has 4D state).

Computation:

```
sog = ||v_enu||                                       (m/s)
cog_rad = atan2(v_enu.x, v_enu.y)                     (east=sin, north=cos; clockwise from north)
cog_deg = wrap_to_0_360(cog_rad * 180/π)
```

σ derivation via polar Jacobian:

Let `θ = cog_rad`. The Jacobian of `(sog, cog) = h(v_east, v_north)` is:

```
∂sog/∂v_east  = sin(θ)
∂sog/∂v_north = cos(θ)
∂cog/∂v_east  = cos(θ) / sog
∂cog/∂v_north = -sin(θ) / sog
```

Then:
```
J = [[sin(θ),  cos(θ)],
     [cos(θ) / sog,  -sin(θ) / sog]]

cov_polar = J · v_cov · Jᵀ
σ_sog² = cov_polar(0, 0)
σ_cog² = cov_polar(1, 1)  (in rad²)
σ_cog_deg = σ_cog_rad * 180/π
```

Singularity at `sog ≈ 0`: `cog` and `σ_cog` are undefined. Decision:
- If `sog < ε_sog` (default 0.01 m/s): output `cog_deg = 0`, `σ_cog_deg = 0`, but keep `is_valid` based on caller's input. Operator interpretation: "track is stationary; heading direction not meaningful". σ_sog is still well-defined.

When `is_valid == false`: zero all four numeric fields and pass through `is_valid = false`. Operator-visible signal that no velocity info exists.

### 4.3 `TrackOutput` — `toTrackOutput`

Input: `Track t`, `geo::Datum datum`.

Computation:

1. Extract position `(t.state(0), t.state(1))` and 2×2 position covariance from `t.covariance.topLeftCorner<2,2>()`. Call `toGeodeticWithCov`.
2. If `t.state.size() >= 4`:
   - Extract velocity `(t.state(2), t.state(3))` and 2×2 velocity covariance from `t.covariance.block<2,2>(2,2)`.
   - `is_valid_v = (t.covariance.block<2,2>(2,2).trace() > 0)` OR explicit signal from a flag if one exists. Currently no explicit flag, so trace-test is the heuristic.
   - Call `toVelocityOutput(v_enu, v_cov, is_valid_v)`.
   Else: `velocity = VelocityGeodeticWithSigma{0, 0, 0, 0, false}`.
3. Copy `id`, `status`, `last_update`, `attributes`, `contributing_sources`, `covariance_is_default` (derived from any non-zero `covariance_is_default` in `recent_contributions` — implementer's call; simplest is to leave it default-false).

For tracks with IMM ensemble carriers (`imm_means.cols() > 0`), v1 ignores the per-mode carriers and reports only the moment-matched `state / covariance`. Future work: a per-mode output variant. Documented in §11.

## 5. Validity semantics

| Track state | velocity is_valid | sog/cog | σ_sog/σ_cog |
|---|---|---|---|
| 2D (px, py only) | false | 0 | 0 |
| 4D with v_cov trace > 0 | true | computed | computed |
| 4D with v_cov trace = 0 | false | computed (just no σ) | 0 |
| sog < 0.01 m/s | based on caller | sog as-is; cog = 0 | σ_sog as-is; σ_cog = 0 |

The `is_valid` flag tells consumers "this velocity info is meaningful". Stationary tracks (sog ≈ 0) keep `is_valid` based on the caller's signal — operators may want to know that a track is confirmed stationary vs. has no velocity info at all.

`covariance_is_default` propagates from any contributing measurement's flag (forwarded via the M→Track→TrackOutput chain). v1: trivial implementation defaults to false; v2 forwards from `Track::recent_contributions`. Documented.

## 6. Public API

```cpp
// core/output/TrackOutput.hpp
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/// Position in geodetic coordinates (lat, lon) with covariance in m^2
/// expressed in the target's local NED frame (north-east). The rotation
/// from datum-ENU to target-local-NED is applied automatically per Option A
/// of the output-interface design (2026-06-04). For tracks within 30 km
/// of the current datum, this rotation is < 0.5° and the magnitudes
/// of sigma_north / sigma_east match the datum-ENU magnitudes within
/// numerical precision.
struct PositionGeodeticWithCov {
  double lat_deg;
  double lon_deg;
  Eigen::Matrix2d position_covariance_m2;   // local NED at target, m²
};

/// Velocity in operator-facing form: SOG (m/s) + COG (deg, true).
/// COG is measured clockwise from true north, in [0, 360).
/// sigma values are derived from the velocity covariance via the
/// standard polar Jacobian. When sog < 0.01 m/s the COG direction is
/// not meaningful and sigma_cog is set to 0.
/// is_valid carries the caller's signal whether the velocity is
/// populated at all (a track with 2D state has no velocity).
struct VelocityGeodeticWithSigma {
  double sog_m_per_s{0.0};
  double cog_deg{0.0};
  double sigma_sog_m_per_s{0.0};
  double sigma_cog_deg{0.0};
  bool is_valid{false};
};

/// Canonical drainable output for one Track. Position in lat/lon;
/// covariance in m²; velocity in SOG/COG. Bundles metadata: stable
/// track id, lifecycle status, last-update timestamp, sensor-derived
/// attributes (mmsi, vessel type, ...), provenance, and a diagnostic
/// flag indicating whether the underlying covariance came from
/// SensorDefaults rather than a real sensor uncertainty.
struct TrackOutput {
  TrackId id;
  TrackStatus status;
  Timestamp last_update;
  PositionGeodeticWithCov position;
  VelocityGeodeticWithSigma velocity;
  TrackAttributes attributes;
  std::vector<std::string> contributing_sources;
  bool covariance_is_default{false};
};

/// Convert a 2D ENU position + 2×2 covariance into geodetic
/// coordinates with covariance rotated into the target's local NED
/// frame.
PositionGeodeticWithCov toGeodeticWithCov(
    const Eigen::Vector2d& enu_xy,
    const Eigen::Matrix2d& cov_enu_m2,
    const geo::Datum& datum);

/// Convert a 2D ENU velocity + 2×2 covariance into SOG/COG with
/// scalar sigmas via polar Jacobian. is_valid is carried through
/// from the caller's signal.
VelocityGeodeticWithSigma toVelocityOutput(
    const Eigen::Vector2d& v_enu,
    const Eigen::Matrix2d& v_cov_m2_per_s2,
    bool is_valid);

/// Build a TrackOutput from a Track and the current datum. Drives
/// the two helpers above and copies metadata fields verbatim.
TrackOutput toTrackOutput(const Track& track,
                          const geo::Datum& datum);

}  // namespace navtracker
```

## 7. Internal: `datumAxisRotation` lift

Currently in `core/tracking/DatumShift.hpp`. Move to `core/geo/AxisRotation.hpp`:

```cpp
// core/geo/AxisRotation.hpp
#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"

namespace navtracker::geo {

/// 2x2 rotation matrix between the ENU axes of two datums.
/// Implements the convergence angle gamma = delta_lon_rad * sin(mean_lat_rad).
/// At equator: gamma = 0. At 60°N over 1° of longitude: gamma ≈ 0.0151 rad.
Eigen::Matrix2d datumAxisRotation(const geo::Datum& old_datum,
                                  const geo::Datum& new_datum);

}  // namespace navtracker::geo
```

`core/tracking/DatumShift.cpp` updates its include to use the new location. `core/output/TrackOutput.cpp` also includes it. Both `navtracker_core` sources; same target.

## 8. Assumptions

- Track covariance is well-defined (PSD, finite) at output time. Tracker invariants guarantee this.
- For 4D states, the top-left 2×2 block of `t.covariance` is the position covariance in ENU m².
- For 4D states, the (2..3, 2..3) block of `t.covariance` is the velocity covariance in m²/s².
- Tracks with 2D state (position-only) are vanishingly rare in the current codebase; the implementation handles them as a defensive branch (velocity output zero, is_valid false).
- ENU velocity components transform to geodetic SOG/COG via the standard polar Jacobian without further corrections at maritime speeds.
- The local NED rotation for covariance is exact within numerical precision for tracks within 30 km of datum.

## 9. Rationale

**Why Option A (covariance in m², target's local NED)?** Operator-friendly: "σ_north = 5 m" reads cleanly. Numerically clean: meters everywhere, no degrees-per-meter scaling that varies with latitude. Mathematically exact at the target's location with a tiny rotation cost. Alternatives in degrees² scale poorly across the globe; raw datum-ENU covariance is < 0.5° off from target's local frame within the 30 km recenter horizon but technically wrong.

**Why polar Jacobian for σ_sog / σ_cog?** Standard closed-form derivation; what operators expect from heading/speed displays. The Jacobian is regular except at sog = 0, where we explicitly handle the singularity.

**Why a single `TrackOutput` struct rather than separate position/velocity/metadata?** Drain loops want one type. Splitting into three forces the consumer to re-assemble the relationship between "this lat/lon" and "this SOG/COG" — they're per-track facts. Bundling is simpler at the call site.

**Why pull-based first (not event sinks)?** Pull-based is the foundation; events that fire at lifecycle transitions reuse the same struct. Designing the struct correctly first means the event spec just decides "when to fire" rather than "what to emit". Pull-based also works for consumers that don't need events (chart overlays, periodic logging).

**Why lift `datumAxisRotation`?** It's pure geo math, currently in `core/tracking/`. Output needs it too. Lifting to `core/geo/` makes both consumers depend on the right layer; neither depends on the other.

**Why `is_valid` flag on velocity instead of `std::optional<VelocityGeodeticWithSigma>`?** Symmetry with `OwnShipPose.velocity_is_valid` from the RMC work. Keeps `TrackOutput` plain trivially-copyable (no optional). Consumers branch on `is_valid` explicitly.

**Why `covariance_is_default` defaults false in v1?** Forwarding from `Track::recent_contributions` requires walking the touches and checking each. Trivial work but not in v1 scope. Forwarding is a one-line follow-up.

**Why no per-IMM-mode output?** Most consumers want the moment-matched Gaussian. Per-mode output is useful for IMM-aware downstream (mode-weighted CPA, etc.) — separate spec, separate struct.

**Why `docs/output-contract.md`?** Operator-facing semantics need a focused doc. Units, validity, what "σ_north = 5 m" means in maritime terms, worked example. Going into CLAUDE.md inflates the architecture doc; a separate file is searchable.

## 10. Documentation plan

### `docs/output-contract.md`

Sections:
1. **Position** — lat/lon in WGS84 degrees, covariance in m² in local NED at target.
2. **Velocity** — SOG (m/s, ≥0), COG (deg, [0,360), clockwise from true north), σ values, is_valid semantics.
3. **Track metadata** — id (stable, monotonic, never reused), status (Tentative/Confirmed/Deleted), attributes (mmsi optional, etc.), contributing_sources.
4. **Diagnostic flag** — covariance_is_default semantics.
5. **Singularities** — stationary tracks, no-velocity tracks, near-datum tracks (where Option A rotation is identity).
6. **Worked example** — drain a Track, show all fields, what they mean operationally.

### CLAUDE.md "Library use" section

Append an "Output contract" subsection (10–20 lines):
- Pointer to `toTrackOutput` as the canonical drain function.
- Pointer to `docs/output-contract.md` for unit/validity semantics.
- Pointer to `app/example.cpp` for the canonical drain pattern.

### `app/example.cpp`

End the example with a drain loop:

```cpp
for (const Track& t : mgr.tracks()) {
  if (t.status != TrackStatus::Confirmed) continue;
  const TrackOutput out = toTrackOutput(t, provider.datum());
  // emit to your sink: lat/lon + σ + SOG/COG
}
```

### `README.md`

One-line addition to "Library use" pointing to `docs/output-contract.md`.

## 11. Ways to improve / what to test next

1. **Event-based sinks** (`ITrackSink::onTrackCreated/Updated/Deleted`). Fires `TrackOutput` (or `TrackId` + cached snapshot) at lifecycle transitions. Real-time UI, persistence, replay logging.
2. **Collision-risk sink** (`ICollisionRiskSink`). Fires when `P(CPA<d) > threshold` for any (own_ship, target) pair. CpaEvaluator runs over all pairs each cycle.
3. **Per-mode IMM output** — `TrackOutputWithModes` carrying per-mode (state, cov, probability). Useful for mode-aware downstream.
4. **Particle-based output** — sample/quantile representation for non-Gaussian posteriors.
5. **Serialization helpers** — `toJson(TrackOutput)`, `toCbor(TrackOutput)`, ROS message converters. Each is a thin wrapper; ship in a separate optional CMake target.
6. **`covariance_is_default` forwarding** — walk `Track::recent_contributions` to set the flag truthfully.
7. **Output in alternative geodetic systems** (NAD83, etc.). Currently WGS84-only.
8. **Per-axis velocity σ** when `OwnShipPose.velocity_std_m_per_s` is promoted from scalar to 2×2 (RMC spec §11). The polar Jacobian handles this once the inputs change.

## 12. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Covariance shape | Option A: m² in target's local NED | Operator-friendly; exact with tiny rotation. |
| Position output | lat/lon degrees (WGS84) | Standard maritime output. |
| Velocity output | SOG (m/s), COG (deg, true) | Operator UI matches. |
| σ derivation | Polar Jacobian | Standard closed-form. |
| Stationary singularity | sog < 0.01 → cog = 0, σ_cog = 0 | Avoid undefined direction. |
| Validity flag | bool `is_valid` on Velocity | Symmetric with OwnShipPose; keeps struct plain. |
| Aggregation | Single `TrackOutput` bundle | Drain loops want one type. |
| Push vs pull | Pull-based (drain via toTrackOutput) | Events are layered later. |
| `datumAxisRotation` location | Lift to `core/geo/AxisRotation` | Shared by output and tracking. |
| Documentation | Inline Doxygen + `docs/output-contract.md` | Operator-facing doc is searchable separately. |
| IMM per-mode | Out of scope; v1 reports moment-matched | Defer for spec-defined mode-aware consumer. |
