# Auto-Datum + Re-Center — Design

**Date:** 2026-06-04
**Status:** Approved, ready for plan

## 1. Motivation

Library users currently must construct a `geo::Datum` at composition time and thread it through every measurement builder, `synthesizeOwnShipTrack`, and adapter. This is bad UX (the operator must understand a concept that exists purely because of the LTP approximation) and physically wrong over long voyages (LTP error grows ~8 m at 10 km, ~200 m at 50 km, ~785 m at 100 km from the datum).

This spec eliminates the user-facing datum concept. `OwnShipProvider` owns the datum, lazy-initialises it from the first pose, auto-re-centers when own-ship moves too far, and emits a sink event so any consumer that cares about ENU coordinates (the 1% case) can react. The 99 % of consumers reading lat/lon never see the re-center.

## 2. Scope

In scope:
- `OwnShipProvider` lazy-initialises a `geo::Datum` from the first `update(pose)` call.
- Auto-re-center when own-ship's distance from the current datum exceeds a configurable threshold (default 30 km).
- New `IDatumChangeSink` port with `onDatumRecentered(old, new)` fired before track shifts are applied.
- Free utility `shiftTracksOnDatumChange(TrackManager&, old, new)` that mutates every track's state and covariance into the new ENU frame (translation + axis rotation).
- `MeasurementBuilders` drop their explicit `const geo::Datum&` argument; pull from `provider.datum()`.
- `synthesizeOwnShipTrack` drops its `const geo::Datum&` argument; takes the provider.
- A small composition-root adapter struct (Pattern A) bridges `IDatumChangeSink` → `shiftTracksOnDatumChange(TrackManager&, ...)`. `TrackManager` stays coordinate-system-agnostic.
- Backward-compat constructor `OwnShipProvider(Datum, ...)` keeps every existing test deterministic.
- Sim adapters' explicit `Datum` arguments stay as-is (sim composition is deterministic and the datum is part of the truth setup).
- `app/example.cpp` and CLAUDE.md updated — no more "you must pick a datum" line.

Out of scope (deferred to §11):
- **Velocity-frame Coriolis correction over very long re-centers.** Negligible at maritime speeds; documented for completeness.
- **Re-projection of cached CPA / path predictions across a re-center.** Re-evaluate on next call instead; predictions held in caller-side caches are the caller's problem.
- **Auto-pick of initial datum when pose isn't strictly the first observable.** For v1 the first `update(pose)` establishes the datum. Edge cases (measurement before pose) are documented limitations.
- **Per-track event hooks** (`onTrackCreated`, `onTrackUpdated`, ...) — that's the broader output-interface conversation, not in this spec.

## 3. Architecture

```
OwnShipProvider
   ├── current_datum_ (std::optional<geo::Datum>)
   ├── history_      (existing ring buffer of OwnShipPose)
   ├── recenter_policy_
   └── sinks_        (vector of IDatumChangeSink*)

update(pose):
   1. If !current_datum_: current_datum_ = Datum(pose.lat, pose.lon, pose.alt).
   2. Else if policy.enable_auto_recenter:
        distance = ||current_datum_.toEnu(pose) − origin||
        if distance > policy.recenter_threshold_km · 1000:
           old = *current_datum_
           current_datum_ = Datum(pose.lat, pose.lon, pose.alt)
           for each sink in sinks_: sink.onDatumRecentered(old, *current_datum_)
   3. Push pose to history (existing).

Composition root:
   OwnShipProvider provider;       // no datum arg
   TrackManager mgr{2, 3};
   TrackShifterSink mgr_sink{&mgr};
   provider.registerDatumSink(&mgr_sink);
   // ... rest of wiring ...

TrackShifterSink:
   struct TrackShifterSink : IDatumChangeSink {
     TrackManager* mgr;
     void onDatumRecentered(const Datum& o, const Datum& n) override {
       shiftTracksOnDatumChange(*mgr, o, n);
     }
   };
```

The provider is the single owner of the datum. The sink port is the seam between "datum changed" and "consumers react". `TrackShifterSink` is a tiny composition-root adapter (Pattern A from the brainstorm) that translates the event into a track-state shift.

## 4. Math

### 4.1 Re-center triggering

Distance from current datum to pose, in meters:

```
enu = current_datum_.toEnu({pose.lat_deg, pose.lon_deg, pose.alt_m})
d = √(enu.x² + enu.y²)
trigger when d > recenter_threshold_m
```

Using just the horizontal distance is correct — re-center is about LTP error, which is horizontal.

### 4.2 Track shift on re-center

For each track `t`:

```
geo_old   = old_datum.toGeodetic([t.state(0), t.state(1), 0])
new_xy    = new_datum.toEnu(geo_old).head<2>()
R         = ENU-axis rotation matrix from old to new frame
            (closed-form, see §4.3)

t.state(0..1) = new_xy
t.state(2..3) = R · t.state(2..3)
t.covariance  = blockRotate(R, t.covariance)

For IMM / particle / multi-mode tracks: apply the same shift to every
mode mean / particle position, and rotate every mode covariance.
```

### 4.3 ENU-axis rotation between two datums

The ENU axes at two nearby points differ by a small rotation determined mostly by meridian convergence (longitudinal motion at high latitudes) and tangent-plane geometry.

For the two datums `D_old(lat_o, lon_o)` and `D_new(lat_n, lon_n)`:

```
Δlon_rad = (lon_n − lon_o) · π/180
mean_lat = (lat_o + lat_n) / 2 · π/180

// Convergence angle (the azimuth rotation between the two local
// north directions when traversing along a parallel).
γ = Δlon_rad · sin(mean_lat)

R = [[ cos(γ), −sin(γ)],
     [ sin(γ),  cos(γ)]]
```

At 60 °N, traversing 30 km eastward (Δlon ≈ 0.54°) gives γ ≈ 0.0082 rad ≈ 0.47°. At the equator γ = 0. At 70 °N over 30 km east, γ ≈ 0.96°.

### 4.4 4×4 covariance rotation

State covariance is 4×4 over `[px, py, vx, vy]`. The rotation acts on the (position) and (velocity) sub-blocks identically:

```
R̄ = [[R, 0_2],
      [0_2, R]]      (4×4 block-diagonal)

cov_new = R̄ · cov_old · R̄^T
```

This is exact under the assumption that the ENU axis rotation between the two frames is the only frame change. The translation (origin shift) doesn't affect covariance.

## 5. Configuration

```cpp
struct DatumRecenterPolicy {
  bool enable_auto_recenter{true};
  double recenter_threshold_km{30.0};
};
```

30 km gives ~70 m worst-case LTP error before re-center, well below typical bearing-projection σ at maritime ranges. Users who want tighter tracking dial it down; users who want fewer re-centers (e.g., for downstream consumers that don't subscribe to the sink event) dial it up.

## 6. Public API

```cpp
// adapters/own_ship/OwnShipProvider.hpp

namespace navtracker {

class IDatumChangeSink {
 public:
  virtual ~IDatumChangeSink() = default;
  virtual void onDatumRecentered(const geo::Datum& old_datum,
                                 const geo::Datum& new_datum) = 0;
};

struct DatumRecenterPolicy {
  bool enable_auto_recenter{true};
  double recenter_threshold_km{30.0};
};

class OwnShipProvider {
 public:
  // Library-friendly: no datum arg. Lazy-init from first update().
  explicit OwnShipProvider(std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  // Backward-compat / determinism: pin the datum at construction.
  OwnShipProvider(geo::Datum initial_datum,
                  std::size_t history_size = 16,
                  DatumRecenterPolicy policy = {});

  void update(const OwnShipPose& pose);
  std::optional<OwnShipPose> latest() const;
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;
  std::size_t historySize() const;

  // Datum access. throws std::runtime_error if !hasDatum().
  const geo::Datum& datum() const;
  bool hasDatum() const noexcept;

  void registerDatumSink(IDatumChangeSink* sink);
  void unregisterDatumSink(IDatumChangeSink* sink);
};

// Free utility. Lives where TrackManager lives (core/tracking/).
void shiftTracksOnDatumChange(TrackManager& mgr,
                              const geo::Datum& old_datum,
                              const geo::Datum& new_datum);

}  // namespace navtracker
```

### 6.1 Builder signatures

```cpp
// Before:
Measurement makeMeasurementFromRelativeBearing(
    SensorKind, std::string, Timestamp,
    double, double, double, double,
    const OwnShipProvider&,
    const geo::Datum&,                  // <-- dropped
    AssociationHints = {});

// After:
Measurement makeMeasurementFromRelativeBearing(
    SensorKind, std::string, Timestamp,
    double, double, double, double,
    const OwnShipProvider&,
    AssociationHints = {});
```

Same change for `makeMeasurementFromTrueBearing`. `makeMeasurementFromEnuPosition` is unchanged — it takes pre-projected ENU coordinates.

### 6.2 `synthesizeOwnShipTrack`

```cpp
// Before:
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const geo::Datum& datum);

// After:
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const OwnShipProvider& provider);
```

Reads `provider.datum()` for the ENU conversion. Throws if `!hasDatum()`.

## 7. Sim adapters

`sim::OwnShipEmitter`, `AisAdapter`, `ArpaAdapter`, `EoIrAdapter` retain their explicit `geo::Datum` arguments. Sim composition is deterministic and the datum is part of the truth setup; pulling from an `OwnShipProvider` would require the sim emitter to hold a reference to it (it already does, indirectly via the adapter). For v1 the sim continues as-is. A future cleanup could harmonise sim with library-side auto-datum if useful.

## 8. Assumptions

- Re-centers happen rarely (every several hours at sea, never on a tactical timescale). Per-track shift cost is amortised to nothing.
- ENU axis rotation between two datums separated by < threshold (30 km) is small enough to be modelled as a single 2×2 rotation (no higher-order corrections needed).
- Position covariance is well-defined (PSD, finite) for every track at re-center time. Tracker invariants guarantee this.
- Tracks are stored in `TrackManager::tracks()` as a contiguous vector. The shift utility walks them in place via the existing accessor.
- Sinks are stable pointers — the registrar owns lifetime. `unregisterDatumSink` is symmetric.

## 9. Rationale

**Why 30 km threshold and not 50 km?** 50 km is where LTP position error hits ~200 m. 30 km hits ~70 m, comfortably below typical maritime measurement σ. Trade-off: 30 km re-centers more often (slightly more sink work); 50 km accepts more LTP error. 30 km is the safer default; users who don't care about ENU precision (most) can raise it.

**Why a port (IDatumChangeSink) instead of a callback std::function?** Consistency with the rest of the codebase, which uses ports for cross-component seams (`IEstimator`, `IDataAssociator`, etc.). Easier to test (mock the port). Composition root assembles via adapter struct.

**Why Pattern A (small adapter struct) instead of TrackManager implementing the port?** Keeps TrackManager coordinate-system-agnostic. The shift utility is testable in isolation. The composition cost is a 4-line glue struct. Matches the architectural invariant from CLAUDE.md ("core has zero I/O and zero sensor-format knowledge"). Coordinate systems aren't I/O or sensor formats, but the spirit is the same: TrackManager shouldn't need to know how the world is parameterised.

**Why sim adapters keep their `Datum`?** Sim composition is test infrastructure; determinism matters more than ergonomics. The sim's `OwnShipEmitter` knows the truth — it doesn't need to discover it.

**Why throw on `datum()` when `!hasDatum()`?** Forces the user to either provide a pose first or use the explicit-datum constructor. Silent fallback to a default datum (e.g., (0, 0)) would silently produce nonsense ENU values.

**Why lazy-init from `update(pose)` and not from `registerDatumSink` or some other event?** The pose carries the actual lat/lon that the datum should be anchored to. Other entry points have no geo data.

**Why fire the sink event before applying track shifts (no-op for TrackShifterSink, since it does the shift)?** Future sinks may need to capture the "before" state for downstream notification (e.g., emit a re-centered event to a UI). Firing before keeps the contract clean: sinks see (old, new) datums and can do whatever they need.

**Why does `MeasurementBuilders` drop the `Datum` argument but not the `OwnShipProvider` argument?** The provider is the source of both the pose and the datum; pulling datum from it is the natural follow-on of the L4 work (commit `cfcafc7`). Dropping both would mean builders take only the static configuration and a measurement value — that's a bigger change to consider as part of the output-interface conversation.

## 10. Test plan

### Unit

1. **`tests/adapters/own_ship/test_own_ship_provider.cpp`** — extend:
   - `NoDatumBeforeFirstUpdate`: `hasDatum() == false`, `datum()` throws.
   - `DatumInitialisesFromFirstUpdate`: push pose at (53.5, 8.0); `datum()` returns Datum(53.5, 8.0, 0.0).
   - `DatumStaysFixedBelowThreshold`: push poses within 30 km; datum unchanged, no sink event.
   - `RecenterFiresAtThreshold`: push poses moving > 30 km; sink fires `onDatumRecentered(old, new)`; new datum at the triggering pose.
   - `RecenterDisabledByPolicy`: `policy.enable_auto_recenter = false`; even at 100 km, no recenter.
   - `MultipleSinksAllFire`: register two sinks; both receive the event.
   - `UnregisteredSinkDoesNotFire`: unregister, then trigger; sink not called.
   - `ExplicitDatumConstructorWorks`: backward-compat construction with explicit datum; `hasDatum() == true` immediately.

2. **`tests/tracking/test_shift_tracks_on_datum_change.cpp`** — new:
   - `PreservesGeodeticPositionUnderShift`: place a track at known (lat, lon); shift to a new datum; verify the new ENU position round-trips to the same (lat, lon).
   - `RotatesVelocityByConvergenceAngle`: track with velocity (1, 0) m/s in old frame; shift to a datum 100 km east at 60 °N; verify new velocity is rotated by ~γ.
   - `RotatesCovarianceBlocks`: 4×4 cov rotates correctly per `R̄ · cov · R̄^T`.
   - `EmptyTrackListNoOp`: empty manager; no crash.
   - `IMMTrackStatesShifted`: track with `imm_means` populated; each mode is shifted.

3. **`tests/types/test_measurement_builders.cpp`** — update:
   - Existing tests that constructed `Datum` and passed it explicitly switch to library-friendly `OwnShipProvider` constructor; the datum lazy-initialises from the test's pose update.
   - Add `BuilderUsesProviderDatum`: provider constructed library-friendly; push pose; build measurement; verify ENU position matches what the explicit datum would have produced.

4. **`tests/collision/test_cpa_synthesize_own_ship.cpp`** — update:
   - Existing tests pass provider via new `synthesizeOwnShipTrack` signature.

### Integration

5. **`tests/scenario/test_datum_recenter_scenario.cpp`** — new:
   - Build a tracker + provider + a track at known (lat, lon, vx, vy).
   - Push own-ship poses crossing the 30 km threshold so re-center fires once.
   - Verify the track's geodetic position is unchanged (to within the LTP error budget) before and after the re-center.
   - Verify the track's ENU position changed (sanity check that the shift actually happened).

### Backward-compat regression guard

All 318 existing tests use one of:
- `OwnShipProvider provider;` followed by a pose update → exercises auto-datum path.
- `OwnShipProvider(Datum)` → exercises explicit-datum path (backward compat).
- Sim composition with explicit Datum → unchanged.

Tests that exercise sensor-builder signatures need their `Datum` arg removed. Mechanical change. Verify by full suite green at 326/326 (current 318 + ~7 new + 1 modified).

### Eval-log

Append "Auto-datum + re-center (2026-06-04)" section. Show a recenter triggered mid-scenario; record:
- Distance own-ship traversed before recenter fired.
- Track position drift before vs after recenter (should be near-zero in geodetic terms; meaningful in ENU).
- Test outcome: no accuracy regression on the §14.9 sweep cells re-run with the library-friendly constructor.

## 11. Ways to improve / what to test next

1. **Configurable recenter strategy.** Today the policy is "recenter to the current pose when threshold exceeded". A smarter strategy could re-center to the centroid of recent poses (less re-centering on a vessel cruising near the threshold). Trade-off: more state, marginal gain.
2. **Recenter event coalescing.** If a vessel oscillates near the threshold, repeated re-centers happen. A hysteresis band (e.g., don't re-center if a re-center happened in the last N minutes) avoids the thrash.
3. **Coriolis / Earth-rotation correction.** Over a very long re-center traverse (e.g., a 1000 km cruise) the inertial frame motion becomes non-negligible. Maritime speeds make this negligible in practice.
4. **Sim adapters consume `OwnShipProvider` datum.** A future cleanup harmonises sim composition with library-friendly composition. Not load-bearing.
5. **Auto-pick initial datum from first sensor measurement.** If a measurement arrives before any pose, use its `sensor_position_enu`-derived position. Today this case throws or returns empty.
6. **TrackManager port (IDatumChangeSink) implementation as default.** If we end up with many composition roots all wiring the same `TrackShifterSink` glue, promote it to `TrackManager` directly. Defer until pattern is proven painful.

## 12. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Approach | Auto-datum + auto-recenter | Eliminates UX problem; fixes LTP error over distance. |
| Recenter threshold default | 30 km | 70 m LTP error budget; comfortably below maritime σ. |
| Sink contract | Port (`IDatumChangeSink`) | Consistent with rest of codebase; testable. |
| Track-shift integration | Pattern A (composition-root adapter) | TrackManager stays coordinate-system-agnostic. |
| Sim adapters | Keep explicit `Datum` argument | Sim is deterministic test infra; ergonomics secondary. |
| Builder signatures | Drop `Datum` arg; pull from provider | Symmetric with L4 (commit cfcafc7). |
| `synthesizeOwnShipTrack` | Takes provider instead of datum | Drops the v1 shim. |
| Backward compat | Explicit-datum constructor of provider stays | Tests get determinism; library users get auto. |
| Empty-pose case | `datum()` throws | Forces explicit handling; no silent fallback to (0, 0). |
