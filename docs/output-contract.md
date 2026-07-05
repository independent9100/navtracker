# Output Contract

How to interpret `TrackOutput` and read the track position, velocity, and metadata fields. Canonical drain function: `toTrackOutput(track, datum)` in `core/output/TrackOutput.hpp`.

## Position

- **`lat_deg`, `lon_deg`**: WGS84 geodetic latitude/longitude, degrees. Range: lat ∈ [−90, 90], lon ∈ (−180, 180].
- **`position_covariance_m2`**: 2×2 symmetric positive-semidefinite matrix, units m². Expressed in the **target's local NED frame** (north-east; row/column 0 = north, row/column 1 = east). The covariance is rotated from the fusion datum's ENU frame into the target's local NED via the convergence angle γ = Δlon·sin(mean_lat). For tracks within the 30 km auto-datum recenter horizon, this rotation is < 0.5°; magnitudes of σ_north and σ_east match the datum-ENU values within numerical precision.
- Extract uncertainties as σ_north = √(position_covariance_m2[0,0]) and σ_east = √(position_covariance_m2[1,1]). These are physical meters at the target's geodetic location.

## Velocity

- **`sog_m_per_s`**: Speed over ground, m/s, ≥ 0.
- **`cog_deg`**: Course over ground, degrees true (clockwise from true north), ∈ [0, 360). Undefined when sog < 0.01 m/s (see stationary singularity below).
- **`sigma_sog_m_per_s`, `sigma_cog_deg`**: 1σ uncertainties derived from the velocity covariance via the standard polar Jacobian. σ_cog is in degrees.
- **`is_valid`**: true only when velocity has actually been observed **and** its covariance is usable. Concretely, `toTrackOutput` requires all of: the track has ≥ 4D state (position + velocity); `track.velocity_observed` is true (≥ 1 update past initiation, so a pure init-prior velocity never reports valid); and the 2×2 velocity covariance is finite and positive-definite (`v_cov(0,0) > 0` **and** `det > 0`). If any of these fails → `is_valid = false` — i.e. a 2D-only state, a velocity that has not yet been observed, or a zero/degenerate velocity covariance.

### Stationary tracks

When sog < 0.01 m/s, the COG direction is not meaningful and **both `cog_deg` and `sigma_cog_deg` are zeroed**. The `is_valid` flag remains set based on the caller's signal. Interpretation: "track is confirmed stationary" (velocity=0) vs. "`is_valid=false`" (no velocity information at all). Two distinct operator signals.

## Track metadata

- **`id`**: Stable, monotonically increasing integer. Never reused after a track is deleted. Two successive drains on the same processing cycle return the same id.
- **`status`**: Enum. Tentative (new, < M-of-N confirmed), Confirmed (M-of-N threshold met), Coasting (was Confirmed, now propagated through a detection gap without a fresh update), Deleted (marked for removal, drained one final time).
- **`last_update`**: Timestamp of the most recent measurement that contributed to this track's state.
- **`attributes`**: Optional fields sourced from identity-carrying sensors (AIS, cooperative fleet partners, a remote/VTS feed): `mmsi`, `platform_id`, `heading_deg`, `nav_status`, `vessel_name`, `vessel_type`, `length_m`, `beam_m`. All optional; client should test presence before use. `mmsi` (AIS) and `platform_id` (cooperative fleet identity, numeric) are **parallel** identity keys — a track may carry either or both. `heading_deg` (true heading, deg [0,360)) and `nav_status` (AIS navigational-status code; 1 = at anchor, 5 = moored) are **target-reported kinematics** from an AIS self-report (backlog #20): `heading_deg` fills the "stationary, direction undefined" gap (an anchored vessel points somewhere even when SOG≈0 makes COG meaningless), and `nav_status` is the anchored/moored operator cue. All four are surfaced last-write-wins from the hints of the measurements a track claims, under **both** the PMBM (winning) and MHT trackers (identity was PMBM-empty before R11). None is the fusion key — that is always the internal `id` (invariant 5); identity/attributes are hints.
- **`contributing_sources`**: Vector of source_id strings ("ais", "arpa", "eoir") that have ever contributed measurements to this track. Order and multiplicity are unspecified.

## Diagnostic flag

- **`covariance_is_default`**: true when any contributing measurement populated its covariance from `SensorDefaults` rather than from a real sensor uncertainty model. Downstream sinks may flag low-confidence or tune gate thresholds accordingly; the tracker behaves identically. v1 defaults to false; v2 forwards truthfully from `Track::recent_contributions`.

## Worked example

**Scenario:** Vessel at lat=53.6°N, lon=8.2°E (North Sea, near Heligoland). Moving due east at 5.0 m/s. Position σ = 5 m (north and east). Velocity σ = 0.5 m/s (speed); direction σ = 5° (course). Tracked by AIS for 45 seconds. Confirmed, id=42.

**TrackOutput dump:**

```
id                      = 42
status                  = Confirmed
last_update             = 123456789 (epoch nanoseconds since fusion start)
position.lat_deg        = 53.600000
position.lon_deg        = 8.200000
position.position_covariance_m2 =
  [ 25.00  0.00 ]   (north, east; 5m sigma each)
  [  0.00 25.00 ]
velocity.sog_m_per_s    = 5.0
velocity.cog_deg        = 90.0  (due east)
velocity.sigma_sog_m_per_s = 0.5
velocity.sigma_cog_deg  = 5.0  (derived from polar Jacobian at sog=5 m/s)
velocity.is_valid       = true
attributes.mmsi         = 211378120
attributes.platform_id  = (absent — set only when a cooperative/remote feed carries one)
attributes.vessel_name  = "EXAMPLE VESSEL"
attributes.vessel_type  = "Cargo"
attributes.length_m     = 190
attributes.beam_m       = 32
contributing_sources    = ["ais"]
covariance_is_default   = false
```

**Interpretation in a UI:**
- Display lat 53.6°N, lon 8.2°E, with a 5 m circular uncertainty disk.
- Display velocity vector pointing east (90°) with label "5.0 m/s" and a heading-deviation arc showing ±5°.
- Display "EXAMPLE VESSEL (Cargo, 190×32 m)" and track id 42 as header.
- Show "Source: AIS" and note that the σ values came from real sensor covariance, not defaults.

## Singularities at a glance

| Condition | Field values | Operator signal |
|---|---|---|
| **Stationary** (sog < 0.01 m/s) | cog_deg=0, σ_cog_deg=0; is_valid=true | "Confirmed stationary; direction undefined." |
| **No velocity info** (2D state, or velocity not yet observed / covariance degenerate) | sog=0, cog=0, σ_sog=0, σ_cog=0; is_valid=false | "No velocity data available." |
| **At datum** (track position = datum origin) | position_covariance_m2 unchanged (R = identity rotation) | Covariance magnitude is exact; no rotation error. |
| **Distant track** (track < 30 km from datum) | Rotation < 0.5°; off-diagonal terms small relative to diagonals | Covariance is exact to within rounding. |

---

## StaticHazardOutput

Canonical drain function: `toStaticHazardOutput(obs)` in `core/output/StaticHazardOutput.hpp`.
This is a separate output type from `TrackOutput`. It represents a charted static
hazard (rock, wreck, pile, platform, buoy, etc.) rather than a kinematic vessel track.

### Position

- **`position.lat_deg`, `position.lon_deg`**: WGS84 geodetic latitude/longitude,
  degrees. The position is taken directly from the `StaticObstacle` chart record
  and is not adjusted for the ENU datum.
- **No NED covariance rotation.** `TrackOutput` rotates the position covariance
  from the fusion datum's ENU frame into the target's local NED frame (a
  kinematic necessity because the ENU frame shifts as own-ship moves). For a
  charted obstacle, the position is a fixed geographic coordinate — it does not
  move with the datum. No rotation is applied or needed; the positional accuracy
  is encoded in `position_uncertainty_m` (see below).

### Geometry fields

- **`footprint_radius_m`**: physical extent of the object in metres. Together
  with `position_uncertainty_m` this forms the hard no-birth core
  `R_hard = footprint_radius_m + position_uncertainty_m` (see §14.10 of the
  design spec and `docs/algorithms/static-obstacle-birth-prior.md`).
- **`keep_clear_radius_m`**: the operationally required clearance margin in
  metres. Must be ≥ `footprint_radius_m`. The `StaticHazardEvaluator` fires a
  proximity alarm when own-ship enters this radius (see "Keep-clear alarm" below).
- **`position_uncertainty_m`**: positional accuracy of the chart record in
  metres. Absorbs survey error (analogous to `W_off` in the coastline model).
  Added to the footprint when computing the hard core.

### Attributes

- **`category`** (`ObstacleCategory`): Rock, Wreck, Obstruction, Pile, Platform,
  Buoy, Beacon, Other, Unknown. Aligned to the S-57/S-101 CATOBS ontology.
- **`water_level`** (`WaterLevel`): AwashCoversUncovers, AlwaysSubmerged,
  AlwaysAboveWater, Floating, Unknown. S-57 WATLEV.
- **`depth_m`**: charted depth in metres (S-57 VALSOU). `NaN` when unknown.
- **`lit`**: whether the hazard carries a navigational light.
- **`aton`** (`AtoNRealism`): NotAtoN, Real, Synthetic, Virtual. A Virtual AtoN
  is an AIS-broadcast hazard mark (AIS Message 21) with no physical structure at
  that position.
- **`source_id`**: provenance string (chart identifier or AtoN MMSI).

### Hazard identity

- **`hazard_id`**: a stable `uint64_t` identifier derived from the obstacle's
  charted position and category via `staticHazardId(obs)`. The function rounds
  lat/lon to approximately 1 m before hashing, so small floating-point jitter in
  the chart record does not change the id between runs. The id is
  **order-independent** (not a list index); re-ordering the obstacle list does
  not change any hazard's id. This mirrors the stable `track_id` guarantee for
  vessel tracks.
- **`is_charted`**: `true` for Stage-1 charted obstacles (the current
  implementation). Will be `false` for Stage-1b / Stage-2 live-detected
  occupancy hazards that have no chart record.

### Keep-clear alarm

The `StaticHazardEvaluator` emits `IStaticHazardSink` events when own-ship
enters or exits the keep-clear ring of a charted obstacle:

- **Entered**: own-ship ENU distance to the obstacle centre falls below
  `keep_clear_radius_m`.
- **Updated**: emitted each cycle while inside (optional; `emit_updates = false`
  by default).
- **Exited**: own-ship ENU distance exceeds
  `keep_clear_radius_m × exit_hysteresis` (default 1.1 — a 10% margin to
  prevent flapping at the boundary).

**This is a static range check, not a CPA.** CPA (Closest Point of Approach)
extrapolates a vessel's current velocity to find its future closest approach.
A charted obstacle has no velocity. The keep-clear alarm answers a different
question: "is own-ship currently inside the required clearance zone?" That
question has a direct geometric answer; no trajectory extrapolation is needed or
appropriate. The vessel-track CPA (`CpaEvaluator`) remains the correct tool for
dynamic hazard assessment against confirmed vessel tracks.
