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
- **`is_valid`**: true when the track carries meaningful velocity information. Set false when the underlying track has 2D state (position-only), or when the velocity covariance is zero (e.g., fresh own-ship pose, or 4D state with zero velocity uncertainty).

### Stationary tracks

When sog < 0.01 m/s, the COG direction is not meaningful and **both `cog_deg` and `sigma_cog_deg` are zeroed**. The `is_valid` flag remains set based on the caller's signal. Interpretation: "track is confirmed stationary" (velocity=0) vs. "`is_valid=false`" (no velocity information at all). Two distinct operator signals.

## Track metadata

- **`id`**: Stable, monotonically increasing integer. Never reused after a track is deleted. Two successive drains on the same processing cycle return the same id.
- **`status`**: Enum. Tentative (new, < M-of-N confirmed), Confirmed (M-of-N threshold met), Deleted (marked for removal, drained one final time).
- **`last_update`**: Timestamp of the most recent measurement that contributed to this track's state.
- **`attributes`**: Optional fields sourced from identity-carrying sensors (AIS, etc.): mmsi, vessel_name, vessel_type, length_m, beam_m. All optional; client should test presence before use.
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
| **No velocity info** (2D state) | sog=0, cog=0, σ_sog=0, σ_cog=0; is_valid=false | "No velocity data available." |
| **At datum** (track position = datum origin) | position_covariance_m2 unchanged (R = identity rotation) | Covariance magnitude is exact; no rotation error. |
| **Distant track** (track < 30 km from datum) | Rotation < 0.5°; off-diagonal terms small relative to diagonals | Covariance is exact to within rounding. |
