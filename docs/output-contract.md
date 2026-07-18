# Output Contract

How to interpret `TrackOutput` and read the track position, velocity, and metadata fields.

## Choosing a drain — covariance-ordering convention (F3, 2026-07-12)

There are **two** canonical drains in `core/output/TrackOutput.hpp`, one per
position-covariance axis ordering. A caller MUST pick one — there is
deliberately no ambiguous `toTrackOutput` (it was removed; the compile-time
break at each call site is the consumer audit, so no caller flips silently):

- **`toTrackOutputENU(track, datum)`** — east-first: `position_covariance_m2`
  slot (0,0) = east variance, (1,1) = north. This is the ordering the internal
  ENU state carries. Stamps `covariance_frame = CovarianceFrame::Enu`.
- **`toTrackOutputNED(track, datum)`** — north-first (operator-facing):
  slot (0,0) = north variance, (1,1) = east. Stamps `covariance_frame =
  CovarianceFrame::Ned`.

The two are identical in every other field (lat/lon, velocity, metadata) —
ONLY the covariance axis ordering and the `covariance_frame` tag differ. Read
`covariance_frame` to know which convention a `TrackOutput` in hand carries;
axis-sensitive consumers may assert on it.

> **Upgrade note (for consumers of the pre-2026-07-12 API).** The old
> `toTrackOutput` documented NED but its covariance was actually emitted in ENU
> (east-first) order — so any consumer that trusted the old *doc* and read slot
> (0,0) as north was silently transposing axes for anisotropic tracks. On
> upgrade your build breaks at every call site; pick the name matching your
> downstream's real expectation. If you were reading the old output as NED and
> it "looked right", your data was isotropic and masked the bug — verify.

## Position

- **`lat_deg`, `lon_deg`**: WGS84 geodetic latitude/longitude, degrees. Range: lat ∈ [−90, 90], lon ∈ (−180, 180].
- **`position_covariance_m2`**: 2×2 symmetric positive-semidefinite matrix, units m², in the target's local frame with axis ordering per the drain used (see above) and recorded in `covariance_frame`. The covariance is rotated from the fusion datum's ENU frame into the target's local frame via the convergence angle γ = Δlon·sin(mean_lat) (a rotation between two ENU frames, **not** an axis relabel); the NED drain additionally permutes the axes to north-first. For tracks within the 30 km auto-datum recenter horizon the rotation is < 0.5°; the σ magnitudes match the datum-ENU values within numerical precision.
- Extract uncertainties per the frame: for **ENU**, σ_east = √(cov[0,0]), σ_north = √(cov[1,1]); for **NED**, σ_north = √(cov[0,0]), σ_east = √(cov[1,1]). These are physical meters at the target's geodetic location.

## Velocity

- **`sog_m_per_s`**: Speed over ground, m/s, ≥ 0.
- **`cog_deg`**: Course over ground, degrees true (clockwise from true north), ∈ [0, 360). Undefined when sog < 0.01 m/s (see stationary singularity below).
- **`sigma_sog_m_per_s`, `sigma_cog_deg`**: 1σ uncertainties derived from the velocity covariance via the standard polar Jacobian. σ_cog is in degrees.
- **`is_valid`**: true only when velocity has actually been observed **and** its covariance is usable. Concretely, both drains require all of: the track has ≥ 4D state (position + velocity); `track.velocity_observed` is true (≥ 1 update past initiation, so a pure init-prior velocity never reports valid); and the 2×2 velocity covariance is finite and positive-definite (`v_cov(0,0) > 0` **and** `det > 0`). If any of these fails → `is_valid = false` — i.e. a 2D-only state, a velocity that has not yet been observed, or a zero/degenerate velocity covariance.

### Stationary tracks

When sog < 0.01 m/s, the COG direction is not meaningful and **both `cog_deg` and `sigma_cog_deg` are zeroed**. The `is_valid` flag remains set based on the caller's signal. Interpretation: "track is confirmed stationary" (velocity=0) vs. "`is_valid=false`" (no velocity information at all). Two distinct operator signals.

## Track metadata

- **`id`**: Stable, monotonically increasing integer. Never reused after a track is deleted. Two successive drains on the same processing cycle return the same id.
- **`status`**: Enum. Tentative (new, < M-of-N confirmed), Confirmed (M-of-N threshold met), Coasting (was Confirmed, now propagated through a detection gap without a fresh update), Deleted (marked for removal, drained one final time).
- **`last_update`**: Timestamp of the most recent measurement that contributed to this track's state.
- **`attributes`**: Optional fields sourced from identity-carrying sensors (AIS, cooperative fleet partners, a remote/VTS feed): `mmsi`, `platform_id`, `heading_deg`, `nav_status`, `vessel_name`, `vessel_type`, `length_m`, `beam_m`. All optional; client should test presence before use. `mmsi` (AIS) and `platform_id` (cooperative fleet identity, numeric) are **parallel** identity keys — a track may carry either or both. `heading_deg` (true heading, deg [0,360)) and `nav_status` (AIS navigational-status code; 1 = at anchor, 5 = moored) are **target-reported kinematics** from an AIS self-report (backlog #20): `heading_deg` fills the "stationary, direction undefined" gap (an anchored vessel points somewhere even when SOG≈0 makes COG meaningless), and `nav_status` is the anchored/moored operator cue. All four are surfaced last-write-wins from the hints of the measurements a track claims, under **both** the PMBM (winning) and MHT trackers (identity was PMBM-empty before R11). None is the fusion key — that is always the internal `id` (invariant 5); identity/attributes are hints.
- **`contributing_sources`**: Vector of source_id strings ("ais", "arpa", "eoir") that have genuinely contributed measurements to this track, cumulative over the track's life and deduplicated (first-seen order). Populated by **all three trackers** — flat, MHT, and PMBM (the deployable) — with the same semantics (no per-tracker asterisk); a sensor that never actually updated the track never appears. For PMBM the set is fed from the F2-truthful claimed-source channel (design spec §14.11, resolved 2026-07-15).

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
  [ 25.00  0.00 ]   (5 m sigma each — isotropic, so both drains agree here)
  [  0.00 25.00 ]
covariance_frame        = Ned   (this dump used toTrackOutputNED)
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
- **No covariance rotation.** `TrackOutput` rotates the position covariance
  from the fusion datum's ENU frame into the target's local frame (a
  kinematic necessity because the ENU frame shifts as own-ship moves; the axis
  ordering is then ENU or NED per the drain chosen). For a charted obstacle,
  the position is a fixed geographic coordinate — it does not move with the
  datum. No rotation is applied or needed; the positional accuracy is encoded
  in `position_uncertainty_m` (see below).

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

---

## BearingWedgeOutput (hazard as a sector)

Canonical drain function: `toBearingWedgeOutput(wedge, anchor)` in
`core/output/BearingWedgeOutput.hpp`. A **third** output type, distinct from both
`TrackOutput` and `StaticHazardOutput`: it represents a hazard that is a
**direction, not a position** — the "never invisible" safety net for a
camera-only contact (a `Bearing2D` stream that cannot initiate a track). Produced
by `BearingWedgeModel` (backlog #17 option 1); plain-English intro in
`docs/learning/28-bearing-wedge-hazard.md`, precise reference in
`docs/algorithms/bearing-wedge-hazard.md`.

### Geometry

- **`apex_lat_deg`, `apex_lon_deg`**: WGS84 geodetic position of the wedge apex —
  own-ship at detection time. Converted from the model's fixed **anchor** ENU
  frame (so a datum recenter does not move it); like `StaticHazardOutput`, no NED
  covariance rotation applies (a wedge has no position covariance).
- **`bearing_true_deg`**: centre bearing, **true** (clockwise from north), in
  `[0, 360)`. (Internally the model carries the `Bearing2D` math convention
  `atan2(dN,dE)`; the output converts to true bearing.)
- **`half_width_deg`**: angular half-width of the sector, `= max(2σ, floor)` where
  σ is the **composed** camera ⊕ heading bearing uncertainty.
- **`max_range_m`** (`std::optional<double>`): the sector's range extent. **Absent
  ⇒ unbounded** — range is genuinely unknown, the defining case of a bearing-only
  contact. Present ⇒ clip the wedge at that range.

### Identity and provenance

- **`hazard_id`**: stable `uint64_t`, unique per physical contact and **never
  reused**. Stays constant while the same contact is continuously reported; a
  contact number that goes quiet and later returns (sensor number-reuse) gets a
  fresh id, not the dead one's.
- **`is_charted`**: always `false` (live-detected, no chart record).
- **`source_id`**: the emitting camera/source id.

### No CPA — and handover by suppression

A wedge has no position or velocity, so **CPA is not computable** for it (unlike a
vessel track). The operator reading is "keep clear along that bearing". A wedge is
**suppressed** from the drained output while a confirmed vessel track occupies its
angular span (the better source has taken over), and **reappears** as soon as no
track does — the claim is recomputed every drain, never latched, so a vessel
crossing the bearing of a still-seen camera contact cannot erase it. Only the
camera going quiet removes a wedge.
