# Sensor Adapters

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec section 5,
`docs/sensors/sensor-reference.md`.

See also (plain-English introductions in the learning series):
[10 — Measurements, frames & time](../learning/10-measurements-frames-time.md),
[21 — Sensor registration bias](../learning/21-sensor-registration-bias.md).

## 1. Polar → ENU projection (`projectRangeBearingToEnu`)

**Math.** `east = own.east + r·sin β`, `north = own.north + r·cos β`.
`J = ∂(east, north)/∂(r, β) = [[sin β, r·cos β], [cos β, −r·sin β]]`.
`Σ_xy = J · diag(σ_r², σ_β²) · Jᵀ`.

**Assumptions.** Bearing is *true* (north-CW); own-ship pose known at the
measurement time; own-ship position uncertainty NOT folded in; flat-Earth ENU.

**Rationale.** Standard polar-to-Cartesian linearization; captures the
anisotropic ellipse (long along the LOS, wide cross-range as `r·σ_β`).

**Ways to improve / test next.** Time interpolation of own-ship pose; fold
own-ship position/heading uncertainty into Σ_xy; range-dependent bias.

## 2. AisAdapter

**Math/Logic.** `value = ENU(lat, lon, datum)`; `R = σ²·I₂` with
`σ = 10 m` (high accuracy / DGNSS) or `σ = 30 m` (low accuracy).

**Assumptions.** Caller pre-decodes AIVDM; alt ignored (surface vessel).

**Rationale.** Cooperative + identity → cleanest path; MMSI becomes a hint.

**Ways to improve / test next.** Plausibility gate; `PositionVelocity2D`
using SOG/COG; data-driven `σ` from reported accuracy.

## 3. OwnShipNmeaAdapter (GGA, HDT)

**Math/Logic.** GGA `ddmm.mmmm` → decimal degrees (`dd + mm.mmmm/60`),
sign by N/S, E/W. HDT `heading,T` → degrees true. Caller supplies a full
`Timestamp` per ingest.

**Assumptions.** One sentence per call; non-GGA/HDT ignored; quality flag
not gated.

**Rationale.** Minimal common baseline; works with any nav feed that emits
position and heading.

**Ways to improve / test next.** Quality-flag gating, RMC parsing, full
UTC→Timestamp parsing, multi-frame combining.

## 4. ArpaAdapter (TTM, TLL)

**Math/Logic.** TTM: distance in `N/K/S` units (NM/km/sm) → meters; bearing
made true if `R` by adding `own_ship_heading_true`; then
`projectRangeBearingToEnu` with `(σ_r, σ_β) = (50 m, 1°)` defaults
(sensor reference §2). TLL: lat/lon → ENU via `Datum`; isotropic
`R = σ²·I` with `σ = 50 m` (own-ship error already folded in by the
radar — sensor reference §2). Sensor track id → `hints.sensor_track_id`
in both.

**Assumptions.** Own-ship pose available for TTM; default `σ`s are
configurable per radar; bearing field correctly tagged `T`/`R`.

**Rationale.** Covers both ARPA target sentences with consistent ENU
output; shares the projection helper.

**Ways to improve / test next.** Per-radar calibrated `(σ_r, σ_β)`; fold
own-ship pose uncertainty into TTM covariance; revisit measurement-vs-
track-level fusion for ARPA tracks if biased.

## 5. EoIrAdapter (bearing + range)

**Math/Logic.** `bearing_true = bearing_relative + own_ship_heading_true`;
then `projectRangeBearingToEnu` using the supplied `(σ_r, σ_β)`.

**Assumptions.** Camera boresight aligned with own-ship bow (no PTZ for the
baseline); range and bearing both available; own-ship pose available.

**Rationale.** Symmetric to ARPA TTM; reuses projection helper. Useful when
EO/IR is equipped with a rangefinder (stereo / lidar / laser).

**Ways to improve / test next.** Bearing-only flow (needs UKF/particle on
the EKF side, or cross-sensor constraint); PTZ + mount calibration;
classification → attribute fusion.
