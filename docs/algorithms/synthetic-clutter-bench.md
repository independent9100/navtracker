# Synthetic Multi-Target + Shore-Clutter Bench — Algorithm Reference

Follows the project documentation standard:
Math / Assumptions / Rationale / Ways to improve.

Companion documents:

- Design spec: [`docs/superpowers/specs/2026-06-30-pmbm-synthetic-clutter-bench-design.md`](../superpowers/specs/2026-06-30-pmbm-synthetic-clutter-bench-design.md).
- Plain-language introduction to the land-clutter prior being tested: [learning §25 — Suppressing tracks on land](../learning/25-land-clutter-prior.md).
- Algorithm-level reference for the prior itself: [pmbm-design.md §10](pmbm-design.md#10-land--coastline-clutter-prior).

This document covers the *bench infrastructure*: the parametric geometry generators and the
stationary shore-clutter injector. It is not a new tracking algorithm.

---

## 1. Math

### 1.1 Parametric geometry generators

All three builders produce a `Scenario` (measurement list + truth list) in ENU metres. Each
target follows a **constant-velocity (CV)** motion model. Truth position at time `t` for target
`i` is:

```
p_i(t) = start_i + v_i · t
```

Measurements are noisy samples of truth:

```
z_i(t) = p_i(t) + ε,    ε ~ N(0, σ²_pos · I₂)
```

where `σ_pos` is the configurable position noise standard deviation (passed as
`pos_noise_std_m`). The RNG is seeded: `std::mt19937(seed)`. Truth ids are assigned
`1..N` in a documented, deterministic order.

**`buildParallelLaneScenario(n_targets, lane_spacing_m, start, velocity, ...)`**

N targets on parallel lanes, all sharing the same heading. The perpendicular direction to
`velocity` is `perp = (-v_y, v_x) / |v|`. Lane `i` (0-indexed) starts at:

```
start_i = start + i · lane_spacing_m · perp
```

All targets have the same velocity. Spacing shrinkage (small `lane_spacing_m`) stresses
track resolution and merge failures.

**`buildCrossingAngleScenario(crossing_angle_deg, speed_mps, crossing_point, ...)`**

Two targets both arrive at `crossing_point` at the mid-time of the scenario. Target A heads
along +x at `speed_mps`. Target B heads at angle `θ = crossing_angle_deg` from +x at the
same speed:

```
v_A = (speed, 0)
v_B = (speed · cos θ, speed · sin θ)
start_A = crossing_point - v_A · t_mid
start_B = crossing_point - v_B · t_mid
```

Truth is emitted in (A, B) order (ids 1, 2) each scan. Sweeping `θ` over 30/60/90° probes
angle-dependent association failures: near-head-on (180°) and near-parallel (small θ) are
the worst cases for track confusion.

**`buildConvoyScenario(n_targets, gap_m, speed_mps, overtaker_speed_mps, ...)`**

N convoy members on the x-axis at `speed_mps`, each starting at `x = -i · gap_m` (i =
0..N-1). One overtaker starts behind the convoy at x = `-(N · gap_m + 100)`, offset
`25 m` laterally, moving at `overtaker_speed_mps`. Truth: convoy members first (ids
1..N), then the overtaker (id N+1) each scan. This stresses in-line association: targets
on the same heading with small lateral separation and a faster approaching target.

### 1.2 Stationary shore-clutter injector

The injector is `addShoreClutter(base, datum, clutter_enu_points, detection_prob,
pos_noise_std_m, seed)`.

Let `C = {c₁, …, c_K}` be the set of K fixed ENU positions (the `clutter_enu_points`).
Let `T = {t₁, …, t_S}` be the set of S distinct scan timestamps already in `base`.

For each scan `t_s` and each clutter point `c_k`, the injector draws:

```
u_{s,k} ~ Uniform(0, 1)     (seeded Bernoulli)

If u_{s,k} < P_D:
    z_{s,k} = c_k + ε_{s,k},   ε_{s,k} ~ N(0, σ²_shore · I₂)
    emit Measurement(position=z_{s,k}, time=t_s,
                     source_id="sim_shore", kind=ArpaTtm)
```

No `TruthSample` is created for any clutter emission. GOSPA and cardinality metrics
therefore count each shore-clutter track as an over-count (false alarm).

**The defining property: same nominal positions every scan.** The set C is fixed. Only the
noise draw `ε_{s,k}` changes between scans. This is the controlled analogue of real shore
clutter (buildings, piers, walls are always there) and contrasts with the existing
`buildClutterCrossingScenario`, which redraws K random positions from a uniform spatial
distribution each scan. A tracker exposed to persistent fixed returns will eventually
accumulate enough evidence to birth a Bernoulli at each position — exactly the phantom-track
failure the land model must prevent.

### 1.3 Synthetic coastline (`buildSyntheticShore`)

The function constructs a `SyntheticShore` struct containing three things:
`CoastlineGeometry`, `geo::Datum`, and `clutter_enu_points`.

**Shoreline geometry.** A rectangular land block occupies
`shore_y_m ≤ y ≤ shore_y_m + land_depth_m`, `−extent_m ≤ x ≤ extent_m`. One
rectangular pier of half-width `pier_width_m/2` protrudes from the shoreline
`pier_length_m` into the water at x = 0. The ENU outline is an 8-vertex polygon:

```
(−extent, shore_y) → (−pier_w/2, shore_y) → (−pier_w/2, shore_y − pier_len)
→ (pier_w/2, shore_y − pier_len) → (pier_w/2, shore_y) → (extent, shore_y)
→ (extent, shore_y + land_depth) → (−extent, shore_y + land_depth)
```

This ENU outline is converted vertex-by-vertex to geodetic (WGS84 lat/lon) via
`datum.toGeodetic(...)` and stored as a `LandPolygon` outer ring inside a
`CoastlineGeometry`. The `CoastlineGeometry` computes the signed-distance ramp as
documented in [pmbm-design.md §10.1](pmbm-design.md#101-math).

**Clutter positions.** `n_clutter` points are placed at ENU y = `shore_y_m +
0.5 · land_depth_m` (half-way into the land block — the hard-gate plateau region, where
`c > 0.95`), spread evenly across x in `[−0.8 · extent_m, 0.8 · extent_m]`.

**Fictitious datum.** A concrete geodetic origin (42.35°N, 71.05°W — Boston harbor area in
the bench fixture) is assigned as the `geo::Datum`. All ENU↔geodetic conversions use this
single datum, ensuring that the clutter generation and the land model's
`clutterPrior(enu_xy)` query path are both anchored to the same coordinate origin.

### 1.4 In-memory land model hook (`ScenarioRun::syntheticCoastline`)

The `ScenarioRun` base class adds one optional override:

```cpp
virtual std::optional<CoastlineGeometry> syntheticCoastline() const {
    return std::nullopt;
}
```

Shore-clutter scenario subclasses (`ShoreClutterOpenScenarioRun`,
`ShoreClutterNearShoreScenarioRun`) override this to return the geometry from
`makeBenchShore()` — the same object that generated the clutter positions in
`generate(seed)`. When `Sweep.cpp` runs with `use_land_model = true`, it checks this
hook: if present, it constructs a `CoastlineModel(geometry, scenario.datum)` in memory
instead of loading a GeoJSON file. One shoreline object, used twice (for clutter injection
and for the live land query), with no I/O and no file-system dependency.

---

## 2. Assumptions

1. **Perfect truth.** Builders generate ground-truth positions analytically from CV
   equations. There is no truth-measurement coupling error, no AIS dropout, and no
   MMSI ambiguity. GOSPA, OSPA, and cardinality metrics are therefore exact — not
   estimated from imperfect AIS-derived truth.

2. **Seeded determinism.** Every source of randomness uses `std::mt19937(seed)` with a
   seed supplied by the caller. Same seed → identical measurement sequence. The bench
   harness explicitly tests that two runs with the same seed produce identical output
   (CLAUDE.md invariant 4).

3. **ENU consistency via fictitious datum.** Shore-clutter scenarios need a geodetic
   query to be consistent with ENU generation. A per-scenario datum (a concrete lat/lon
   origin) is used for all ENU↔geodetic conversions, both in clutter generation and in
   the land model's prior query. By construction, a clutter point well inland returns
   `c > 0.95` from the scenario's own `CoastlineModel`.

4. **Shore clutter is deep-inland (hard-gate plateau).** Clutter points are placed at
   `y = shore_y_m + 0.5 · land_depth_m`, putting them squarely in the ramp's `c = 1.0`
   region (well past the hard-gate threshold of 0.95). A near-waterline clutter band is
   covered by the `shore_clutter_nearshore` scenario, which places a **real** target at
   `y = shore_y_m − 10 m` (10 m offshore) with `c ≈ 0.4` — the soft-ramp region.

5. **No RCS or multipath modelling.** Shore-clutter returns are a fixed-position point
   process, not a physics-based sensor simulation. The injected returns are statistically
   stationary in position (modulo noise); in reality, multipath creates
   range-correlated, angle-spread returns. This is a deliberate scope boundary.

6. **Single scan rate.** All builders emit one measurement per target per integer-second
   timestamp. There is no multi-rate or staggered-scan complexity. The land model query
   is per birth candidate, not per scan.

---

## 3. Rationale

### 3.1 Fixed-position shore clutter, not uniform-Poisson

The existing `buildClutterCrossingScenario` generates uniform-Poisson false alarms:
K positions are drawn uniformly at random over a bounding box each scan. This models a
sensor environment where clutter is spatially random — airborne chaff, rain clutter,
intermittent reflectors.

Shore clutter is fundamentally different. Buildings, piers, and breakwaters are at fixed
lat/lon. They appear at the **same ENU position every scan**. This persistence is exactly
what fools a PMBM tracker: after enough scans, the Bernoulli at each shore position
accumulates posterior evidence and crosses the confirmation threshold — a phantom track.

Uniform-Poisson clutter at new positions each scan does not reproduce this failure mode.
A fixed-position injector does.

### 3.2 In-memory shared land mask enables a clean A/B

To measure the land model's effect cleanly, the land query must use the **same shoreline
that seeded the clutter**. If the clutter is generated from one geometry and the land
model queries a different file (e.g. the philos `boston.geojson`), misalignment of
polygon coordinates could create false positives or false negatives in the test.

The in-memory hook (`syntheticCoastline()`) enforces this by construction: one
`buildSyntheticShore()` call returns both the clutter points and the geometry. The bench
passes both to the Sweep in the same run. There is no GeoJSON file, no I/O, and no
possible mismatch.

### 3.3 Synthetic shoreline rather than the philos GeoJSON fixture

The philos dataset (`boston.geojson`) has three limitations as a bench fixture:

- **Imperfect AIS-derived truth.** Ground truth is inferred from AIS transponder
  positions, which have dropout, MMSI collisions, and positional error. GOSPA scores
  against philos truth are lower-bounded by these imperfections.
- **Anchored-ship ambiguity.** Philos has vessels moored near the Boston waterline.
  We cannot verify from AIS alone whether a near-shore detection is a real anchored
  vessel or a shore return. The "inland-only hard gate must not delete a near-shore
  vessel" invariant cannot be confirmed — only inferred statistically.
- **No controllable target geometry.** The philos targets are at fixed historical
  positions. We cannot sweep crossing angle, lane spacing, or convoy gap.

A synthetic shoreline with perfect truth resolves all three: we know exactly which
returns are shore clutter (we placed them), we place one real vessel 10 m offshore
deliberately (the `shore_clutter_nearshore` scenario), and we control every geometric
parameter.

### 3.4 Reuse of existing metrics and configs

The bench adds no new tracker behaviour. Sweeping `imm_cv_ct_pmbm_coverage` vs
`imm_cv_ct_pmbm_coverage_land` on the new scenarios directly measures the land model's
effect on GOSPA, cardinality error, and false-track count. The success criterion is
unambiguous: `land ON` must drive shore-clutter cardinality error to near zero without
dropping the near-shore real target.

---

## 4. Ways to improve / what to test next

**RCS and multipath-modelled shore returns.**
Real shore clutter is not a neat point process. A pier at 1 km range generates a spread
of returns across multiple range-azimuth bins due to multipath and reflection geometry.
An RCS-weighted cluster model (position = fixed nominal + range-correlated offset) would
better represent the challenge a tracker faces in harbour approaches. This requires a more
elaborate injector but does not change the tracker — same `ILandModel` interface.

**Range-bearing injector.**
Current shore clutter is injected as `Position2D` (ENU x/y). Real radar produces
`RangeBearing2D` returns. Injecting shore clutter in the range-bearing domain and
projecting via the radar adapter would stress the full measurement-to-ENU pipeline and
expose any heading-covariance artefact near the waterline.

**Near-shore real-target offshore-distance sweep.**
The `shore_clutter_nearshore` scenario places a real target at exactly 10 m offshore.
A parametric sweep of offshore distance from −50 m (inland, hard-gate should fire) to
+150 m (open water, no suppression) would map the soft-ramp survival boundary:
at what offshore distance does the real target's lifetime drop below an acceptable ratio?
This would characterise the `W_off` / `W_in` sensitivity without philos ambiguity.

**Multi-pier harbour geometry.**
The current shoreline has one pier. A multi-pier geometry (e.g. two parallel piers
forming a berth slot) would create clutter returns inside the berth and near-shore vessel
targets between the piers — the most challenging maritime geometry for the spatial prior.
This is achievable by adding more notches to the `buildSyntheticShore` polygon.

**Coupling with the clutter-map model.**
The `ClutterMapDetectionModel` (chapter 13) can learn a spatial clutter intensity from
observed false tracks. A future experiment would run the shore-clutter bench with both
the static coastline prior AND the learned clutter map active, to measure how much the
learned map adds to the static prior (or whether the static prior alone is sufficient at
the soft-ramp boundary).
