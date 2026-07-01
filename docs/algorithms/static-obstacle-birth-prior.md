# Static-obstacle birth prior

**Algorithm reference.** Corresponds to `StaticObstacleModel::birthSuppression` in
`core/static/StaticObstacleModel.hpp`, `PmbmTracker::birthScale` in
`core/pmbm/PmbmTracker.hpp`, and `StaticHazardEvaluator` in
`core/collision/StaticHazardEvaluator.hpp`.
ADR 0002 (`docs/adr/0002-static-objects-track-vessels-map-environment.md`) is the
scope decision.
See also: [26 — Static obstacles (learning chapter)](../learning/26-static-obstacles.md)
for the plain-English introduction and a zone diagram.

---

## 1. Math

### 1.1 Coordinate frame

Charted obstacles are stored in WGS84 geodetic (`StaticObstacle::position`).
At startup — and whenever the datum recenters — `StaticObstacleModel` converts
each obstacle position to ENU metres via `datum.toEnu(position)` and caches
the result. All proximity queries during a processing cycle operate in the same
ENU frame as the tracker; no geodetic arithmetic happens on the hot path.

### 1.2 Per-obstacle suppression ramp `c(d)`

Let `d` be the Euclidean distance in ENU metres from the birth-candidate position
to obstacle `i`'s cached ENU centre:

```
d = ‖ p_enu − o_enu ‖₂
```

Define two radii from the obstacle's fields:

```
R_hard = footprint_radius_m + position_uncertainty_m
R_soft = max(keep_clear_radius_m, R_hard)
```

The per-obstacle suppression value `c_i` is a linear ramp capped to `[0, 1]`:

| distance | c_i |
|---|---|
| `d ≤ R_hard` | `1.0` (footprint core — hard zone) |
| `R_hard < d ≤ R_soft` and `R_soft > R_hard` | `soft_max · (R_soft − d) / (R_soft − R_hard)` (linear buffer ramp) |
| `d > R_soft` | `0.0` (clear water — no suppression) |

The combined `birthSuppression` across all `N` obstacles takes the element-wise
maximum so an obstacle never cancels another one:

```
c_static = max( c_1, c_2, …, c_N )
```

Default `soft_max = 0.9`. The constraint `soft_max < static_obstacle_hard_gate`
(default 0.95) is enforced by configuration: the outer keep-clear buffer is
**always soft-only**. Only the hard footprint interior (`d ≤ R_hard`) can trigger
the hard gate.

### 1.3 Combined birth scale

`PmbmTracker::birthScale` combines the land prior and the static-obstacle prior
**multiplicatively**:

```
scale = (1 − c_land) · (1 − c_static)
```

where `c_land` comes from `ILandModel::clutterPrior` and `c_static` from
`IStaticObstacleModel::birthSuppression`, both evaluated at the 2D ENU position
of the birth candidate.

A **hard-drop** is triggered when either prior exceeds its own gate:

```
if (c_land > land_birth_hard_gate [default 0.95])  → return −1 (drop)
if (c_static > static_obstacle_hard_gate [default 0.95]) → return −1 (drop)
```

A return value of −1 means "discard this birth entirely". A return value in
`[0, 1]` multiplies the adaptive birth intensity `λ_birth`, scaling it downward
without eliminating it.

**Safe-by-construction:** when `use_static_obstacle_model = false` (or no model
is wired via `setStaticObstacleModel`), `c_static = 0` and `birthScale` is
identical to the land-only expression. No model wired and no land model wired →
`scale = 1.0`, bit-identical to the pre-Stage-1 baseline.

### 1.4 Why hard-drop only at the footprint, not the full keep-clear ring

The hard gate (`c > 0.95 → drop`) applies only to the footprint interior where
`c_i = 1.0`. Outside that core, `c_i ≤ soft_max = 0.9 < 0.95`, so the
keep-clear buffer is **never a hard gate** — a real vessel loitering in the
keep-clear ring can still birth through repeated detections as the soft-scaled
intensity `(1 − c_static) · λ_birth` accumulates evidence over several scans.
This is the *anchored-vessel protection*: suppress phantom tracks from a fixed
structure, not real vessels passing close to it.

### 1.5 Keep-clear proximity alarm

`StaticHazardEvaluator` performs a **static range check** — not a CPA
calculation — each time `evaluate(own_ship_enu, datum, t)` is called:

```
d_own = ‖ own_ship_enu − o_enu_in_datum_frame ‖₂

Entered  when d_own < keep_clear_radius_m               (and was not inside)
Updated  while d_own < keep_clear_radius_m              (if emit_updates=true)
Exited   when d_own > keep_clear_radius_m · exit_hysteresis (default 1.1)
```

The hysteresis band (`exit_hysteresis > 1.0`) prevents flapping when own-ship
loiters at the boundary. There is no trajectory extrapolation, no TCPA
computation, and no velocity information is required. This is intentional: the
charted-obstacle alarm is about **current proximity to a fixed point**, not the
closest point on a future trajectory. CPA-style trajectory math applies only to
the vessel MTT branch (see `CpaEvaluator`).

---

## 2. Assumptions

1. **Charted position accuracy is bounded by `position_uncertainty_m`.** The
   hard-core radius `R_hard = footprint_radius_m + position_uncertainty_m`
   absorbs survey error in the same way `W_off` absorbs coastline-polygon
   imprecision. If positional error exceeds `position_uncertainty_m`, the hard
   gate can fire next to (rather than at) the real obstacle.

2. **Obstacles are static.** A charted obstacle does not move. If a drifting
   vessel happens to sit where a charted rock is, the model does not
   distinguish them. Vessel-vs-environment disambiguation remains the operator's
   or the AIS layer's responsibility.

3. **`keep_clear_radius_m` encodes the required clearance, not the physical
   size.** The physical footprint is `footprint_radius_m`; the keep-clear radius
   is an operationally set margin larger than the footprint. The soft ramp
   operates between these two boundaries.

4. **`soft_max < static_obstacle_hard_gate`.** The inequality `0.9 < 0.95`
   (default values) must hold. If this is violated by config, the outer
   buffer becomes a hard gate and a real vessel in the keep-clear ring would
   be suppressed rather than softened.

5. **A real vessel gets repeated detections.** The soft ramp allows a real
   vessel passing through the keep-clear ring to accumulate enough existence
   evidence across several radar scans to birth and confirm a track. A single-scan
   detection at `d ≈ R_hard` (where `c_static ≈ 0.9`) starts a Bernoulli at
   `0.1 · λ_birth` — weak but not zero. A single-scan birth at `d = R_soft`
   starts at exactly zero (the ramp reaches zero at `R_soft`).

6. **Datum recenters are wired.** `StaticObstacleModel` implements
   `IDatumChangeSink`. When own-ship moves more than 30 km and the datum
   recenters, the ENU cache is rebuilt deterministically. Without this wiring,
   obstacle positions drift in ENU space as the datum shifts.

---

## 3. Rationale

### 3.1 Why a separate port, not a subclass of `ILandModel`

ADR 0002 decision 4: charted discrete hazards are semantically different from
the coastline. The coastline is a large region you never navigate behind; it maps
cleanly to a signed-distance ramp from a polygon. A rock or a pillar is a discrete
object in navigable water with its own footprint, positional uncertainty, depth,
type, and keep-clear radius. Folding it into the land polygon would create a tiny
no-birth zone around each obstacle (the anchored-vessel trap) and lose all
attributes (depth, category, AtoN status) that a polygon cannot carry. The
`IStaticObstacleModel` port exists so the tracker's birth logic calls exactly
one interface per concern.

### 3.2 Why multiplicative combination `(1 − c_land)(1 − c_static)`

The two priors suppress births for orthogonal reasons: the land prior fires near
the coastline, the obstacle prior fires near discrete hazards in open water. The
two regions do not generally overlap — a rock in navigable water is not on land.
The multiplicative form is the correct independence assumption: being near a rock
**and** near the shore suppresses births more than either alone, without requiring
explicit joint modelling. If either prior is zero (one model not wired), the
expression reduces to the other, so enabling or disabling a model never affects
the other path.

### 3.3 Why hard-gate only at the footprint interior

The footprint interior (`d ≤ R_hard`) is where a real vessel *cannot* be — it
would be inside the physical structure. The hard gate there is correct and safe.

The keep-clear ring is navigable water — a vessel *can* legitimately be there
(passing close, anchoring nearby). Hard-gating that ring would silently suppress
valid tracks, repeating the ADR 0001 no-birth-cliff failure at every charted
obstacle. The soft ramp allows the tracker to weaken births near the hazard
without blocking them outright.

### 3.4 Why geodetic store + ENU cache

navtracker rides own-ship. Charts are in WGS84. The tracker's birth logic works
in ENU metres. The natural solution is to store the ground-truth position
geodetically (so it survives datum recenters as a stable quantity) and convert to
ENU on first use (and on each datum recenter). This mirrors the approach for the
coastline geometry. The "fixed geographic frame" shortcut — precompute once and
reuse — works for a shore-mounted radar (Herrmann et al. 2025, arXiv:2508.16169
(local copy: `docs/references/2508.16169v1-herrmann-2025-hybrid-tbd-coastal-radar.pdf`);
their land mask is in the radar body frame, valid because the radar does not move).
navtracker moves, so the ENU cache must be rebuilt when the datum shifts. See
design spec §14.10 "Caveat — we move."

### 3.5 Why a proximity alarm, not a CPA

CPA requires a velocity estimate for the obstacle: `TCPA = −(r · v_rel) / |v_rel|²`.
A charted rock has no velocity. Estimating CPA to a static point degenerates to
`d_current / |v_own|`, which is just the current range scaled by speed — the
same information a proximity check gives, with more arithmetic. The `StaticHazardEvaluator`
uses the simpler and more honest formulation: "are we inside the keep-clear ring?"
CPA-to-vessel math remains in `CpaEvaluator` for the dynamic track branch.

---

## 4. Ways to improve / what to test next

1. **Stage 1b — reframe the clutter-map primitive (open).** The `ClutterMapDetectionModel`
   accumulates persistent spatial returns. Wiring it to emit labelled "static occupancy"
   via dominant-hypothesis `1 − r` from PMBM (the parked design, ADR 0002 §3) turns
   the existing code into a first live-static estimator without a new subsystem.
   The output becomes a `StaticHazardOutput` with `is_charted = false`.

2. **Stage 2 — Bayesian/evidential occupancy grid.** A Dempster-Shafer / DOGMa
   (Nuss et al. IJRR 2018, arXiv:1605.02406) occupancy grid — free / occupied-static /
   occupied-dynamic / **unknown** — is the principled replacement for the crude λ_C
   clutter map. The explicit **unknown** state is the honest fix for the ADR 0001
   no-birth cliff: carry "I have not observed this cell" rather than "it must be clear."
   DOGMa is in the same RFS family as PMBM, so the two branches are mathematically
   compatible siblings.

3. **Stationary IMM mode.** A low-PSD "stopped" mode (or a zero-velocity pseudo-
   measurement gated on |v| < threshold) would keep a moored vessel's covariance
   tight and give CPA a clean stationary-hazard flag. Today the IMM has CV + CT modes
   only; a moored track holds but with inflated covariance.

4. **ENC / AIS AtoN ingest.** `StaticObstacle` is schema-aligned to the S-57/S-101
   ontology (`CATOBS`, `WATLEV`, `VALSOU`, real/virtual AtoN). The next step is
   an ENC adapter (IHO S-57 parser → `StaticObstacle` objects) and an AIS Message 21
   adapter (AtoN position reports → `StaticObstacle` with `aton = Virtual`).

5. **A real fixture with charted obstacles.** There is no measured A/B benchmark
   improvement yet. None of the current test fixtures (AutoFerry, philos) contains
   charted static hazards, so the birth-prior effect is untested end-to-end on real
   data. A fixture with known rocks/buoys, ground-truth vessel tracks, and an OSPA/
   GOSPA before/after comparison is the right follow-up measurement.

6. **Sensor-aware near-shore birth discriminator (ADR 0001 A3).** EO/IR or AIS
   corroboration of a near-shore return that overlaps a charted obstacle can
   distinguish a real vessel from the structure. Without this, the current ramp
   suppresses both equally in the buffer zone. Build when EO/IR or AIS near shore
   is available in a fixture.
