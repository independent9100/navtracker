# Live static-occupancy layer (`LiveOccupancyModel`)

**Algorithm reference.** Corresponds to `LiveOccupancyModel` in
`core/static/LiveOccupancyModel.{hpp,cpp}`, the feed port
`ports/ILiveOccupancyFeed.hpp`, and the tuning struct `LiveOccupancyParams`
(in `LiveOccupancyModel.hpp`). Wired into the tracker as *two faces on one
object*: `IStaticObstacleModel` (birth suppression + hazard output, via
`PmbmTracker::setStaticObstacleModel`) and `ILiveOccupancyFeed` (the per-scan
accumulation feed, via `PmbmTracker::setLiveOccupancyFeed`).

This is the shipped **Stage 1b** live static estimator: it learns persistent,
spatially extended structure (piers, breakwaters, shoreline, uncharted fixed
obstructions) from the PMBM clutter feed and suppresses vessel *births* there —
and only there — emitting each learned structure region as an uncharted static
hazard so nothing persistent is suppressed into nothing (the ADR 0002
"presence over classification" invariant).

Design of record:
`docs/superpowers/specs/2026-07-01-honest-static-occupancy-stage1b-design.md`
(1b-i) and
`docs/superpowers/plans/2026-07-03-stage1b-ii-structure-detector.md` (1b-ii:
coarse-grid detector, coverage-aware decay, chart + camera corroboration).
ADR 0002 (`docs/adr/0002-static-objects-track-vessels-map-environment.md`) and
its 2026-07-03 amendment are the scope decision.

See also:
- [27 — Live static occupancy (learning chapter)](../learning/27-live-static-occupancy.md)
  for the plain-English introduction and diagrams.
- [Static-obstacle birth prior](static-obstacle-birth-prior.md) — the charted
  sibling that shares the `IStaticObstacleModel` / `birthScale` seam this model
  plugs into. §4 item 1 of that doc tracked this as future work; it is now DONE.
- [PMBM design §8 — Coverage / Visibility Channel](pmbm-design.md#8-coverage--visibility-channel-isensoractivity)
  for the `CoverageSector` descriptor the coverage-aware decay consumes.

---

## 1. Math

### 1.1 Coordinate frame and grid

Everything is in the **ENU local tangent plane** (metres). The grid is anchored
to a **fixed datum** at construction (`anchor_`); the tracker's *current* datum
(`current_`) can drift as own-ship moves. Incoming ENU positions (expressed in
the current datum) are transformed to the anchor frame at accumulate/query time:

```
toAnchorEnu(p) = p                                   if current_ == anchor_   (identity fast path)
               = anchor_.toEnu( current_.toGeodetic(p) )   otherwise
```

The identity fast path is exact and drift-free for the single-fixed-datum case
(the bench). Cells are integer index pairs over the anchor frame:

```
cellOf(p)     = ( floor(p.x / cell_size_m), floor(p.y / cell_size_m) )
cellCenter(c) = ( (c.i + 0.5)·cell_size_m, (c.j + 0.5)·cell_size_m )
```

Only touched cells are stored (an ordered `std::map` → deterministic iteration
and component labeling). Times are Unix seconds; camera bearings are radians.

### 1.2 Per-scan feed and EWMA persistence

Each scan the model receives, via `observe()`, one `ScanObservation` bundle per
(sensor, detection-model): a list of **clutter positions** with per-position
weights. The weight is the PMBM dominant-hypothesis mass `1 − r`: unclaimed
returns carry weight `1.0`, weakly-claimed returns carry `1 − r`, and returns
claimed by a near-certain track (`r ≈ 1`) are excluded (weight `0`). An empty
weight vector means weight `1.0` per return; non-positive weights are skipped.

Let `α = ewma_alpha`. The update, in order, per scan:

1. **Coverage-aware decay.** For every stored cell, decay
   `p ← (1 − α)·p` **only if the cell was observable this scan** — i.e. inside
   some bundle's `CoverageSector` footprint. If no bundle carries a valid
   footprint, full coverage is assumed and every cell decays (the legacy /
   synthetic behaviour, bit-identical). Absence of returns where no sensor
   looked is *not* evidence of vacancy — this is what separates a departed
   vessel (returns cease while still in coverage → decays out) from a cell that
   merely left coverage (frozen).
2. **Touch weights.** For each cell touched this scan, take the **largest**
   feed weight `w_max` reaching it (order-independent, so replay is
   deterministic).
3. **EWMA add.** For each touched cell: `p ← p + α·w_max` (decay from step 1
   already applied), i.e. the recursion is `p ← (1 − α)·p + α·w_max` for a
   touched, observable cell.
4. **Prune.** Drop any cell with `p < erase_floor` to bound memory; drop camera
   streaks whose cell no longer exists.

### 1.3 Structure classification (persistent AND extended)

A cell is **persistent** when `p ≥ bar`. In the default (absolute-bar) mode
`bar = persistence_bar`. In **clutter-adaptive** (detector) mode the bar is
raised above the estimated uniform-clutter background:

```
bar = max( persistence_bar, clutter_reject_factor · median{ p : cell ∈ grid } )
```

The median over live cells estimates the clutter background because the feed is
clutter-dominated; structure sits far above its *own* clutter density even where
absolute persistence does not separate it, so dense clutter is rejected relative
to itself (no death-spiral) while sparse structure still classifies.

Persistent cells are grouped by **4-connected flood fill** (deterministic,
key-sorted BFS). A component is **structure** iff its cell count
`≥ extended_cells_min`. Compact persistent regions (a single anchored boat's
watch circle) fall below the extent floor and are **not** structure — never
suppressed.

### 1.4 Emitted hazards and derived birth suppression

Each structure component becomes exactly **one** synthesized `StaticObstacle`:

```
centroid  = mean of cellCenter(c) over the component
conf      = max over cells of min(1.0, p)            # peak persistence, capped
reach     = max over cells of ‖ cellCenter(c) − centroid ‖
footprint_radius_m  = reach + 0.5·cell_size_m        # encloses every cell
keep_clear_radius_m = footprint_radius_m + suppression_radius_m
category = Obstruction ; water_level = AlwaysAboveWater ; source_id = "live_occupancy"
```

`birthSuppression(q)` is **derived entirely from the emitted hazard set** — a
soft ramp over the obstacles, evaluated in the anchor frame:

```
d = ‖ obstacle_center − q ‖
ramp = 1                          if d ≤ footprint_radius_m        (hard core)
     = (kc − d) / (kc − fr)       if fr < d ≤ kc  (kc = keep_clear, fr = footprint)
     = 0                          otherwise
c = max over obstacles of ( suppression_max · conf · ramp )
```

Because suppression is a function of the emitted hazards, `c > 0` implies `q`
lies inside some emitted hazard's keep-clear ring — **suppression into nothing
is structurally impossible** (the ADR 0002 conservation invariant). The cap
`suppression_max = 0.9 < 0.95` (the tracker's `static_obstacle_hard_gate`)
keeps live suppression **soft-only**: even a mislabeled large stationary vessel
is never hard-dropped from the birth channel. The combined birth scale and
hard-gate semantics are those of the shared seam — see
[static-obstacle-birth-prior.md §1.3](static-obstacle-birth-prior.md#13-combined-birth-scale).

### 1.5 Chart corroboration (label only)

If `setChartedStructure()` has been fed a charted structure point cloud
(cached in the anchor frame), an emitted hazard is flagged **corroborated** when
a charted point lies within `chart_corroboration_radius_m` of its centroid.
This is a **label only** — suppression and hazards are unchanged. It feeds
operator confidence and the eviction policy (§1.7): an uncorroborated pin is the
eviction candidate; a corroborated one is held.

### 1.6 Camera corroboration (observed-empty streaks)

`observeCamera(frame)` advances a per-cell **observed-empty streak** from one
live camera frame. For each stored cell, its bearing from the camera is
`atan2(dN, dE)` (absolute ENU math bearing). A cell:

- **out of the FOV** (`|remainder(brg − fov_center, 2π)| > fov_half_width`) is
  left untouched — absence outside the FOV is never evidence of absence;
- **in FOV with a detection within `match_tolerance_rad`** of its bearing has
  its streak reset (something is there);
- **in FOV with no matching detection** has its streak extended (or started).

A hazard whose centroid cell has been continuously observed-empty for
`≥ camera_empty_sustain_s` is flagged `camera-observed-empty`. Label only until
eviction is enabled.

### 1.7 Camera eviction (behaviour, opt-in)

When `evict_camera_empty = true`, a pre-pass in `observe()` (before
reclassification) **spends** (erases) the persistence of any structure cell that
is (a) in a **chart-unconfirmed** component, (b) has a **matured** streak
(`span = last − first ≥ camera_empty_sustain_s`), and (c) is **recent**
(`now − last ≤ camera_empty_recency_window_s`). Evidence precedence: a
chart-confirmed component is held regardless of the camera.

Erasing (not merely dropping the hazard) is necessary because coverage-aware
decay **freezes** the persistence of an unobserved departed cell; dropping only
the hazard would let the frozen persistence re-emit it next scan (a blinker), so
eviction resets the cell to accumulate from fresh returns. Evidence is keyed by
**cell** and accrues even while the cell is off-stage, so eviction fires the
instant a flickering cell re-enters the structure set. Conservation-safe by
construction: suppression is re-derived from the post-eviction persistence, so
lifting suppression can only free a birth, never orphan one.

### 1.8 Datum re-anchoring

`LiveOccupancyModel` implements `IDatumChangeSink`. On
`onDatumRecentered(old, new)` it sets `current_ = new` — the grid data
(persistence, streaks, charted points) stay attached to geography in the fixed
anchor frame, and subsequent `toAnchorEnu` calls re-express incoming
current-datum ENU into the anchor frame. Wire the model as a datum sink (see
CLAUDE.md "Auto-datum pattern") or its ENU cache silently goes stale after a
recenter.

---

## 2. Assumptions

1. **The feed is the PMBM dominant-hypothesis `(position, 1 − r)` clutter feed**
   — unclaimed + weakly-claimed returns, with near-certain-track returns already
   excluded (weight 0). The layer never sees raw radar cells; it operates on the
   same measurement positions the tracker already consumes, inside the domain
   contract. It deliberately does **not** touch `λ_C` / `p_D` (that indiscriminate
   coupling is what the Stage 1b design rejects).
2. **Structure is persistent AND spatially extended; a boat's watch circle is
   not.** The extent floor (`extended_cells_min`) is the 1b-i discriminator, set
   high enough that a single anchored boat never qualifies. This is an interim
   geometric proxy with two documented failure directions — a large anchored
   non-AIS ship (extended → wrongly suppressed) and a single compact pile
   (compact → wrongly kept). 1b-ii moves discrimination to corroboration; under
   the ADR 0002 amendment a compact anchored boat *may* legitimately be shown as
   a keep-clear hazard, provided it recovers to a moving track once underway.
3. **Uniform sea clutter never becomes persistent + extended**, so it is never
   suppressed (no `dense_clutter` regression). In detector mode the
   clutter-adaptive bar additionally rejects *dense* clutter relative to its own
   estimated background.
4. **A departed vessel that leaves coverage keeps its pin frozen, not decayed.**
   Coverage-aware decay only forgets a cell that was actually observed empty; a
   truly-departed vessel is resolved by camera eviction (or chart abstention),
   not by radar absence alone. Permanently-unobserved cells never decay — an
   unobserved-decay floor for hour-long steady state is deferred (Stage 2 /
   Layer-2).
5. **Recovery is inherent in EWMA decay.** When a suppressed region's returns
   cease (a mover gets underway) while it is still observed, its cells forget
   below the bar within a bounded number of scans, the hazard drops from the
   set, and suppression lifts so the mover births normally. `ewma_alpha` trades
   recovery latency against structure stability.
6. **Datum recenters are wired** (`IDatumChangeSink`), or the anchor-frame cache
   drifts after a > 30 km own-ship move.

---

## 3. Rationale

### 3.1 Why a dedicated feed port, not `ClutterMapDetectionModel`

The Stage 1b spike proved that feeding a persistence map straight into the
detection channel (`λ_C`) fixes the structured philos over-count but **regresses
uniform clutter** — it suppresses *any* persistent return, including real boats
and targets buried in noise. `ILiveOccupancyFeed` is therefore a separate port
that influences the **birth channel only**, via the `IStaticObstacleModel`
face. Wiring an occupancy feed cannot couple the learned map into association.
The grid is also datum-stable (geodetic anchor + ENU cache), unlike the
body-relative `ClutterMapDetectionModel`, because navtracker moves (design spec
§14.10 "we move").

### 3.2 Why EWMA persistence rather than a full evidential grid

The learned quantity is a scalar EWMA per cell — cheap, order-independent,
deterministic, and directly interpretable as "how reliably has structure-like
mass touched this cell recently". It is *not* a Bayesian evidential grid. The
principled Stage 2 replacement is a **Dempster-Shafer / DOGMa** occupancy grid
(Nuss et al., IJRR 2018, arXiv:1605.02406) carrying four masses —
free / occupied-static / occupied-dynamic / **unknown** — where the explicit
*unknown* state is the honest fix for "I have not observed this cell" (versus
today's decay-toward-empty). DOGMa is in the same random-finite-set family as
PMBM, so the two branches are mathematically compatible siblings. EWMA is chosen
for 1b because it is a single-scalar, provably-conservative first estimator that
ships without a new inference engine; the gap it leaves — no explicit unknown,
no dynamic/static separation, decay-as-forgetting instead of evidence-discounting
— is exactly what Stage 2 closes.

### 3.3 Why suppression is derived from the emitted hazards

Making `birthSuppression` a pure function of `obstacles()` turns the ADR 0002
conservation invariant ("suppress nothing into nothing") into a structural
property, not a test we hope stays green: `suppression(q) > 0` can only happen
inside an emitted hazard's keep-clear ring. One hazard per component, with a
footprint that encloses every cell, keeps operator output clean while preserving
conservation.

### 3.4 Why extent is interim and corroboration is the real discriminator

R4 (eval-log 2026-07-03) measured that on real philos data, dwell/persistence
does **not** separate the ~35 % KEEP craft from the ~50 % SUPPRESS structure
(anchored boats are as persistent and compact as piers); no occupancy-grid
tuning can make that split. Only **chart / AIS / camera corroboration** can.
Dalhaug, Stahl, Mester & Brekke 2025 (arXiv:2502.18368) reach the same
conclusion — they discriminate by camera instance segmentation, not geometry,
and independently confirm ENC-only chart layers under-cover real harbour
structure. Hence 1b-ii adds coverage-aware decay, chart corroboration, and
camera observed-empty eviction as the discriminators, keeping extent only as a
single-return noise floor.

---

## 4. Ways to improve / what to test next

1. **AIS KEEP-guard.** Veto suppression within a radius of a recent AIS vessel
   fix (an AIS-known vessel must track). Belt-and-suspenders to the unclaimed-only
   feed rule; guards the not-yet-tracked case. Pending on synthetic/HAXR fixtures
   (all labelled philos clips are zero-AIS).
2. **Stage 2 — Dempster-Shafer / DOGMa evidential grid** with an explicit
   *unknown* mass (§3.2), replacing the EWMA scalar. This is the genuinely-future
   work; the EWMA layer is not a partial DOGMa.
3. **Unobserved-decay floor for hour-long steady state.** Coverage-aware decay
   freezes permanently-unobserved cells forever; HAXR-hours churn needs a slow
   unobserved-decay floor (or corroboration-driven eviction as the sole release)
   before steady-state over-count reduction is measurable.
4. **Structure-set membership hysteresis.** A frozen-persistence cell near the
   clutter-adaptive bar blinks in/out of the structure set as the live-cell
   median moves, so hazards blink in operator output regardless of camera. Fix:
   enter/exit thresholds on membership (the `CpaEvaluator` hysteresis pattern).
5. **Stationary IMM mode** so an anchored track recovered from a suppressed
   region keeps a tight gate (zero-velocity / constant-position, locking the
   unobservable turn-rate), rather than holding with inflated covariance.
6. **Layer-2 HAXR-hours A/B.** The 20 s philos clips cannot show confirmed-cohort
   re-birth; the steady-state phantom-reduction benefit must be measured on
   hour-long real churn (plan increment 8).
7. **Bench camera arm.** `observeCamera` is currently wired only in the replay
   harness; a bench `Config` evict arm lands when camera enters the Sweep for the
   HAXR-hours A/B.

---

## Appendix — `LiveOccupancyParams` defaults

All values are the struct defaults in `core/static/LiveOccupancyModel.hpp`. The
`imm_cv_ct_pmbm_occupancy_detector` bench arm overrides some (notably
`cell_size_m = 100`, `extended_cells_min = 1`, `clutter_adaptive = true`) per the
1b-ii detector design.

| Field | Default | Units | Meaning |
|---|---|---|---|
| `cell_size_m` | `25.0` | m | Metric grid resolution. |
| `ewma_alpha` | `0.3` | — | Persistence EWMA rate per fed scan. |
| `persistence_bar` | `0.5` | — | Absolute EWMA at/above which a cell is persistent. |
| `extended_cells_min` | `4` | cells | Connected persistent cells required to be "structure". |
| `suppression_max` | `0.9` | — | Suppression cap; `< 0.95` hard gate ⇒ soft-only. |
| `suppression_radius_m` | `25.0` | m | Keep-clear ramp distance beyond the footprint. |
| `erase_floor` | `1e-3` | — | Drop cells whose EWMA decays below this. |
| `clutter_adaptive` | `false` | bool | Raise the bar above the estimated clutter background (detector mode). |
| `clutter_reject_factor` | `1.5` | — | Multiplier on the median live-cell persistence for the adaptive bar. |
| `chart_corroboration_radius_m` | `100.0` | m | Centroid-to-charted-point radius that confirms structure (label). |
| `camera_empty_sustain_s` | `2.0` | s | Continuous observed-empty duration to flag a cell camera-empty. |
| `evict_camera_empty` | `false` | bool | Enable eviction-by-camera as behaviour (spend persistence). |
| `camera_empty_recency_window_s` | `5.0` | s | Max staleness of a streak's last frame for it to evict now. |
