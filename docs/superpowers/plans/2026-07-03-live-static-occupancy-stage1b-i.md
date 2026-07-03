# Stage 1b-i — Live static-occupancy layer (implementation plan)

- **Date:** 2026-07-03
- **Status (2026-07-03):** Layer BUILT + unit-tested + wired + measured. Result:
  classification works and the layer is SAFE, but **birth-only suppression is
  INERT** on all available synthetic fixtures — `suppress_hits ≈ 0`,
  `_occupancy` ≈ byte-identical to `_land`. Root cause: within a scan births
  precede the feed, so the pier cohort confirms before the layer classifies, and
  `smart_birth_skip_existing` then owns the region (see eval-log 2026-07-03). The
  effective mechanism needs the philos dense regime (unavailable here) or an
  existence channel (deferred to 1b-ii for safety). **Direction decision pending
  — do not treat steps 5–7 below as done.**
- **Design:** `docs/superpowers/specs/2026-07-01-honest-static-occupancy-stage1b-design.md`
- **North-star:** Cl-3 (`docs/algorithms/comparison-baselines.md`)
- **Motivation carried in from the PDA promotion HOLD (2026-07-03):** the residual
  urban regression the land-aware pool could not close is *in-water structured
  clutter* (piers/floating structure and moored craft) that a land mask cannot
  flag. Stage 1b-i is the right tool for the *structure* half: learn persistent
  extended occupancy live and remove it from the birth channel. Moored *vessels*
  are NOT this layer's job (ADR 0002: SOG≈0 is never a reason to call something
  environment) — they converge via the unclaimed-only PDA pool once they become a
  zero-velocity track (Stage 2 stationary mode). Same seam, two doors.

## Launch conditions (verified 2026-07-03)

- Gate scenarios already landed on the review-fixes branch: `harbor_boat_near_pier`
  (R6), `harbor_large_anchored_ship` KEEP + `harbor_compact_dolphin` SUPPRESS (R3),
  `harbor_charted_pier` (R5). R2 true-assignment feed, R1 pre-suppression floor,
  R4 field-check all landed. So no gate scaffolding to build first.
- Corrected harbor baseline: card_err +11.64, gospa 50.63, gospa_false 2362,
  lifetime 0.974 (post truth-sort fix).

## Architecture (reuses Stage-1a seams end to end)

`LiveOccupancyModel` (`core/static/`) is ONE object that plays two roles:

1. **Suppression side** — implements `IStaticObstacleModel`
   (`birthSuppression(enu)` + `obstacles()`), wired via the existing
   `PmbmTracker::setStaticObstacleModel(&live)` seam + `use_static_obstacle_model`.
   Birth-channel only.
2. **Accumulation side** — receives the per-scan `(position, 1−r)` feed the
   shipped `feed_clutter_map` producer already computes. Fed through a NEW,
   minimal hook `PmbmTracker::setLiveOccupancyFeed(ILiveOccupancyFeed*)` that is
   **independent of `detection_model_`** → NO λ_C coupling (the design's hard
   requirement; the reason we do not reuse `ClutterMapDetectionModel`).

Datum-stable like `CoastlineModel`/`StaticObstacleModel`: grid anchored to a fixed
datum, `IDatumChangeSink` updates the current-datum transform on recenter. Bench
uses a single fixed datum ⇒ anchor==current ⇒ identity transform ⇒ deterministic.

### The occupancy grid (math)

- Cells: integer `(i,j) = floor(enu_anchor / cell_size_m)` (default 25 m).
- Persistence: per-cell EWMA `p ← (1−α)·p + α·w_max` where `w_max` is the largest
  feed weight (`1−r`, or 1.0 unclaimed) touching the cell this scan; untouched
  cells decay `p ← (1−α)·p`. Cells below a floor are erased (sparse hash map).
- Persistent cell: `p ≥ persistence_bar`. Uniform sea clutter never persists.
- Extent: connected components (4-conn, deterministic key-sorted BFS) over
  persistent cells; a component is **structure** iff `size ≥ extended_cells_min`
  (default 4 ⇒ ≥ ~100 m span). Compact persistent region (a boat's watch circle)
  is NOT structure.
- `birthSuppression(enu)`: soft ramp on the nearest *structure* component
  confidence, capped at `suppression_max` (default 0.9 < hard gate 0.95 → soft-only,
  never a hard no-birth kill; safe for a mislabeled large vessel per R3). 0 for
  compact/transient cells → no `dense_clutter` regression, boats never suppressed.
- `obstacles()`: one synthesized `StaticObstacle` (Obstruction, source
  `live_occupancy`) per structure component for the hazard output (`is_charted=false`).

## Steps (TDD; RED→GREEN each)

1. **`ports/ILiveOccupancyFeed.hpp`** — 1-method port `observe(const
   std::vector<ISensorDetectionModel::ScanObservation>&)`.
2. **`core/static/LiveOccupancyModel.{hpp,cpp}`** — the grid above. Unit tests:
   persistence accumulation/decay, extent classification (pier extended vs boat
   compact), suppression ramp, datum-recenter re-anchor, determinism.
3. **`PmbmTracker::setLiveOccupancyFeed` + producer hook** — build bundle if
   `feed_clutter_map || occupancy_feed_`; feed detection only under
   `feed_clutter_map`, feed occupancy under the pointer. Off ⇒ bit-identical.
4. **Bench wiring** (`Sweep.cpp`) + config `imm_cv_ct_pmbm_occupancy`
   (= `_land` + `feed_clutter_map=false`, `use_static_obstacle_model=true`,
   wire `LiveOccupancyModel` as both static-obstacle model and occupancy feed).
5. **Gate tests / A/B:**
   - anchored-boat preservation (compact persistent → still births + confirms);
   - `dense_clutter` clean (uniform clutter never suppressed → lifetime ~0.90);
   - off/null bit-identical + determinism;
   - philos win retained; clean-geometry byte-identical (targeted foreground A/B,
     never full-matrix background — OOM lesson).
6. **Cheap ablation (carried from review):** claimed returns fed at weight 0 vs
   1−r — one bench run, record which drives any `dense_clutter` movement.
7. **Docs:** `docs/algorithms/live-static-occupancy.md` (four-part), a
   `docs/learning/` chapter + figure, ADR 0002 staging update,
   comparison-baselines Stage-1b row, eval-log entry with the 1b-i "before"
   numbers for the R3 KEEP/SUPPRESS gates.

## Non-goals (documented limitations, deferred to 1b-ii / Stage 2)

- Large non-AIS anchored ship (extended → wrongly suppressed) — R3 known limit;
  fixed by classification/AIS corroboration in 1b-ii.
- Dense marina read as one extended region — `extended_cells_min` set high enough
  that a single watch circle never qualifies; principled fix is 1b-ii shape/chart.
- Moored *vessel* recovery is Stage 2 (stationary IMM mode), not this layer.
