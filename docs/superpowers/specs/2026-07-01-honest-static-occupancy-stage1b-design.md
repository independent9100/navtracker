# Honest live static-occupancy layer (Stage 1b) — design

- **Date:** 2026-07-01
- **Status:** Draft (for review)
- **Related:** ADR 0002 (static objects), design spec §14.10, eval-log 2026-07-01
  (Stage 1b spike + chart-coverage + anchored-boats finding), Stage 1a
  (`IStaticObstacleModel` / `StaticHazardOutput` / `birthScale` seams).

## The insight that shaped this

The philos PMBM "over-count" is **not pure clutter**. Investigation +
operator field-check (Google Maps satellite) established it is a **mix**:
1. **Fixed infrastructure** (piers, breakwaters, shoreline) — must be
   *suppressed* (environment, not vessels).
2. **Real anchored / moored boats with no AIS** — must be *tracked* as
   stationary vessels (the "Confirmed stationary; direction undefined" state),
   NOT suppressed. The philos truth is AIS-only, so these real boats are absent
   from truth — which is why the Stage 1b spike's raw clutter-map feed *looked*
   like it removed "false" tracks (gospa_false −58%) while actually **partly
   suppressing real vessels**. That is the trap this design must avoid.

The Stage 1b spike (`feed_clutter_map`, committed 2457951) proved live spatial
suppression fixes the structured philos over-count (gospa 63→52) but **regresses
uniform clutter** (`dense_clutter` lifetime 0.90→0.26), because it suppresses
*any* persistent return — including real boats and targets buried in uniform
noise. So the raw feed is opt-in only. This design makes it **honest and
universal** by suppressing *only* what is genuinely fixed structure.

## The discriminators (both from measurement positions — no raw data)

navtracker consumes `Measurement`s (positions + covariance), **not raw radar
cells**. Both discriminators are computed from the positions the tracker
already receives — sensor-agnostic, inside the domain contract.

1. **Extent — spatial size of the persistent occupancy region (position-based).**
   A pier scatters returns over a large area → over scans, a **large connected
   region** of occupied cells (tens–hundreds of m). An anchored boat's returns
   fall in one small watch-circle → **one cell**. Extent = connected-component
   size of contiguous high-occupancy cells in the tracker's OWN occupancy grid
   (spatial bins over measurement positions — NOT the radar's raw range/azimuth
   cell). Works on any position-reporting sensor.
   *(The raw radar `n_cells` field — philos-specific, from the offline DBSCAN
   chain — is **validation-only**: it corroborated this split offline (compact
   `n_cells` 4–107 vs extended 1000+) but is **not** part of the `Measurement`
   contract and is never required. At most an optional corroborating hint if a
   sensor happens to report plot size — never load-bearing.)*
2. **Persistence — from the occupancy accumulation.** A cell/region touched by
   unclaimed returns over many scans (the Stage 1b spike's `1−r` feed) is
   persistent; transient uniform clutter is not.

**Suppression rule:** suppress births ONLY where occupancy is **persistent AND
extended**. This is the vessel-vs-environment rule (ADR 0002) made operational:

| return | persistent? | extent | → action |
|---|---|---|---|
| pier / breakwater | yes | large `n_cells` | **suppress** + emit static hazard |
| anchored/moored boat | yes | small `n_cells` | **track** (stationary vessel) |
| moving vessel | no | any | track (normal) |
| uniform sea clutter | no | small | ignore (not suppressed → no dense_clutter regression) |

## Architecture

Reuses the Stage-1a seams end to end; no new hot-path association code beyond
the (already-shipped) `feed_clutter_map` producer.

```
sensor Measurements (positions) ─▶ PMBM
                                     │
PMBM per-scan 1−r labeling (feed_clutter_map, shipped) ─▶ unclaimed returns
                                     │  (positions accumulate; no raw data)
                                     ▼
     LiveOccupancyModel  (geodetic, datum-stable; EWMA persistence per cell;
                          connected-component EXTENT over its own cells)
                                     │  implements IStaticObstacleModel
                    ┌────────────────┴─────────────────┐
                    ▼ birthSuppression(enu)             ▼ obstacles()
      PMBM birthScale seam (suppress births         StaticHazardEvaluator
      only in PERSISTENT + EXTENDED regions;        → StaticHazardOutput
      compact regions = boats → never suppressed)     (is_charted = false)
```

**Frame:** geodetic storage + ENU cache rebuilt on datum recenter
(`IDatumChangeSink`), exactly like `CoastlineModel`/`StaticObstacleModel` — the
"we move" requirement (design spec §14.10 caveat). NOT the existing
`ClutterMapDetectionModel` grid (ENU-body-relative, not datum-stable, and
coupled to λ_C — the spike showed feeding it changes tracking indiscriminately).

## Components

1. **`LiveOccupancyModel`** (`core/static/`) — a geodetic occupancy grid,
   datum-stable (`IDatumChangeSink`, ENU-cache rebuild like `CoastlineModel`).
   - **Accumulate:** each scan, the shipped `feed_clutter_map` producer hands it
     the unclaimed returns (positions + `1−r` weight). Cells hold an EWMA
     persistence score over positions — **no raw data**.
   - **Classify (extent):** run connected-component over contiguous
     high-persistence cells; a component's cell-count is its extent. A region is
     "structure" iff it is BOTH **persistent** (EWMA above a bar) AND
     **extended** (component size ≥ `kExtendedCellsMin`, initial ~a few cells @
     25 m ≈ >50–75 m; validated on synthetic truth). A compact persistent region
     (single/few cells = a boat's watch circle) is NOT structure.
   - **`birthSuppression(enu)`:** soft ramp on the nearest *structure* region's
     confidence — **0 for compact regions** (boats are never suppressed) and for
     transient cells (uniform clutter never persists → no `dense_clutter`
     regression).
   - **`obstacles()`:** the structure regions as `StaticObstacle`-like entries
     (`is_charted=false`) for the hazard output.
2. **Wiring:** `PmbmTracker::setStaticObstacleModel(&live)` (the Stage-1a setter
   — live occupancy feeds the SAME `birthScale` seam); `StaticHazardEvaluator(&live)`
   emits the live hazards; a new opt-in bench config `imm_cv_ct_pmbm_occupancy`
   turns on `feed_clutter_map` + wires the `LiveOccupancyModel`.

**No `Measurement` / `ScanObservation` contract change** — the occupancy layer
works entirely from the positions the producer already carries.

## Testing (the promotion gates)

**REVISED 2026-07-03 after measuring across regimes + R4 (eval-log
2026-07-03).** Two gates below were mis-specified and are corrected here; the
churn variant is now a PERMANENT gate member.

- **`harbor_complete_truth` (P_D 0.9) is NOT a suppression gate — it is
  structurally unpassable by ANY birth-channel suppressor.** At P_D 0.9 the
  pier's phantom cohort confirms in scans 1–2 and never dies, so the birth
  channel never gets a second query (measured: `suppress_hits ≈ 0`, byte-
  identical). It stays the *classification* + off-bit-identical + boat-
  preservation gate, NOT the suppression gate. Recording this so the gate/
  mechanism mismatch is never re-litigated.
- **`harbor_complete_truth_churn` (P_D 0.4) — NEW PERMANENT gate member.** The
  low-P_D pier decays and must re-birth, so a birth suppressor is measurable at
  all. This is the complete-truth suppression gate: a valid birth-channel
  mechanism must cut pier `gospa_false` here WITHOUT dropping the three anchored
  boats' lifetime (measured with a tuned classifier: −78 false, lifetime held
  0.975, `suppress_hits` 26). Any future suppression change is evaluated here,
  not on the P_D-0.9 yardstick.
- **philos gate is BOUNDED by R4, not a bare "gospa ≈ 52".** R4 measured the
  removable ceiling: only **~50% of the persistent over-count mass is
  chart-confirmed structure**; **~35% is real craft (KEEP)**. So the honest
  philos gate is: suppress toward the SUPPRESS canaries, never the KEEP canaries
  (`charts/philos_cluster_classification.csv`); `card_err` must NOT overshoot
  negative and total suppressed mass must NOT exceed the ~50% structure ceiling
  (the raw λ_C spike deleted ~58% → into KEEP mass → rejected). philos gospa is
  a *reality check bounded by canaries*, never a bare scalar to minimise.
- **anchored-boat preservation (synthetic COMPLETE truth):** a compact,
  ~zero-velocity, non-AIS return that persists near shore MUST still birth +
  confirm. R4 shows why this is load-bearing: on real data **dwell/persistence
  does not separate craft from structure** (p50 0.64 vs 0.68), so a suppressor
  that keys on persistence alone WILL kill moored boats — the gate that catches
  it.
- **dense_clutter:** clean — transient uniform clutter never becomes
  persistent+extended → not suppressed → lifetime back to ~0.90. (Measured:
  byte-identical at every occupancy tuning — safety holds.)
- **clean geometry / near-shore:** byte-identical / no regression.
- **determinism + null/off bit-identical** (model unwired → today's behaviour).

## Staging

**Extent is an INTERIM discriminator (R3, 2026-07-02).** Separating KEEP
(compact) from SUPPRESS (persistent AND extended) by geometry is a stopgap with
two documented failure directions: a **large anchored ship** (~150–200 m,
non-AIS) is extended → wrongly SUPPRESSED (dangerous); a **single compact
pile/dolphin** is compact → wrongly KEPT as a vessel (cardinality pollution). The
literature discriminates by **classification, not geometry**: Dalhaug, Stahl,
Mester & Brekke 2025 (arXiv:2502.18368, `docs/references/`) exclude anything
boat-shaped from the static map via camera instance segmentation, measured both
failure directions (a pre-trained model read a floating dock as two boats), and
independently confirm ENC-only layers under-cover real harbour structure (our
~⅓ chart-coverage finding). So 1b-ii's discriminator is corroboration, not
extent. Gate scenarios `harbor_large_anchored_ship` (must be KEPT) and
`harbor_compact_dolphin` (SUPPRESS target) exist to record the 1b-i "before"
numbers as known limitations.

- **1b-i (this design):** extent field + `LiveOccupancyModel` + birth-prior
  suppression + hazard output + the four tests above. Measured. Extent is the
  interim discriminator per the note above.
- **1b-ii / Stage 2 (follow-on):** replace extent with **corroboration** — S-57
  chart overlay (`PONTON`/`MORFAC`/`ACHARE`), AIS (nav-status 1/5, Message 21
  AtoN), then camera instance-segmentation classification (Dalhaug 2025). Plus a
  **stationary IMM mode** (zero-velocity / constant-position, locking the
  unobservable turn-rate) so moored tracks keep a tight gate, and the full
  Dempster-Shafer evidential grid with an explicit "unknown".
  **R4 (2026-07-03) is the quantitative mandate for this:** on real philos,
  persistence and extent do NOT separate the ~35% KEEP craft from the ~50%
  SUPPRESS structure (dwell p50 0.64 vs 0.68; extents overlap). No occupancy-grid
  tuning can make that split — corroboration is not a refinement, it is the only
  discriminator that works on real data. The 1b-ii detector must fire on real
  sparse/projection-smeared returns (100 m cells classify philos structure where
  25–50 m do not) AND gate suppression on chart/AIS/camera corroboration; the
  birth channel is adequate once the detector fires (Stage 1b-i measurement).

## Docs (required)

Four-part algorithm doc (`docs/algorithms/live-static-occupancy.md`) + a
`docs/learning/` chapter (extent + persistence, boat-vs-structure, plain
English) + figure; update ADR 0002 staging + comparison-baselines + eval-log.

## Known limitations (documented, not silently accepted)

- A **large stationary non-AIS vessel** (e.g. an anchored 200 m ship with no
  AIS) has a large occupancy footprint and would be wrongly suppressed by extent
  alone. Mitigation deferred to 1b-ii (shape discriminator + AIS/chart
  corroboration). In practice such vessels almost always carry AIS (the
  source-aware path exempts cooperative returns).
- A **dense marina** (many boats packed together, swinging in watch circles) can
  read as one medium extended region. Extent alone may over-suppress it; the
  shape/motion discriminator (1b-ii) and chart-`SMCFAC`/`PONTON` corroboration
  are the principled fix. The initial `kExtendedCellsMin` should be set high
  enough that a single boat's watch circle never qualifies as structure.
