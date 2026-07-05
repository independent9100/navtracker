# Stage 1b-ii — live structure detector (presence over classification)

- **Date:** 2026-07-03
- **Status:** IN PROGRESS (implementer). Design of record below; increments TDD.
- **Governing decisions:** ADR 0002 + its 2026-07-03 amendment
  ("presence over classification"); the 2026-07-01 honest-static-occupancy
  design (superseded on the detector by the 2026-07-03 regime measurement + R4).
- **North-star:** Cl-3 (`docs/algorithms/comparison-baselines.md`).

## Why this exists (the two measurements that set the design)

1. **Regime measurement (eval-log 2026-07-03 follow-up):** birth-only occupancy
   suppression works + is safe on tuned synthetic churn but is inert on real
   philos at every tuning — fine cells never classify (sparsity + own-ship
   projection smear); coarse 100 m cells classify but the phantoms are already
   confirmed tracks the birth channel can't reach. The wall is the *detector*,
   not the channel.
2. **R4 (eval-log 2026-07-03):** on real philos, dwell/persistence does NOT
   separate the ~35 % KEEP craft from the ~50 % SUPPRESS structure — anchored
   boats are as persistent + compact as piers. **No occupancy-grid tuning can
   make that split; only chart/AIS/camera corroboration can.** ~50 % of the
   over-count mass is chart-confirmed structure (suppressible); ~35 % is real
   craft; ~14 % unknown.

## The goal (ADR 0002 amendment, do not re-litigate)

Presence over classification. For every persistent object in the water:
1. **Never invisible** — it is a track OR a static hazard, never neither.
2. **Correct classification preferred, not required** — anchored-vessel-shown-
   as-static-hazard-with-keep-clear is an acceptable degraded mode when no
   sensor can tell.
3. **Static → moving must recover** — anything represented as static is promoted
   to a moving track within bounded latency once it moves.

This reframes R4's 35 % KEEP: those craft need not stay *tracks*; they may be
*hazards*, PROVIDED conservation + recovery hold. That is far more achievable
than "never suppress a real vessel", and strictly safer than today's silent
over-count.

## Design

`LiveOccupancyModel` evolves (still one object; params-configured). The detector
is a config (`imm_cv_ct_pmbm_occupancy_detector`), not a new class.

1. **Coarse grid.** Default detector cell 100 m (R4: fires on philos; 25–50 m
   don't). Grid stays datum-stable (anchor frame; IDatumChangeSink).
2. **Persistence, low/no extent floor.** EWMA persistence per cell as today. The
   extent≥4 gate is RETIRED as the KEEP protector (the amendment lets a compact
   anchored boat be suppressed-as-hazard). A small floor (≥1–2 cells) only
   rejects single-return noise. Discrimination moves to corroboration.
3. **Conservation by construction (the load-bearing safety property).**
   `birthSuppression(q)` is DERIVED from the emitted hazard set: it is a ramp
   over `obstacles()` (1.0 inside `footprint_radius_m`, linear to 0 at
   `keep_clear_radius_m`), so `suppression(q) > 0 ⇒ q ∈ some emitted hazard's
   keep-clear ring`. Suppression into nothing is impossible. Each persistent
   structure component emits ONE `StaticObstacle` (`is_charted=false`,
   `source_id="live_occupancy"`) whose footprint covers ALL its cells → clean
   operator output, still conserved.
4. **Corroboration KEEP-guard (same milestone).** Optional inputs:
   - *Chart:* a live hazard coincident with a charted obstacle → classification
     confirmed (label; suppression stays — it IS structure).
   - *AIS:* suppression is VETOED within a radius of a recent AIS vessel fix (an
     AIS-known vessel must track, amendment rule where-we-can-discriminate).
     Belt-and-suspenders: tracked AIS vessels are already excluded from the
     occupancy feed by the unclaimed-only rule; this guards the not-yet-tracked
     case.
5. **Recovery (static → moving).** Inherent in EWMA decay: when a suppressed
   region's returns cease (vessel underway), its cells forget below the bar →
   drop from the hazard set → suppression lifts → the mover births normally. α
   tuned so recovery latency ≤ N scans while genuine structure still holds.

## Gates (all TDD)

- **Conservation invariant (unit):** over a query grid, every `q` with
  `birthSuppression(q) > 0` is inside some emitted hazard's `keep_clear_radius`.
- **Recovery gate (scenario):** `harbor_anchored_gets_underway` — an anchored
  non-AIS boat classified into a structure region gets underway mid-run; must
  confirm as a moving track within N scans; the vacated cells decay out of the
  hazard set (no permanent pin). Complete truth.
- **AIS KEEP-guard (unit/scenario):** a persistent region under a recent AIS fix
  is NOT suppressed.
- **Layer 1 — philos:** the detector CLASSIFIES real structure
  (`occ_peak_structures > 0` at 100 m; SUPPRESS canaries hit, KEEP canaries not).
- **Layer 2 — HAXR hours:** birth-only suppression actually reduces the phantom
  over-count at steady state on real hour-long churn (the 20 s philos clips are
  too short to show confirmed-cohort re-birth).
- **dense_clutter:** byte-identical (uniform clutter never persists).
- **off / determinism:** unwired ⇒ bit-identical; deterministic replay.

## Increments

1. DONE — conservation refactor (suppression ⊆ hazards) + unit test (61e2293).
2. DONE — recovery bounded-latency unit test (≤5 scans; mover never suppressed).
3. DONE — coarse grid + extent floor 1 + clutter-ADAPTIVE bar (self-estimated
   from feed median) + config imm_cv_ct_pmbm_occupancy_detector (b6a4280).
   Layer-1 CONFIRMED: fires on real philos (structures 8, suppress_hits 94; no
   card_err overshoot → KEEP anchorage safe). Weak on 20 s clips (confirmed-
   cohort horizon limit → Layer-2).
4. DONE — datum-bearing dense-clutter death-spiral guard. New scenario
   `dense_clutter_datum` (dense_clutter + a datum so the layer wires); bench
   gate `OccupancyDetectorGates.DenseClutterDatumNoDeathSpiral`. land lifetime
   0.845 → detector 0.836, gospa 13.07 → 13.09: no spiral. (Commit: this batch.)
5. DONE — conservation/presence at BENCH. New per-truth Sweep column
   `occ_truth_in_hazard:truth_<id>` (truth's final position ∈ an emitted hazard
   ring — pure geometry); gate `OccupancyDetectorGates.PresenceOverClassifica-
   tionOnHarbor` enforces the THREE-WAY split (presence hard-gate / movers
   lifetime / classification-quality reported). M2 gate formalized in
   synthetic-clutter-bench.md §5.6 + eval-log. Boats stay tracked at P_D 0.9
   (confirmed-cohort wall) so the boat→hazard trade is negligible there.
6. IN PROGRESS — corroboration KEEP-guard. Order (2026-07-03 steer): coverage-
   aware decay FIRST, then AIS veto, then chart corroboration.
   - **6a DONE — coverage-aware decay, MODEL mechanism (inert-by-default).**
     `ScanObservation` gains an optional `CoverageSector` (sensor ENU + max range
     + azimuth sector, mirroring `DetectionParams`; disc = degenerate full
     circle). `LiveOccupancyModel::observe()` decays a cell only when it is
     inside some bundle's footprint (observable) and empty; no valid footprint ⇒
     full coverage ⇒ universal decay (legacy, bit-identical — proven by the 10
     pre-existing model tests staying green). 3 new unit tests: out-of-range and
     out-of-sector cells do NOT decay, observed-empty cells DO. Full suite 928/928.
   - **6b DONE — producer wiring.** `CoverageSector::fromReturns(sensor, points,
     az_pad, range_pad)` self-estimates the swept sector as the SMALLEST arc
     covering a scan's return bearings (largest-circular-gap method; handles the
     ±180° wrap), range = farthest return + padding; under-estimates coverage
     (safe). Unit-tested (`tests/tracking/test_sensor_detection_models.cpp`:
     sector/range, degenerate empty/single, wraparound). Producer (`PmbmTracker.cpp`
     bundle builder) sets `obs.coverage` per bundle when the new config flag
     `estimate_coverage_sector` is on (default OFF → no footprint → universal
     decay → bit-identical; synthetics stay off, a per-burst radar turns it on).
     Wiring tested via `SpyOccupancyFeed` (flag-on populates a covering sector,
     flag-off leaves it invalid). Full suite 933/933.
   - **6c DONE — clip validation (sunset_cruise + close_approach A/B).** New
     coverage-aware config arm `imm_cv_ct_pmbm_occupancy_detector_coverage`
     (differs from `_detector` in `estimate_coverage_sector` ALONE, asserted by
     `Config.OccupancyDetectorArmsDifferOnlyInCoverageFlag`). Harness extended
     additively (per-scan hazard set + coverage-sector introspection; existing
     label tests bit-identical). Tests: `test_philos_occupancy_coverage_6c.cpp`.
     **Findings (eval-log 2026-07-03):** mechanism bites — 1328 sub-circle
     sectors, median 12.6° (≈3° sweep + 2×5° pad), zero full-circle. Coverage-
     aware **improves structure presence** (astern_blob held 9→31 scans; final
     hazards 0→11) and **KEEP_MIXED presence** (sailing_dock 0.964→0.998,
     far_bank 0.494→0.616 by keep-clear coverage) with **zero conservation loss**;
     presence is monotone ≥ universal by construction (decays a subset of
     universal's events). **The loiterer is NOT resolved-as-departed by radar
     alone** — its cell is swept 0/283 scans after t94 (departure is a *camera*
     fact), so coverage-aware correctly holds it as a conserved hazard; universal
     drops it only by forgetting all structure. This is the corroboration wall,
     confirmed → **next: AIS veto** (synthetics/HAXR only — all labelled philos
     clips are zero-AIS), then chart corroboration. **Layer-2 caveat (increment
     8):** permanently-unobserved cells never decay → need a slow unobserved-decay
     floor or corroboration-driven eviction before HAXR-hours steady-state.
     **Scan-batching MEASURED 2026-07-03 (unblocked 6c): philos bursts ARE
     sub-360°.** Grouping fixture rows by identical `tod`: cadence 0.063–0.26 s,
     median azimuth span per burst 2.9° (sunset_cruise) / 2.8° (close_approach) /
     4.7° (ais_ferry_near); zero bursts span >350° in any labelled clip. The
     self-estimated sector collapses to full circle NOWHERE — the mechanism
     bites on philos. Consequence: a cell is observed ~once per antenna
     rotation, so decay proceeds per-rotation, not per-burst — expected.
     **Caveat found — non-contiguous bursts overclaim coverage.** 68/1330 (5%)
     sunset_cruise and 146/882 (17%) close_approach bursts span >30°, and 100%
     of those are MULTI-CLUSTER (largest internal bearing gap 80–169°; a 0.08 s
     burst cannot sweep 175° — these are separate echo clusters sharing a
     timestamp). Smallest-covering-arc claims the unswept gap between clusters
     as observed-empty → over-decay there, the UNSAFE direction.
     **GUARD DONE (before 6c, this batch):** `CoverageSector::fromReturns` now
     splits a scan's return bearings into contiguous clusters (internal gap >
     `cluster_gap_rad`, default ~20°) and keeps ONLY the largest cluster's arc —
     the single-sector safe fallback (under-claims; other clusters wait for their
     own narrower bursts). Config `coverage_cluster_gap_rad`. New unit test
     `FromReturnsKeepsLargestClusterNotTheInterClusterGap` (the 45°-gap point is
     no longer claimed); the synthetic estimator/wiring tests were retightened to
     realistic <20°-span clusters. Full suite 934/934. The remaining 6c work is
     the actual clip validation (loiterer/ferry decay-out + KEEP_MIXED
     protection), now safe to run.
   - **CHART CORROBORATION DONE (2026-07-04, jumped ahead of AIS veto per the
     post-6c steer — chart owns the largest measured target, R4's ~49.5%).**
     `LiveOccupancyModel::setChartedStructure` tags each emitted live hazard
     chart-confirmed when its centroid is within `chart_corroboration_radius_m`
     (default 100 m) of a charted structure point. LABEL ONLY (suppression/tracks
     unchanged); feeds operator confidence + increment-8 eviction. Inert-by-
     default (13 pre-existing model tests green). Chart source:
     `charts/export_philos_chart_structure.py` densifies `radar_clutter.geojson`
     to 8 m points scoped to the philos bbox → `tests/fixtures/philos/charts/
     radar_structure_points.geojson` (15 974 pts; radar_clutter = the radar-
     visible layer, R4's dual-layer AND was a DELETION bound, CONFIRM needs one
     layer). **sunset_cruise: 6220/8114 (76.7%) hazard-scans chart-confirmed;
     astern_blob 31/31 (nearest 16 m); loiterer 0/122 (nearest 134 m — the
     departed-vessel discriminator radar couldn't provide); ferry_v1_a 0/538;
     UNKNOWN ranks_84_95 0/955.** close_approach Charles-basin KEEP_MIXED is 277–
     432 m from charted structure (FLOATING docks) → chart correctly ABSTAINS,
     that region needs camera/AIS. TDD'd (3 model unit tests); 1 replay measurement
     test (`test_philos_occupancy_coverage_6c.cpp`). Chart is the first eviction-
     by-evidence source (increment 8).
   - **CAMERA CORROBORATION (i) DONE (2026-07-04, label-only).**
     `LiveOccupancyModel::observeCamera` (dedicated API, NOT the clutter feed)
     advances a per-cell observed-empty streak (in-FOV + live frame + no detection
     within tolerance of the cell bearing → extend; matching detection → reset;
     out-of-FOV → untouched). A hazard whose centroid cell is observed-empty ≥
     `camera_empty_sustain_s` (2 s) is flagged. Label only; inert until fed.
     TDD'd (3 model unit tests). **FOV gate FIRST (the "before designing"
     check): loiterer 100% (1537/1537) in center ~±22° FOV after t94; its bearing
     0 detections within ±10° over t100–120 (20 s) — camera viable on this clip.
     Result:** ferry_v1_a (vacated outbound berth) 538→41 = the clean demo (real
     vessel that moved, stale pin marked departed); loiterer 122→1 (flagged t118);
     astern_blob (out of FOV) 31→0 (unobserved, held by chart). All camera-flagged
     cells chart-UNconfirmed → eviction candidates. **Honest caveat:** the
     loiterer's low count is NOT a camera limit (its bearing is cleanly empty) —
     its *hazard* is intermittent post-departure (adaptive-bar flicker on the
     frozen-persistence cell), rarely coexisting with the matured streak; the
     ferry berth (stable held cell) is the robust demo.
   - **CAMERA CORROBORATION (ii) DONE (2026-07-04, eviction as BEHAVIOUR).**
     New `LiveOccupancyParams.evict_camera_empty` (default false) + `camera_empty_
     recency_window_s` (default 5 s). Eviction is a pre-pass in `observe()`: a
     structure cell whose per-cell observed-empty streak is matured (≥ sustain)
     AND recent (last frame within the window of the scan time) AND whose component
     is chart-UNconfirmed has its persistence SPENT (erased), not just its hazard
     dropped — because coverage-aware decay FREEZES an unobserved departed pin, and
     dropping only the hazard would let the frozen persistence re-emit it (a
     blinker). Evidence is keyed by CELL and accrues while the cell is off-stage, so
     eviction fires the instant a flickering cell re-enters — fixing the increment-i
     loiterer coincidence. Evidence precedence: chart-confirmed → hold regardless.
     Conservation-safe by construction (suppression re-derived from post-eviction
     persistence). Refactor: extracted `structureComponents()` (shared by recompute
     + eviction). **Synthetic PROMOTION GATE** (`test_live_occupancy_model.cpp`,
     6 eviction unit tests + 2 scenario gates): `EvictionSceneDepartedEvictsHeld-
     StructuresStayFlat` (3 co-present frozen structures — departed/chart-held/
     camera-blind — eviction A/B: departed→0, both held BYTE-IDENTICAL = tracks_on_
     keep flat) and `CameraEvictionSurvivesAdaptiveBarFlicker` (the loiterer
     pathology as a deterministic regression — pin blinks out via the adaptive bar,
     matures off-stage, evicts on re-entry; proven non-vacuous by flipping the flag
     → RED). **Real-data DEMO** (`test_philos_occupancy_coverage_6c.cpp`:
     `SunsetCameraEvictionRemovesDepartedPinsHoldsChartStructure`) — sunset_cruise
     coverage+chart+camera, eviction A/B: total hazard-scans 8114→7722; the ferry's
     vacated OUTBOUND berth (post-t98 phantom) 180→42 = the clean departed-evicts;
     the loiterer's BEFORE-departure hazards retained 121→121 (vessel present, camera
     sees it — correctly not evicted); chart-confirmed astern_blob held 31→31.
     **Honest correction to the increment-i framing:** the loiterer is NOT a
     persistent post-departure phantom in this config (adaptive bar fades it — off
     has 1 hazard-scan after t100), so its 1/122 flag was mostly CORRECT (121 are
     pre-departure); the SYNTHETIC flicker gate carries its pathology. Logged caveat:
     eviction also removed ~145 ferry hazards pre-move (camera saw the docked berth
     empty ≥ sustain) — correct-vs-over-eviction needs kinematic truth (Layer-2).
     **Deferred (increment 8 / Layer-2):**
     no bench `Config` arm — the bench Sweep does not feed camera to the occupancy
     model (observeCamera is wired only in the replay harness), so a bench evict arm
     would be inert; it lands when camera enters the Sweep for the HAXR-hours A/B.
     **Backlog item — DONE 2026-07-05 (commit 9773510):** a frozen-
     persistence cell blinks in/out of the structure set as the clutter-adaptive
     live-cell median moves → hazards blink in operator output regardless of camera;
     fixed via `LiveOccupancyParams.membership_exit_factor` + `persistent_prev_`
     enter/exit thresholds (the CpaEvaluator pattern; default 1.0 = bit-identical).
   - 6d TODO — docs (folds into increment 9): live-static-occupancy.md four-part +
     learning chapter + figure for coverage-aware decay + chart + camera.
7. DONE — recovery gate SCENARIO `harbor_anchored_gets_underway` (stop→go boat
   via new `addStopGoBoat` builder); gate
   `OccupancyDetectorGates.AnchoredGetsUnderwayRecovers` (truth_6 lifetime 0.972
   ≥ 0.4 while suppression active). Bounded-latency decay is the model unit test.
8. TODO — Layer-2 HAXR hours A/B (steady-state churn the 20 s clips cannot show).
9. TODO — docs: live-static-occupancy.md four-part + learning chapter + figure;
   ADR 0002 staging; comparison-baselines row; eval-log.

Interleaved (before increment 6, per 2026-07-03 steer — "build the exam before
the student"):
- DONE — R8.1 label fixture (`sunset_cruise_labels.csv`) + loader
  `core/benchmark/ExistenceLabel` (unit-tested).
- DONE — R8.2 label-aware decomposition (false_on_suppress 3070 / tracks_on_keep
  1633 / false_unlabeled 18295 under land) +
- DONE — R8.3 binary gates (KEEP canaries all covered; stop→go id 13 stable +
  2.89 m/s late), in `tests/replay/test_philos_sunset_labels.cpp`, pass TODAY
  under `imm_cv_ct_pmbm_land`. sunset_cruise is zero-AIS/no-truth so it is run
  through the tracker directly (empty truth ⇒ no Sweep slices).
- DONE — R8.6 items 1/4/5: `KEEP_MIXED` label class + `close_approach_labels.csv`
  (2 video-verified regions); shared replay harness `PhilosLabelReplay.hpp`
  (sunset bit-identical after extraction); close_approach KEEP-stress baseline
  (`tracks_on_keep=5570`, `false_on_suppress=0`, `false_unlabeled=15182`; both
  KEEP_MIXED canaries COVERED, 0.14 m / 1.40 m); eval-log entry + R4 ceiling
  correction. Full suite 925/925. **R8.6 items 2 (CPA fixture) + 3 (sensor-doc
  note) DONE 2026-07-05 (commit bb47031):** first real-data collision-alarm test
  (`test_philos_close_approach_cpa.cpp` — collider tracked to 10.2 m, CpaEvaluator
  Entered 4.1 s before contact, keyed to the collider's track id); sensor-reference
  raw-plot note (15 m floor is a last-metres blind zone only; radar tracked the 4 m
  collider to 17 m). `almost_cross`/`sailboats_busy` anchorage canaries still
  deferred pending FOV-confirming video passes.
- TODO — R8.4 (observed-empty / coverage-aware decay) folds into increment 6.
  Coverage descriptor decided (R8.6 amendment): feed carries (sensor ENU, max
  range, azimuth sector); disc = degenerate [0°,360°); synthetics pass full
  circle (bit-identical), philos self-estimates each burst's sector.
- TODO — R8.5 comparison-baselines philos-metric note.
