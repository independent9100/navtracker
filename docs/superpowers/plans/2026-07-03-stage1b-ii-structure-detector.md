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
   - 6b TODO — producer wiring: `feed_clutter_map` in `PmbmTracker.cpp` (~L1664)
     self-estimates each burst's sector from its plot azimuth span + conservative
     padding + sensor pose; synthetics emit no footprint (full coverage, bit-
     identical). Under-estimated coverage is the safe direction.
   - 6c TODO — validate on `sunset_cruise` (loiterer cessation t≈94 resolves as a
     DEPARTED vessel; ferry-vacated cells after t≈98) + KEEP_MIXED departure
     protection on `close_approach`; then AIS veto (synthetics/HAXR only — all
     labelled philos clips are zero-AIS), then chart corroboration.
   - 6d TODO — docs (folds into increment 9): live-static-occupancy.md four-part +
     learning chapter + figure for coverage-aware decay.
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
  correction. Full suite 925/925. R8.6 items 2 (CPA fixture) + 3 (sensor-doc
  note) deferred as an independent task; `almost_cross`/`sailboats_busy`
  anchorage canaries deferred pending FOV-confirming video passes.
- TODO — R8.4 (observed-empty / coverage-aware decay) folds into increment 6.
  Coverage descriptor decided (R8.6 amendment): feed carries (sensor ENU, max
  range, azimuth sector); disc = degenerate [0°,360°); synthetics pass full
  circle (bit-identical), philos self-estimates each burst's sector.
- TODO — R8.5 comparison-baselines philos-metric note.
