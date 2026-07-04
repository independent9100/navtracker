# Increment 8 — HAXR occupancy steady-state A/B (2026-07-04, autonomous run)

Front matter: veto production wiring SHIPPED (commit 0472eae); D1 extraction
done (9 station-hours, time-sorted). This plan executes the reviewer-approved
increment-8 measurement campaign. **HAXR truth is AIS-only → cross-check, NOT
the gate** (`harbor_complete_truth` stays the gate).

## The compute reality (timing probe, 2026-07-04)

PMBM + occupancy on undecimated HAXR (kattwyk_08 t40: 302k plots, 285 s, ~169
plots/scan) = **1 h 35 min / config** (`imm_cv_ct_pmbm_coverage_land`), ~172×
EKF+GNN (33 s), ~20× slower than realtime. The occupancy-OFF baseline carries
~150 phantom Bernoullis → Murty over 150×169/scan. Full-hour PMBM ≈ 18 h/station;
9 × A/B ≈ two weeks. **Full hours are ruled out; the lever is plot density.**

## Plan (user calls, 2026-07-04)

1. **Decimate — clustering-first, NOT amplitude.** Amplitude threshold deletes
   weak returns = small non-AIS craft whose miss-rate we care about. Aggressive
   grid-cluster reach (`extract_plots.py --cluster-eps-m`) instead merges
   extended structure (the phantom source) while point targets survive as single
   plots. Target ~60–100 plots/scan. Chosen: **eps=50 → ~82 plots/scan** (sweep:
   eps15=186, eps40=87, eps50≈82, eps70=56). Amplitude only as trim (not needed).
   **Provenance:** both A/B arms see the IDENTICAL decimated feed; record
   extraction params + row counts + checksums; eval-log carries the
   direction-of-bias note (decimation removes phantom load from the baseline →
   UNDERSTATES the occupancy benefit; conservative, acceptable) + the honest
   sentence that nobody feeds 169 plots/scan to a production tracker, so the
   decimated regime is arguably MORE deployment-representative.
2. **Window — re-probe timing on ONE decimated config first** (cost super-linear
   in density; 2× plot cut may buy >2× speed). Prefer 285 s if affordable
   (steady-state settling — the detector bar + confirmed cohort need the room;
   ~114 scans vs ~48 for 120 s), else 120 s. **3 stations first — one per SITE**
   (Kattwyk / Seemannshöft / Parkhafen, same hour 08 UTC) for geographic
   diversity per compute dollar. Confirm phantom-reduction DIRECTION, then scale.
3. **wall_time + peak RSS as first-class A/B columns.** If occupancy-ON cuts ~150
   phantom Bernoullis it should also collapse the Murty cost — "the static layer
   makes PMBM N× faster on dense harbor feeds" is worth as much as the GOSPA
   delta. wall_seconds is already a bench metric; RSS captured via
   `/usr/bin/time -v` per (station × config) invocation.
4. **AIS as a THIRD arm (radar+AIS), NOT folded into the core A/B.** Core =
   radar-only, ON vs OFF (clean, comparable to philos). AIS arm answers what only
   it can: veto mechanics with real AIS (vessel-fix hit counts, no suppression
   near AIS vessels, no cardinality damage in AIS lanes). **HARD RULE
   (circularity):** AIS is input AND truth there → NO accuracy-vs-AIS claims from
   AIS-fed runs; write that sentence into the eval-log BEFORE the numbers.

## A/B configs

**CORRECTION (2026-07-04, mid-run):** the ON arm is the Stage-1b-**ii** DETECTOR,
NOT the 1b-i standard config. Measured: `imm_cv_ct_pmbm_occupancy` and
`imm_cv_ct_pmbm_occupancy_sensitive` both classify **occ_peak_structures = 0** on
decimated HAXR → inert → A/B byte-identical to OFF (the same two-layer wall as
philos). The config that ENGAGES is `imm_cv_ct_pmbm_occupancy_detector` (1b-ii,
coarse 100 m grid + clutter-adaptive bar; occ_peak_structures = 31), whose own
comment says "A/B vs imm_cv_ct_pmbm_land." That is the correct ON arm.

- ON: `imm_cv_ct_pmbm_occupancy_detector` (Stage 1b-ii; universal decay).
- ON-6c: `imm_cv_ct_pmbm_occupancy_detector_coverage` (identical + coverage-aware
  decay — the increment-6c stale-pin arm).
- OFF: `imm_cv_ct_pmbm_land` (land inert on HAXR — no Hamburg coastline — so the
  delta isolates the occupancy layer; = imm_cv_ct_pmbm behaviourally on HAXR).

The first 3-site A/B (2026-07-04) mistakenly used `imm_cv_ct_pmbm_occupancy`
(inert) → byte-identical, which is itself the "standard-classifier inert on real
radar" finding (2nd geography after philos). Re-run with the _detector arm.

**Harness prerequisite (DONE, this plan): HaxrScenarioRun now (a) sets a nominal
fixed anchor datum** — HAXR is a local metre frame, and the Stage-1b occupancy /
land / obstacle wiring is gated on `scen.datum.has_value()`; without it the ON
arm is silently bit-identical to OFF — **and (b) reads HAXR_PLOTS_CSV /
HAXR_AIS_CSV / HAXR_STATIONS_CSV / HAXR_STATION** so the bench can point at
decimated per-station CSVs. Guard test:
`ReplayScenarioRun.HaxrScenarioCarriesDatumSoOccupancyWires`.

## Sequence

decimate (eps=50) → re-probe PMBM timing on decimated kattwyk_08 (also confirms
`occ_peak_structures>0`, i.e. the layer actually wires) → pick window → decimate
seemannshoeft_08 + parkhafen_08 → 3-station radar-only A/B (ON vs OFF, wall+RSS)
→ if phantom-reduction signal confirms → all 9 overnight + the AIS arm.

Bench invocation (per station × config), from project root:
```
HAXR_PLOTS_CSV=tests/fixtures/haxr_cfar/out/<st>_08_dec50.csv \
HAXR_AIS_CSV=data/dlr/<st>_08-UTC.csv HAXR_STATION=<st> \
/usr/bin/time -v ./build/bench/navtracker_bench_baseline --with-haxr \
  --scenario-filter haxr --config-filter <config> --out <dir>/ --run-id haxr_<st>_<cfg>
```

## RESULTS — DONE 2026-07-04

**Headline: the persistence occupancy detector is near-inert on real dense-harbor
radar.** Full numbers in the eval-log (2026-07-04 increment-8 entry). Summary:

- **Phantom-over-count reduction (the core question): near-null.** The engaged
  Stage-1b-ii detector (`_occupancy_detector` / `_detector_coverage`) cuts card_err
  by < 3 % across 9 station-hours (3 sites × 08/09/11), once slightly POSITIVE.
  Suppression is DECOUPLED from reduction (10 041 suppress hits → −1.18 card_err):
  the classifier catches persistent structure, but the over-count is diffuse
  clutter elsewhere. lifetime/missed identical (safe). No "N× faster".
- **Decimation confound RESOLVED** (undecimated kattwyk t40): the near-null holds
  undecimated (−0.45 %) as well as decimated (−0.72 %); decimation removed phantom
  load (baseline 113 vs 51) but hid no benefit. Real, not an artifact.
- **AIS third arm:** the corroboration veto is mechanically LIVE end-to-end on real
  AIS (740 fixes/site shift occ_suppress_hits + structure classification), no
  cardinality damage. Isolated benefit unmeasurable without a veto-ON/OFF toggle
  (entangled with AIS-tracking); accuracy circular → no accuracy claims. FOLLOW-UP:
  add a veto-disable flag to A/B the veto with AIS held constant.
- **Self-heal:** N/A on HAXR (camera-eviction property; HAXR is radar-only).

**Strategic conclusion:** persistence does NOT discriminate the phantom sources on
real radar — confirmed on 2 geographies (HAXR + philos) × 2 decimation regimes.
The over-count is diffuse clutter, not concentrated structure. The forward path is
CORROBORATION (AIS/camera — the veto) + a different clutter/birth model for the
diffuse over-count, NOT persistence tuning. This is a robust negative result that
redirects Stage 1b-ii away from persistence-suppression.

**Harness shipped this pass:** e5be99b (HAXR datum + per-station env — was inert),
5ae6117 (bench --config-eq), 0472eae (veto production wiring — was inert). All 9
station-hours decimated (clustering-first, 55–100 plots/scan) + windowed, md5-pinned.
