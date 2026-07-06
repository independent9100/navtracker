# Data-Expansion TODOs (D1–D12) — 2026-07-02 (D7–D12 added 2026-07-03)

Source: D1–D6 from review of `data/` (manifest: `data/README.md`, fetched
2026-06-04) against what the bench actually wires. D7–D12 from an external
sweep (2026-07-03) for free datasets/benchmarks the manifest missed. Today the replay bench uses only
**philos** (1 of 7 extracted clips), **autoferry** (9 scenarios), and **HAXR**
(one station-hour, `kattwyk_08`, first 40 s). Everything else (~2.3 GB) is
fetched but unused; `pohang_radar`, `pohang_lidar`, and `smd_eo` already have
half-built fixtures under `tests/fixtures/`.

Ranked by value to the current front (Cl-3 / Stage 1b / anchored-vessel
work). Each ticket is self-contained.

---

## D1 — HAXR multi-station / multi-hour expansion [Cl-3, serves the M2 gate; ~1–2 days]

**What.** Bench currently wires exactly one HAXR station-hour
(`tests/fixtures/haxr_cfar/out/kattwyk_08_t40.csv`, first 40 s). On disk:
3 stations × 3 hours (`kattwyk`, `seemannshoeft`, `parkhafen` × 08/09/11 UTC,
`data/dlr/`), loader shipped (`HaxrScenarioRun`), extraction script exists
(`tests/fixtures/haxr_cfar/extract_plots.py`), and the HAXR truth-sort bug was
just fixed (commit 3aa9c58) so multi-hour replay is safe.

**Why.** Hamburg port is a **second real harbor** — piers, shore structure,
non-AIS traffic — i.e. the same phantom-track workload as philos but a
different geography and radar. Exactly the cross-check Stage 1b needs to show
the occupancy layer is not overfit to Boston. Same AIS-only-truth caveat as
philos (non-AIS targets score as "false"), so like philos it is a
**cross-check, not the gate** (`harbor_complete_truth` stays the gate).
Multi-station also opens a future overlapping-coverage test no other source
offers.

**Do.**
1. Run `extract_plots.py` for the remaining 8 station-hours → `out/` CSVs.
2. Register them as bench scenarios (parametrize `HaxrScenarioRun` on
   station/hour instead of the single hardcoded path).
3. Baseline under `imm_cv_ct_mht`, `imm_cv_ct_pmbm`, `imm_cv_ct_pmbm_land`
   (needs a Hamburg coastline GeoJSON for the land model to be non-inert —
   scope that separately if not trivial); record in the eval log.

**Acceptance.** ≥ 8 new `haxr_*` bench scenarios, deterministic, truth
time-sorted (contract test), baselines in the eval log, north-star row updated.

---

## D2 — Stone Soup GOSPA cross-validation [all claims — measurement integrity; ~half day] — **DONE 2026-07-06**

**Status (2026-07-06): SHIPPED, PASS.** Exporter `core/benchmark/GospaExport`
(+ `--export-states-dir` on `navtracker_bench_baseline`, dumping the harness's
own `BenchResult` so the *same* tracks are re-scored) + venv-local
`tools/stonesoup_gospa_crosscheck.py`. Stone Soup's `GOSPAMetric` at matched
convention (c=20, p=2, α=2, switching=0) == `core/scenario/Gospa.hpp` to
floating-point epsilon (max per-scan |Δ| = 1.42e-14) on **one sim
+ one real** run: `harbor_complete_truth` (40 scans) and `philos` (20 scans,
real ARPA). Per-scan loc/missed/false and cardinality counts agree on every
scan. Result + convention table: `docs/algorithms/gospa-crosscheck.md`;
eval-log entry 2026-07-06. Parked: time-series/switching (T-GOSPA) arm, OSPA
arm, and (out of scope by design) Stone Soup's own trackers as a baseline.

**What.** Score at least one benched run's tracks with
`stonesoup.metricgenerator`'s GOSPA (repo already cloned at
`data/stonesoup/Stone-Soup/`) and confirm `core/scenario/Ospa.hpp` /
`Gospa` agree within tolerance.

**Why.** Every promotion decision hangs on GOSPA deltas, and the harness has
now had **two** truth-fragmentation bugs silently corrupt metrics (autoferry
2026-06-10, harbor 2026-07-02). An externally-authored metric implementation
agreeing with ours is the cheapest possible hedge against a metric bug.
`data/README.md` already names this play.

**Do.** Export one scenario's (truth, tracks) per-step sets to CSV/JSON;
small Python script feeds them to Stone Soup's GOSPA with our (c, p, α);
compare per-step and mean values; commit the script + a doc note (where the
comparison lives, tolerance, any definitional differences found).

**Acceptance.** Documented agreement (or a documented, explained
discrepancy) for total GOSPA + localisation/missed/false decomposition on at
least one synthetic and one replay scenario.

---

## D3 — MarineCadastre/DMA anchored-vessel mining + density soak [Cl-3 (R3/1b-ii input) + robustness; ~1–2 days]

**What.** Two independent uses of the bulk AIS archives
(`data/marinecadastre/AIS_2024_01_01.zip`, `data/dma/*.7z`):

1. **Anchored-vessel statistics for the 1b-ii AIS-corroboration design.**
   MarineCadastre carries the `Status` column (nav-status 1 = at anchor,
   5 = moored). Mine real distributions: how anchored vessels report, drop
   out, and transition to underway; dwell times; position jitter at anchor
   (watch-circle size). Output: a short stats doc + a replay/synthetic
   scenario dense with genuinely anchored vessels, as ground material for
   the "nav-status 1/5 → vessel, never suppress" corroboration rule (review
   ticket R3) and the stop→go stable-id regression test (north-star
   anchored-vessel-safety row).
2. **Scale soak.** Hundreds of simultaneous AIS targets through the tracker:
   runtime, memory, track-id churn, association scaling. Not accuracy-grade
   (AIS is both input and truth) — a robustness/perf scenario, not an OSPA
   scenario.

**Acceptance.** Stats doc committed; one anchored-dense scenario registered;
soak numbers (targets, wall time, peak tracks, id churn) in the eval log.

---

## D4 — Pohang radar: moving-platform occupancy validation [Cl-3 Stage 2; BLOCKED — park until Stage 2]

**What.** `tests/fixtures/pohang_radar/` is half-built
(`pohang00_plots.csv` exists) but its README lists three unresolved
conventions (max-range scale m/px, heading column, image azimuth reference),
and there is **no target ground truth** until the PoLaRIS extension ships —
so no OSPA-grade scoring.

**Why later.** Canal walls and port structure seen from a **moving** vessel
is precisely the geo-referenced-occupancy problem the design spec's "we move"
caveat calls out (§14.10). When Stage 2's occupancy grid exists, Pohang is
the qualitative validation that the grid doesn't smear under ego-motion; the
lidar plots CSV (`tests/fixtures/pohang_lidar/out/`) gives the same geometry
from a second modality.

**Unblock steps (when picked up).** Resolve the three conventions per the
fixture README's recovery recipes; then wire a `PohangScenarioRun` with
cardinality/occupancy-stability assertions (no truth → no GOSPA).

---

## D5 — SMD EO bearing-only fixture [Cl-2/Cl-3 later; park until camera axis opens]

**What.** `tests/fixtures/smd_eo/` is mostly extracted (near-perfect
bearing-only detections from a *stationary* shore camera + per-frame GT).
Exercises the bearing-only path cleanly.

**Why later.** Its real role is the sensor-aware near-shore birth work
(ADR 0001 A3 "needs camera" — north-star anchored-vessel-safety row): a
camera corroboration channel needs bearing-only test material. Pick this up
when that ticket starts, not before.

---

## D6 — Kystverket realtime soak [side quest; unscheduled]

**What.** 5 realtime GeoJSON snapshots on disk + a no-auth polling endpoint.
Long-duration soak: genuine arrival/departure dynamics and MMSI churn over
hours.

**Why unscheduled.** Advances no current claim; infra-level stability value
only. Revisit when a long-running deployment soak is actually needed.

---

---

# External candidates from the 2026-07-03 sweep (D7–D12)

Context for the sweep: every real radar source we currently use (philos,
HAXR, autoferry-radar) scores against **AIS-only truth** — non-AIS targets
count as "false", which is the standing caveat on all real-data claims
(and the reason `harbor_complete_truth` is the gate). D7 and D8 are the
first candidates that break that limitation. All D7–D12 are
**evaluate-before-extract**: dataset papers oversell; each ticket's first
step is a license/format/truth-quality check, and the ticket dies cheaply
if that check fails.

---

## D7 — MOANA: real radar with NON-AIS truth + anchored vessels [Cl-3, 1b-ii evidence; evaluate ~half day, extract ~1–2 days]

**What.** Multi-radar maritime dataset (marine X-band + W-band scanning
radar) with ground-truth object labels derived from **camera + lidar, not
AIS**, and scenes that explicitly contain many anchored large vessels.
Paper: arXiv 2412.03887 ("MOANA: Multi-Radar Dataset for Maritime Odometry
and Autonomous Navigation Application").

**Why.** Hits our two sorest evidence gaps at once: (1) real marine radar
where a non-AIS target is *labelled*, not silently scored as false — the
first real-data source where "false track" actually means false; (2) real
radar signatures of anchored vessels, the exact object class the 1b-ii
detector must NOT suppress (KEEP-guard material that today rests entirely
on philos video labelling).

**Do.** (1) Feasibility: license, download size, label quality/coverage,
whether X-band plots are extractable to our `radar_plots.csv` shape.
(2) If pass: fixture extraction + a labelled replay scenario; score the
occupancy detector's SUPPRESS/KEEP split against the dataset's own labels
(same decomposition as the R8 philos gates, but with real truth).

**Acceptance.** Feasibility note in the eval log (go/no-go). If go: one
fixture + one label-scored replay scenario; detector confusion numbers
(structure vs anchored-vessel) recorded.

---

## D8 — R-BAD: 69 h radar berthing dataset + synced video [Cl-3 steady-state + labels; evaluate ~half day]

**What.** Radar-Based Berthing-Aid Dataset: 69 hours of FMCW radar point
clouds with timestamped synced video of ship docking operations, stated
freely accessible. Paper: MDPI Electronics 14(20):4065 (2025).

**Why.** Berthing = piers, moored vessels, near-shore structure — the
philos regime — but at *hours* of duration instead of 20 s clips, so it
can answer the steady-state / confirmed-cohort question (increment 8)
on a second geography, and the synced video supports the same
label-pass workflow we built for philos (R8) without us shooting our own
footage.

**Do.** (1) Feasibility: license, radar type/geometry vs our plot model,
video alignment quality. (2) If pass: extract 1–2 station-hours as
fixtures; video label pass (same method as sunset_cruise/close_approach);
register a label-scored replay.

**Acceptance.** Feasibility note (go/no-go); if go, ≥1 hour-scale fixture
+ labels + baseline in the eval log.

---

## D9 — DLR extended-target campaigns: overlap check [cheap curiosity; ~1 h]

**What.** The DLR institute behind HAXR also publishes separate marine
radar datasets for (multiple) extended target tracking with AIS reference
(elib.dlr.de/129565, Sensors 2021 10.3390/s21144641; download page:
dlr.de → Institute of Communications and Navigation → nautical systems).
Our manifest only covers the HAXR station-hours.

**Why.** Possibly distinct campaigns (different geometry/platform) at
near-zero marginal cost — we already have their HDF5 access layer and
loader conventions. Extended-target emphasis is also relevant to R3
(extent-is-interim).

**Do.** Ten-minute check whether these are the same files as `data/dlr/`;
if distinct, size up and decide whether to fold into D1's extraction pass.

**Acceptance.** One-line manifest note: same/distinct, and if distinct,
fetch-or-skip decision with reason.

---

## D10 — Global Fishing Watch anchorages + events [replaces most of D3's mining; ~half day]

**What.** Free (registration) curated global anchorage database —
locations where ≥20 unique vessels sat stationary since 2012 — plus
vessel-event APIs (anchorage visits, port calls).
globalfishingwatch.org/datasets-and-code-anchorages/.

**Why.** Two uses. (1) D3's goal — anchored-vessel dwell/transition/
watch-circle statistics for the AIS-veto and corroboration design — may
be largely pre-computed here, cheaper than mining raw MarineCadastre.
(2) The anchorage polygons themselves are a candidate **chart-corroboration
prior** for the 1b-ii KEEP-guard: "this water is a known anchorage →
bias KEEP" (the exact opposite polarity of the charted-structure
SUPPRESS prior — both feed the same corroboration seam).

**Do.** Register; pull the anchorage layer for the philos/HAXR
geographies; sanity-check the Boston 42.3585N anchorage (our video-
verified KEEP cluster) appears; assess event-API stats vs D3's needs.

**Acceptance.** Anchorage extract for ≥1 bench geography committed as a
fixture candidate + a note on how much of D3 it renders unnecessary.

---

## D11 — Reeds: moving-platform radar with precision ego-truth [Stage 2 alternative to D4; park until Stage 2]

**What.** Chalmers/RISE instrumented boat (Gothenburg): 360° radar, 3
lidars, 6 cameras, fibre-optic-gyro + 3-antenna GNSS ego-truth.
reeds.opendata.chalmers.se.

**Why later.** Same role as D4 (Pohang): does the geo-referenced
occupancy grid smear under ego-motion — but *unblocked* (documented
conventions, precision ego pose) where Pohang has three unresolved
conventions and no truth. When Stage 2 starts, evaluate Reeds FIRST and
keep Pohang only if Reeds fails feasibility. Caveat: raw volumes are
huge (GB/s-class cameras) — extract radar + ego pose only, never mirror.

---

## D12 — WaterScenes: truth-complete 4D radar (wrong sensor physics) [cross-check only; unscheduled]

**What.** USV with automotive-style 4D mmWave radar + camera, point-level
radar annotations (arXiv 2307.06505; waterscenes.github.io).

**Why unscheduled.** Truth-complete radar — but mmWave on inland water is
not marine nav radar; clutter/extent statistics don't transfer. Legit
only as a logic cross-check of the detector (does SUPPRESS/KEEP behave
sanely under a *different* radar with full truth), never for tuning.
Revisit if we want a third independent detector sanity check after
MOANA/R-BAD.

**Camera benchmarks (MVTD, MODS, SeaDronesSee, LaRS):** noted, parked
with D5 — they become relevant when the camera corroboration axis
(ADR 0001 A3) opens, not before.

---

## Suggested order

**D1 + D2 now** (both cheap; both strengthen the M2 gate everything else
leans on) → **D7 + D8 feasibility checks** (half day each; first real
radar with non-AIS truth — directly de-risks the 1b-ii promotion story)
→ **D9** whenever someone touches `data/dlr/` next → **D10** before the
AIS-veto/chart-corroboration design lands (it may supply the prior) →
**D3** only for what D10 doesn't cover → **D11** at Stage 2 (before D4)
→ **D5** with ADR 0001 A3 → **D6/D12** unscheduled.
