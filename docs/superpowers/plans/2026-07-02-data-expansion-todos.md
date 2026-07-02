# Data-Expansion TODOs (D1–D6) — 2026-07-02

Source: review of `data/` (manifest: `data/README.md`, fetched 2026-06-04)
against what the bench actually wires. Today the replay bench uses only
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

## D2 — Stone Soup GOSPA cross-validation [all claims — measurement integrity; ~half day]

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

## Suggested order

**D1 + D2 now** (both cheap; both strengthen the M2 gate everything else
leans on) → **D3** as a named input before R3/1b-ii design work starts →
**D4** at Stage 2 → **D5** with ADR 0001 A3 → **D6** unscheduled.
