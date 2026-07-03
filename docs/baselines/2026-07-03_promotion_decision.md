# PDA promotion decision — real Trondheim coastline reality check

**Date:** 2026-07-03
**Question:** Promote `imm_cv_ct_pmbm_land_pda` (with the land-aware pool,
`imm_cv_ct_pmbm_land_pda_wateronly`) from opt-in to the recommended default,
replacing `imm_cv_ct_pmbm_land`?
**Decision: HOLD — do NOT promote. Keep opt-in.**

## Setup

The land-aware PDA pool was validated in sim (`shore_clutter_transit`, on-land
quay clutter → 10/10 seeds, pos_rmse halved). The standing methodology (sim
validates the mechanism; real replay is the reality check; never promote on sim
alone) requires a real-data confirmation. AutoFerry had been chartless for the
land model — the loader never set `Scenario::datum`, so no coastline could wire.

Fixed by sourcing the **real Trondheim inner-harbour coastline from OpenStreetMap**
(`tests/fixtures/autoferry/trondheim_harbor.geojson`, Overpass: natural=coastline
+ the Kanalen/Ravnkloløpet/Nidelva canals; assembled about the Piren datum;
100% of AutoFerry ground truth verified in water) and wiring it + the Piren datum
onto every AutoFerry scenario. Real geometry — not a hand-draped mask — is
deliberate: the land-aware pool's behaviour depends on where the land actually is
relative to the clutter that caused the regression, so a hand-drawn coast at the
load-bearing input would make the "real confirmation" half-synthetic (and could
be unconsciously drawn to coincide with the clutter → falsely optimistic).

Baseline: `docs/baselines/2026-07-03_promotion_autoferry_real_coast.csv`
(land / land_pda / wateronly × 18 AutoFerry scenarios, real coastline live).
Candidate for promotion = `_wateronly` (= `_land_pda` + land-aware pool, the best
PDA variant). philos numbers from `2026-07-02_pda_landaware_ab.csv`.

## Result (candidate `_wateronly` vs current default `_land`)

| Regime | metric | `_land` | `_land_pda` | `_wateronly` | Δcand | verdict |
|---|---|---|---|---|---|---|
| **Urban** (13/16/17/22) | pos_rmse | 15.67 | 18.88 | **17.77** | **+2.10** | ❌ regression not closed |
| | gospa_mean | 15.82 | 16.56 | 16.25 | +0.43 | ❌ |
| **Open-water** (2–6) | pos_rmse | 13.51 | 12.74 | **12.74** | **−0.77** | ✅ win retained |
| | gospa_mean | 17.69 | 17.41 | 17.41 | −0.29 | ✅ |
| **Anchored** (×9) | gospa_mean | 3.98 | 4.01 | 4.01 | +0.03 | ✅ flat |
| | lifetime | 0.918 | 0.917 | 0.917 | −0.001 | ✅ flat |
| **philos** | gospa_mean | 63.13 | 63.08 | 63.08 | −0.05 | ✅ flat |

Promotion required **all four** criteria to hold. Three do (open-water win
retained, anchored flat, philos flat). The load-bearing one — **urban regression
closed** — does **not**.

## Why the urban regression is only partly closed

The land-aware pool recovers ~⅓ of the plain-PDA urban regression (18.88 → 17.77)
but `_wateronly` is still **+2.10 m pos_rmse worse than `_land`**. On the real
Trondheim channels much of the clutter PDA pools is **in the water**, not on land:
moored vessels, floating structures, and near-shore-but-offshore returns whose
`clutterPrior < 0.5`. The land mask, by construction, cannot flag these — it only
excludes on-land quay returns. The AutoFerry loader's own detection-table comments
already note the urban camera excess is "persistent structured returns
(shoreline, **moored vessels**)"; the real geometry confirms a large in-water
share. The sim fixture (on-land dock clutter) was therefore necessary but not
sufficient: it proved the mechanism works *for on-land clutter*, and the real
replay shows on-land clutter is only part of the real problem.

Open-water is unaffected (land far from the vessels ⇒ `_land == adapt`; the PDA
open-water win survives), and `_wateronly == _land_pda` there and on philos (no
in-gate shore returns / dense-scene pool ≈ {winner}).

## Decision & consequence

- **`imm_cv_ct_pmbm_land_pda` and `_wateronly` stay OPT-IN.** `imm_cv_ct_pmbm_land`
  remains the recommended general/coastal default.
- **The K=1 north-star item stays "shipped (opt-in)", not promoted.** The PDA
  soft update + land-aware pool close the open-sea/open-water K=1 gap and are
  safe everywhere, but do not net-improve the mixed real-world workload because
  of the in-water urban clutter they cannot address.
- **Residual / next idea (not this gate):** the in-water structured-clutter pull
  is an association/existence problem, not a land-mask one — candidates are a
  β₀ miss term, confirmed-only softening
  (`pda_soft_detected_branch_on_confirmed_only`), or treating persistent in-water
  structure via the live static-occupancy layer (Stage 1b). None promoted here.

This is the sim-primary / real-reality-check split working as intended: the sim
gate said "mechanism sound", the real gate said "not a universal default yet".
