# AutoFerry real-data A/B — PDA soft detected-branch update

**Date:** 2026-07-02
**Question:** Should `imm_cv_ct_pmbm_land_pda` (PDA soft detected-branch
update, commit 68c845e) be promoted from opt-in to the recommended
general/coastal default?
**Data:** `docs/baselines/2026-07-02_autoferry_pda_ab.csv`
**Command:**

```
navtracker_bench_baseline --run-id 2026-07-02_autoferry_pda_ab \
  --config-filter imm_cv_ct_pmbm_land --scenario-filter autoferry
```

A = `imm_cv_ct_pmbm_land` (baseline) · B = `imm_cv_ct_pmbm_land_pda`
(only delta: `use_pda_soft_detected_branch = true`). 9 canonical +
9 anchored AutoFerry replays (open-water env 1 = scenarios 2–6;
urban-channel env 2 = 13/16/17/22). Replays are deterministic
(seed 0); the per-scenario spread is the real-data error bar.

## Verdict: DO NOT promote. Keep opt-in.

The result is **regime-split**, not a clean win:

| Regime | n | lifetime Δ | gospa_mean Δ | pos_rmse Δ | gospa_false Δ | id_switch Δ | read |
|---|---|---|---|---|---|---|---|
| **Open-water** (2–6) | 5 | +0.0014 | −0.29 | **−0.77** (5/5 better) | −0.6 | −1.1 | mild **win** (as designed) |
| **Urban channel** (13/16/17/22) | 4 | +0.0068 | **+0.70** (4/4 worse) | **+3.2** (4/4 worse, +20 %) | +9.4 | +1.4 | mild **regression** |
| **Anchored** (all 9) | 9 | −0.0007 | +0.03 | +0.10 | +0.05 | +0 | **flat** |
| **Net canonical** | 9 | +0.0038 | +0.15 | +0.99 | +3.8 | +0 | wash / slightly negative on accuracy |

- **Open-water = the target regime** (the open-sea K=1 gap PDA was
  built to close). It improves exactly as designed and as the sim
  predicted: gospa_missed −3.5, gospa_mean −0.29, id_switches −1.1,
  and **pos_rmse lower on all 5 scenarios** (13.51 → 12.74 m). No
  open-water regression.
- **Anchored (all 9) = flat.** The anchored-scenario regression that
  disqualified "just raise K" is **not** tripped by PDA — the one hard
  gate passes. (Anchored replays inject a truth-AIS anchor, tracks are
  well-established, in-gate returns are already claimed ⇒ pool ≈ 1 ⇒
  the update reduces to today's hard update. Matches the design.)
- **Urban channel = mild regression.** gospa_mean +0.70 (all 4 worse),
  **pos_rmse +3.2 m / +20 %** (all 4 worse), gospa_false +9.4
  (3/4 worse; scenario16 alone +34), id_switches +1.4. The unclaimed
  structured **shore/dock clutter** enters the PDA pool and pulls
  tracks toward it. The sim harbor over-count *drop* (a large target's
  own hull returns pooling constructively) did **not** generalise to
  real urban shore clutter.

Net across the 9 canonical scenarios the urban regression roughly
cancels the open-water gain, leaving accuracy metrics (gospa_mean,
ospa, pos_rmse) slightly *worse* while lifetime is marginally up. That
is not a promotion case for a config that is also the general default.

## Why this is the right call (methodology)

Sim + philos (last turn) validated the *mechanism* cleanly — open-sea
lifetime 0.823 → 0.847, over-count down, philos flat, flag-off
byte-identical. The real replay is the reality check, and it caught
**model-matched optimism**: the extended-target over-count *drop* was a
sim artefact (constructive hull-return pooling) that does not transfer
to real structured shore clutter, which instead *adds* false pull. This
is exactly the sim-primary / real-reality-check split doing its job —
we do not ship on sim alone.

## Important caveat: AutoFerry ships no coastline

The land mask is **inert** on AutoFerry (no coastline loaded ⇒
`imm_cv_ct_pmbm_land` is byte-identical to plain `adapt`). So this A/B
measures **PDA in isolation, with no shore suppression on the pool**.
The urban regression is precisely unclaimed-shore-clutter pull that a
wired coastline + a land-gated pool would exclude. It is therefore a
*pessimistic* view for a charted coastal deployment — but a fair view
for the chartless open-water/general case the default must also serve.

## Principled next step (blocks promotion until done)

**Land/coastline-aware PDA pool.** Exclude returns that fall inside the
land-clutter zone (the ADR 0001 offshore band / signed-shoreline ramp)
from the β pool, so PDA softens against *water* clutter only. This
should keep the open-water win and remove the urban regression. It ties
directly to ADR 0001 (land-clutter no-birth-zone) and the sensor-aware
suppression principle. Re-run this exact A/B **with a coastline wired
for the urban scenarios** to confirm before reconsidering promotion.

Secondary candidates (smaller, do after the pool fix):
- β₀ miss-term variant (leave existence/covariance un-shrunk on the
  missed-detection share) — may lift open-water lifetime further.
- `pda_soft_detected_branch_on_confirmed_only` (restrict softening to
  confirmed tracks) — could cut the urban tentative-track pull.

## Status

`imm_cv_ct_pmbm_land_pda` **stays opt-in**. `imm_cv_ct_pmbm_land`
remains the recommended general/coastal default. Promotion is blocked
on the land-aware-pool fix + a coastline-wired urban re-measure.
