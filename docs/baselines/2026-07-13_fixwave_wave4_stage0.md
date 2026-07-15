# Wave-4 Stage 0 — Cl-4 headline rows re-measured post-W2.4 (ADR-0003 reconciliation input)

Branch `fixwave-wave4` off post-merge master `34367f6` (which includes the
wave-2 merge). Purpose (ticket Stage 0): the wave-2 A/B scored the bench
*scenarios*, not the two frozen ADR-0003 headline rows. This re-runs the exact
adoption-freeze reproduce commands on post-W2.4 master and gives the arbiter the
three-row before/after so the dated ADR-0003 reconciliation addendum can be
written. **Delivered separately, ahead of the rest of wave 4.**

## Result — env-2 revival HELD 8/8; harbor unchanged; neither stop-trigger fired

| Row | Frozen (pre-W2.4, W25 adoption) | Post-W2.4 (this run) | Δ |
|---|---|---|---|
| **env-2** — autoferry 13/16/17/22, GOSPA (mean of 4 base) | **8/8, 13.38** | **8/8, 13.75** | **+0.37** |
| **env-1** — autoferry 2–6, GOSPA (mean of 5 base) | 16.57 | **15.49** | **−1.08 (improved)** |
| **harbor_complete_truth** — card_err (mean of 5 seeds) | 9.53 | **9.53** | **0.00** |

- **env-2 8/8 revival CONFIRMED**: all 8 shore-hugger targets tracked
  (`lifetime_ratio > 0`), 2/2 in each of sc13/16/17/22. GOSPA 13.75 (not the 0/8
  collapse value ~20), so no target loss — the +0.37 is a small localization /
  miss-timing shift, not a revival regression.
- **Neither STOP-AND-REPORT trigger fired** (env-2 ≥ 8/8; harbor did not move).
  The Cl-4 *claim* stands; only the env-2 GOSPA number moves (+0.37) and env-1
  improves (−1.08).
- **Cross-check that the "before" is right**: the frozen values reproduce
  exactly as the wave-2 W2.4 A/B *fix-OFF* base-scenario means (env-2 13.376→13.38,
  env-1 16.57, harbor 9.53) — so this Δ is purely the W2.4 effect, measured two
  independent ways.

## Attribution (from the wave-2 A/B mechanism)

The move is **W2.4a (coverage measured from own-ship, not the datum origin)** —
autoferry is a moving platform with no cooperative channel, so W2.4b (identity)
is inert here. Correct own-ship-centred coverage changes which sweeps count as
"covered", shifting miss-timing on the moving platform: env-1 (open-water)
tracking improves; env-2 (shore-strip) localization is fractionally worse but all
8 targets still revive. Per-scenario post-W2.4 GOSPA:

- env-2: sc13 14.74, sc16 11.10, sc17 15.58, sc22 13.60 (mean 13.75).
- env-1: sc2 11.53, sc3 18.45, sc4 14.46, sc5 15.52, sc6 17.50 (mean 15.49).

## philos (out of Stage-0 scope, noted)

The frozen philos row (+17.35 card / GOSPA 84.6, an accepted ADR-0003 deviation)
is **byte-identical under W2.4** (the wave-2 A/B showed philos fix-ON == fix-OFF —
philos own-ship drift stays inside the radar range so no coverage decision flips,
and its tracks carry AIS identity). So the accepted philos deviation stands
unchanged; W2.4 does not touch it.

## Reproduce (env unset = W25 shipped, on `fixwave-wave4` / post-merge master)

```
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_coverage_land_ivgate \
   --scenario-filter autoferry_scenario --seeds 1 --run-id af --out <dir>
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_coverage_land_ivgate \
   --scenario-eq harbor_complete_truth --seeds 5 --run-id hb --out <dir>
# env-2 = mean gospa_mean over autoferry_scenario{13,16,17,22}; revival = count of
# those scenarios' truths with lifetime_ratio>0 (8 total). env-1 = mean over {2..6}.
# harbor = mean card_err_mean over the 5 seeds. (Use LC_ALL=C for the arithmetic.)
```

**For the arbiter:** this is the ADR-0003 reconciliation input. env-2 revival is
intact (8/8); the deployable config's env-2 GOSPA moves 13.38→13.75 and env-1
improves 16.57→15.49 under the (correct) W2.4 own-ship coverage; harbor and
philos are unchanged. No re-freeze performed here.
