# Cl-4 endgame — the coverage-cliff price list (gate-floor sweep 0.05–0.10)

**Date:** 2026-07-11 · **Branch:** `cl4-endgame-cliff-sweep` · **North-star:** Cl-4 endgame.
**Measurement only — no shipped default change, no recommendation.** The price *decision*
is the user's. Ticket: `docs/superpowers/plans/2026-07-11-cl4-endgame-cliff-reprice-ticket.md`.

## What was swept

One dial: `min_new_bernoulli_existence` ∈ {0.05…0.10} on the dossier candidate
`imm_cv_ct_pmbm_coverage_land_ivgate`, everything else fixed. Applied via the sanctioned
env-sweep knob `PMBM_MIN_NEW_BERN` (the A1 method; `probeEnvD`, byte-identical when unset —
suite verified). Under this config the birth path has no occupancy model, so pre-gate birth
existence is exactly `r_new = birth_existence_target·(1 − c_land) = 0.1·(1 − clutterPrior)`,
and the floor admits a birth iff `r_new ≥ floor`. The whole curve is that one inequality.

## The price list (no winner declared)

| floor | env-2 tracked | env-2 GOSPA | env-1 GOSPA | philos card_err | philos GOSPA | philos gospa_false | harbor card_err (5-seed) | harbor life |
|---:|:--:|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | **8/8** | 18.34 | 16.57 | **+36.15** | 100.03 | 9000 | 9.53 | 0.974 |
| 0.06 | **8/8** | 18.34 | 16.57 | +29.35 | 93.11 | 7640 | 9.53 | 0.974 |
| 0.07 | **8/8** | 17.14 | 16.57 | +25.00 | 89.44 | 6810 | 9.53 | 0.974 |
| **0.08** | **8/8** | **13.72** | 16.57 | +19.35 | 83.76 | 5700 | 9.53 | 0.974 |
| 0.09 | 3/8 | 18.45 | 16.57 | +14.10 | 79.46 | 4770 | 9.53 | 0.974 |
| 0.10 | 0/8 | 20.00 | 16.57 | +6.90 | 73.06 | 3550 | 9.53 | 0.974 |

(0.10 = the shipped default = the A1 "floor 0.10" endpoint; 0.05 reproduces the A1 "full
revival" endpoint at card_err +36 / gospa 100. "env-2 tracked" = of the 8 shore-hugger
targets, those with lifetime_ratio ≥ 0.1.)

**No collateral, and provably so.** Harbor (5-seed card_err 9.53, lifetime 0.974) and env-1
GOSPA (16.57) are **identical at every floor**. Reason: both are open-water/chart-free, so
their births have `c_land = 0 → r_new = 0.1 ≥` every swept floor — no birth is ever gated.
The floor touches **only** land-ramp (in-band) births. Imazu shares this property (the
Imazu battery carries no coastline, so it is floor-inert by the same construction; its
fixtures are absent from the worktree — a named skip — so it was not run directly, but it
cannot move). This also means the floor never touches the confirmed harbor **pier**
phantoms (they are chart-free, r_new 0.1), consistent with the harbor row not moving.

## The env-2 cliff, per target (why revival is binary)

| floor | per-target env-2 lifetimes (8 targets) | tracked |
|---:|---|:--:|
| 0.07 | 0.86 0.66 0.65 0.61 0.45 0.15 0.56 0.45 | 8/8 |
| 0.08 | 0.77 0.66 0.65 0.61 0.40 0.15 0.47 0.41 | 8/8 |
| 0.09 | 0.00 0.39 0.61 0.13 0.00 0.00 0.00 0.09 | 3/8 |
| 0.10 | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | 0/8 |

Revival is **all-or-nothing between 0.08 and 0.09**: full at ≤ 0.08, near-total collapse at
0.09, total at 0.10. The philos phantom price, by contrast, is **smooth** (each 0.01 of
floor ≈ 1200–1400 gospa_false, ≈ +5 card_err).

## Why the curve bends there — the r_new distributions

`r_new = 0.1·(1 − c_land)`, per in-band birth:

- **env-2 shore-huggers (vessels):** r_new ∈ **[0.065, 0.092]**, median ~0.078 — a tight
  band. Fraction of a target's in-band positions that survive the floor: ~100% at ≤ 0.07,
  38–79% at 0.08 (still enough to initiate → tracked), 0–4% at 0.09 (gated → collapse). The
  vessels are ~20–35 m offshore, sitting on the middle of the ramp.
- **philos near-shore clutter (radar in-band, 1706 returns):** **bimodal** — median r_new
  = 0 (returns right at the shoreline, `c ≈ 1`, gated at every floor) with a **tail up to
  0.10**. Fraction surviving: 20% at 0.05, 8% at 0.07, 5% at 0.08, 2% at 0.09, 0% at 0.10.

**The bands overlap.** The vessel band (0.065–0.092) sits *inside* the philos clutter tail
(which reaches 0.10). So there is no floor that admits the vessels while excluding the
clutter tail: any floor ≤ 0.08 (needed for full revival) necessarily admits ~5% of philos
clutter (≈ the +19 card_err), and lower floors admit progressively more of the tail. The
separation the knee would need — vessel r_new sitting *above* clutter r_new — **does not
exist**; the clutter tail and the vessels occupy the same 0.07–0.09 band. (This is the
occupancy-space echo of the Phase-1c result: near-shore clutter and near-shore vessels are
not separable by the signal in hand — here, ramp depth.)

## Knee analysis

Ticket knee test: **a floor where ≥ 6/8 env-2 track AND philos card_err ≤ ~+12.**

- ≥ 6/8 env-2 requires floor **≤ 0.08** (8/8 at 0.08; only 3/8 at 0.09).
- At floor 0.08 philos card_err is **+19.35** (> +12).
- philos card_err drops to ≤ +12 only at floor **≥ ~0.093** — where env-2 is already < 6/8.

**No knee.** The margin-robust region for "both" is empty; it is a graded trade, and the
two axes cross on the wrong side. Stated plainly per the house rule: there is no free
lunch here — the env-2 cliff (0.08) and the philos ≤+12 threshold (~0.093) do not overlap.

## The trade, named (what each candidate setting costs and when it hurts)

- **floor 0.08 — "full revival" point.** Every env-2 shore-hugger tracks (8/8) and env-2
  GOSPA is **13.72 — better than `pmbm_land`'s 17.74** (the per-geography best), so this
  *meets* the Cl-4 env-2 definition-of-done with margin. **It hurts on philos**: card_err
  +6.9 → **+19.35**, GOSPA 73.1 → **83.76**, gospa_false 3550 → 5700 — i.e. ~5% of the
  near-shore clutter tail is admitted as phantom tracks. Where it bites operationally:
  extra phantom tracks at cluttered fixed-shore harbours (the philos/Boston regime), traded
  for real tracking of vessels hugging an urban channel (the env-2/Trondheim regime).
- **floor 0.10 — shipped default.** philos clean (+6.9 / 73.1) and harbor untouched, but
  env-2 collapses (0/8, the documented < 50 m no-birth zone). Hurts exactly where Cl-4
  exists to help: shore-hugging channel traffic is invisible.
- **floors 0.05–0.07 — deeper revival, higher price.** No extra env-2 benefit over 0.08
  (already 8/8; GOSPA slightly *worse* at 0.05–0.07 because more clutter is admitted), and
  a steadily larger philos phantom bill (up to +36 card_err / gospa 100 at 0.05). Strictly
  dominated by 0.08 for env-2; only relevant if a deployment wants margin below the 0.08
  vessel r_new floor.
- **floor 0.09 — the cliff's edge.** philos +14.10 (near the +12 comfort line) but env-2
  already 3/8 — the worst of both: most vessels lost AND philos degraded.

## Decision (the user's)

If a single global floor must ship, the meaningful choices are **0.08** (revive the
channel, pay ~+12.5 philos card_err) or **0.10** (keep philos clean, accept the no-birth
zone). There is no floor that does both. A per-geography floor would rebuild the exact seam
Cl-4 exists to remove, so it is not on the table as a "one config." **No shipping default is
recommended here** — the follow-up (setting the default + ADR-0001 third amendment, guide,
learning docs) is a separate ticket once the price point is picked.

## Reproduce

```
# byte-identical when unset; sweep one value per run
for F in 0.05 0.06 0.07 0.08 0.09 0.10; do
  PMBM_MIN_NEW_BERN=$F ./build/bench/navtracker_bench_baseline \
    --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-filter autoferry_scenario --seeds 1 --run-id af_$F --out <dir>
  PMBM_MIN_NEW_BERN=$F ./build/bench/navtracker_bench_baseline \
    --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-eq philos --seeds 1 --run-id ph_$F --out <dir>
  PMBM_MIN_NEW_BERN=$F ./build/bench/navtracker_bench_baseline \
    --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-eq harbor_complete_truth --seeds 5 --run-id hb_$F --out <dir>
done
# r_new distributions (the explanation)
./build/bench/navtracker_cl4_a3_census --mode rnew --scenario autoferry_scenario16
PHILOS_CLIP=ais_ferry_near ./build/bench/navtracker_cl4_a3_census --mode rnew --scenario philos
```
Fixtures symlinked from the main tree (gitignored, not committed).
