# Cl-4 endgame — the coverage-cliff price list (gate-floor sweep 0.05–0.10)

**Date:** 2026-07-11 · **Branch:** `cl4-endgame-cliff-sweep` · **North-star:** Cl-4 endgame.
**Measurement only — no shipped default change, no recommendation.** The price *decision*
is the user's. Ticket: `docs/superpowers/plans/2026-07-11-cl4-endgame-cliff-reprice-ticket.md`.

> **PART 1 (original, floor-only) is below.** **PART 2 (ticket amendment, 2026-07-11
> spatial-split directive)** adds the second dial `offshore_halfwidth_m`, the
> distance-from-shore geography, the 2-D price surface, and the phantom *map* (where the
> re-admitted phantoms sit). Jump to [Part 2](#part-2--the-spatial-split-2-d-surface--phantom-map).
> The Part-2 headline: the two dials are **redundant on total cost** (iso-cost ridges), but
> **not on phantom geography** — narrowing `offshore_halfwidth_m` revives all 8 env-2
> vessels at the clean floor (0.10) while adding phantoms almost entirely inside the
> near-land strip and *not* growing the far open-water phantom field, whereas lowering the
> floor spills phantoms into open water. Under the user's ruling (strip phantoms accepted,
> open-water leak not), that makes **`W_off=25 m, floor=0.10` the spatially-cleaner knee
> candidate** — reported with counts; the carriable-count decision is the user's.

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

---

# Part 2 — the spatial split: 2-D surface + phantom map

The user's 2026-07-11 directive reshaped the question: keep the strict zone where structure
actually lives (inner strip) but be permissive in the outer strip where channel vessels hug,
because near-land waters are operator-supervised — a phantom *there* is an accepted price,
an invisible real mover is not. This adds a **second dial**, the offshore ramp half-width
`offshore_halfwidth_m` ∈ {50, 35, 25, 15} m, swept alongside the floor. Instrument: the same
sanctioned env-sweep method, a new `PMBM_OFFSHORE_HALFWIDTH_M` knob applied at the GeoJSON
`CoastlineModel` construction site (`Sweep.cpp`), byte-identical when unset (suite verified),
NOT a deployment surface — the ramp math in `CoastlineGeometry` is untouched, only its two
half-width params move. Mechanically: `c(d, W_off) = clamp((W_off − d)/(W_off + W_in), 0, 1)`
with `W_in` fixed at 50 m, so narrowing `W_off` slides the `c=0` boundary **shoreward** — a
vessel beyond the narrowed strip jumps to `c=0 → r_new = 0.1`, escaping the gate at every floor.

## The geography measurement FIRST — does the split *can* work?

Signed distance-from-shore (m, `<0` inland), via a wide-ramp reference `CoastlineModel`
(`W_off=W_in=5000 m` never saturates in-band, so `d = 5000 − 10000·c` is exact). Tool:
`--mode geo`.

| population | n | median d | where it sits |
|---|---:|---:|---|
| env-2 vessels (truth, in-band) | 7874 samples | ~25–31 m | band **6–42 m**, none inland |
| philos in-band radar — **inland (structure)** | 1361 | −111 m | behind the waterline (radar on land) |
| philos in-band radar — **offshore (water clutter)** | 345 | 14 m | **0–50 m, dense at 0–10 m (137), tail to 40–50 m (41)** |

The fact the split hangs on: **the philos offshore water clutter and the env-2 vessels
overlap in distance-from-shore.** The clutter is densest at 0–10 m and *thins* with distance
(137 → 78 → 52 → 37 → 41 across the 0–50 m decades), while the vessels ride the 20–40 m band —
so there is a density gradient (inner strip is clutter-dominated, vessels live mid-strip), but
the clutter tail reaches through and past the vessel band (41 returns at 40–50 m). No inner/outer
cut cleanly separates them. (Aside: the shipped `LiveOccupancyModel` persistence grid flags **0**
philos structure cells at 25 m / bar 0.5 — the in-band clutter is diffuse, not a stationary
point-structure, corroborating the increment-8 "diffuse clutter not structure" finding. On the
autoferry synthetic coastline the same grid *does* flag 17–33 cells, so the tool is working.)

## The 2-D price surface (`offshore_halfwidth_m` × floor)

env-2 tracked = of the 8 shore-hugger targets (4 scenarios × 2), those with `lifetime_ratio ≥
0.1`. env-2 GOSPA = mean of the four scenarios' `gospa_mean`. philos single-seed replay.

| W_off | floor | env-2 trk | env-2 GOSPA | philos card_err | philos GOSPA | gospa_false |
|---:|---:|:--:|---:|---:|---:|---:|
| 50 | 0.08 | **8/8** | 13.72 | +19.35 | 83.76 | 5700 |
| 50 | 0.09 | 3/8 | 18.45 | +14.10 | 79.46 | 4770 |
| 50 | 0.10 | 0/8 | 20.00 | +6.90 | 73.06 | 3550 |
| 35 | 0.09 | **8/8** | 13.49 | +20.95 | 85.36 | 6000 |
| 35 | 0.10 | 4/8 | 17.44 | +13.00 | 79.92 | 4770 |
| **25** | **0.10** | **8/8** | **13.38** | **+17.35** | 84.58 | 5640 |
| 15 | 0.10 | **8/8** | 17.37 | +24.00 | 91.23 | 6970 |

(Full 24-cell grid in the reproduce block / `scratchpad/surface`. env-1 GOSPA = **16.57 at
every cell**; harbor 5-seed card_err = **9.53** and lifetime 0.975 at every cell — both dials
provably no-collateral: open-water/chart-free births have `c=0 → r_new=0.1 ≥` every floor and
sit beyond every `W_off`, so they are never gated.)

**Reading the surface:** narrowing `W_off` shifts the env-2 revival cliff to *higher* floors —
8/8 revival needs `floor ≤ 0.08` at W_off=50, `≤ 0.09` at 35, and holds all the way to **0.10**
at W_off ≤ 25. But the philos cost tracks it one-for-one: the cheapest 8/8 cell is
**W_off=25 / floor=0.10 at card_err +17.35** (best env-2 GOSPA 13.38, beating `pmbm_land` 17.74),
essentially the same bill as the floor-only **W_off=50 / floor=0.08 (+19.35)**. **The two dials
are redundant on total cost** — iso-cost diagonal ridges — because both re-admit the *same*
overlapping clutter tail. No 8/8 cell drops below **+17 card_err**; the ≥6/8-AND-≤+12 knee of
Part 1 remains empty on the surface too.

## Analytic phantom footprint — where the re-admitted *births* sit

`--mode phantom` (r_new = 0.1·(1−c(d,W_off)), re-admitted iff ≥ floor). At floor 0.10, the
births re-admitted by narrowing W_off are exactly the offshore clutter tail, at the vessel
distances:

| W_off | re-admitted births | distance band |
|---:|---:|---|
| 50 | 0 | — (the no-birth zone) |
| 35 | 58 | 30–50 m |
| 25 | 98 | 20–50 m |
| 15 | 168 | 10–50 m |

Every re-admitted **birth** is in-band (`d ≤ 50 m`) by construction — births are always near
shore. The operator concern is where the resulting **tracks** end up, which the footprint
cannot answer. That needs the real map.

## The phantom MAP — where the re-admitted *tracks* end up

`--mode phantomtrack` reads a bench `--export-states-dir` states CSV, classifies each estimated
track-sample as phantom (not matched to truth within 20 m — the same criterion `gospa_false`
uses) and bins its distance-from-shore. Distinct-tracks caveat: counts are per-scan track
*samples*; and philos AIS truth is sparse, so some "phantoms" are real vessels missing from
truth — the same caveat the whole `gospa_false` axis carries.

| corner | phantom samples | in-strip (<50 m) | near (50–150 m) | far (>150 m) | max d |
|---|---:|---:|---:|---:|---:|
| W50 / f0.10 (baseline, no revival) | 355 | 37 | 230 | 88 | 264 m |
| **W25 / f0.10** (halfwidth revival, 8/8) | 564 | **228** | 248 | **88** | **264 m** |
| W50 / f0.08 (floor revival, 8/8) | 570 | 194 | 259 | **117** | **5000 m** |
| W15 / f0.05 (max pressure) | 1074 | 684 | 268 | 122 | 5000 m |

**The spatial discriminator (the Part-2 payoff):** the far open-water phantom field (>150 m) is
**dial-invariant by construction** — those tracks are born from open-water returns (`c=0`,
`r_new=0.1`), admitted regardless of floor or W_off. Baseline = 88 far samples.
- **Narrowing W_off to 25 m** revives all 8 vessels and grows the phantom field almost entirely
  *inside the strip* (37 → 228 in-strip; +18 near) while the far field stays **88 → 88** and the
  worst flyer stays at 264 m.
- **Lowering the floor to 0.08** revives the same 8 vessels but grows the far field (88 → **117**)
  and produces extreme flyers (max **5000 m** = a phantom track that wandered > 5 km offshore).

So the two dials cost the same *total* phantom bill but place it differently: the halfwidth dial
keeps the added phantoms in the operator-supervised near-land strip; the floor dial spills a
fraction into open water. This **partially vindicates the user's spatial-split intuition** —
even though it does not open a *cheaper* door, it opens a *spatially cleaner* one.

## Amended knee analysis (the spatial criterion)

Amended test: a `(W_off, floor)` point with **≥6/8 env-2 tracked AND the re-admitted phantoms
confined to the near-land strip at a reportable count** (report it; do not decide what the
operator framing can carry).

**Knee candidate: `W_off = 25 m, floor = 0.10`.**
- env-2: **8/8** (≥6/8 ✓), env-2 GOSPA 13.38 (best on the whole surface, beats `pmbm_land` 17.74).
- Phantom count to carry: philos card_err **+17.35** (baseline +6.90 → **+10.45 extra tracks per
  scan**). Of the phantom track-samples, 40 % sit in-strip (<50 m) and 44 % in the 50–150 m near
  band; **the far open-water field (>150 m) does not grow vs baseline (88 → 88)** and there are
  **no extreme flyers** (max 264 m). Contrast the equal-cost floor route (W50/f0.08), which grows
  the far field to 117 and produces 5000 m flyers.
- No collateral: harbor 9.53 / env-1 16.57 unchanged.

**What the candidate does *not* clear:** it is not "confined to the strip" in an absolute sense —
228 in-strip and 248 near-band phantom-samples remain, and card_err +17.35 is well above the
Part-1 +12 comfort line. Whether +10.45 extra tracks/scan, mostly in the supervised near-land
strip, is a carriable price is the user's call — **not decided here.** What the surface *does*
settle: if the price is paid, the **halfwidth dial pays it in the strip**, so `W_off=25/f0.10`
dominates the equal-revival floor route `W50/f0.08` on phantom geography.

## Decision (the user's)

Two axes, both the user's to pick, no recommendation here:

1. **Whether to revive at all** — pay a philos phantom bill (`+10.45` extra tracks/scan at the
   cheapest 8/8 setting) to make the shore-hugging channel visible, or keep the philos count
   clean and accept the <50 m no-birth zone. There is no free lunch: no 8/8 cell on the whole
   surface costs less than **+17 card_err**.
2. **If reviving, which dial** — the surface settles this one on evidence: the halfwidth dial
   (`W_off=25 m, floor=0.10`) and the floor dial (`W_off=50 m, floor=0.08`) revive the same 8
   vessels at essentially the same total cost, but the **halfwidth dial places the added
   phantoms inside the operator-supervised near-land strip and leaves the far open-water field
   unchanged**, while the floor dial spills some into open water and produces multi-km flyers.
   Under the user's ruling, `W_off=25/f0.10` dominates on phantom geography.

A per-geography setting would rebuild the exact seam Cl-4 exists to remove, so it is not on the
table as a "one config." **No shipping default is recommended here** — the follow-up (setting
the default + ADR-0001 third amendment, guide, learning docs) is a separate ticket once the
user picks the price point and the dial.

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
# r_new distributions (Part 1 explanation)
./build/bench/navtracker_cl4_a3_census --mode rnew --scenario autoferry_scenario16
PHILOS_CLIP=ais_ferry_near ./build/bench/navtracker_cl4_a3_census --mode rnew --scenario philos

# --- Part 2: the 2-D surface (offshore_halfwidth_m x floor) ---
# both env knobs byte-identical when unset; sweep one cell per run
for W in 50 35 25 15; do for F in 0.05 0.06 0.07 0.08 0.09 0.10; do
  PMBM_OFFSHORE_HALFWIDTH_M=$W PMBM_MIN_NEW_BERN=$F ./build/bench/navtracker_bench_baseline \
    --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-filter autoferry_scenario --seeds 1 --run-id af_w${W}_f${F} --out <dir>
  PMBM_OFFSHORE_HALFWIDTH_M=$W PMBM_MIN_NEW_BERN=$F ./build/bench/navtracker_bench_baseline \
    --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-eq philos --seeds 1 --run-id ph_w${W}_f${F} --out <dir>
done; done
# env-2 tracked = count of lifetime_ratio:truth_N >= 0.1 over the 4 non-anchored {13,16,17,22};
# env-2 GOSPA = mean of their gospa_mean.

# geography — distance-from-shore distributions (the fact the split hangs on)
./build/bench/navtracker_cl4_a3_census --mode geo --scenario philos
./build/bench/navtracker_cl4_a3_census --mode geo --scenario autoferry_scenario16
# analytic phantom birth footprint
./build/bench/navtracker_cl4_a3_census --mode phantom --scenario philos --floor 0.10
# real phantom-track map: export states at a corner, then bin distance-from-shore
PMBM_OFFSHORE_HALFWIDTH_M=25 PMBM_MIN_NEW_BERN=0.10 ./build/bench/navtracker_bench_baseline \
  --config-eq imm_cv_ct_pmbm_coverage_land_ivgate --scenario-eq philos --seeds 1 --run-id ptc --out <dir> --export-states-dir <dir>
./build/bench/navtracker_cl4_a3_census --mode phantomtrack --scenario philos \
  --states <dir>/imm_cv_ct_pmbm_coverage_land_ivgate__philos__seed0.states.csv
```
Fixtures symlinked from the main tree (gitignored, not committed).
