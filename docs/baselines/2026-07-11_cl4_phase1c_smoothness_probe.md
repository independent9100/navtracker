# Cl-4 Phase 1c — smoothness + real-association displacement: does the pier/vessel overlap survive honest measurement?

**Date:** 2026-07-11 · **Branch:** `cl4-phase1c-smoothness-probe` · **North-star:** Cl-4 Phase 1c.
**Measurement only — zero shipped behaviour change.** Ticket:
`docs/superpowers/plans/2026-07-11-cl4-phase1c-smoothness-probe-ticket.md`.
Tool: `tools/cl4_a3_census.cpp` (extended: `--motion` Tier-A motion-model chainer, `--vcap`).

## Verdict (one line)

**NO-BUILD — confirmed, and now robustly so.** Both Phase-1b escape hatches close under
honest measurement, and they close *harder* than hoped. **Honest displacement makes the
pier walk WORSE, not better** (hook 2 backfires): under motion-consistent association the
pier walks its 120 m linear extent to **104–130 m** net displacement (vs the plain-NN
proxy's 34–84 m). And **the smoothness split does not exist on a fair footing** (hook 1
was an artifact): measured as chains/tracks — the level the floor actually keys on — the
env-2 vessels are **as jumpy as the pier** (Tier A max-step p95 ≈ 19 m/s for both; Tier B
vessel-track median max-step 31 m/s). No `(M, D, S)` separates the env-2 shore-huggers
from the harbor pier in **either** displacement **or** smoothness. This is the ticket's
NO-BUILD branch #2: real association launders the pier walk into vessel-like motion.

**Root cause (one sentence):** the harbor pier is a 120 m **linear** structure, so a chain
walking along its axis is geometrically a constant-velocity transit — same displacement,
same per-step statistics as a real vessel. No kinematic feature distinguishes them.

## Step 1 — honest displacement (old NN proxy vs honest)

Harbor pier max net displacement per seed (pier-born, chain/track length ≥ 8):

| seed | Tier 0 — plain NN (Phase 1b) | **Tier A — motion-model (vcap 20 m/s)** | **Tier B — real PMBM confirmed pier tracks** |
|---|---:|---:|---:|
| 0 | 34.4 | 114.5 | 69.0 |
| 1 | 49.9 | 129.5 | 104.4 |
| 2 | 83.8 | 124.0 | 42.4 |
| 3 | 82.7 | 120.7 | 101.8 |
| 4 | 52.3 | 90.9 | 93.8 |
| **max** | **83.8** | **129.5** | **104.4** |

The Phase-1b worry — that the greedy-NN chainer *over*-stated the pier walk and honest
association would keep it below the 72.6 m K1 ceiling — is **falsified**. Motion-consistent
chaining *sustains* the walk along the linear pier (a steady apparent velocity predicts the
next pier point and locks on), and the real PMBM's own confirmed pier tracks reach 104 m.
Across 5 seeds, Tier A puts **5–14 pier chains ≥ 50 m** per seed; Tier B, **9 pier tracks
≥ 50 m (3 ≥ 72.6 m)** pooled. The D-only window does **not** reopen — it closes.

env-2 targets under Tier A (truth-based, `--mode target`) are unaffected: all 8 in-band
100 %, reach **≥ 69 m within 30 s** (69–95 m). K1 remains feasible at D ≤ 72.6 m *on
displacement alone* — the problem is the pier reaches the same band.

## Step 2 — smoothness census (the missing philos measurement + the fair-footing correction)

**The Phase-1b hook 1 was apples-to-oranges.** It compared env-2 vessel *truth* per-step
speed (≤ 7.9 m/s, smooth) against the pier *chain* max-step (16–20 m/s, jumpy). Measured on
the **same footing** — both as re-detection chains / tracks, which is what a floor keying on
re-detection history sees — the smoothness gap vanishes:

Tier A motion-model chains (in-band; disp ≥ 40–30 m, len ≥ 8):

| population | net_disp med/max (m) | **max-step med / p95 / max (m/s)** |
|---|---|---|
| env-2 vessel chains | 68 / 158 | **13.8 / 19.2 / 19.4** |
| harbor pier chains | 75 / 129 | **14.7 / 19.1 / 20.0** |

Identical. Tier B confirms at the confirmed-track level: env-2 vessel tracks have median
max-step **31 m/s** (max 274 — tracker churn/teleports), while pier tracks include a *smooth*
tail (max-step 7–9 m/s at 55–69 m disp). So to admit env-2 vessel tracks a smoothness cap S
would have to sit at ≥ ~19 m/s (Tier A) or ≥ ~31 (Tier B) — which admits the whole pier;
to reject the pier's smooth tail S would have to sit below ~7 — which rejects essentially
all vessels. **No S admits vessels while rejecting the pier.**

**Philos in-band radar (1140 chains), per-step-speed by Phase-1b class** (the missing
measurement): stationary structure (net_disp < 10 m): n=66, max-step med 3.7 / p95 13.6.
Moving tail (net_disp ≥ 20 m): n=116, max-step med 15.8 / p95 19.2 (mostly jumpy). Of that
tail, **29 chains are smooth** (max-step ≤ 8) — plausible real unlabelled near-shore craft
that a smoothness rule would *correctly admit* (not re-admission failures; flagged, not
counted against K2 — but they are unlabelled, so the credit is unprovable). Harbor sea
clutter: length-1 chains, no steps (trivial, as expected).

## Step 3 — the augmented `(M, D, S)` rule and the overlap

Rule: `(chain ≥ M in 30 s) AND (net displacement ≥ D) AND (max single-step speed ≤ S)`,
displacement/smoothness measured honestly (Tier A/B). The two populations the rule must
split occupy the **same region of (D, S) space**:

| quantity | env-2 vessels (admit) | harbor pier (reject) | separable? |
|---|---|---|---|
| net displacement, 30 s | 69–95 m | Tier A 90–130 m, Tier B ≤ 104 m | **no** (pier ≥ vessel) |
| max-step, chain (Tier A) | med 13.8, p95 19.2 | med 14.7, p95 19.1 | **no** (identical) |
| max-step, track (Tier B) | med 31 | smooth tail 7–9 | **no** (overlap both ways) |

There is no `(M, D, S)` point — with or without the #24 margin (S ± 2, D ± 10) — that
revives ≥ 2/3 env-2 targets and admits zero pier. The margin-robust feasible region is empty.

## K1–K4 verdict

- **K1 (revival):** displacement alone is fine (8/8 reach ≥ 69 m in 30 s), but any S that
  admits the env-2 vessel *chains/tracks* (≥ ~19 m/s) also admits the pier. **Not
  independently satisfiable together with K3.**
- **K2 (philos guard):** not binding — the pier fails the build first. Informational: a
  smoothness rule would admit ~29 smooth philos movers (mixed real craft / clutter,
  unlabelled) and reject ~87 jumpy ones; the ~185 stationary structure returns stay
  rejected by D. philos would have passed on its own.
- **K3 (harbor guard):** **FAILS** under both honest tiers — pier chains/tracks reach
  50–130 m net displacement (5–14 per seed, all 5 seeds) with max-step indistinguishable
  from vessels. Loosening S by 2 or lowering D by 10 only admits more. Zero-pier is
  unreachable in the K1-feasible region.
- **K4 (fast-vessel blind spot):** degenerate here — because no S separates, any S low
  enough to exclude the pier's smooth tail (< 7 m/s) excludes essentially the entire vessel
  population (env-2 chain p95 ≈ 19 m/s). The "blind spot" is total: the per-step cap cannot
  be set to a value that keeps real vessels. (Documented limitation, as specified — but
  moot, since the rule does not build.)

## Why (the durable lesson) and what it leaves for the arbiter

The conditional coverage floor — in every form measured (plain-D Phase 1b; augmented
D+S Phase 1c) — dies on the same rock: **an extended linear structure is kinematically a
vessel transit.** Displacement can't see the difference (the pier is *longer* than the
targets' 30 s reach); smoothness can't either (once you measure vessels as chains/tracks
in real clutter, they are as jumpy as the pier walk). The floor cannot key on a suppressed
birth's re-detection *kinematics* to exclude the pier, because association — NN,
motion-model, or the real PMBM — launders the walk into constant-velocity motion.

Per the ticket, the arbiter now chooses between **path (c) re-pricing** and **a documented
per-geography residual in the Cl-4 claim** (open-water/coastal deployments get the coverage
stack; dense linear-structure harbors keep `pmbm_land` or accept the near-shore no-birth
zone). What would move the needle is orthogonal to kinematics: a *map/extent* signal (the
charted-obstacle / static-structure prior of ADR-0002, which knows the pier is a fixed line)
— i.e. tell the floor *where the structure is*, rather than trying to infer it from motion.

## Reproduce

```
cmake --build build --target navtracker_cl4_a3_census navtracker_bench_baseline
# Tier A honest displacement + smoothness (motion-model chainer)
./build/bench/navtracker_cl4_a3_census --mode chain --scenario harbor_complete_truth --kind radar --seed 1 --motion 1 --gate 15 --vcap 20 --csv h.csv
./build/bench/navtracker_cl4_a3_census --mode chain --scenario autoferry_scenario16 --kind pos --inband-only 1 --motion 1 --gate 15 --vcap 20 --csv e.csv
# Tier B real PMBM pier tracks
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm --scenario-eq harbor_complete_truth --seeds 5 --export-states-dir st/
# env-2 revival + latency (truth)
./build/bench/navtracker_cl4_a3_census --mode target --scenario autoferry_scenario16 --latency 30
```
Fixtures (`data/autoferry`, `tests/fixtures/philos/out`) symlinked from the main tree
(gitignored, not committed).
