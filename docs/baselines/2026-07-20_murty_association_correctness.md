# Murty association-correctness cycle (backlog #34: M5 + M3)

**Date:** 2026-07-20 → 2026-07-23
**Branch:** `murty-association-correctness` off master `3606c25`
**Deployable config under test:** `imm_cv_ct_pmbm_coverage_land_ivgate`
**Outcome:** ship the correctness fix (M5 + M3) + two review hardenings (F2, F4);
the ambiguity-keyed adaptive-K lever was built and **evaluated NO-GO**; the K=1
near-tie limitation ships as a **named, structurally-ticketed cost**.

## Commits (shipped branch, master → HEAD)
| sha | what |
|---|---|
| `026ec81` | **M3** — `murtyKBest` per-row degradation on an infeasible seed (was: return EMPTY) |
| `83be3c7` | **M5** — reconciled detection-pricing term in the PMBM assignment cost |
| `8229008` | **F2** — Murty children degrade on genuinely-unexplainable columns (review Finding 2) |
| `68cec4e` | **F4** — cache the per-Bernoulli miss decision, single source of truth (review Finding 4) |
| `4daa5ea` | **F1** — drop the Phase-0/Step-1 diagnostic probe from the shipped merge (review Finding 1) |

Diagnostic + lever preserved (NOT for merge): the Phase-0/Step-1 probe lived at
`29b930a..d790e4e`; the lever + Step-1 tie-margin probe are on side branch
**`murty-lever-wip` (`4296dc7`)** for the structural follow-up.

## The two defects
- **M5** (`core/pmbm/PmbmTracker.cpp`): the Murty cost for a Bernoulli detection
  cell was `−log(r·ℓ)`, omitting the detection-pricing term
  `−log(p_D/(1−r·p_D))` the applied hypothesis weight carries. Under the K=1
  hard-commit the K-best argmin ≠ the max-posterior assignment → wrong child
  committed. Fixed cost: `−log(r·p_D·ℓ) + M_i`, where `M_i` is the *exact*
  applied misdetection log-weight (reconciled form — see the fix-form fork).
- **M3** (`core/association/Murty.cpp:69`): `murtyKBest` returned EMPTY on any
  infeasible seed edge, defeating the per-row degradation contract both callers
  (PmbmTracker, MhtTracker) were built against.

## Fix-form fork (arbiter ruling 2026-07-20): reconciled, not textbook
The corrected cost's miss-baseline `M_i` could be the **textbook** unconditional
`log(1−r·p_D)` or the **reconciled** exact applied weight (conditional
surveillance-miss `opp.p_D` under `use_sensor_activity`; `compute_miss_pD`
legacy; `0` when the scan could not have observed the target). The binding
invariant is self-consistency with the *applied* weights. Phase-0 probe measured
the two forms diverge on **1.32 %** of cost matrices gauntlet-wide (harbor 12.9 %,
dense_clutter 37.5 %) → the reconciled form was built (faithfulness over
code-cheapness). F4 later made this invariant *structural*: cost and applied
weight read one cached `MissEval` per Bernoulli.

## Phase-0 probe (blast radius, GO gate)
Bench-side, env-gated, byte-identical when unset. M5 flips (argmin changes) were
**material**: 1.16 % of deployable cost matrices overall, concentrated in
high-`n_meas` scans (philos 28.6 %, dense_clutter 35.5 %). M3 = **0** infeasible
seeds gauntlet-wide (latent; the `(n+m)×m` matrix gives every measurement a finite
new-target cell when `ρ_l > 0`). → GO to build.

## Adversarial hot-path review
**No CONFIRMED correctness defect.** Weight-consistency invariant re-derived
across all five miss branches; M5 algebra sound; M3 correct and strictly better
than master; probe a pure read. Four findings, all folded in: F2 (children
degrade — activates under the lever's K>1), F4 (memoize the miss decision;
byte-identical; also fixes the "most fragile coupling"), F1 (drop the probe), F3
(`p_D=0` latent, note only).

## Step-1 localization (deployable A/B, seed-0; replays are deterministic)
| scenario | ospa m→b | gospa_false m→b | gospa_missed m→b |
|---|---|---|---|
| autoferry_scenario2 | 91 → 183 | 24.7 → 1.4 (win) | 37.8 → 104.9 (loss) |
| autoferry_scenario16 | 213 → 284 | 0 → 65 (loss) | 161.5 → 161.5 (flat) |
| imazu_12 | 75 → 125 | 292 → 268 (win) | 282 → 302 (loss) |
| imazu_18 | 74 → 140 | 290 → 259 (win) | 283 → 325 (loss) |
| **philos** | 84.6 → 58.0 | 5640 → 1360 (−76 %) | ~flat | (card_err +17.35 → −4) |

**Mechanism (a) — near-tie hard commit (dominant, deployable).** Every branch↔
master divergence originates at a Murty recon-flip, and **every damaging flip is a
near-tie** (all ≤ 1.83 nats): imazu_12 = ONE 0.63-nat flip cascading to all its
losses; imazu_18 t=400 flip = 0.09-nat dead tie on an n_meas=11 scan → truth-182
loss begins that scan; autoferry_scenario2 = 17 flips all ≤ 0.45 nats (a
pervasively-ambiguous 2-target scene, baseline top-2 margin median 1.42 nats).
Downstream = coalescence + miss-starvation (the starved truth's track is missed
80–95 % of its loss window while its mass slow-decays). autoferry_scenario16 is
the same fiat inverted → over-split (a 3rd track, +34 phantom scan-instances).

**Mechanism (b) — confident-clutter persistence: NOT deployable.** imazu phantom
counts branch == master (8==8, 6==6); autoferry_scenario2 branch has FEWER (0 vs
3). (b) is confined to the 5 non-deployable research-probe configs
(dense_clutter / cluttermap / occupancy / LOS-shadow), where cheaper confident
detection lets clutter-born tracks persist in dense/low-P_D regimes. Nothing
touches the birth channel or miss-P_D (§3.2.2 invariants).

## Step-2 lever — ambiguity-keyed adaptive K — EVALUATED NO-GO
Design: on a parent whose baseline K is 1, keep every assignment within
`neartie_margin_nats` of the best (up to `neartie_k_cap`) so the alternative
association survives as a hypothesis instead of dying by argmax. Per-instance,
default OFF. 5 TDD tests (split / no-split / margin-controls / determinism).

Threshold sweep (margin × k_cap) on the four regression scenarios,
ospa / gospa_false / gospa_missed:

**autoferry_scenario2** (master 91/24.7/37.8; M5 183/1.4/105):
| margin | ospa | false | missed |
|---|---|---|---|
| 0.5 | 197.6 | 76.6 | 62.2 |
| 1.0 | 212 | 112 | 60 |
| 1.5 | 231–234 | 149–162 | 51–61 |
| 2.0/k2 | 234.8 | 193.3 | 44.0 |

Fails at every setting: false explodes (≥ 76 vs master 24.7, vs M5 1.4), ospa
worse than master AND M5 everywhere. k_cap irrelevant.

**imazu_18** (master 74/283 ospa/missed; M5 140/325): fixed **only** at m2.0/k2
(51.5 / 283.8); m1.5/k2 = 216 / 419 (worse than M5) — a **knife-edge**, not a
±0.5-nat plateau, and m2.0/k2 is exactly autoferry_scenario2's worst point.
**imazu_12**: fixed at margin ≥ 1.5 (ospa 56 < master 75). **autoferry_scenario16**:
ospa improves (→162) but false worsens (0 → 75+) at every setting.

**Verdict: NO-GO.** No `(margin, k_cap)` passes the binding kill-criteria; the
requirements directly conflict (imazu_18's only fix point maximises
autoferry_scenario2 damage), there is no plateau, and the wide-margin K>1 solve is
costly on exactly the dense ambiguous scans (sc2 8-config sweep 3m17s, ~25 s/config
vs seconds no-lever). Root cause: a single global near-tie margin cannot separate
an *isolated* close-pass near-tie (real starved target → keep) from *pervasive*
ambiguity (spurious interpretation → phantom); the cost margin alone does not
distinguish them per-scan.

## Shipped named cost + follow-up
The K=1 hard commit resolves genuine association near-ties by fiat: a real vessel
can be coalesced away for minutes under sustained dense ambiguity, and close
passes can break tracks or over-split. This is ADR-0002's failure class, now
named and ticketed. Flip side, same fix: gauntlet-wide phantoms −8.9 %, philos
accepted deviation +17.35 → −4, imazu_15/22 improved. Structural follow-up
(improvement backlog): *scene-adaptive ambiguity handling — a discriminator beyond
a global margin* (ambiguity-density keying, starvation-history keying, coalescence
detection), starting from the `murty-lever-wip` (`4296dc7`) evidence and this
sweep table.
