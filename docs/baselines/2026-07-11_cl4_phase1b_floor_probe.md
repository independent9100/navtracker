# Cl-4 Phase 1b — conditional coverage floor (re-detection) evidence probe

**Date:** 2026-07-11 · **Branch:** `cl4-phase1b-floor-probe` · **North-star:** Cl-4 Phase 1b.
**Measurement only — zero shipped behaviour change.** Ticket:
`docs/superpowers/plans/2026-07-11-cl4-phase1b-coverage-floor-probe-ticket.md`.
Tool: `tools/cl4_a3_census.cpp` (extended: `--mode chain`, `--mode target`).

## Verdict (one line)

**NO-BUILD for the rule as specified** (`chain length ≥ M within N scans` **AND**
`net displacement ≥ D`). The env-2 shore-huggers and the harbor pier **overlap in
net displacement**: reviving ≥2/3 of the env-2 targets within 30 s needs **D ≤ 72 m**,
but excluding the harbor pier robustly needs **D > 84 m** (the pier's 120 m extent lets
association walk it to ~84 m net displacement on some seeds). Empty feasible set for D.
The philos guard (K2) is **not** the binding constraint — it passes comfortably. The
**harbor pier is the bridging population** — exactly the K3 trap the ticket named.
A per-step-speed (smoothness) term would break the overlap but changes the rule family
(arbiter's call — see hooks).

## Method (fixed up front)

- **In-band** = tracker's own `CoastlineModel::clutterPrior(pos) > 0` (coverage_land
  <50 m no-birth zone). Harbor is chart-free → land inert → in-band N/A (chain all radar).
- **Chain** = greedy nearest-neighbour re-detection linking of birth-candidate
  Position2D measurements: a point extends the nearest active chain within
  **r_chain = 25 m** whose time gap ≤ **5 s** (absolute revisit tolerance, covers the
  philos rotating-radar re-visit and the denser autoferry stream); else starts a new
  chain. Per chain: length (scans), duration, **net** displacement (start→end),
  path length, mean speed, **max single-step speed** (jumpiness).
- **Birth-candidate population** = radar (ArpaTtm) — the common birth sensor and the
  shore-clutter source on all three workloads; env-2 also uses lidar (radar+lidar union
  for the per-target revival test). EO/IR excluded (bearing-only, cannot seed — Phase 1a).
- **K1 latency** measured authoritatively from truth: per target, the max net truth
  displacement achievable in any ≤ 30 s window (`--mode target`).

## Step 0 — suppression inventory

| workload | what suppresses the near-shore births | suppressed birth-candidate population |
|---|---|---|
| **env-2** (autoferry sc13/16/17/22) | **land ramp no-birth zone** — both targets in-band 100 % of scans; `coverage_land` produces **0 tracks** (Phase-1 confirmed). | all in-band radar+lidar returns |
| **philos** (ais_ferry_near) | **land ramp** (near-shore structure, ~185 stationary returns) **+ gate** (the transient water residual A1 catches). | 1140 in-band radar chains, dominated by stationary structure |
| **harbor** (sim) | **chart-free → land ramp inert.** The coverage/sensor-activity component manages existence (dossier: canonical `card_err 11.1 → coverage_land 8.0`, ≈3.6 phantom pier tracks suppressed; the 8.0 residual is unsuppressed confirmed pier phantoms). A floor rides the suppressed fraction, so **the pier is in scope**. | pier returns (extended persistent structure, 13 pts / 120 m) |

## Step 1 — chain census, three populations

**env-2 (per-target, authoritative truth, latency window 30 s):**

| scen·tgt | in-band | re-detect scans | truth net disp (m) | max disp in 30 s (m) | per-step speed med/p95 (m/s) |
|---|---:|---:|---:|---:|---|
| 13·1 | 100% | 897 | 274 | 77.7 | 2.49 / 3.11 |
| 13·2 | 100% | 730 | 294 | 78.8 | 0.00 / 6.63 |
| 16·1 | 100% | 954 | 280 | 91.8 | 2.79 / 3.74 |
| 16·2 | 100% | 882 | 216 | 72.6 | 0.53 / 6.04 |
| 17·1 | 100% | 548 | 143 | 69.0 | 1.91 / 2.86 |
| 17·2 | 100% | 559 | 164 | 71.6 | 0.34 / 6.08 |
| 22·1 | 100% | 546 | 186 | 92.6 | 2.91 / 3.74 |
| 22·2 | 100% | 649 | 226 | 95.2 | 0.00 / 7.91 |

All 8 move (net 143–294 m), re-detect on hundreds of scans, and reach **≥ 69 m within
30 s**. Per-step speed is **smooth**: p95 ≤ 7.9 m/s (tgt1 ≤ 3.7).

**philos (in-band radar, 1140 chains):** dominated by stationary structure (median net
disp 0). Moving tail — chains satisfying `n≥M AND net_disp≥D`, clutter only:

| M \ D | 20 | 30 | 50 | 72 |
|---|---:|---:|---:|---:|
| 3 | 57 | 31 | 8 | 2 |
| 5 | 20 | 14 | 7 | 2 |
| 8 | 3 | 3 | 2 | 2 |
| 12 | 1 | 1 | 1 | 1 |

At (M≥8, D≥50): **2 chains** = 0.2 % of the 1140 A1-admitted residual. The smooth movers
sit ≤ 3 m/s; the moving tail is a mix of drift and (unlabelled) real near-shore craft.

**harbor (radar; 5 seeds):** sea clutter = length-1 chains, **never** satisfies any
(M,D). Anchored boats (reference) = stationary, net disp ≈ 0 (never revived — see scope
note). **Pier** = long chains (n 26–36, persistent) that walk between its 10 m-spaced
points on missed detections; net displacement is usually small (back-and-forth) but
reaches a **monotonic-walk maximum of ~84 m**:

| r_chain | seed0 | seed1 | seed2 | seed3 | seed4 | max pier net disp (m) |
|---|---|---|---|---|---|---|
| 15 m | 37.6 | 34.0 | **50.6** | 39.7 | 40.1 | 50.6 |
| 25 m | 34.4 | 49.9 | **83.8** | **82.7** | **52.3** | 83.8 |

Pier walks are **jumpy**: max single-step speed **16–20 m/s** (vs env-2 vessels ≤ 8).

## Step 2 — (M, N, D) score and the overlap

N = 30 s window throughout. Representative points:

| (M, D) | env-2 revived /8 (latency) | philos clutter re-admitted | harbor pier/clutter re-admitted |
|---|---|---|---|
| (8, 30) | 8/8 (<30 s) | 3 (0.3 %) | 1 pier (borderline) |
| (8, 50) | 8/8 (<30 s) | 2 (0.2 %) | **0–2 pier** (seed-dependent; ≥50 m on 3/5 seeds at rc25) |
| (8, 72) | 6/8 | 2 | 0–1 pier (83.8 m on seed2) |
| (8, 85) | 3/8 ✗ K1 | 2 | 0 pier |

- **K1 (revival ≤ 30 s):** feasible for **D ≤ 72.6 m** (6/8; 8/8 at D ≤ 69 m). M met with
  huge margin (targets re-detect on 546–954 scans).
- **K2 (philos guard ≤ 10 % of A1 residual):** **passes** for all D ≥ 30 m — at (M≥8,
  D≥50) only 2 chains (0.2 %), projected `card_err` ≈ +0.05 (≪ the +10 budget; ≪ the A2
  -unconditional +40.15 anchor). philos is not the blocker.
- **K3 (harbor: zero pier/clutter chains):** the pier reaches **~84 m net displacement**
  on some seeds → to exclude it robustly requires **D > 84 m**.

**The gap:** K1 needs D ≤ 72.6 m; K3 needs D > 84 m. **No D satisfies both.** The env-2
shore-huggers move 69–95 m per 30 s; the harbor pier's 120 m extent lets association walk
it to ~84 m net displacement — the two populations overlap in the exact quantity (net
displacement) the rule keys on. **NO-BUILD as specified.**

## Hooks for the arbiter (do not decide here)

1. **Smoothness breaks the overlap.** The pier walk is jumpy (max single-step 16–20 m/s)
   because it teleports between 10 m-spaced points; env-2 vessels are smooth (per-step
   p95 ≤ 8 m/s, tgt1 ≤ 3.7). A rule augmented with a **per-step-speed cap (~10 m/s)**
   would keep all 8 env-2 targets and reject the pier walk. This **changes the rule
   family** (the ticket specified length + net-displacement only) and must be re-scored
   on philos (clutter smoothness there unmeasured) — an arbiter decision, not a Phase-2
   given.
2. **Association-proxy sensitivity (possible over-rejection).** The pier walk is produced
   by a greedy single-link NN chainer with r_chain = 25 m; a real PMBM Bernoulli with a
   motion model resists teleporting between pier points, so the real pier displacement may
   be **smaller** than ~84 m. The NO-BUILD is thus conservative on K3; a Phase-1c that
   keys displacement on the real tracker's Bernoulli tracks could refine whether the
   overlap survives. (Direction, not a request to reopen this verdict.)
3. **Scope (stated, not fixed here).** A displacement floor never revives **stationary**
   near-shore vessels — harbor anchored boats 3–5 (net disp ≈ 0) and any anchored env-2
   craft. That is unchanged from today; they remain the ADR-0002 static-hazard channel's
   responsibility. Quantified: the 3 harbor anchored boats never satisfy any D > 0.

Per the ticket's NO-BUILD branch, the arbiter chooses between path (c) re-pricing, a
smoothness-augmented rule (hook 1), a real-association Phase-1c (hook 2), or accepting a
documented per-geography residual in the Cl-4 claim.

## Reproduce

```
cmake --build build --target navtracker_cl4_a3_census
# env-2 per-target revival + latency + smoothness
./build/bench/navtracker_cl4_a3_census --mode target --scenario autoferry_scenario16 --latency 30
# philos / harbor chain census (per-chain CSV)
PHILOS_CLIP=ais_ferry_near ./build/bench/navtracker_cl4_a3_census --mode chain --scenario philos --kind radar --inband-only 1 --csv philos.csv
./build/bench/navtracker_cl4_a3_census --mode chain --scenario harbor_complete_truth --kind radar --seed 2 --csv harbor.csv
```
Fixtures (`data/autoferry`, `tests/fixtures/philos/out`) are symlinked from the main tree
(gitignored, not committed).
