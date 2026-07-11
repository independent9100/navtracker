# Cl-4 Phase 1d — occupancy floor-veto: can the persistence grid exclude the pier without blinding the floor?

**Date:** 2026-07-11 · **Branch:** `cl4-phase1d-occveto-probe` · **North-star:** Cl-4 Phase 1d.
**Measurement only — zero shipped behaviour change** (offline `LiveOccupancyModel`, default
params; model unmodified). Ticket:
`docs/superpowers/plans/2026-07-11-cl4-phase1d-occupancy-veto-probe-ticket.md`.
Tool: `tools/cl4_a3_census.cpp` `--mode veto`.

## Verdict (one line)

**BUILD-ELIGIBLE — the binding criteria K1–K3 are met, with margin, on all measured
workloads** — the first affirmative result of the Cl-4 Phase-1 arc. The extent-gated
persistence veto wins the race against the pier decisively (T_veto 1–3 s ≪ T_floor
14–30 s) under **both** Tier-A chains and Tier-B real PMBM tracks, all 5 seeds, at D = 50
and 70 m, while never firing on the env-2 vessels (zero added latency at nominal). **One
coverage gap must gate Phase-2** (below): no fixture exercises a vessel transiting *within
the ramp of* extended structure, and env-2's fed stream carries no dense shore returns — so
the hard vessel-adjacent-to-structure case is unmeasured here and must be closed on the
full gauntlet. This is **not** a NO-BUILD (the race is won); it is a build with a named
Phase-2 gate.

## Step 0 — wiring conventions (pinned before measuring)

- **Veto signal = `birthSuppression(endpoint) > 0`** (the shipped, **extent-gated**
  structure+ramp face), not the bare persistent cell. This is load-bearing: a pier (13
  points / 120 m → ≥ 4 connected persistent cells) classifies as structure; a *compact*
  dwelling vessel's watch circle does not (`extended_cells_min = 4`). Bare-cell persistence
  is reported alongside but is **not** the veto (it would flag dwelling vessels).
- **Fed scan = one unique measurement timestamp**; all returns fed as `clutter_positions`
  weight 1.0 (offline proxy — no tracker r-labeling; a mover hits each cell ~once so its
  persistence stays low, a persistent pier accrues). EWMA is per fed scan, so cadence
  matters (see philos).
- **Cell grid = model's own `floor(p/cell)`** (matched exactly, so flag-time and
  chain-endpoint share cells). Fixed datum per workload (bench frame; the model is an
  `IDatumChangeSink` but recenter never fires here).
- Params: defaults (`cell_size_m 25`, `ewma_alpha 0.3`, `persistence_bar 0.5`). Floor
  corner: M = 8, D ∈ {50, 70} m, N = 30 s (the Phase-1b/1c K1-feasible corner).

## Step 1 — harbor: the flag side and the race (5 seeds)

- **Pier flags fast:** structure `t_veto` = **1–3 s** (2–3 hits at α 0.3, as predicted).
- **Race won, Tier A (motion-model chains):** every floor-satisfying pier chain vetoed,
  0 revived, all seeds, both D:

  | | seed0 | seed1 | seed2 | seed3 | seed4 |
  |---|---|---|---|---|---|
  | D=50 satisfy / revived | 5/0 | 14/0 | 8/0 | 13/0 | 8/0 |
  | D=70 satisfy / revived | 2/0 | 11/0 | 6/0 | 6/0 | 5/0 |

- **Race won, Tier B (real PMBM confirmed pier tracks — authoritative):** every
  floor-satisfying pier track vetoed, 0 revived, all seeds, D = 50 and 70. (The Phase-1c
  laundered pier tracks that reached 104 m are caught: their endpoint cells flag by 1–3 s,
  long before they satisfy the floor at ≥ 14 s.)
- **Sea clutter never flags** as structure (transient uniform returns never accrue
  persistence); the only structure cells lie on the pier line ± its 25 m ramp
  (y ∈ [−412, −262]).
- **Anchored boats** (ids 3–5) show as *persistent* cells but never *structure* (compact,
  not extended) → not vetoed; they fail D anyway (stationary) and remain with the
  static-hazard channel (ADR-0002). Reported, not fixed.

## Step 2 — env-2: the pass side and the latency cost (4 scenarios)

**At nominal params the veto NEVER fires on env-2: `structure_cells = 0` on all four
scenarios.** The dwelling vessels form persistent cells (16–29) but never a ≥ 4-connected
*extended* component, so `birthSuppression = 0` everywhere → no vessel chain is vetoed →
**all 16 floor-satisfying vessel chains revived, added latency = 0** (the veto is inert, so
first-revival-with-veto == first-revival-without). The extent gate does exactly its job.

The dwelling-vessel kill-risk the step exists for does not bite: even where dwelling
structure *does* form (lower bar, below), a vessel satisfies D only by *moving*, and its
floor-satisfying endpoint is a fresh, unflagged cell — the claim that saves the floor,
confirmed.

## Step 3 — philos guard (K2)

The veto is **monotone (removes revivals only)** so K2 can only improve. On philos the veto
is **inert** (`structure_cells = 0`): under per-timestamp feeding the rotating radar
revisits each shore cell too rarely for the EWMA to reach the bar. So re-admission with the
veto = the no-veto floor count (1 moving-tail chain at M = 8, D = 50) — **K2 satisfied by
equality** (the floor's own philos re-admission is already tiny; Phase 1b). Documented
consequence: on this workload the veto neither helps nor hurts; the floor carries the
philos guard.

## Binding kill-criteria — verdict

| criterion | result | margin (bar ± 0.15, 2× cell) |
|---|---|---|
| **K1** ≥ 2/3 env-2 revived, +latency ≤ 15 s | **PASS** — 16/16 revived, +0 s latency (veto inert at nominal) | 15–16/16 across bar {0.35,0.5,0.65} × cell {25,50} — holds |
| **K3** race won every pier chain/track, all 5 seeds; sea clutter never flags | **PASS** — Tier A + Tier B, D 50 & 70, 0 pier revived; sea clutter/boats never structure | **0 pier revived at every** bar × cell corner |
| **K2** philos re-admission ≤ no-veto | **PASS** — veto monotone/inert on philos, = no-veto (1) | n/a (monotone) |
| **K4** (named, not gated) | dwelling latency = 0 (nominal); a vessel *moored alongside* flagged structure is not revived (static-hazard channel, standing scope); deployment must register the occupancy model as a datum sink (CLAUDE.md) | — |

**All three gates pass with margin.** The margin test is genuinely robust: K3 holds 0-pier
at all four bar × cell corners (the pier's 120 m extent gives it ≥ 4 connected cells even at
50 m resolution; its flag-time margin over the floor is ~11–27 s).

## The coverage gap Phase-2 must close (frame the trade)

The criteria are met, but two facts bound what has actually been shown, and they point at
the same untested case — **a vessel transiting within ~25 m of extended structure**:

1. **No fixture puts a moving vessel next to extended structure.** env-2's vessels are
   near-shore (100 % in-band) but the autoferry stream carries **no dense shore radar
   returns** (the shore is a coastline polygon, not returns), so no shore structure forms →
   the veto is inert there by data, not by discrimination. Harbor has the pier but no
   vessel passing it (the anchored R6 boat is stationary → fails D). So the veto's behaviour
   on a vessel whose floor-satisfying endpoint lands inside a real structure's 25 m ramp is
   **unmeasured**. There it would fire transiently and add latency until the vessel clears
   the ramp — the moving analogue of the K4 moored-alongside boundary.
2. The env-2 K1 "zero latency" is therefore a property of *this* data as much as of the
   mechanism. The extent gate is the reason it *should* generalise (a compact vessel never
   becomes structure), but that is demonstrated pier-vs-nothing, not vessel-vs-adjacent
   structure.

Per constraint 4 a Phase-2 build is judged on the full promotion-dossier gauntlet; that
gate **must** include a vessel-transits-past-structure case (harbor_boat_near_pier made to
move, or real coastal data with shore returns) and bound the transit latency against the
15 s K1 budget. Recommend the arbiter's Phase-2 ticket make this the primary acceptance
test, not an afterthought.

## Reproduce

```
cmake --build build --target navtracker_cl4_a3_census navtracker_bench_baseline
# harbor race (Tier A) + flag-grid dump
./build/bench/navtracker_cl4_a3_census --mode veto --scenario harbor_complete_truth --kind radar --seed 1 --M 8 --D 50 --dump-flags flags.csv
# env-2 pass side (structure_cells=0 => veto inert => latency 0)
./build/bench/navtracker_cl4_a3_census --mode veto --scenario autoferry_scenario16 --kind pos --inband-only 1 --M 8 --D 50 --chain-radius 25
# Tier B: export real PMBM harbor tracks, race against flags.csv (tools/… python in the probe scratch)
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm --scenario-eq harbor_complete_truth --seeds 5 --export-states-dir st/
```
Fixtures symlinked from the main tree (gitignored, not committed).
