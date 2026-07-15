# ADR 0003 — Near-shore birth policy: narrow the no-birth strip to 25 m on the deployable config

- **Status:** Accepted
- **Date:** 2026-07-12
- **Deciders:** navtracker maintainers (user decision, Cl-4 endgame option (a), 2026-07-12)
- **Related:** ADR-0001 (partially superseded — see its status line), ADR-0002
  (presence-over-classification, bounded-latency promotion),
  `docs/algorithms/pmbm-design.md` §10 (land clutter prior) and §3.2.2
  (λ_C-cancellation invariant),
  `docs/baselines/2026-07-11_cl4_cliff_price_list.md` (Parts 1+2, the sweep +
  phantom map), `docs/baselines/2026-07-12_cl4_pending_band_probe.md` (shape
  collapse + the parked pending band),
  `docs/superpowers/specs/comparison-baselines.md` (Cl-4 claim card),
  `docs/algorithms/evaluation-log.md` (2026-07-12 freeze).

## Context

Cl-4 is the north-star requirement of **one deployable PMBM config** (user-set
2026-07-10): a single named configuration a consumer can run everywhere, rather
than a per-geography zoo. The named candidate is
`imm_cv_ct_pmbm_coverage_land_ivgate`.

Its blocker was ADR-0001's near-shore **no-birth zone**. Under `coverage_land`
the birth floor equals the birth target (`min_new_bernoulli_existence ==
birth_existence_target == 0.1`), which turns the shoreline clutter ramp's inner
50 m into a hard no-birth strip: *a vessel within 50 m of shore never
initiates*. On the Cl-1 env-2 workload (the Trondheim urban channel, targets
hugging the shore) this **collapsed the coverage stack to zero tracks**
(promotion dossier 1a: env-2 "COLLAPSE", lifetime 0, all targets missed), while
the same config is the open-water leader on env-1.

## Today's prior default, and why it existed

`W_off = 50 m` with `gate == target` was a **deliberate** ADR-0001 decision, not
an oversight. It bought the philos shore-clutter win: the 50 m strip suppresses
the dense near-shore radar clutter that would otherwise birth phantom tracks, and
philos `card_err` stays at **+6.9**. The no-birth zone was ADR-0001's **accepted
cost**, and it was priced on **GOSPA alone** — a metric that, on the sparse-AIS
philos truth, did not register the env-2 channel loss as sharply as an operator
would.

## The measured dead-ends (why the fix is a strip-narrowing, not something cleverer)

Every more-principled alternative was built and measured, and each failed. Each
has a baseline link in the eval-log / the two cliff-price + pending-band dossiers.

- **A1 — lower the gate** (`min_new_bernoulli_existence` < target,
  unconditionally): re-admits philos near-shore **water** clutter wholesale
  (philos gospa 73.1 → 100, card_err balloons). Rejected in ADR-0001.
- **A2 — pre-suppression floor** applied unconditionally to the land-only case:
  **philos card_err +6.9 → +40.15** (ADR-0001 amendment, eval-log 2026-07-02 R1).
  A2 survives only *scoped to static-obstacle composition*; land-only is exactly
  the zone A1 was rejected for.
- **A3 — sensor-typed exemption** (exempt births corroborated by a clutter-free
  sensor near shore): the guard is **unscoreable cross-workload** — the camera is
  itself a clutter source near shore (79–85 % of near-shore camera returns are
  off-target), and the revival and guard workloads share no clutter-free sensor
  (`docs/baselines/2026-07-11_cl4_phase1_a3_probe.md`). NO-BUILD.
- **Kinematic conditional floor** (revive a near-shore contact once it moves like
  a vessel): association **launders a linear-structure walk into a CV transit** —
  honest displacement makes the pier walk 104–130 m, indistinguishable from a
  real transit (`2026-07-11_cl4_phase1c_smoothness_probe.md`). NO-BUILD.
- **Occupancy floor-veto** (let a persistence grid veto births on structure):
  wins the pier race but **blinds a vessel transiting past structure** — the
  spatial veto vetoes the whole pass (`2026-07-11_cl4_phase2_stage0...md`).
  NO-BUILD.
- **Ramp/bar SHAPE** (a cleverer suppression curve): **provably cannot help.**
  Under adaptive birth every candidate's pre-suppression existence is pinned to
  `birth_existence_target = 0.1` (§3.2.2), so `r_new(d)` is monotone in distance
  and any one-shot ramp × bar reduces to a single admit-boundary distance `d*`
  already on the swept surface. Two non-linear shapes at `d* = 25` reproduce the
  linear cell exactly (env-2 identical, philos within seed noise) — proof in
  `2026-07-12_cl4_pending_band_probe.md` Step 1.

The distance-to-shore data settles what remains: **env-2 vessels ride 6–42 m
offshore (median 25–31 m); philos clutter is densest 0–10 m and thins outward;
1361 of 1706 in-band philos returns are inland structure (hard-gated).** There is
no clean inner/outer cut, but there is a graded trade, and the phantom **map**
(not just the count) discriminates *where* the re-admitted phantoms land.

## Decision

**Set `offshore_halfwidth_m = 25 m` (inland plateau unchanged at 50 m) on the
deployable config `imm_cv_ct_pmbm_coverage_land_ivgate` only, at today's floor
0.10.** Scoped as an explicit per-config parameter (`Config::coastline_prior_params`);
the `CoastlinePriorParams` struct default stays 50/50 for every other config, and
the `PMBM_OFFSHORE/INLAND_HALFWIDTH_M` env vars stay as research levers.

Why 25 m specifically, from the measured surface:
- It **revives the 25–50 m band** where the env-2 vessels ride (at floor 0.10 the
  admit boundary *is* `W_off`), taking env-2 from **0/8 (collapse) to 8/8**, GOSPA
  **13.38** (best on the whole 2-D surface, beats `pmbm_land` 17.74).
- The **phantom map** shows the cost lands **in-strip**: the re-admitted philos
  phantoms sit near shore (max **264 m** from shore; open-water field FLAT at 88
  samples vs baseline). The equal-revival *floor-lowering* alternative
  (`W50/f0.08`) instead spills the far field to 117 and produces **5 km flyers** —
  same total cost, worse geography. The half-width dial pays the bill in the
  operator-supervised near-land strip.

## Accepted costs (named, dated)

1. **philos-regime harbors gain ~+10.45 phantom tracks/scan, in-strip** (philos
   `card_err` +6.9 → **+17.35**). The user re-priced this **2026-07-11/12 as
   acceptable**: near-land waters are operator-supervised, and by ADR-0002's own
   principle an **invisible real moving vessel is the worse failure** — a mover
   has no static-hazard fallback, whereas an in-strip phantom in a cluttered
   harbor is a supervised nuisance. This is a **recorded deviation** from the
   Cl-4 "~10 % everywhere" definition-of-done, on the philos row only.
2. **A residual 0–25 m blind band remains.** A vessel that stays within 25 m of
   shore for its entire track still never initiates. This is a smaller zone than
   ADR-0001's 50 m, not its elimination.

## The parked third operating point — the pending band

An alternative measured 2026-07-12 (`2026-07-12_cl4_pending_band_probe.md`):
in-band births admitted only **after K re-detection scans** (a "waiting room" for
near-shore contacts — it uses TIME, which distance alone cannot). It beats
unconditional-A2 at any K≥2, **matches** the W25 front's phantom cost at K≈5–8,
covers the **full 0–50 m band** (closing the residual above) at ~7–17 s
first-track latency, phantoms stay in-strip. It is **PARKED, not rejected** — it
matches W25 rather than beating it, and costs a tracker build + latency.

**Reopen trigger:** a deployment that operates vessels inside the **0–25 m band**
(quay/dock operations), where the residual blind band bites. Then the pending
band is the priced escalation, and its bounded first-track latency is consistent
with ADR-0002's own bounded-latency promotion language.

## Revisit when

- The reopen trigger above fires (0–25 m operations); or
- a real near-shore workload shows the philos in-strip phantom price materially
  mis-estimated (the +10.45 was measured on one geography, Boston).

## Consequences

- One deployable config now initiates near-shore movers in the 25–50 m band; the
  Cl-4 claim closes with a named, dated deviation rather than a clean sweep.
- Consumers wiring the deployable config must build their `CoastlineModel` with
  `offshore_halfwidth_m = 25` (see `docs/integration-guide.md`). Every other
  named config is byte-identical to master (proven: `Config,
  Cl4OffshoreStripScopedToDeployableConfigOnly`; the R3 two-class A/B).

## Reconciliation addendum (2026-07-15) — post-fix-wave re-pin of the headline rows

The pre-release fix wave landed two correctness fixes that touch the
gauntlet's autoferry family: W2.4 (sensor-activity coverage measured from
own-ship, not the ENU origin; identity-keyed cooperative retirement,
merged 34367f6) and the wave-3/wave-4 estimator repairs (merged
b284f8f/738e542). Frozen rows re-measured on post-W2.4 master (wave-4
Stage 0, `docs/baselines/2026-07-13_fixwave_wave4.md`):

| Row | Frozen (d6cc871) | Post-W2.4 | Δ |
|---|---|---|---|
| env-2 (channel) | **8/8**, GOSPA 13.38 | **8/8**, 13.75 | +0.37 |
| env-1 | 16.57 | 15.49 | −1.08 (improved) |
| harbor (5-seed) | 9.53 | 9.53 | 0 |

**The claim STANDS**: env-2 revival held 8/8 (the decision-carrying fact),
philos byte-identical, harbor unchanged. The headline numbers are hereby
re-pinned to the post-W2.4 values (13.75 / 15.49); the small env-2 shift
and the env-1 improvement are correct-behavior effects of measuring sensor
coverage from own-ship. Wave 4's estimator fixes left all three rows
bit-identical to Stage 0.

**Named parked item — the env-2 EO/IR anchored-mode bias seed.** The
`_anchored` diagnostic seed (7.0°/4.9°) predates the wave-3 bias-chain
repair and was additionally mis-derived (mean |residual| ≈ 0.8·σ noise
under min-|residual| association, not a signed bias). Corrected signed
re-derivation (wave-3 hand-back): EO +1.66°, IR −1.92° (IR sign OPPOSITE
the seed), both dwarfed by ~7–9° noise. Adoption is PARKED: the deployment
path is byte-identical (seeds affect the `_anchored` diagnostic mode
only), and adopting any new seed would perturb the re-pinned rows for a
diagnostic-only benefit. Revisit if the anchored mode is ever used as a
claim input, and consider "seed small or not at all" first.
