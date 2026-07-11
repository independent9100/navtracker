# Implementer prompt — Cl-4 endgame: the coverage-cliff price list (floor sweep 0.05–0.10)

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(`docs/algorithms/comparison-baselines.md` §Cl-4). The revive-near-structure
arc is exhausted (Phases 1a–1d + Phase-2 Stage 0, all merged): no mechanism
separates a transiting vessel from a linear structure. The user chose
**Option C — re-price the cliff**: measure the never-measured middle of the
gate-floor dial and produce the price list for a one-config decision. Only
the endpoints exist today (2026-06-30 A1): floor 0.10 = zero channel
revival / philos clean (gospa 73.1, card_err +6.9); floor 0.05 = full
revival / philos +36.2 card_err, gospa 100. Budget ~1 day. North-star tag:
Cl-4 endgame. Measurement only — the price *decision* is the user's.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4c -b cl4-endgame-cliff-sweep`, own build
dir; ALL fixtures from the MAIN tree — symlink; skips named BY NAME).
Zero shipped behavior change: this is a bench config sweep over the
EXISTING `min_new_bernoulli_existence` field — no new code paths, no
default changes. Reuse the 2026-06-30 A1 method.

## The sweep

Config: `imm_cv_ct_pmbm_coverage_land_ivgate` (the dossier candidate) with
`min_new_bernoulli_existence` ∈ {0.05, 0.06, 0.07, 0.08, 0.09, 0.10}
(finer steps near a knee if one appears). Everything else untouched — one
dial, one curve.

Per floor value, measure:

1. **Channel revival (env-2, all 4 scenarios):** are the shore-huggers
   tracked? Report lifetime ratio, GOSPA, and per-target tracked/invisible
   — the "real boats let in" column. Also env-1 for no-regression.
2. **Philos phantom price:** gospa, card_err, gospa_false — the "fakes let
   in" column. This is the load-bearing cost axis.
3. **No-collateral spot rows:** harbor (5-seed card_err/lifetime — the
   floor also gates non-land phantom births, so check it doesn't move),
   one autoferry row, and the Imazu battery aggregate. Flag ANY movement
   beyond seed noise.

## The explanatory measurement (why the curve bends where it bends)

Per env-2 target and for the philos in-band clutter population: the
distribution of pre-gate birth existence `r_new` (post-suppression). The
curve is just these distributions sliced by the floor — showing them makes
the price list explainable instead of magic, and tells us whether a knee
is real separation (vessel r_new sits above clutter r_new) or coincidence.
The census tool's plumbing from Phases 1a–1d should get you most of the way.

## Deliverable — the price list, no winner declared

One table, floor value per row: env-2 targets tracked (n/8) · env-2 GOSPA ·
philos card_err · philos gospa · gospa_false · harbor/autoferry/Imazu
no-regression flags. Plus the r_new distribution figure/table. Write-up as
`docs/baselines/2026-07-11_cl4_cliff_price_list.md` with a knee analysis:
IF a floor value exists where ≥6/8 env-2 targets track AND philos card_err
stays ≤ ~+12 (≈ the harbor yardstick's known phantom scale), name it as the
knee candidate; if the curve is a straight trade with no knee, say that
plainly. Frame the trade per house rule — name what each candidate setting
costs and when it hurts (phantoms at harbor exits vs invisible movers in
channels). The decision is the user's; do not recommend a shipping default.

## Constraints

1. Zero shipped changes (sweep configs live in the bench run only).
2. Do not modify suppression/gate arithmetic — this sweep is the existing
   dial at existing math (`pmbm-design.md` §3.2.2 invariant untouched).
3. Determinism: fixed seeds, reproduce commands in the write-up.
4. Full suite green in your worktree, skips named BY NAME. Commit on your
   branch; do not merge or push master.

## Checkpoint

Hand the price list back to the arbiter. If a knee exists, the follow-up
(setting the one-config default + docs: ADR-0001 third amendment, guide,
learning) is a separate ticket after the user picks the price point.
