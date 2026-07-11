# Implementer prompt — Cl-4 Phase 1c: smoothness + real-association displacement — does the pier/vessel overlap survive honest measurement?

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(`docs/algorithms/comparison-baselines.md` §Cl-4). Phase 1b measured the
plain conditional floor `(chain ≥ M in N scans) AND (net displacement ≥ D)`
to **NO-BUILD** (merged d5e68e8): env-2 revival needs D ≤ 72.6 m, but a
greedy NN chainer can *walk* the harbor pier's 10 m-spaced points to ~84 m
net displacement — empty feasible set. The probe left two measured escape
hatches, and this phase measures BOTH, together, because they attack the
same overlap from different sides:

1. **Smoothness:** pier walks are jumpy (max single-step speed 16–20 m/s —
   teleports between pier points on missed detections); env-2 vessels are
   smooth (per-step p95 ≤ 7.9 m/s). A per-step-speed term may separate with
   ~2× margin.
2. **The chainer was a conservative K3 proxy:** at r_chain 15 m the pier
   walk peaks at 50.6 m (BELOW the 72.6 m K1 ceiling), and a real PMBM
   Bernoulli under a motion model resists teleports. True pier displacement
   under honest association may reopen the D-only window without any new
   rule term.

Budget ~1–1.5 days (extends the existing census tool). North-star tag:
Cl-4 Phase 1c. Same probe discipline: kill-criteria below are binding and
agreed before measuring; NO-BUILD is a success outcome.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4p1c -b cl4-phase1c-smoothness-probe`,
own build dir; autoferry + philos + harbor fixtures from the MAIN tree —
symlink; skips named BY NAME). MEASUREMENT ONLY — zero shipped behavior
change. Extend `tools/cl4_a3_census.cpp` (which already has `--mode
chain/--mode target` from Phase 1b).

## Step 1 — honest displacement (hook 2): replace the NN-walk proxy for K3

Re-measure the pier's achievable net displacement under association that a
real tracker would actually perform, two tiers (do both; they bracket):

- **Tier A (cheap):** motion-model-consistent chaining — a chain carries a
  CV state (position+velocity, process noise matched to the bench config);
  a new detection extends the chain only if its innovation is plausible
  under the predicted state (state the gate; use the tracker's own
  association gate scale). This kills teleports by construction. Report the
  pier's max net displacement per seed, same 5-seed table as Phase 1b, at
  r_chain 15 m and 25 m equivalents.
- **Tier B (authoritative):** run the actual PMBM (`imm_cv_ct_pmbm`, the
  bench config) on harbor and measure, per confirmed pier-born track (the
  phantoms are identifiable — no truth within gate), the track's net
  displacement over its life and its max per-step speed. This is what a
  floor keyed on *Bernoulli* re-detection history would actually see.
  (Per-scan state export exists — the dossier and Phase-1a used it.)

Also re-run the env-2 targets under Tier A (their displacement should be
unaffected — they genuinely move — but confirm, don't assume).

## Step 2 — smoothness census (hook 1): the missing philos measurement

Phase 1b never measured philos chain smoothness. For the philos in-band
population (the 1140 chains), report the per-step-speed distribution
(median / p95 / max) split by the Phase-1b classes: stationary structure,
the moving tail (the 2–57 chains that satisfied various (M,D) points), and
overall. The question: does the moving tail contain smooth vessel-like
chains (real unlabelled near-shore craft — which a smoothness rule
correctly ADMITS, so they are not re-admission failures; say so if so) vs
jumpy drift/multipath (which the rule must reject)? Same census for harbor
sea clutter (expected trivial: length-1 chains have no steps).

## Step 3 — score the augmented rule family

Rule: `(chain ≥ M in N scans) AND (net displacement ≥ D) AND (max
single-step speed ≤ S)` — with displacement measured per Step 1 (report
both the old NN numbers and the honest numbers so the delta is visible).
Sweep the (M, D, S) grid (N = 30 s fixed, as Phase 1b; state the grid up
front). Report per point: env-2 revived (of 8) + latency, philos
re-admitted chains (by class), harbor pier/clutter chains admitted.

## Binding kill-criteria (agreed now)

**BUILD** requires a feasible (M, D, S) point satisfying ALL of, **with
margin** (the #24 knife-edge lesson — a config that flips on an epsilon is
not deployable):

- **K1:** ≥ 2/3 of the 8 env-2 targets revived within 30 s — and still
  satisfied when S is tightened by 2 m/s and D raised by 10 m from the
  chosen point.
- **K2:** philos re-admission of *non-vessel* chains ≤ 10% of the A1
  residual (projected gospa ≤ ~76). Smooth vessel-like philos movers that
  the rule admits are reported separately, not counted as failures — but be
  honest about the ambiguity (they are unlabelled).
- **K3:** zero pier tracks/chains admitted under the HONEST (Tier A and
  Tier B) displacement+smoothness measurement, across all 5 seeds — and
  still zero when S is loosened by 2 m/s and D lowered by 10 m.
- **K4 (new — boundary named, not gated):** quantify the fast-vessel
  boundary: a per-step cap S excludes a real vessel moving faster than S
  from revival while it stays in-band. Report, for the chosen S, how long a
  vessel at 1.5×S would remain in the 50 m band on a shore-parallel course
  (i.e. how big the accepted blind spot is). This goes in the write-up as a
  documented limitation, ADR-style — it does not gate the build.

**NO-BUILD outcomes:** the margin-robust feasible region is empty (report
which population bridges it and in which measured quantity); or Tier B
shows real Bernoulli pier tracks with vessel-like smooth displacement
(association itself launders the walk) — then the floor cannot key on
per-track history at all, and the arbiter decides between path (c)
re-pricing and a documented per-geography residual in the Cl-4 claim.

## Constraints (all binding — same as Phases 1a/1b)

1. Zero shipped behavior change; census tooling only (research bench
   target).
2. Do not touch `birth_existence_target` / gate arithmetic
   (`pmbm-design.md` §3.2.2) or ADR 0002 presence guarantees.
3. Extraction boundary: plots as fed today; chaining stays census-side.
4. Phase-2 build (if any) is judged on the FULL promotion-dossier gauntlet.

## Checkpoint (mandatory)

Hand back: Tier A/B displacement tables (old-vs-honest delta explicit),
philos smoothness census, the (M,D,S) score table with the margin test,
and the K1–K4 verdict. The arbiter writes the Phase-2 build ticket (or the
pivot). Full suite green in your worktree, skips named BY NAME. Commit on
your branch; do not merge or push master.
