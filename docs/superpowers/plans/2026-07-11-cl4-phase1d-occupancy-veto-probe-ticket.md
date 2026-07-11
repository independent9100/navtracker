# Implementer prompt — Cl-4 Phase 1d: occupancy floor-veto — can the persistence grid exclude the pier without blinding the floor?

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(`docs/algorithms/comparison-baselines.md` §Cl-4). Phases 1a–1c closed every
kinematic form of the conditional floor: association launders a walk along
a linear extended structure into a CV vessel transit (Phase 1c, merged
e676d67 — pier reaches 104–130 m under honest association vs the 72.6 m
revival ceiling; smoothness gap vanishes on equal footing). The missing
signal is **extent/map**, orthogonal to motion — and we already ship it:
`LiveOccupancyModel` (`core/static/LiveOccupancyModel.hpp`) keeps an EWMA
persistence grid (`ewma_alpha = 0.3`, `persistence_bar = 0.5`) with
`persistentCells()` / `persistenceCells()` queries, is chart-free, and was
explicitly retained as a corroboration substrate. The candidate composition:

> Floor: revive a suppressed birth chain that satisfies (M scans in 30 s AND
> net displacement ≥ D) — **UNLESS the chain's current endpoint cell (or its
> neighborhood) is occupancy-flagged as persistent structure.**

Why this should separate: a transiting vessel constantly enters *fresh*
cells (its endpoint is never flagged), while a pier walk only ever visits
pier-point cells — each re-hit ~90% of scans, flagged within a few scans at
alpha 0.3. This probe measures whether that geometry actually holds, on the
same three workloads, with binding criteria. It does NOT contradict the
increment-8 negative: that closed occupancy as a *general* phantom
suppressor on HAXR diffuse clutter; this is a narrow veto on a floor that
does not exist yet.

Budget ~1–1.5 days. North-star tag: Cl-4 Phase 1d. Same discipline:
measurement only, kill-criteria binding, NO-BUILD is a success outcome.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4p1d -b cl4-phase1d-occveto-probe`, own
build dir; autoferry + philos + harbor fixtures from the MAIN tree —
symlink; skips named BY NAME). Extend `tools/cl4_a3_census.cpp`: feed each
workload's measurement stream into an offline `LiveOccupancyModel` instance
(shipped machinery, default params first) alongside the existing Tier-A
chainer, and evaluate the veto at each chain's rule-satisfaction moment.

## The central quantity: the race

The veto only works if the grid flags pier cells FASTER than a pier chain
can satisfy the floor rule. Both clocks start at scenario start:

- **T_floor(chain):** first scan where the chain satisfies (M in 30 s AND
  net disp ≥ D) — use the Phase-1b/1c feasible-for-K1 corner, e.g. M = 8,
  D = 50 m, plus a second point at D = 70 m.
- **T_veto(cell):** first scan where the chain-endpoint cell's EWMA crosses
  `persistence_bar`.

Veto wins iff T_veto(endpoint cell at T_floor) ≤ T_floor. Report this per
chain, per seed.

## Step 0 — wiring inventory (half day max)

Grid cell size and neighborhood semantics (`suppression_radius_m` = 25 m
ramp — decide and STATE whether the veto tests the bare cell or the ramp);
what "a fed scan" is on each workload (the EWMA is per *fed scan* — philos
rotating radar vs autoferry stream vs harbor sim cadence normalize
differently; pin the feeding convention up front); datum anchoring per
workload (the model is an `IDatumChangeSink`; the census uses fixed datums,
say so).

## Step 1 — harbor (the flag side + the race)

5 seeds, radar stream → occupancy. Report: (a) time-to-flag for each pier
point cell (expected: 2–3 hits at alpha 0.3 — confirm); (b) for every pier
chain/PMBM pier track that satisfies the floor rule, the race verdict —
was its endpoint cell flagged at T_floor? (c) sea-clutter cells: none
should ever flag (transient); confirm. (d) anchored boats: their dwell
cells WILL flag — that is fine (they fail D anyway and stay with the
static-hazard channel; report, don't fix).

## Step 2 — env-2 (the pass side + the latency cost)

4 scenarios, radar+lidar → occupancy. The kill-risk this step exists for:
**the env-2 shore-huggers dwell** (median per-step speed 0.0–0.5 m/s on
half the targets) — a dwelling vessel flags its own cell within ~3 scans.
The claim that saves the floor: when the vessel *moves* (the only time it
can satisfy D), its endpoint is a fresh unflagged cell. Measure it:

- Per target: at every scan where the (M, D) rule is satisfied, was the
  endpoint cell flagged? → revival preserved yes/no, and the added latency
  vs the no-veto floor (first no-veto satisfaction vs first veto-passing
  satisfaction).
- Also report the fraction of each target's path cells that end up flagged
  (how much structure-shadow a dwelling vessel paints), and whether a
  target ever re-enters its own flagged wake.

## Step 3 — philos (the guard, unchanged expectation)

The veto only removes revivals, never adds — so K2 can only improve. Confirm:
the Phase-1b moving-tail chains (the 2 at M≥8, D≥50) — endpoint flagged or
fresh? Report the re-admission count with veto active.

## Binding kill-criteria (agreed now)

**BUILD** requires, at a stated (M, D, bar, neighborhood) point **with
margin** (re-test at persistence_bar ± 0.15 and at 2× cell size — a veto
that flips on grid resolution is not deployable):

- **K1:** ≥ 2/3 of the 8 env-2 targets still revivable with the veto
  active, added latency ≤ 15 s over the no-veto floor.
- **K3:** the race is won for EVERY floor-satisfying pier chain/track,
  all 5 seeds (zero pier revivals); sea clutter never flags.
- **K2:** philos re-admission ≤ the Phase-1b no-veto count (sanity: veto
  is monotone — if it increases, something is miswired).
- **K4 (named, not gated):** boundaries in the write-up — the dwelling-
  vessel latency cost (measured); a vessel moored alongside flagged
  structure is not revived (static-hazard channel, consistent with the
  standing scope note); deployment wiring note that the occupancy model
  must be registered as a datum sink (CLAUDE.md already documents this).

**NO-BUILD outcomes:** the race is lost on any seed (pier satisfies the
floor before its cells flag); or env-2 revival latency blows the bound
(dwelling vessels paint too much shadow); or the margin test fails. Then
the remaining Cl-4 options are path (c) re-pricing or the documented
per-geography residual — arbiter + user decide; do not pick.

## Constraints (all binding — same as 1a/1b/1c)

1. Zero shipped behavior change; census tooling only. Instantiating
   `LiveOccupancyModel` offline inside the census tool is exactly the
   intended use of shipped machinery — do not modify the model itself. If
   the probe NEEDS a model change to work at all, that is a stop-and-report,
   not a patch.
2. Do not touch `birth_existence_target` / gate arithmetic
   (`pmbm-design.md` §3.2.2) or ADR 0002 presence guarantees.
3. Extraction boundary: plots as fed today.
4. A Phase-2 build (floor + veto composed into the coverage stack) is
   judged on the FULL promotion-dossier gauntlet.

## Checkpoint (mandatory)

Hand back: Step-0 wiring conventions, the per-seed race tables (harbor),
the env-2 revival/latency table, the philos guard count, and the K1–K4
verdict with the margin test. The arbiter writes the Phase-2 build ticket
(or presents the endgame decision). Full suite green in your worktree,
skips named BY NAME. Commit on your branch; do not merge or push master.
