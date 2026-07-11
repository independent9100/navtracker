# Implementer prompt — Cl-4 Phase 1: does sensor identity separate near-shore vessels from near-shore clutter? (ADR 0001 A3 evidence probe)

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
(one deployable PMBM configuration, user-set north star 2026-07-10 — see
`docs/algorithms/comparison-baselines.md` §Cl-4). The known blocker is the
<50 m offshore **no-birth zone** in the coverage stack (ADR 0001): the
`coverage_land` gate==target equality turns the soft shore ramp into a hard
cliff, so `imm_cv_ct_pmbm_coverage_land_ivgate` tracks NOTHING in the Cl-1
env-2 shore-hugging channel (dossier: lifetime 0, all targets missed), while
`pmbm_land` survives there (17.74) but has no coverage discipline. The
ranked fix is **ADR 0001 A3 — sensor-aware suppression**: shore clutter is
overwhelmingly a *radar* artifact; a camera/lidar/AIS report near shore is
strong evidence of a real vessel. This probe measures — offline, before any
birth-path code — whether that discriminator actually works on OUR gauntlet
data. Budget ~2 days. North-star tag: Cl-4 Phase 1.

**Precedent that binds this ticket:** the 2026-06-30 A1 measurement (naive
gate-lowering revives the channel but regresses philos 73.1→100 by
re-admitting water clutter) and the Phase-2a / clutter-campaign §5.0
discipline — kill-criteria are agreed BEFORE measuring, and **NO-BUILD is a
success outcome** (it killed two doomed builds already; zero waste).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-cl4p1 -b cl4-phase1-a3-probe`, own build
dir; philos + autoferry/Helgesen fixtures live in the MAIN tree — symlink
them; skips named BY NAME in the handoff). MEASUREMENT ONLY — zero shipped
behavior change in this phase. Instrumentation may be throwaway (reverted)
or default-off per-instance ctor-threaded; never global/static.

## The question

A3 says: exempt (or corroborate-override) births seeded by clutter-free
sensors so a near-shore real vessel initiates while radar shore clutter
stays suppressed. Two sub-approaches (ADR 0001 §A3):

1. **Sensor-typed exemption** — do not apply (or sharply reduce) the land
   scale for births seeded by camera/lidar/AIS measurements.
2. **Cross-sensor corroboration override** — a near-shore birth corroborated
   by ≥2 sensors at the same location overrides the prior.

The probe answers, with confusion tables, whether either rule would
(a) **revive** the env-2 channel targets the coverage stack currently
misses, WITHOUT (b) **re-admitting** the philos shore/water clutter the
prior exists to kill. Both sides on real gauntlet data, no code in the
birth path.

**Read the ADR caveat first** (ADR 0001 §A3, "Important caveat"): under
adaptive birth, `r_new` is pinned to `birth_existence_target` *before* the
land scale and is algebraically independent of sensor count (the λ_C
cancellation invariant, `pmbm-design.md` §3.2.2 — do not touch that math).
So more sensors do NOT emergently lift a birth past the gate; the
*suppression rule itself* must be conditioned on sensor identity. This
probe measures evidence availability and discrimination — the r arithmetic
comes in Phase 2, if built.

## Step 0 — sensor-stream inventory (grounding; half a day max)

Per gauntlet workload, what actually feeds the tracker:

- **Cl-1 env-2 channel replay** (Helgesen/Autoferry, Trondheim urban
  channel): which streams does OUR bench feed per condition (no-AIS =
  radar + lidar + EO + IR? which of these seed births vs update-only)?
  The dossier's collapse rows are the no-AIS condition — inventory that one
  precisely.
- **philos**: radar plots + camera-bearing fixtures (2f0261c pipeline,
  local-only) + AIS. Do the camera-bearing windows overlap the shore-clutter
  windows in time and bearing coverage? If they do NOT overlap enough to
  score the guard side, STOP-AND-REPORT (the probe can't be honest without
  it).
- **harbor_complete_truth sim**: expected radar-only → A3 inert there. Say
  so explicitly; the harbor headroom (card_err 11.64→7.43, charted-pier
  A/B — see `docs/baselines/2026-07-10_harbor_truthsort_reconcile.md`) is
  the *static/charted* prior, a separate Cl-4 rider, not this probe.

## Step 1 — revival side (env-2): do the missed targets have the evidence?

For every real target the coverage stack misses in env-2 (the collapse
set): at each scan where it is inside the <50 m band and detectable, was
there a camera/lidar/AIS measurement that would seed or corroborate a birth
at its location (state the association radius + time window you use; keep
them physically justified and fixed before scoring)? Deliverable: a
per-target table — target id, time-in-band, scans with clutter-free-sensor
evidence (per rule variant), verdict "revivable under variant i: yes/no".

## Step 2 — guard side (philos): what would the rule re-admit?

For every birth `coverage_land` currently suppresses on philos (the ~185
near-shore stationary returns + the offshore-ramp water-clutter residual
that the gate catches — the thing A1 re-admitted): would each rule variant
re-admit it? Count HONESTLY — cameras see piers, moored objects, and
background structure too (the ADR names this), and camera-corroborated
clutter is exactly the failure mode that would kill A3. Deliverable: counts
of re-admitted births per variant, split radar-only / camera-corroborated /
AIS-corroborated, with positions (on-land / in-band / open-water).

## Step 3 — score the variants

Variants to score (at minimum): (i) exempt AIS-seeded/corroborated only,
(ii) exempt AIS + camera(+lidar), (iii) two-sensor corroboration override.
For each: the confusion table — env-2 targets revived vs philos clutter
births re-admitted — plus a projected philos damage estimate anchored to
the A1 measurement (A1 re-admitted the full residual and cost gospa
73.1→100, card_err +6.9→+36.2; scale linearly by re-admission fraction as
a first-order projection, and say it's first-order).

## Binding kill-criteria (agreed now; do not renegotiate after measuring)

**BUILD** (any variant qualifies) requires BOTH:

- **K1 (revival):** the variant revives ≥ 2/3 of the env-2 in-band missed
  targets (by the Step-1 table — evidence present on enough scans to
  plausibly confirm, ≥3 corroborated scans per target).
- **K2 (guard):** the variant's philos re-admission is ≤ 10% of what A1
  re-admitted (first-order projected philos gospa stays ≤ ~76, card_err
  ≤ ~+10 — inside the noise band of the 73.1 guard).

**NO-BUILD outcomes (equally valuable — write them up, don't force K1/K2):**

- K1 fails because the env-2 misses lack clutter-free-sensor evidence →
  A3 cannot be the Cl-4 fix on this gauntlet; recommend pivot to ranked
  path (b), the conditional coverage floor (arbiter writes that probe).
- K2 fails for every variant (camera corroborates structure too well) →
  A3 dead as specified; record which corroboration signal leaked.
- Split verdict (e.g. AIS-only passes K2 but fails K1 because env-2 no-AIS
  is the deployment-relevant condition) → report it as the trade it is; the
  arbiter decides. Frame the trade — name each failure mode and when it
  hurts; do not declare a winner.

## Constraints (all binding)

1. **Zero shipped behavior change.** No config, no default, no birth-path
   code lands in this phase. Census instrumentation is throwaway or
   default-off per-instance; if kept, it must be byte-identical when off.
2. Do not touch `birth_existence_target` / gate arithmetic (λ_C
   cancellation invariant, `pmbm-design.md` §3.2.2) or the ADR-0002
   presence guarantees.
3. Extraction boundary: everything operates on plots/measurements as fed
   today; no new extraction.
4. Any fix candidate is judged on the FULL promotion-dossier gauntlet in
   Phase 2 — this probe deliberately front-loads the two workloads that
   disagree (env-2 revive vs philos guard).

## Checkpoint (mandatory before any Phase-2 work)

Hand back: Step-0 inventory, Step-1/2 tables, Step-3 confusion matrices,
and the K1/K2 verdict per variant. The arbiter writes the Phase-2 build
ticket (or the pivot). Full suite green in your worktree, skips named BY
NAME vs the expected fixture-gated set. Commit on your branch; do not
merge or push master.
