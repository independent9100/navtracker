# Implementer prompt — clutter/birth-model campaign (Tier-3 pick; the increment-8 redirect)

Status: ready to hand off. Paste everything below the line. Origin: the
increment-8 verdict (eval-log 2026-07-04/05) — persistence suppression closed
NEGATIVE; the real dense-harbor over-count is **diffuse clutter**, and the
named redirect is a clutter/birth-model investigation (spatially-varying λ_C
under PMBM). This is the LAST open quality front before the water test.
Everything this campaign needs was deliberately pre-built: the sim
discriminator gate, 18× faster replays, the two-class pricing method.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
including the parallel-work convention). Mission: reduce the PMBM dense-scene
over-count by improving the CLUTTER and/or BIRTH model, with proof on both
controlled and real data, without damaging anything the standing gates
protect. This is a CAMPAIGN: three phases, two arbiter checkpoints. Budget
~1 week total.

## Setup

- `git worktree add ../navtracker-clutter -b clutter-birth-campaign` off
  current master. Own build dir. Conan sandbox gotcha per CLAUDE.md.
- The natural owner is the sim-gates implementer — you built the
  discriminator this campaign is judged by.

## Phase 0 — defuse backlog #21 first (~half day, its note says so)

`harbor/pmbm_adapt_k3` has a cardinality decision on a 1e-15 knife-edge
(backlog #21) that flips under epsilon perturbation. This campaign will run
that gate constantly; fix the fragility FIRST per #21's own instruction:
either a robust assertion (tolerance/cardinality band) or nudge that config's
birth threshold off the borderline. Small, separate commit, priced.

## The target, quantified (your baselines)

- `sim_ms_clutter_burst` (the DESIGNED discriminator): uniform-λ trackers
  over-count on compound-K clutter — MHT card_err +2.51, PMBM +3.48. A
  spatially/statistically honest clutter model must beat uniform-λ HERE,
  measurably.
- HAXR decimated (kattwyk_08_dec50_w285, md5 304cdeb8…): card_err ~48.8
  (PMBM coverage_land) — the real-data over-count.
- philos: the historical regression territory. KEEP safety is ABSOLUTE.

## Known minefield (read the history before designing — this ground is mined)

1. **λ_C ↔ miss-P_D coupling.** The philos diagnosis (2026-06-24): the
   wrong-math miss-P_D is the LOAD-BEARING BRAKE on over-counting; naive λ_C
   changes re-open the philos over-count. Any clutter-model change must be
   analyzed jointly with the miss-P_D term, not in isolation.
2. **λ_birth is competed against λ_C** (absolute-not-ratio was a diagnosed
   defect). Changing one silently retunes the other. Treat (clutter, birth)
   as one design surface.
3. **dense_clutter spiral + KEEP over-deletion** are the two historical
   failure modes of clutter tuning — both have standing gates; run them
   early and often, not at the end.
4. **ADR 0001 no-birth zone is KEPT ON PURPOSE** (<50 m offshore) — do not
   "fix" it in passing; the philos win depends on it.
5. **Do NOT resurrect occupancy-based suppression** — increment 8 closed it
   (negative, two geographies). The occupancy layer stays what it is:
   hazard/presence + corroboration.
6. Parked prior art to weigh in the design (don't rediscover): clutter-aware
   PPP-birth (Phase-3 parking lot, 2026-06-21 — parked as marginal, but that
   was pre-TPMBM and pre-discriminator); the cmap primitive (measured INERT
   under PMBM, 2026-07-01 — understand why before proposing anything
   map-shaped); per-sensor-unit λ_C (autoferry finding: a single λ_C across
   sensor units is wrong under multi-sensor).

## Phase A — design (~1.5 days) → ARBITER CHECKPOINT 1

Deliverable: a design note (docs/superpowers/plans/, dated) containing:
- Candidate models, each with the four-part standard (math, assumptions,
  rationale-vs-alternatives, what-to-test). Candidates to at least evaluate:
  (a) spatially-varying λ_C (gridded intensity, learned online from
  unclaimed returns — state how it differs from the inert cmap and why it
  won't be inert); (b) non-Poisson clutter count (negative-binomial /
  gamma-modulated — matches what the compound-K sim proved trackers suffer
  under); (c) clutter-aware birth weighting (the parked PPP-birth,
  re-priced); (d) per-sensor λ_C. Combinations allowed; pick ≤2 to build.
- The joint (λ_C, miss-P_D, λ_birth) analysis for each candidate — mine #1/#2
  above, addressed explicitly.
- The experiment matrix: which gates, which order, what success looks like
  as NUMBERS (see Phase C), predicted failure modes.
- STOP here. Hand the design to the arbiter; Phase B starts on approval.

## Phase B — implement (~2 days)

Approved candidate(s) behind per-instance config (default-off = bit-identical,
proven — the standing toggle rules). TDD; unit tests for the model math
itself (intensity estimates, count distributions), not just end-to-end.

## Phase C — price (~1.5 days) → ARBITER CHECKPOINT 2

The full battery, in this order (fail-fast on the cheap gates):
1. `sim_ms_clutter_burst`: the discriminator — card_err must drop vs
   uniform-λ, stated with the number. Also the other 5 sim scenarios
   (no regression).
2. harbor_complete_truth + dense_clutter_datum: unchanged-or-better; no
   spiral.
3. philos KEEP replays: safety metrics ABSOLUTE (byte-identical or
   fp-noise); tracks_on_keep flat.
4. HAXR decimated (+ raw if cheap): the real-data payoff — card_err delta
   reported with lifetime/gospa_missed guard columns (real vessels
   untouched).
5. Wall/RSS + per-scan p99/max deltas (the model must not undo the perf
   arc; a clutter map that doubles scan cost needs to say so).
Deliverables: results doc + eval-log entry + frontier-style table; promotion
recommendation (default-on / named-config / not-promoted) left to the
arbiter. An honest "built, measured, not better" is an acceptable outcome —
it closes the redirect either way.

## Acceptance

1. Phase 0 committed + priced separately.
2. Checkpoint 1 design note (four-part per candidate, joint-coupling
   analysis, experiment matrix) — no implementation before approval.
3. Phase C battery complete with the ordered fail-fast discipline; suite
   green; default-off bit-identical proven.
4. Docs: learning-doc addition if a genuinely new concept ships (a
   spatially-varying clutter field or negative-binomial counts qualify —
   subsection + figure); algorithm doc for whatever lands; integration-guide
   entry if any Config surface is added (drift-guard will insist).
5. Stop-and-report triggers: any KEEP safety movement beyond fp-noise at any
   point; the discriminator NOT separating candidates (gate failure — that's
   a finding about the gate); single-run times exploding past ~5 min on the
   standard workloads.
