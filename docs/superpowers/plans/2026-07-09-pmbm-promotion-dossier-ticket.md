# Implementer prompt — PMBM promotion dossier: every deployment-decision number, one commit, one page

Status: ready to hand off. Paste everything below the line. Origin: the
canonical-tracker question is finally decidable — Cl-1 cold-start closed
(PMBM 18.62 beats paper 20.37, d70572a), #25 close-pass loss mitigated (the
innovation gate: CPA-overlap loss 163→6 s, `backlog25-phase2b`), veto
isolation measured. But the evidence is scattered across ~a dozen baseline
docs measured at DIFFERENT commits with different harness states. The user
will make the promotion call (formal canonical is still `imm_cv_ct_mht`,
2026-06-24 decision) — this ticket compiles the dossier they decide FROM.
North-star tag: Cl-3 #3 ("PMBM as canonical, if the gain holds") — this is
that milestone's measurement half. MEASUREMENT ONLY: no tuning, no config
changes, no recommendation of a winner — frame the trade (house rule).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-dossier -b pmbm-promotion-dossier`, own
build dir; ALL fixture families wired from the MAIN tree — this ticket
needs philos + autoferry + HAXR + harbor + imazu + sim_ms; skips must be ∅
or named). Budget ~1–2 days, almost all of it compute.

## The two contenders (fixed, do not add more)

- **Champion:** `imm_cv_ct_mht` — today's formal canonical.
- **Candidate:** `imm_cv_ct_pmbm_coverage_land_ivgate` — the deployment-
  shaped PMBM stack (coverage + land prior + innovation gate deweight@400).

Where a workload historically used a different PMBM variant (e.g. Cl-1 ran
`imm_cv_ct_pmbm_land`), ALSO run that variant on that workload and show both
rows, so the dossier is comparable backward AND forward. Note every such
difference explicitly.

## The gauntlet (one commit, one table)

All runs at YOUR branch-base commit, recorded in the doc header. Per
workload, per contender, the SAME column set: gospa (RMS where the workload's
convention says so), card_err, gospa_false, gospa_missed, lifetime,
track_breaks, id_switches, and per-scan latency (scan_proc_ms mean/p95/max,
`--fast-metrics` columns) where the harness provides it.

1. **Autoferry / Cl-1 protocol** — env-1 + env-2, no-AIS AND truth-AIS
   (the Helgesen frozen protocol; cite `docs/baselines/helgesen2022_reference.md`
   and reproduce its dated-section numbers as the cross-check that your
   harness state matches — if they don't reproduce, STOP and report before
   generating anything else).
2. **philos clips** — the KEEP-config discipline: candidate vs champion on
   the standard clips; expectation from history is near-parity (73.1 vs
   69.4 class) with the gate never firing — CONFIRM the gate fire-count is
   0 on real data and say so.
3. **HAXR 3 sites** — decimated standard arms (the increment-8 setup),
   veto at its default (ON).
4. **Harbor yardstick** — `harbor_complete_truth` (the M2-gate metrics:
   card_err, lifetime).
5. **Imazu 22 battery** — full 22 + the 6-dying-case cut: loss-seconds-
   overlapping-CPA, re-acquire ids, id_switches (candidate should reproduce
   the Stage-2 numbers; champion's churn numbers from the #11 forensics
   should reproduce too).
6. **Sim multi-sensor gates** — the 6 seeded scenarios incl. fusion-vs-
   radar-only dropout continuity.
7. **Runtime** — decimated + raw philos worst-scan latency vs the 148 ms
   scan interval for both contenders (the realtime criterion: worst-scan <
   interval with margin; cite the perf-arc method).

## The dossier document (the deliverable)

`docs/baselines/2026-07-10_pmbm_promotion_dossier.md`:

1. One summary table: workloads × contenders, the column set above, with a
   per-workload one-line reading (no winner-declaring language — "candidate
   better on X, pays Y" style).
2. A TRADE section naming each failure-mode axis explicitly, with its
   number: cardinality/false tracks (champion's #11 conveyor), continuity
   (candidate's breaks/lifetime, post-gate), identity (switches both ways),
   close-pass CPA behavior (post-gate candidate vs champion), runtime,
   and the standing residuals (+0.77 crossing-independent over-count;
   philos near-parity; anything NEW your runs surface).
3. An OPEN-QUESTIONS section: anything the dossier cannot answer without
   the water test or deployment facts — say so plainly rather than
   extrapolating.
4. Dated eval-log entry: exact commands, CSV artifacts, checksums, commit.
5. NO promotion recommendation. The final section is titled "The decision
   in front of the user" and states the trade in ≤10 plain-English lines.

## Acceptance

1. Every gauntlet row measured at one commit; historical cross-checks
   reproduced (or divergence root-caused and reported, not papered over).
2. Zero config/core changes. Zero tuning. If a workload can't run a
   contender without wiring changes, report the gap instead of building it.
3. Full suite green in your worktree, skips named (∅ expected with full
   wiring).
4. Stop-and-report: the Cl-1 dated-section cross-check fails; a contender
   config errors on any gauntlet workload; or any single number contradicts
   its source baseline doc by more than noise — that's a drift finding,
   surface it before continuing.
