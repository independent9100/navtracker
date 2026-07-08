# Implementer prompt — backlog #11: diagnose the Imazu identity churn (MHT) and its PMBM mirror image

Status: ready to hand off. Paste everything below the line. Origin: the
Imazu-22 suite (e771dad, `docs/baselines/2026-07-08_imazu22.md`) gave backlog
#11 its controlled instrument: MHT churns identity up to 72 mean
switches/truth on dense multi-target crossings (imazu_17/20) while position
holds (RMSE ~25–28 m); every single-target case is 0-switch. PMBM holds
identity (≤8.3 switches) but over-counts (+0.77 card_err). North-star tag:
Cl-2/Cl-3 diagnostic (identity stability is a headline metric on both).
This is a DIAGNOSIS pass — no config change, no knob promotion; findings for
the arbiter.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
parallel-work convention + fixture-trap note). Worktree:
`git worktree add ../navtracker-b11 -b backlog11-imazu-churn`, own build dir.
The `imazu_*` sim fixtures live in the MAIN tree only — symlink them in
(same for `sim_ms_*` if needed) and state ran-vs-skipped in the handoff.
Budget ~1 day.

## Question 1 — WHICH churn mechanism is it on Imazu? (MHT)

History matters here: #11 was diagnosed TWICE with different answers.
2026-06-11 assumed pair swaps; 2026-06-12 forensics showed the real sc5
mechanism was a CONVEYOR of short-lived duplicate tracks (only 21 of 182
events were swaps), rooted in filter over-confidence (→ item 12, honest R
landed 2026-06-16 and halved the churn). Imazu is a different regime:
dense CROSSINGS with clean-ish sim sensors — genuine swap-at-crossing is
plausible there in a way it wasn't on sc5.

Run the per-event switch dump (the 2026-06-12 forensics method — per-event
classification: pair swap at close pass vs duplicate-birth handoff vs
break+re-confirm) on the worst cases (imazu_17, imazu_20, plus one mid-churn
case as contrast) under the CURRENT MHT canonical. Deliverable: an event
census per case — % swaps / % conveyor / % break-reconfirm — with the
geometry named (which vessels, which crossing).

## Question 2 — is PMBM's over-count the mirror image?

Same close passes, PMBM canonical: are the +0.77 card_err extra tracks
duplicate Bernoullis born during the crossings (the same ambiguity expressed
as cardinality instead of identity), or unrelated (e.g. birth-model noise)?
Report where and when the extra ids appear relative to the crossing times.

## Question 3 — do the existing opt-in #11 knobs help HERE? (measurement only)

The three opt-in knobs from 2026-06-12 (`share_ambiguous_bearings`,
per-sensor static `gate_threshold`, `gate_recapture_tau_s`/`max_scale`)
were measured on autoferry and left OFF (recapture's lifetime cost was
catastrophic there). Measure them on the Imazu family — id_switches,
track_breaks, lifetime, gospa per case. The historical trade-off (switches
down, lifetime down) is the thing to watch; a knob that behaves differently
on crossing geometry than on bearing-carried tracks is a finding either way.
NO promotion in this ticket regardless of result — table + recommendation
only.

## Acceptance

1. Event census for ≥3 cases (Q1) + PMBM mirror analysis (Q2) + knob table
   (Q3), committed as a dated section in
   `docs/baselines/2026-07-08_imazu22.md` (or a follow-up doc referencing
   it) + dated eval-log entry with exact commands and checksums.
2. Zero config/algorithm changes. Findings + a ranked recommendation for
   the arbiter (including "do nothing; PMBM is the deployment tracker and
   holds identity" if that is what the data says).
3. Full suite green in YOUR worktree, fixtures symlinked, ran-vs-skipped
   stated, strict pass check (`grep '^100% tests passed'`).
4. Knife-edge rules apply: if you add any test, band assertions only —
   never exact pins on association outcomes.
5. Stop-and-report: the forensics tooling from 2026-06-12 no longer runs /
   can't be pointed at sim scenarios without core changes (report what it
   would take instead of building it unasked).
