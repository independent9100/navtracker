# Implementer prompt — D8 R-BAD extraction: first fixtures + label-scored replay

Status: ready to hand off. Paste everything below the line. Origin: the D8
feasibility GO (2026-07-06, eval-log; merged e807cb6) — CC-BY-4.0, radar
detections as CSV + synced video, 31.6 GB, provided labels + video for
independent label passes. This executes the feasibility note's named next
step: extract 1–2 station-hours as fixtures + a label-scored replay. The
REGIME CAVEAT governs all claims: automotive mmWave FMCW (60–81 GHz), NOT
marine X-band — this corroborates the berthing scene on a NEW SENSOR CLASS;
it is not a third marine geography and philos/HAXR tuning is not expected to
transfer.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
including the parallel-work convention AND its fixture-trap note: this ticket
CREATES fixtures, which live in the MAIN tree's `tests/fixtures/`; do code in
a worktree (`git worktree add ../navtracker-d8 -b d8-rbad-extraction`) but
place/verify fixture data + run fixture-gated tests against the main tree,
and state ran-vs-skipped in your handoff. Budget ~1–1.5 days.

## Step 1 — download + resolve the four confirm-at-extraction flags (~2 h)

Zenodo record 16936465. Before writing any code, resolve and record the four
flags the feasibility note could not confirm (MDPI full text was
bot-blocked): (a) detection-CSV columns + coordinate frame; (b) annotation
provenance (how were the labels made — from the radar itself? then they are
mechanics-grade for us, circularity rule); (c) own-ship/ego pose presence and
rate; (d) radar range/scan characteristics. Any of these may reshape the
ticket — if (c) says NO ego pose, stop and report (a berthing scene without
own-ship motion may still work as a fixed-frame scene, but that's an arbiter
scope call).

## Step 2 — extraction (~half day)

- Pick 1–2 contiguous station-hours covering at least one arrival or
  departure (the berthing archetype) + some port-idle.
- Extraction script in `tests/fixtures/rbad/` (committed source, data local —
  the philos/sim gitignore-negation pattern; pin deps). Output: our standard
  CSV shapes (radar plots w/ per-plot sigmas if derivable, ownship if ego
  pose exists, labels in whatever honest grade Step 1 determined).
- Fail-loud integrity guards from the start (the R8.8 lesson): assert
  plausible rates, dynamic values, non-placeholder columns. Checksums in the
  eval-log entry.

## Step 3 — replay + label-scored pass (~half day)

- Wire an `RbadScenarioRun` (env-pointed, skip-guarded, datum set — the
  HAXR/simms pattern). Bench flag `--with-rbad`.
- Run MHT default + `imm_cv_ct_pmbm_coverage_land`. Determinism check once.
- Score against the provided labels at the grade Step 1 assigned (existence/
  region mechanics vs kinematic truth). Report the standing metrics with the
  regime caveat stated ON the table — nobody quotes an mmWave number as a
  marine-radar result.
- Interesting questions to answer (report, don't tune): does the tracker
  hold berthing-speed targets (slow, maneuvering, close-range) without
  fragmenting? What does the over-count look like on a sensor class with
  completely different clutter statistics? Does the anchored/moored logic
  fire on stationary labeled objects?

## Acceptance

1. Four flags resolved + recorded; extraction committed (source only) with
   integrity guards; checksums in a dated eval-log entry.
2. Replay wired, skip-guarded, deterministic; results table with the regime
   caveat inline; suite green — fixture-gated tests RUN against main-tree
   fixtures, ran-vs-skipped stated in the handoff.
3. Findings for the arbiter (no config changes, no tuning to this dataset —
   it's a reality-check arm, not a tuning target).
4. Stop-and-report: no ego pose (Step 1c); label provenance circular for
   everything (Step 1b — then it's video-label material for a later
   user/analyst pass, report that); single replay > ~10 min.
