# Implementer prompt — Imazu 22: the canonical encounter suite as named sim scenarios

Status: ready to hand off. Paste everything below the line. Origin: named
follow-up from the multi-sensor sim survey (2026-07-06) — the 22 canonical
Imazu encounter situations (head-on / crossing / overtaking singles up to
3–4-ship combinations) transcribed as seeded sim scenarios: a citable
regression suite for exactly what crossing geometry stresses — identity
stability through close passes.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` first —
parallel-work convention + fixture-trap note apply: worktree
`git worktree add ../navtracker-imazu -b imazu22-suite`, generated fixtures
live in the MAIN tree, fixture-gated tests must RUN (state ran-vs-skipped).
Budget ~1 day.

## Source of the geometries

The Imazu problem set is standard in the COLREG/collision-avoidance
literature; encodings exist in research repos (e.g. the CORALL repo
validates over all 22). Transcribe the 22 initial geometries (own-ship +
1–3 targets, bearings/ranges/courses/speeds) into
`tests/fixtures/sim_multisensor/generator/` as a new scenario family
`imazu_01..imazu_22`. Cite your exact source for the parameters in the
generator docstring — if sources disagree on a case, pick one, say which,
and note the divergence. Own-ship follows its nominal course (we are a
TRACKER suite, not a COLAV planner — own-ship does NOT maneuver; the
encounters are allowed to pass close or even collide-geometry through;
that's the stress).

## Scenario construction

- Reuse the existing Layer-2 observation models exactly (radar per-scan +
  M.1371-lite AIS): radar+AIS arm per scenario, one seed each (22 scenarios
  is the sweep; multi-seed only if a case proves flaky — that's a finding).
- Truth at 1 Hz, standard CSV schemas, checksums in the eval-log entry;
  regeneration deterministic (two clean runs identical — spot-check 3).
- Keep duration short (each case resolves in a few minutes of sim time);
  the full 22-battery replay should stay cheap (~minutes total).

## Bench + gate

- Extend `SimMultisensorScenarioRun` scenario discovery to the imazu family
  (env-pointed, skip-guarded — same pattern; do NOT fork a new class).
- Run MHT default + `imm_cv_ct_pmbm_coverage_land` over all 22. Results
  table `docs/baselines/2026-07-08_imazu22.md`: per case — id_switches,
  track_breaks, card_err, gospa (honest truth by construction).
- Committed gate test: determinism per case (sampled) + the headline
  assertion at the level the data supports: report id_switches per case;
  assert only a COARSE band (e.g. no case exceeds N switches, N from the
  measured table + margin — a tripwire, not a pin; the knife-edge lessons
  apply: no exact-value pins on association outcomes).
- NO tuning to make cases pretty. A case where identity churns is a
  FINDING (candidate evidence for backlog #11 bearing-driven identity
  churn) — report it with the geometry named.

## Acceptance

1. 22 scenarios generated + checksummed; source cited; determinism shown.
2. Battery table committed + dated eval-log entry; suite green with
   fixture-gated tests RUN against main-tree fixtures (ran-vs-skipped
   stated in the handoff).
3. Findings (churning cases → #11 evidence) listed for the arbiter; no
   config changes.
4. Stop-and-report: geometry sources contradict each other beyond
   parameter noise; or a case degenerates (vessels spawn overlapped).
