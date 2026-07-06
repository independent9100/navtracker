# Implementer prompt — ais_ferry_far + almost_cross measurement pass (pre-water Tier 2 #10)

Status: DONE 2026-07-06 (09edb64, suite 1052/1052). Headline: PMBM ~46-54 m position accuracy (radar-only vs AIS truth, 19 s spot check, missed-dominated); radartruth relabeled everywhere (AIS-in-radar-frame, consistency check only); independence audit CLEAN (no shipped result was load-bearing on it); almost_cross ADR-0002 canary PASS. Findings for arbiter: #20 SOG/CoG unreachable via replay (loadAisCsv Position2D-only); bench cannot score truthless clips. Paste everything below the line to the implementer
agent. Origin: pre-water window item 10 (`docs/superpowers/plans/
2026-07-05-pre-water-window.md`). Held-out duty was discharged on
sailboats_busy (2026-07-05, 3H/1M/4P), so these two philos clips are released
for measurement. `ais_ferry_far` is the ONLY philos clip where
accuracy-vs-truth is honestly measurable — this pass adds the last real-data
accuracy number available before the water test.

---

You are working in the navtracker repo (C++17, CMake+Conan, read `CLAUDE.md`
first). Mission: a MEASUREMENT-ONLY pass over the two remaining philos clips.
No default touched, no algorithm changed. Runs are cheap now (a philos replay
is ~8.5 s post-Murty-fix), so favor completeness over shortcuts.

## Ground rules (these are standing project law — do not bend)

1. **Circularity rule.** philos video-derived labels and any truth built from
   a sensor THAT ARM CONSUMES yield mechanics/corroboration statements only,
   never accuracy claims. Concretely: an arm that ingests AIS may NOT be
   scored for accuracy against AIS-derived truth. Radar-only (and
   camera-bearing) arms scored against AIS truth are honest. Arms scored
   against the independent radar-derived truth follow the existing
   `philos_radartruth` pattern (`ReplayScenarioRun.cpp` ~L436) which exists
   precisely to kill AIS circularity. EVERY reported number carries its
   (arm sensors → truth source) pair; any circular combination is either
   omitted or explicitly labeled "mechanics only".
2. **Measurement-only.** If a result looks like it wants a config change,
   write it as a finding for the arbiter.
3. **Fixtures are local-only** (all of `tests/fixtures/` is gitignored).
   Committed tests must be skip-guarded on fixture absence, exactly like the
   existing philos tests. Record emitted-file checksums in the eval-log entry
   (the tracked drift-guard, per the camera-bearing precedent).

## What already exists (verified 2026-07-06 — do not re-extract)

- `tests/fixtures/philos/out/ais_ferry_far/`: `radar_plots.csv`,
  `ownship.csv`, `ais.csv`, `radar_truth.csv`, `meta.txt`.
- `tests/fixtures/philos/out/almost_cross/`: same minus `radar_truth.csv`.
- Existing wiring to reuse: `PhilosScenarioRun` in
  `adapters/benchmark/ReplayScenarioRun.cpp` (truth-source variants
  `philos` / `philos_union` / `philos_radartruth`) currently points at one
  clip via compile-time constants. Parameterize by env var mirroring the
  `HAXR_*` pattern in the same file (empty/unset = today's clip, bit-identical
  — prove with one baseline run before/after wiring). Do NOT fork new
  scenario classes per clip.
- Read each clip's `meta.txt` FIRST and put its facts (duration, target
  count, AIS density, any warnings) at the top of your results doc — clip
  assumptions have bitten before (sailboats_busy ran 120 s vs an assumed
  20–80 s and that amplified a prediction miss).

## The pass

Configs: the canonical MHT default + `imm_cv_ct_pmbm_coverage_land` (the
deployment-shaped arm). Add `pmbm_adapt` if time allows (autoferry-class
choice). Ten seeds where the harness supports it; these are replays, so
determinism green is expected and must be checked once per clip.

### Arm A — ais_ferry_far accuracy (the headline)

Radar-only arm scored vs AIS-derived truth AND vs `radar_truth.csv`
(two truth sources = a cross-check on the truth itself; disagreement between
them is a finding, not something to average away). Report the standing
metric set (gospa + decomposition, card_err, lifetime_ratio, id_switches)
per config. This is the number the pass exists for: real-data accuracy with
honest truth, on a clip nobody tuned against.

### Arm B — ais_ferry_far fusion mechanics

Radar+AIS arm (and radar+camera if `camera_bearings.csv` can be emitted with
the existing pinned pipeline — optional, note if skipped): mechanics
statements only vs AIS truth (circular), honest accuracy vs `radar_truth.csv`
only. Interesting questions: does AIS fusion hold one track per vessel
(no dual-track), does identity (mmsi) surface on the output (R11), do the
#20 paths fire if the AIS carries SOG/COG/nav-status (check `ais.csv`
columns — if SOG/COG present this is the FIRST real-data exercise of the
increment-2 velocity path; say so either way).

### Arm C — almost_cross

Determine from `meta.txt` + `ais.csv` what truth is honestly available; if
AIS is sparse/own-ship-only, accuracy claims are off the table and this clip
is a lifecycle/cardinality pass: track count sanity vs visible vessel count
(from meta/labels if present — labeled counts are mechanics-grade),
ID stability through the crossing geometry (its name suggests a
near-crossing — the classic association stress), and the DEFERRED anchorage
canaries (eval-log ~L777 deferred anchorage canaries for almost_cross:
anchored contacts must persist as tracks or hazards, never vanish —
ADR 0002). Also record whether the crossing produces an id-switch under
each config — that geometry is what M-of-N/association changes get judged
on later.

## Deliverables

1. `docs/baselines/2026-07-06_philos_farcross.md` + a metrics CSV next to it:
   per (clip × config × arm × truth-source) rows, circularity label column.
2. Eval-log entry (dated, observations only) with fixture checksums and the
   honest one-paragraph takeaway: "on the one philos clip with honest truth,
   the tracker's accuracy is X (config Y best)".
3. Skip-guarded scenario smoke tests per clip (existence + determinism +
   the anchorage-canary assertion for almost_cross), committed.
4. Env-parameterization change proven bit-identical on the existing clip.
5. Any finding that wants action → listed for the arbiter, not acted on.
6. Full suite green. Budget ~1 day; if the env-parameterization fights the
   scenario-run structure for more than ~2 h, stop and report the shape of
   the problem instead of forcing it.
