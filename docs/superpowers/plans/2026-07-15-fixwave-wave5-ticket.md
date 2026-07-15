# Implementer prompt — fix wave, wave 5: the pipeline/lifecycle tail + the Section-D test backfill batch

Status: ready to hand off AFTER the arbiter confirms the wave-3+4 merge.
Paste everything below the line. Origin: the pre-release review synthesis
(`docs/reviews/2026-07-09-prerelease-open-points.md` §B Themes 5+6, §C
tail, §D) — the last confirmed code defects of the fix wave, plus the
test holes the synthesis paired with them. Budget ~2–2.5 days. TDD
throughout; teeth per the #24 standard. Read the verifier evidence in
`docs/reviews/2026-07-09-prerelease-review/10-bughunt-findings.md` first.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-fw5 -b fixwave-wave5` off post-merge
master, own build dir, fixtures inner-level symlinked; suite runs under
`NAVTRACKER_REQUIRE_FIXTURES=1` — 0 skips enforced. Commit on your branch;
never merge or push master. `fixwave-wave1` (held F2) stays untouched —
the F2 cycle is a SEPARATE ticket running in parallel; if you find your
W5.2 fix collides with `PmbmTracker` regions the F2 cycle touches,
coordinate through the arbiter rather than resolving unilaterally.)

## W5.1 — `Tracker::process()` drops soft (JPDA) association results (HIGH)

`Tracker.cpp:144`: on the soft-association path no track is updated at
all. Fix per the finding's mechanism; end-to-end test with the JPDA
associator wired (a two-track ambiguous scan: both tracks receive
weighted updates; the pre-fix code leaves both un-updated — teeth).

## W5.2 — PMBM mixed-timestamp scans double-propagate (MED, Cl-4-relevant path)

`PmbmTracker.cpp:964`: in a mixed-timestamp scan a detected Bernoulli's
`last_update` is rewound while others advanced → the interval is
propagated through F/Q twice. Fix + the missing Section-D test that IS
this bug's home: **same-scan two-sensor fusion of one target end-to-end**
(radar+AIS same timestamp → single track, single propagation — assert
covariance does not inflate double).

## W5.3 — `Tracker::processBatch` missing the backlog-#15 batch sort (MED)

MHT/PMBM got the unsorted-batch sort fix; the plain Tracker path never
did — unsorted batches stale-drop. Apply the same sort; test with an
out-of-order batch.

## W5.4 — Tentative→Coasting promotion on miss (MED, operator-facing)

`TrackManager.cpp:64`: a one-hit clutter blip that misses becomes
Coasting — which is CPA-ELIGIBLE (Tentative is not) → false collision-risk
events; also violates the documented Coasting definition ("was
Confirmed"). Fix the lifecycle transition (a missed Tentative stays
Tentative / dies per M-of-N — never Coasting). **A/B REQUIRED:** this is a
lifecycle change — report bench deltas (id churn, track counts, CPA event
counts); Cl-4 candidate rows = finding rule, same as waves 2–4.

## W5.5 — MHT deferred-commitment leaf protection inert (MED)

`MhtTracker.cpp:531`: protection flags are always one `branch()` behind.
Fix per the finding; a test where the protected leaf survives a prune it
previously lost.

## W5.6 — the Section-D backfill batch (each small; each pins an invariant)

1. **AIS sentinels:** SOG=1023 / COG=3600 fed to `AisAdapter` → velocity
   treated unavailable, never as 102.3 m/s (the sentinel-as-velocity hole).
2. **`loadAisCsv` timestamp parsing:** ISO-8601 and BaseDateTime rows parse
   (the path that silently drops all DMA rows today — if parsing is
   genuinely broken, FIX the loader; if it's format-gated by design,
   assert the reject is loud, not silent).
3. **ID-never-reused invariant:** a delete→create cycle test asserting the
   architecture invariant no test currently pins.
4. **End-to-end determinism through the real adapters:** NMEA/CSV in →
   TrackOutput out, twice, byte-identical (the replay contract currently
   pinned only at core level).
5. **Housekeeping:** bump the `VetoIsolationHaxrAB` ctest TIMEOUT (280 s
   standalone vs 300 s cap — the wave-3 hand-back flagged it starving
   under parallel load).

## Acceptance

1. TDD paper trail + teeth per finding; the W5.2 same-scan fusion test and
   the W5.4 A/B table are the two load-bearing deliverables.
2. Full suite green under `NAVTRACKER_REQUIRE_FIXTURES=1`; adversarial
   review before handoff (tracker hot path + lifecycle).
3. Write-up `docs/baselines/2026-07-15_fixwave_wave5.md` + eval-log entry.
4. Commit on your branch; do not merge or push master.
