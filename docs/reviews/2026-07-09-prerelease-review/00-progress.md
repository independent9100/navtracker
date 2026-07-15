# Pre-release deep review — 2026-07-09 — progress/state

Purpose: in-depth pre-release review of navtracker @ master 317ecfd.
Requested checks: (1) test sufficiency, (2) test-data correctness, (3) deep bug
hunt, (4) reviewer-chosen release-readiness checks. Findings collected
incrementally here so nothing is lost if the session dies mid-way.

Final deliverable: `docs/reviews/2026-07-09-prerelease-open-points.md`

## Workstreams and status

| # | Workstream | Status | State file |
|---|-----------|--------|------------|
| W1 | Build + full suite + skip-list diff + ASan/UBSan (isolated worktree `../navtracker-review-build`, fixtures symlinked) | IN PROGRESS | 50-build-suite-report.md |
| W2 | Bug hunt: 15 module reviewers + 5 cross-cutting lenses (determinism, time/dt, angles/units/frames, numerics, memory/UB), each finding adversarially verified | IN PROGRESS | 10-bughunt-findings.md |
| W3 | Test-sufficiency audit (per-module gaps, assertion quality, CMake registration, fixture-skip audit, required-scenario coverage) | IN PROGRESS | 20-test-sufficiency.md |
| W4 | Test-data audit (fixtures + data/: licensing/provenance, units vs loaders, timestamp monotonicity, truth alignment; sim generators) | IN PROGRESS | 30-test-data.md |
| W5 | Release-readiness sweep (integration-guide accuracy, output-contract vs code, TODO/FIXME triage, licensing summary) | PENDING (after W1-W4) | 40-release-readiness.md |
| W6 | Completeness critic + synthesis → open-points doc | DONE | ../2026-07-09-prerelease-open-points.md |

## REVIEW COMPLETE 2026-07-11
All 5 workstreams done. Final deliverable:
`docs/reviews/2026-07-09-prerelease-open-points.md`. W5 readiness run inline
by orchestrator (Opus) after Fable quota exhausted; W2 bug hunt ran full-depth
on Fable 5 with adversarial verification. Verdict: NOT ready — 1 critical, ~13
confirmed high, 1 licensing blocker, coverage holes coincident with the bugs.

## EVENT 2026-07-09 ~19:10 — session usage limit hit

All 48 subagents across W2/W3/W4/W5 failed with "You've hit your session
limit - resets 9:30pm (Europe/Berlin)". No agent returned results (journals
empty of result lines). W1 (local build) unaffected and produced findings
F-BUILD-1..3 (see 50-build-suite-report.md).

RESUME PLAN — superseded by BATCH MODE (user directive ~19:30): run small
batches sequentially, wait for each, save results, then next. Subagent
capacity probe returned alive at ~19:25.

## Batch mode protocol

Bug-hunt batch script:
`~/.claude-work/projects/-home-andreas-workspace-navtracker/662c12f1-3115-4df7-b3b5-4774657269c8/workflows/scripts/prerelease-bughunt-batch-wf_d7e733d2-3bd.js`
— invoke with `args = [unit keys]`. Only high/critical findings get one
skeptic verifier (effort high). Batches:

- B1 DONE (wf_b60d9329-dd6): estimation-* → 13 findings (3 HIGH CONFIRMED:
  RangeBearing2D initiate-as-Cartesian in EKF/UKF/IMM/PF; UKF bearing mean
  across ±π wrap). Saved in 10-bughunt-findings.md + 10-bughunt-raw/B1-….json.
- B2 DONE (wf_7f418447-9ee, attempt 2): association, pmbm, pipeline → 15
  findings (CONFIRMED: Tracker.cpp:144 soft-assoc drops updates HIGH;
  PmbmTracker:1666 spurious SourceTouch HIGH; PmbmTracker:964 double-propagate
  MED; MhtTracker:531 leaf protection inert MED).
- B3 finders DONE (wf_d5cdecce-6a9): tracking-types, geo-ownship, bias → 21
  findings; 5 verifiers died on limit (reset 11:40). Orchestrator personally
  confirmed AxisRotation.cpp:17 SIGN FLIP (numeric ECEF check, see findings
  file); geo finder separately CONFIRMED antimeridian wrap bug same function.
  B3 verify resume DONE 12:05: bias trio ALL CONFIRMED HIGH (closed-loop
  double-subtraction converges to half-bias; per-sensor loop same shape;
  v1 pair extractor bearings about datum origin not own-ship). Antimeridian
  wrap CONFIRMED HIGH. AxisRotation sign + Tentative->Coasting CONFIRMED,
  downgraded to MEDIUM (bounded/self-correcting; still test-enshrined).
- B4 RUNNING 12:06 (wf_9c0b3794-813): collision-output, static-land,
  adapters-sensor.
- B4 DONE (wf_9c0b3794-813): 19 findings, 7 CONFIRMED incl. CRITICAL
  OwnShipNmeaAdapter GGA no-validation (no-fix -> (0,0) pose/datum), HIGH
  TrackOutput NED-vs-(east,north) contract mismatch (Foxglove adapter already
  bitten), HIGH Cpa.cpp:135 tcpa-Jacobian sign error (spec has same error),
  HIGH adapters hold stale Datum copy across recenter, HIGH×2
  DeclaredSensorActivity (coverage from origin not own-ship; identity-blind
  retirement).
- B5 DONE (wf_d9b355ca-d14): 20 findings (external CSV schema paths: knots-as-
  m/s, DMA timestamps unparseable, ownship CSV unvalidated; tgospa_smooth
  degenerate CONFIRMED; example.cpp bearing sign doc).
- B6 DONE (wf_431d712a-214, resumed): BUG HUNT COMPLETE (20/20 units).
  Lenses cross-confirmed PmbmTracker:1666/:964, TrackOutput ordering,
  RangeBearing2D init, heading-bias sign inconsistency now CONFIRMED HIGH.
  New: Tracker::processBatch missing backlog-#15 sort (MED); PMBM no
  stale-scan guard (MED). Determinism lens: extensive CLEAN bill (RNG, sorts,
  container iteration, no wall-clock in state paths).
- MODEL SWITCH 2026-07-11: Fable 5 HARD quota reached (not rolling limit).
  User switched session to Opus 4.8 (1M ctx). Bug hunt (all 20 units) was
  completed on Fable 5. Remaining audits (W3/W4/W5) now run on Opus 4.8 —
  acceptable per plan (mechanical audits, effort high). Agents inherit
  session model.
- W3 test-sufficiency DONE on Opus (T1 wf_1f2a81ec-69e, T2 wf_380bd5fd-2c4,
  T3 wf_dac9d37e-408): 42 findings saved to 20-test-sufficiency-findings.md.
  Highlights: HIGH same-scan 2-sensor fusion untested (=PMBM bug path);
  HIGH AIS sentinels never fed to adapter; HIGH ISO/DMA timestamp parse
  untested; HIGH crossing/overtaking continuity via degenerate metric;
  HIGH no fixture-skip guard; HIGH cwd-gated scenario tests skip under ctest
  (=F-BUILD-3). ID-never-reused invariant unasserted. One HIGH REFUTED
  (head-on IS covered by test_crossing anti-parallel geometry).
- W4 test-data DONE on Opus (D1 wf_314ab709-a16, D2 wf_1c2a830e-fa9,
  D3 wf_36ba18a6-4b7): 38 findings -> 30-test-data-findings.md. HEADLINE:
  THREE non-commercial datasets present — philos CC-BY-NC-SA (HIGH, derivatives
  COMMITTED to git, blocker), pohang CC-BY-NC (HIGH, 683MB on disk), SMD no
  license at all (HIGH, but untracked+unused). autoferry CLEAN (CC-BY-4.0,
  fully validated). Baselines lack provenance headers; ~2.3GB staged data
  unused. HAXR provenance REFUTED (it is recorded).
- W5 R1 RUNNING on Opus (wf_3a37b765-12b): integration-guide, output-contract,
  learning-docs, todo-triage.
- Queued: R2 [api-surface,adr-consistency,readme-claude,repo-hygiene].
  Then synthesis -> docs/reviews/2026-07-09-prerelease-open-points.md.

USER-APPROVED TRIMMED PLAN (2026-07-10 ~17:10) — after B5:
- B6 (last bug-hunt batch): lens-determinism, lens-time-dt, lens-angles-units
  (numerics + memory-ub lenses DROPPED — ASan/UBSan clean incl. 36 real-data
  replays; module reviewers covered numerics).
- W3 test-sufficiency at effort=high, 3 batches:
  T1 [estimation, association-pmbm, pipeline-tracking, geo-ownship-bias]
  T2 [collision-output-static, adapters, scenario-bench-sim]
  T3 [required-scenarios, assertion-quality, registration-fixtures]
- W4 test-data at effort=high, 3 batches:
  D1 [licensing-provenance, philos, haxr]
  D2 [autoferry, pohang, rbad, sim-multisensor]
  D3 [geo-misc, sim-emitters, baselines]
- W5 readiness at effort=high, 2 batches:
  R1 [integration-guide, output-contract, learning-docs, todo-triage]
  R2 [api-surface, adr-consistency, readme-claude, repo-hygiene]
- Scripts already patched for args-filtering + effort high (same paths as
  original W3/W4/W5 scripts above). Then synthesis by orchestrator.
- Limit economics: ~1.5 batches per 5h window at xhigh (B1 750k, B2 854k,
  B3 500k+resume). Plan around ~11:40/16:40/21:40 window boundaries.
- B3: tracking-types, geo-ownship, bias
- B4: collision-output, static-land, adapters-sensor
- B5: adapters-replay, scenario-metrics, periphery
- B6: lens-determinism, lens-time-dt, lens-angles-units
- B7: lens-numerics, lens-memory-ub

Then W3 test-sufficiency (10 units, 3 batches), W4 test-data (10 units,
3 batches), W5 readiness (8 units, 2 batches) — same batching approach,
reusing their original scripts converted to args-filtered form.
Results appended per batch to 10-bughunt-findings.md / 20-… / 30-… / 40-….

Inline observations queued for the relevant unit finders:
- TrackOutput NED-vs-ENU covariance ordering question → collision-output
  unit hints (datumAxisRotation applies no axis swap; header claims NED).

## Background run IDs (this session)

- W1 build+suite+ASan: bash task bznype25f, logs in
  `/home/andreas/workspace/navtracker-review-build/review-logs/`
- W2 bug hunt: workflow wf_87caaa25-3bf
- W3 test sufficiency: workflow wf_e5ad4f4a-9ae
- W4 test data: workflow wf_76cd1cdf-55f
- W5 release readiness: workflow wf_1ed537ac-2b3
- Workflow journals: `~/.claude-work/projects/-home-andreas-workspace-navtracker/662c12f1-3115-4df7-b3b5-4774657269c8/subagents/workflows/<run-id>/journal.jsonl`

## Notes for a resuming session

- Build worktree: `/home/andreas/workspace/navtracker-review-build` (detached
  @ master 317ecfd, `tests/fixtures` and `data` symlinked to main tree).
  Remove with `git worktree remove --force` when done.
- Do NOT build in the main tree (other sessions active — see CLAUDE.md).
- Known/by-design items excluded from findings: λ_C cancellation in PMBM birth
  existence (pmbm-design.md §3.2.2), ADR-0001 offshore no-birth zone, backlog
  #25 PMBM CPA track loss, backlog #11 MHT Imazu churn, extraction-is-upstream
  ruling, RTS anchored-mode regression (parked), Sweep's fixed-datum design.
