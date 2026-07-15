# Pre-release fix wave — wave 5 (2026-07-15)

Branch `fixwave-wave5`, originally off post-merge master `068a30f` (waves 3+4 merged,
ADR-0003 reconciliation addendum in), **rebased onto master `3a5fb63`** after the F2
provenance cycle merged (`6fcd44e`) — see Handoff. Origin: the pre-release review synthesis
(`docs/reviews/2026-07-09-prerelease-open-points.md` §B Themes 5+6, §C tail, §D),
ticket `docs/superpowers/plans/2026-07-15-fixwave-wave5-ticket.md`. TDD + #24 teeth
throughout; suite run under `NAVTRACKER_REQUIRE_FIXTURES=1` (the ceremony standard).
`fixwave-wave1` (held F2) untouched. Commits separated per finding.

Wave 5 is the pipeline/lifecycle tail + the Section-D test backfill: the last
confirmed code defects of the fix wave, each paired with the coverage hole it
lived in.

## Findings (commit order)

### W5.6.5 — VetoIsolationHaxrAB ctest timeout (`97efbe9`)
`gtest_discover_tests` global `TIMEOUT 300 -> 600`. The three-site
`VetoIsolationHaxrAB` veto A/B runs ~290 s standalone (measured **290.96 s** on
the baseline run) — a knife-edge under a 300 s cap that starved past it under
parallel `-j` load (wave-3 hand-back). Comment updated; config-lint teeth
(parsed TIMEOUT ≥ 600). Pure housekeeping, zero behavior change.

### W5.5 — MHT deferred-commitment leaf protection (`fb21eb6`, MED)
**Finding.** `TrackTree` leaf-protection flags were always one `branch()` behind:
the global solve sets `is_protected` on the top-K leaves at scan end, but the next
scan's `branch()` (first step) demotes every leaf to internal and makes fresh
children with `is_protected=false`, never inheriting — so `pruneKLocal` /
`mergeBranches` / `pruneNScan` never saw a protected leaf. Deferred-commitment
TOMHT was fully inert.

**Fix.** `branch()` propagates `is_protected` to both child kinds (miss + hit);
self-limits to one scan (MhtTracker clears+resets each solve). New teeth test
`TrackTree.ProtectedAlternativeSurvivesPruneAcrossBranch` runs the real order
(`branch()` THEN `pruneKLocal(1)`) — RED pre-fix (protected alternative left
leafless), GREEN after.

**Regression fix (found by making protection work).** The whole-tree
below-threshold delete-sweep's `any_protected` grace would then keep a starving
tree alive forever (its sole miss-leaf is re-selected as its own K=1 best and
re-protected every scan) — `SustainedMissesDeleteTrack` broke. Removed that
grace: a tree flagged `score_dead`/`existence_dead` has its BEST (max-score /
max-existence) leaf below threshold, so every leaf — including any protected
alternative — is below threshold; there is nothing viable to keep. Whole-tree
deletion is now **byte-identical to the (inert-protection) baseline**;
deferred-commitment leaf protection applies only WITHIN surviving trees.

**Cross-lane T2T gate recalibration — ARBITER / F2-implementer please review.**
The T2T scenario arms run an MHT tracker (`T2tViewHarness::runArm`), so activating
deferred-commitment reduced per-arm churn (dropout scenario: inits 10→8, deletes
7→5, distinct fused ids 8→7, dropout-survivors 1→2 — a continuity improvement) and
shifted two PRELIMINARY directional gates. Both directions SURVIVE (per the
arbiter's riders); recalibrated:
- `DropoutContinuityAndLatencySkew` (a): **shape fix**. The old aggregate
  mean-cov-trace over ALL tracks was churn-confounded (in-drop 196514 = diverging
  duplicate tracks; W5.5's churn reduction collapsed it to ~510, inverting the
  aggregate). Replaced with a geometry-controlled per-track metric — the surviving
  target's cov-trace in adjacent windows straddling the dropout onset (both
  sensors → radar-only, near-identical range). Measured inflation **2.07×** on the
  fixed MHT; floor 1.5 (still fails if inflation stops). Robust to future MHT
  churn changes.
- `PerArmNeesCalibrationAndBandViolationGate` (a): threshold **1.4 → 1.25**.
  Reduced churn tightened the CI-fused NEES mean 2.08→2.40, moving the
  (test-acknowledged fragile, tail-driven) naive/CI mean-ratio 1.52→1.32. The
  robust load-bearing gates — naive breaches `band_hi`, CI covers truth better
  with margin — both still hold with headroom.

**Frozen Cl-4 untouched** — the deployable is PMBM
(`imm_cv_ct_pmbm_coverage_land_ivgate`); W5.5 touches only MHT + the MHT-fed T2T
arms. MHT is the champion comparator, not the deployable.

### W5.4 — Tentative→Coasting on miss (`684ae8a`, MED, operator-facing)
**Finding.** `TrackManager::recordMiss` unconditionally set `Coasting` on any
sub-delete-threshold miss, including a never-confirmed Tentative one-hit blip.
Coasting is CPA-eligible (`CpaEvaluator` gates on `evaluate_tentative ||
Confirmed || Coasting`, default false) while Tentative is not → false
collision-risk events; also violates the documented Coasting definition
("was Confirmed").

**Fix.** Guard the demotion on prior status — only an already-Confirmed (or
still-Coasting) track Coasts on miss; a Tentative track keeps its status and ages
toward deletion per M-of-N. Delete timing and counter resets unchanged.

**Tests.** `TentativeMissStaysTentativeNeverCoasting` (unit teeth, RED pre-fix);
`CpaEvaluator.NoFalseRiskFromNeverConfirmedTentativeBlipThatMissed` (operational
A/B teeth).

**A/B (rider-required).** Behavior change confined to the classic Tracker +
TrackManager + CpaEvaluator pipeline (PMBM/MHT set status independently, never
call `recordMiss`):
- **CPA event counts (the metric that moves): 2 → 0** false events in the focused
  collision-bearing-blip scenario.
- **Bench id-churn / track-counts / OSPA / GOSPA: BYTE-IDENTICAL.** `BenchRunner`
  scores only Confirmed tracks (`BenchRunner.cpp:107/187/252`); W5.4 changes only
  the Tentative↔Coasting label of never-Confirmed tracks. Confirmed empirically:
  `ekf_cv_gnn` × 26 sim scenarios, 1675 rows, **0 substantive diff** pre/post
  (only wall-clock timing rows differ).
- **Cl-4 candidate rows: NONE move** (deployable is PMBM, bypasses `recordMiss`).

### W5.2 — PMBM mixed-timestamp double-propagate (`86b03ab`, MED, Cl-4-relevant path)
**Finding.** `PmbmTracker` enumerateChildren stamped a detected Bernoulli's
`last_update = scan[l].time`. `processBatch` predicts every component to `t_max`
before enumeration and the EKF update does not re-predict, so the child lives at
`t_max`. On a MIXED-timestamp scan `scan[l].time < t_max` rewinds `last_update`
below the state's real time, and the NEXT predict applies F/Q over
`[scan[l].time, t_max]` a SECOND time (double-counted process noise).

**Fix.** `det.last_update = current_time_` (== `t_max`). Byte-identical on uniform
scans (every bench/test — `HarnessBatched` groups strictly-equal timestamps);
observable only on mixed-timestamp scans, which no bench feeds.

**Birth-path sibling (`074928f`, found by the adversarial review).** The PPP-birth
path (`buildNewTargetCandidates`) had the identical bug: it moment-matches the
newborn's state at `current_time_`/`t_max` but stamped `last_update = scan[l].time`.
The fix is PATH-SPECIFIC — the adaptive-birth path (`initiate(z)`) is genuinely at
`z.time`, so a blanket `current_time_` would be wrong. Carried the state's physical
time on `NewTargetCandidate::state_time` (PPP → `current_time_`; adaptive → `z.time`)
and stamp from it. Teeth: `MixedTimestampPppBirthStampsBernoulliAtPhysicalTime`
(RED 0.0 → GREEN 0.9, proven by stash-out).

**Tests.** `MixedTimestampScanStampsBernoulliAtPhysicalTime` (teeth, RED pre-fix:
0.0 vs 0.9); `MixedTimestampPppBirthStampsBernoulliAtPhysicalTime` (birth-path
teeth); `SameScanTwoSensorFusionSingleTargetSinglePropagation` (the Section-D
same-scan fusion test — radar+AIS same timestamp → single propagation, covariance
not double-inflated).

**F2-CYCLE COORDINATION (arbiter, please route).** Textually isolated (line 964 vs
F2's SourceTouch walk ~:1666, no merge conflict), but a FUNCTIONAL coupling: the F2
walk matches a Bernoulli's claimed measurement via `z.time == b.last_update`
(~:1689). My re-stamp makes `b.last_update == t_max`, so on a mixed-timestamp scan
that keying no longer matches the claimed (earlier) measurement. The robust key
already exists — `det.last_claimed_meas_index` (set at :965) — and the F2 walk
should switch to it. Uniform scans (all benches/tests) unaffected either way.

### W5.3 — plain Tracker::processBatch batch sort (`e8d99af`, MED)
**Finding.** MHT/PMBM got the backlog-#15 batch sort; the plain single-hypothesis
`Tracker::processBatch` never did. It took the scan instant from raw arrival-order
`front()`, so an unsorted fixed-rate batch was processed at the wrong instant and,
with the stale guard on, a front older than the high-water mark stale-dropped the
whole batch.

**Fix.** The identical `is_sorted`/`stable_sort` preamble MHT/PMBM use — bind
`scan_in` to a time-ordered view. Bench-inert (`is_sorted` fast-path → already-
sorted input bit-identical; every bench/replay feeds sorted scans).

**Test.** `PlainTrackerUnsortedBatchMatchesSortedNoStaleDrop` — sorted vs reversed
batches must match, no fresh tail dropped. RED pre-fix (order-dependent).

### W5.1 — Tracker::process() drops soft (JPDA) association (`9cf83d0`, HIGH)
**Finding.** The single-measurement `process()` consumed only
`AssociationResult::matches` (hard path). A soft associator (JPDA) fills
`betas`/`beta_0`, never `matches`, so with JPDA wired every fix that gated to an
existing track left it un-updated and spawned a DUPLICATE track — unbounded churn,
zero fusion, silently.

**Fix.** `process(z)` delegates to `processBatch({z})` (which dispatches on both
hard and soft branches), deleting ~70 lines of duplicated hard-path logic. For a
one-element batch this is behaviour-identical to the old hard path (sort no-op,
`t == z.time`, identical stale-guard/predict/associate/initiate/stale-timeout and
innovation + provenance emission) — all existing `test_tracker*` + innovation-emit
suites green — while the soft path now works.

**Test.** `TrackerProcessJpda.SingleMeasurementSoftUpdatesInsteadOfDuplicating` —
JPDA driven through `process()`; the second fix soft-updates track 1 (size stays
1). RED pre-fix (size 2, original frozen).

### W5.6.1 — AIS SOG impossible-band gate (`64f3fe0`, Section-D)
The literal 1023/3600 sentinels were already guarded (regression-pinned). The live
defect: the SOG gate `< 1023.0` admitted the physically-impossible band
`(102.2, 1023)` kn — a raw/mis-scaled field became a ~100 m/s velocity. Fixed:
`<= kAisMaxValidSogKnots` (102.2). COG gate already tight, unchanged. Teeth:
SOG=200 kn → Position2D (RED pre-fix: PositionVelocity2D).

### W5.6.2 — loadAisCsv DMA date parsing (`4c8adae`, Section-D)
ISO-8601 / BaseDateTime already parse (regression-pinned). The genuinely-broken
ADVERTISED format is DMA (`dd/mm/yyyy`): `detectColumns` recognizes the DMA
headers but `parseTimeString` only handled year-month-day, so DMA rows were
silently dropped (loader returned empty). Fixed by adding a `dd/mm/yyyy` branch
(FIX not gate — the format is advertised). Teeth: DMA rows load 2 measurements at
the correct UTC times (RED pre-fix: 0 rows).

### W5.6.3 + W5.6.4 — ID-never-reused + e2e determinism (`df83762`, Section-D)
Two pure test additions (invariants hold today; convert to executing guards).
- `TrackIdsAreNeverReusedAcrossDeleteCreateCycles` pins architecture invariant #5
  via a delete→create cycle.
- `EndToEndReplayThroughAdapterIsByteIdentical` pins CLAUDE.md invariant #4
  through the REAL adapter: loadAisCsv → Tracker → `toTrackOutputENU` twice,
  byte-identical at `setprecision(17)`, with a non-vacuity guard.

## Verification

- **Full suite under `NAVTRACKER_REQUIRE_FIXTURES=1 ctest --test-dir build -j8`
  (post-rebase onto master `3a5fb63`):** **1222/1222 pass, 0 failed, 0 timeouts.**
  Strict mode ⇒ **0 fixture skips**. That is +15 over master's 1207 — the wave-5
  backfill plus the new mixed-timestamp provenance composition test. (Pre-rebase this
  branch measured **1218/1218** off `068a30f`; the rebase onto master picks up the F2
  cycle's tests and adds the composition pin.) The composition holds across the whole
  suite: F2's `MisdetectedTrackDoesNotInheritForeignSource` / `T2tLivePedigreeContent`
  / `Cl4AisDropoutContinuity` are green alongside the wave-5 tests. W5.6.5 validated:
  `VetoIsolationHaxrAB` took **398 s** under `-j8` load on the pre-rebase run (would
  have TIMED OUT under the old 300 s cap; the 600 s bump was load-bearing, not
  precautionary) — this post-rebase run it came in at **323 s**.
- **Teeth proven for every code fix** (RED-before-green above); the two pure Section-D
  pins carry non-vacuity guards.
- **A/B** on W5.4 (above): CPA events 2→0, bench byte-identical, no Cl-4 move.
- **Adversarial review** (11-skeptic panel across every finding — 2 skeptics each on
  the HIGH W5.1 and the riskiest W5.5 — + a completeness critic hunting the
  sibling-miss class, then an independent verify pass on every claimed defect). 6
  claims raised; **3 confirmed, 3 refuted**:
  - CONFIRMED → FIXED IN-WAVE: **W5.2 PPP-birth-path** double-propagate sibling
    (`074928f`, above) — a genuine structural sibling in W5.2's own territory.
  - CONFIRMED → **RESOLVED BY REBASE** (was routed to the arbiter): the **W5.2
    SourceTouch provenance walk** coupling. On this branch's original base the walk
    keyed on `z.time == b.last_update`, wrong on a mixed-timestamp scan post-W5.2.
    The F2 cycle (merged to master as `6fcd44e`) re-keyed the walk onto
    `Bernoulli::last_claimed_meas_index` — independent of `last_update` — so after
    rebasing this branch onto master `3a5fb63` the coupling is gone and the two
    fixes are orthogonal. Pinned by the new composition test
    `MixedTimestampMultiSensorAttributesEachClaimedSource` (teeth: reverting to the
    old key → RED). See Handoff.
  - CONFIRMED → docs updated: algorithm docs for W5.2 (`pmbm-design.md` §2.2, incl.
    the uniform-scan provenance invariant) and W5.4 (`pipeline.md` lifecycle).
  - REFUTED: W5.5 per-track dropout metric "cherry-picks a spurious track"
    (`sim_ms_headon` has exactly two genuine targets — the dropout survivors ARE
    them, no spurious survivor to pick); and the compound docs-LOW (backlog #15 is
    already marked DONE).

## Handoff

- All merge-ready on `fixwave-wave5`; commits per finding; **rebased onto master
  `3a5fb63`** (after the F2 cycle merged at `6fcd44e`); not merged/pushed.
  `fixwave-wave1` untouched. The rebase touched three files (`CMakeLists.txt`,
  `PmbmTracker.cpp`, `evaluation-log.md`); only `evaluation-log.md` conflicted (both
  sides appended a dated entry — kept both). `PmbmTracker.cpp` auto-merged cleanly
  (W5.2's stamp at `:986/:1197` is disjoint from F2's walk at `:1719`).
- **Composition verified.** W5.2's physical-time stamping and F2's re-keyed
  SourceTouch walk are orthogonal (F2 keys on `last_claimed_meas_index`, not
  `last_update`). The stale in-code F2-coordination comment was refreshed, and the
  positive mixed-timestamp attribution case F2's own uniform-scan regression never
  reached is now pinned by `MixedTimestampMultiSensorAttributesEachClaimedSource`
  (teeth: old-key revert → RED on both discriminating assertions).
- **One item remains for the arbiter:**
  1. **W5.5 cross-lane T2T gate recalibration** — the F2-implementer / T2T-gate
     author sanity-checks the shape fix (geometry-controlled per-track dropout
     inflation **2.07×**) + the **1.4→1.25** threshold at merge review, per the
     recalibrate-vs-defer riders (direction survived; documented inline).
  - *(The former item 1 — the W5.2↔F2 SourceTouch coupling — is resolved by this
    rebase; see the review section. No arbiter decision needed.)*
- **No Cl-4 gauntlet headline row moves** (deployable is PMBM; W5.5/W5.4 touch
  MHT + classic-Tracker paths, W5.2 is uniform-scan byte-identical).
- Findings-file marks deferred to the arbiter (`10-bughunt-findings.md` untracked).
