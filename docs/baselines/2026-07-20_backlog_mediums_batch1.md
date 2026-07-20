# Backlog mediums — batch 1 (#26–#38 triage): write-up

**Date:** 2026-07-20
**Branch:** `backlog-mediums-batch1` (off master `d063558`)
**Ticket:** `docs/superpowers/plans/2026-07-15-backlog-mediums-batch1-ticket.md`
**Arbiter GO:** top-5 + both riders; all its-own-cycle flags confirmed (nothing pulled forward).

This batch pulls the first slice of the 24 PLAUSIBLE-OPEN medium findings filed
as improvement-backlog entries #26–#38. Selection ranked by (severity ×
deployable-path relevance × fix-cost); the arbiter approved the top five plus
two cheap riders. **Every landed item is byte-identical on the deployable
config `imm_cv_ct_pmbm_coverage_land_ivgate` for well-formed input** — the fixes
are edge-hardening, fail-loud guards, a determinism guard, a datum-origin
invariant, and a non-tracking collision-output correction. One item (#28) is a
PMBM hot-path toucher and got a dedicated adversarial review.

## Landed items

| # | Item | Commit | Touches tracking path? |
|---|------|--------|------------------------|
| #26 | Finish validate-at-the-edges on input adapters (M17/M16/M29/M18/M22/M20) | `174964f` (+ `a9f151f`) | No (adapters / provider datum-init) |
| #28 | PMBM cross-batch stale-scan / high-water guard | `8dea53a` | Yes (PMBM `processBatch`) — adversarial review |
| #34 M6 | `clutter_intensity>0` fail-loud ctor guard | `85cb0a4` | PMBM ctor (inert on valid) |
| #35 M1 | dimension-aware measurement-covariance guard | `39a520e` | Estimator entry (inert on valid) |
| #29 | CPA advances own-ship from `pose.time` (+ L24 sanity) | `a2b2f9c` | No (read-only CPA evaluator) |
| #36 M23 | Comparator neutral arrows for signed/target metrics | `772f8e7` | No (bench reporting) |
| #37 M25 | `example.cpp` relative-bearing sign doc | `772f8e7` | No (doc/UX) |

## Per-item

### #26 — input-adapter edge validation (HIGH)
- **M17 (hang):** `wrapDegToPi` / `gyroRateRadPerSec` used unbounded `while(rad>π)`
  loops that never terminate on an Inf parsed angle (`Inf-2π == Inf`); a single
  corrupt HDT (~1/256 pass the 8-bit checksum) froze the ingest thread. Guard
  non-finite input; the wrap is now total. *Teeth:* `NonFiniteHdtHeadingDoesNotHangAndIsRejected`
  (RED = the test hung under a 12 s `timeout`, exit 124; GREEN = returns instantly, rejected).
- **M16:** HDT/HDG/RMC heading & COG parsed via bare `strtod` → `""`→0.0 published
  as an authoritative sample, overflow→Inf into the wrap. New `parseFiniteField`
  rejects empty/non-finite required fields (`skippedNonFinite()`). *Teeth:*
  `EmptyHdtHeadingRejectedNotPublishedAsZero`, `EmptyRmcSogRejected`.
- **M29:** `OwnShipProvider::update` anchored the datum from the FIRST pose
  unconditionally → a heading-only sentence before the first GPS fix pinned the
  datum at Null Island (and a stray (0,0) after a real fix tripped the 30 km
  auto-recenter). Fixed **provider-side**: only a real (finite, in-range,
  non-(0,0)) position anchors/recenters; the pose is still stored so the
  documented heading-before-fix contract (multi-heading tests) is preserved.
  *Teeth:* `Hdt/HdgBeforeFirstFixDoesNotAnchorDatumAtNullIsland`,
  `OwnShipProviderTest.{NullIslandPoseStoredButDoesNotAnchorDatum,
  ImplausibleFirstPoseDoesNotAnchorDatum, NullIslandPoseAfterRealFixDoesNotRecenter}`.
- **M18:** `RemoteTrackAdapter` gated only lat/lon; NaN/Inf/non-PSD stated
  covariance and non-finite (accepted) velocity flowed into the Measurement.
  Validate stated covariances (finite+PSD) and velocity at the edge. *Teeth:*
  `RejectsNonFinite/NonPsdStatedCovariance`, `RejectsNonFiniteVelocityWhenAccepted`,
  `ValidCovarianceAndVelocityStillAccepted`.
- **M22:** `loadOwnshipCsv` did no finite/range/null-island check — a blank row
  became a (0,0) pose poisoning every body-frame projection. Mirror the AIS
  loader's inline validation. *Teeth:* `OwnshipCsvReader.SkipsBlankImplausibleAndNullIslandRows`.
- **M20:** `GeoJsonCoastline` leaked a raw `nlohmann::parse_error` and used const
  `operator[]` on a possibly-missing `"coordinates"` key. Mirror the R7.2
  `GeoJsonStaticObstacles` hardening. *Teeth:* `MalformedJsonThrowsRuntimeError`,
  `PolygonMissingCoordinatesKeyIsSkippedNotCrash`, `NonNumericCoordinatesSkippedNotThrow`.

**Follow-up `a9f151f`:** the M22 fix first used `edge::isPlausibleLatLon`, which
pulled `adapters/util/EdgeValidation` into `navtracker_replay`'s symbol set —
fine for `navtracker_tests` (links everything) but an undefined reference for
`navtracker_bench_baseline` (links `navtracker_replay` without that TU). Caught
by building ALL targets. Inlined the check like the sibling AIS loader; behavior
unchanged.

### #28 — PMBM cross-batch stale-scan / high-water guard (MEDIUM→HIGH, extends #1)
`processBatch` sorted within a batch but had no cross-batch reject-stale guard;
an out-of-order batch reached `predict()`, whose `dt≤0` branch **rewinds**
`current_time_` and returns un-propagated, then the update ran against newer
states. Mirrors the `MhtTracker` guard (`Config::reject_stale_measurements`,
default on; `staleDropped()`). **Forced divergence** (condition 1): PMBM
processes an empty scan as a first-class all-miss step, so the guard is scoped
to non-empty batches — documented in `pmbm-design.md §2.3`. *Teeth:*
`PmbmStaleGuard.{RejectsStaleBatchAndDoesNotRewindClock, InOrderBatchesAreNeverDropped,
GuardCanBeDisabledForOrderRobustCallers}`.

### #34 M6 — fail-loud on non-positive clutter_intensity (HIGH-impact)
`clutter_intensity==0` makes an unclaimed measurement's cost column all-`+inf`,
collapsing the whole MBM to empty in one scan (every track lost). Ctor throws
`std::invalid_argument` (mirrors the R9 guard). Default 1e-4 and any positive
value are unchanged. **M3/M5 (Murty ranking) and M4 (JPDA) NOT pulled** —
its-own-cycle (M5/M3 park with the #25 Murty cycle). *Teeth:*
`PmbmConfigGuard.{RejectsZero, RejectsNegative, DefaultAccepted}ClutterIntensity`.

### #35 M1 — dimension-aware measurement-covariance guard (MEDIUM)
`isMeasurementCovariancePsd` never saw the measurement dimension, so a
square-but-wrong-size R passed the PSD check and then mismatched in
`H·P·Hᵀ + z.cov` (Eigen abort / OOB). New `(R, expected_dim)` overload; callers
pass `z.dim()`. **M2 (softUpdate z0.covariance) NOT pulled** — its-own-cycle.
*Teeth:* `MeasurementModels.PsdGuardRejectsWrongSizeRForMeasurement` (unit),
`EkfEstimator.WrongSizeMeasurementCovarianceIsRejectedNotCrash` (behavioral).

### #29 — CPA advances own-ship from pose.time (MEDIUM)
`synthesizeOwnShipTrack` stamped the CPA QUERY time while its state is the raw
fix, so own-ship's extrapolation `dt=0` and it was frozen at the old position —
a `|v_own|·(t_ref−pose.time)` error in `cpa_distance`/`tcpa`/P(below) on stale
GPS. Now stamps the fix time (extrapolates symmetrically with targets; velocity
uncertainty grows through the Jacobian). Folds in L24 (state/cov sanity check in
`CpaEvaluator`). *Teeth:* `SynthesizeOwnShipTrack.StampsFixTimeSoCpaExtrapolatesOwnShip`
(cpa 141.4 m vs the pre-fix 100 m).

### Riders — #36 M23, #37 M25 (LOW)
- **M23:** `Comparator::isLowerBetter` gave signed/target metrics (`card_err_*`,
  `nees_*`, `nis_*`) a false improvement arrow on a mere decrease. New
  `metricDirection` classifier with a Neutral category → non-directional `~`.
  *Teeth (condition 4):* `Comparator.TargetMetricDecreaseIsNotFlaggedAsImprovement`
  (fails on today's arrow logic), `SignedAndTargetMetricsAllGetNeutralIndicator`.
- **M25:** `example.cpp` relative-bearing comment corrected (+0.5 rad is to
  STARBOARD of bow, marine 0=bow / clockwise-positive).

## A/B — deployable config `imm_cv_ct_pmbm_coverage_land_ivgate`

Per the finding rule, any deployable-config row change is a finding. **No
deployable row changed** — the three tracking-path touchers are byte-identical
on the deployable config by construction, and this is mechanized by the
deployable-config-pinning tests in the strict suite (raw numbers below; arrows
are decoration only, per condition 4):

| Tracking-path item | Byte-identical basis (deployable) | Mechanized by |
|--------------------|-----------------------------------|---------------|
| #28 stale guard | in-order deterministic replay never trips the (t_max-keyed) guard | `PmbmScenario.ReplayDeterminism`, `PmbmTrackerUpdate.ReplayDeterministicallyReproducesState`, `PmbmClutterFeed.DeterministicFeed`, all PMBM replay/scenario tests |
| #34 M6 ctor guard | deployable `clutter_intensity` = 1e-4 > 0 (no `=0` config exists in-tree) | ctor unchanged for valid λ_C; full PMBM group green |
| #35 M1 R-dim guard | every model has `value.size()` == required R dim → correct R passes identically | full estimator + PMBM/MHT/Tracker scenario groups green |

Deployable-config gate tests (all green, unchanged assertions):
`Adr0002Promotion` (promotion_latency gate on `imm_cv_ct_pmbm_coverage_land_ivgate`),
`Cl4AisDropoutContinuity`, `VetoIsolationHaxrAB`, `LosGuardHaxrAB`,
`Philos*` OSPA/label replays, `Imazu22ScenarioRun`. A fresh 25-min bench sweep
was **not** re-run: the changes cannot alter deployable-config output (no
out-of-order batches, no `clutter_intensity=0`, no wrong-size R, no NMEA/GeoJSON
in the PMBM bench path, 0/35 ownship.csv rows rejected), and the above tests
pin the deployable numbers and are green.

**Condition-2 report (M18/M22 real-fixture row deltas):** M22 rejected **0 rows
across all 35 `ownship.csv` fixtures** (philos ×7, sim_multisensor ×28). M18: no
fixture/bench path exercises the `RemoteTrackAdapter` live feed. No changed input
row count → no finding.

**Condition-3 report (#29 CPA hysteresis):** `FullStackIntegration.NmeaTargetTrackingBiasAndCpaAllCompose`
green — no Entered/Exited timing shift. Existing `CpaScenario` tests are
byte-identical (they use `pose.time == t_ref` or zero own-ship velocity, so
`dt=0`).

## Verification

- **Full suite (strict, `NAVTRACKER_REQUIRE_FIXTURES=1`):** **1258/1258 passed, 0 failed** (483 s; 1228 baseline + 30 new tests — the two extra vs the first cut are the #28 follow-up's overlapping-`t_max` and same-instant guard tests). **0 skips** — proven, not merely observed: under strict mode the fixture-guard macro (`NAVTRACKER_REQUIRE_FIXTURE_OR_SKIP`) turns a would-be fixture skip into `FAIL()`, and there are no raw `GTEST_SKIP()` calls outside it, so 0 failures ⟹ 0 skips. The heavy fixture tests (Veto/Los/Haxr/Philos/Occupancy/Imazu) ran with real runtimes.
- **Adversarial review (#28 / #34 M6 / #35 M1 hot-path):** #35 M1 and #34 M6 **clean**
  (verified `z.dim()` == R dim for all four measurement models; no in-tree config
  sets `clutter_intensity=0`). #28 — the review **found a real correctness gap**
  in the first cut (`8dea53a`): the front-keyed guard did not prevent the
  `t_max`-based rewind, so an *overlapping* batch (fresh front, stale `t_max`)
  was accepted and rewound the clock with `staleDropped()==0`. **Fixed in
  `00627cc`** — the guard now keys on `t_max` vs `current_time_`; the reviewer's
  exact trigger is pinned by `PmbmStaleGuard.RejectsOverlappingBatchWhoseLatestPrecedesFilterTime`.
  (Not a regression — pre-#28 PMBM rewound in these cases too; but the commit's
  guarantee is now actually delivered.) Re-verified byte-identical on the PMBM +
  determinism groups.
- Build of ALL targets green (caught the `a9f151f` bench-link regression).

## Not pulled (arbiter-confirmed its-own-cycle / deferred)

- **#34 M5 + M3** (Murty cost term + empty-on-infeasible) — change association
  ranking on the deployable adaptive-K PMBM; park with the #25 close-pass Murty
  cycle (next dedicated behavior cycle).
- **#35 M2** (softUpdate z0.covariance) — changes IMM update math on the
  deployable config.
- **#31** (own-ship maneuver-gate envelope) — substantive estimator change;
  standalone cycle.
- **#34 M4** (JPDA global event enumeration) — non-deployable associator.
- **#27** (lifecycle Coasting→Confirmed restore) — GNN/MHT lifecycle behavior change.
- **#30 / #32 / #33** (ClutterMap datum sink / coastline ring-closure / AIS
  knots→m/s) — lower deployable relevance; available as future batches.
- **#36 M24, #37 M8** — bench-truth / doc-diagnostic; deferred.
