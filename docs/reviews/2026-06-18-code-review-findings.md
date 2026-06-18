# navtracker code review — findings

Date: 2026-06-18
Reviewer: Claude (Opus 4.8), file-by-file pass
Scope: full `core/`, `ports/`, `adapters/`, `app/` source tree.

Severity legend: **[BUG]** correctness defect · **[GAP]** missing/incomplete
behavior · **[MINOR]** consistency/clarity · **[PERF]** scaling ·
**[OK]** notable correct-and-good.

---

## Status update (2026-06-18)

**Fixed: ranks 1–3 (#1, #2, #3).** Implemented + unit-tested
(`tests/pipeline/test_review_fixes.cpp`); full suite 641/641 green.
Verified **bench-neutral**: a before/after focused sweep (9 MHT configs ×
10 synthetic scenarios × 2 seeds) is bit-for-bit identical on every metric
row — the fixes add identity/provenance, the MHT datum hook, and the
single-hypothesis prune without touching tracker kinematics or lifecycle.
- #1 → `MhtTracker` now accumulates per-tree `attributes.mmsi` +
  `contributing_sources` (birth + committed hits) and copies them into the
  emitted Track view; pruned when a tree dies.
- #2 → `MhtTracker::onDatumRecentered(old,new)` shifts every tree node's
  (state, cov, IMM) via the new `shiftStateOnDatumChange` helper (factored
  out of `shiftTracksOnDatumChange`); wire it to an `IDatumChangeSink`.
- #3 → `Tracker` prunes `recent_contributions` to a 2 s window after each
  append (parity with the MHT path).

**Fixed: rank 2 / #15 (live adapter edge validation).** New shared helper
`adapters/util/EdgeValidation.{hpp,cpp}` (`isPlausibleLatLon`,
`isPlausibleRange`, `isFiniteValue`); `AisAdapter`, `ArpaAdapter`
(TLL lat/lon + TTM range/bearing), and `EoIrAdapter` now reject
sentinel/out-of-range/NaN/non-positive-range input at the edge instead of
emitting phantom measurements. Unit-tested
(`tests/adapters/util/test_edge_validation.cpp` + rejection cases in each
adapter test); full suite 651/651 green. Verified **bench-neutral**: a
before/after sweep (17 configs × 10 synthetic scenarios × 5 seeds, 28166
rows) is bit-for-bit identical — guards fire only on invalid input, which
the clean synthetic/replay data never produces.

**Fixed: correctness batch — #4, #11, #12; #18 verified.** Unit-tested
(`test_jpda_contributing_sources`, `test_bias_correction_orthogonality`,
+ PF/SigmaPoints cases); full suite 658/658 green. Verified **bench-neutral**:
before/after sweep (17 configs × 10 scenarios × 5 seeds, 28166 rows)
bit-for-bit identical.
- #4 → JPDA soft-update path now appends each fusing source to
  `contributing_sources` (dedup), parity with the hard/GNN path.
- #11 → `ParticleFilterEstimator` sizes its ensemble from
  `motion_->stateDim()` (no more 4-state hard-wire / dimension crash) and
  `predict` uses the model's per-particle `propagate(x, dt)` so a CT model
  actually turns each particle instead of silently collapsing to CV. New
  `init_omega_std` ctor arg (appended; defaulted) seeds the ω state.
- #12 → `computeSigmaPoints` checks `llt.info()`; on a non-PD `scale·P` it
  falls back to an eigen PSD square root (negative eigenvalues clamped),
  keeping the UKF/IMM step finite instead of NaN-poisoned.
- #18 → **verified, not a defect.** Heading bias (angular, platform-global,
  removed upstream in the adapter via `IHeadingBiasProvider`) and per-sensor
  bias (translational/bearing, via `ISensorBiasProvider` in the Tracker) are
  applied by disjoint mechanisms on disjoint quantities — no code path
  double-applies. Locked by `test_bias_correction_orthogonality`. System-level
  separability of a far-target heading offset vs a per-sensor position offset
  relies on multi-range geometry, which is the documented design
  (`SensorBiasPairExtractor.hpp`), not a bug.

**Fixed: #17 (rank 6) — optimal metric assignment.** `ospaGreedy`,
`gospaGreedy`, and `assignPerStep` now use the optimal (min-cost) Hungarian
assignment instead of greedy NN (names kept for call-site stability; docs
updated). OSPA uses the clipped-distance² matrix; GOSPA an augmented
(|X|+|Y|)² matrix with per-target miss/false dummy slots; `assignPerStep`
a gated truth×track matrix. New distinguishing unit tests
(`Ospa.UsesOptimalAssignmentNotGreedy`, `Gospa.UsesOptimalAssignmentNotGreedy`)
on a collinear geometry where the global-min edge is *not* in the optimal
matching. Full suite 660/660.
⚠ **Intentional re-baseline (NOT bench-neutral, by design).** Before/after
sweep (17×10×5, 28166 rows): **226 rows differ (~0.8 %)**, all
assignment-dependent metrics (ospa/gospa, id_switches, pos/sog/cog rmse,
nees) and only in close-crossing geometry (head-on). `id_switches` moves
**both** directions — greedy both over-counted (spurious churn) and
under-counted (real ID swaps it masked by locking a stale pairing); optimal
is the correct metric in each case. The dated baselines in `docs/baselines/`
are frozen historical decision records and are intentionally left as-is;
cross-comparing id_switches/OSPA across the #17 boundary is not
apples-to-apples (see eval-log entry 2026-06-18).

## Ranked summary

Ranked by impact × likelihood × blast radius. "Always-on" = triggers on
normal input; "conditional" = needs a specific trigger (bad input, large
move, specific config).

| Rank | # | Finding | Sev | Trigger | Why here |
|------|---|---------|-----|---------|----------|
| 1 | 1 | MHT drops track identity (MMSI/name/dims) ✅ FIXED | BUG/GAP | always-on | Output contract violated on **every** track in the canonical pipeline |
| 2 | 15 | Live adapters skip edge validation ✅ FIXED | BUG/GAP | bad input | One AIS 91°/181° sentinel or NaN → phantom/garbage track; safety-relevant |
| 3 | 2 | MHT has no datum-recenter shift ✅ FIXED | BUG | >30 km move | Internal `trees_` desync from measurements; tracks jump. Severe but conditional |
| 4 | 3 | `Tracker.cpp` never prunes `recent_contributions` ✅ FIXED | BUG | time | Unbounded memory + O(n²) extractor cost on the single-hypothesis path |
| 5 | 11 | PF hard-wired to 4-state ✅ FIXED | BUG | config | Crash / silent CV-only if wired to a 5-state motion model |
| 6 | 17 | Greedy (not optimal) GOSPA/OSPA/id-switch assignment ✅ FIXED (re-baseline) | EVAL | crossing geometry | Confounds estimator A/B decisions you act on |
| 7 | 4 | JPDA soft path omits `contributing_sources` ✅ FIXED | BUG | JPDA mode | Provenance empty under soft association |
| 8 | 18 | Possible heading × per-sensor double-debias ✅ VERIFIED (no bug) | VERIFY | ARPA/EO-IR | Could double-correct one physical offset; needs a targeted test |
| 9 | 12 | SigmaPoints ignores `llt.info()` ✅ FIXED | MINOR | non-PD cov | NaN sigma points poison the whole UKF/IMM step |
| 10 | 9 | `TrackManager::index()` O(n) → O(n²)/cycle | PERF | many tracks | Fine now; bites at scale |
| 11 | 13 | Velocity `is_valid` true whenever `trace>0` | MINOR | always-on | Consumers get a COG from pure init prior |
| 12 | 14 | HeadingBias v1 AIS-ARPA path has no outlier gate | MINOR | bad pair | One bad pair enters bias state unfiltered |
| 13 | 6 | Relative-bearing builder drops heading σ | MINOR | always-on | Under-reports projected covariance |
| 14 | 16 | ArpaAdapter magic numbers bypass config | MINOR | always-on | No per-deployment noise tuning without recompile |
| 15 | 8 | Cross-sensor extractor wasteful size gate | MINOR | rare | Harmless wasted work (TTM+TLL same source_id) |
| 16 | 5 | Hard-batch path omits `covariance_is_default` | MINOR | batch mode | Diagnostic flag wrong in one path |
| 17 | 7 | `Cpa.cpp` bare `1e-12` not `kEpsDv2` | MINOR | cosmetic | Constant-naming consistency |
| — | 10 | Hungarian BIG_M re-validation | RESOLVED | — | Murty already re-validates; non-issue |

Suggested cut lines: **P0 = ranks 1–2**, **P1 = ranks 3–6**,
**P2 = ranks 7–10**, **P3 = ranks 11–17**.

---

## Part 1 — core hot path, bias subsystem, estimation, math

Deep-read: `types/*`, `pipeline/Tracker.cpp`, `pipeline/MhtTracker.cpp`
(key sections), full `bias/`, `MeasurementModels`, `geo/Wgs84`,
`collision/Cpa`, `tracking/TrackManager`, `association/Hungarian` +
`JpdaAssociator` (setup), `estimation/Resampling` + `ImmEstimator` +
`EkfEstimator` (parts), `MeasurementBuilders`, `util/Nmea`,
`OwnShipProvider`, `tracking/DatumShift`.

### [BUG] 1. MHT pipeline drops track identity attributes (MMSI/name/dimensions)
`TrackTreeNode` has no `attributes` field, and the `view` Track rebuilt
each scan (`MhtTracker.cpp:653-671`) never sets `attributes` or
`contributing_sources`. `estimator.initiate()` sets `t.attributes.mmsi`,
but MHT only copies the node's state/cov/imm — attributes are discarded.
Net effect: in the canonical MHT path, `TrackOutput.attributes.mmsi` is
always empty and `contributing_sources` is empty, despite the output
contract advertising both. Largest functional gap found.

### [BUG] 2. MHT pipeline has no datum-recenter handling
Zero datum references in `MhtTracker.{cpp,hpp}`. The documented auto-datum
pattern wires `shiftTracksOnDatumChange(TrackManager&, …)`, but MHT keeps
authoritative state in its own `trees_`, not a `TrackManager`. If
auto-recenter fires (own-ship > 30 km) during an MHT run, internal tree
states stay in the old ENU frame while new measurements arrive in the new
frame → tracks jump by the recenter distance. Violates invariant #4 / the
library contract for long-running MHT deployments. Benchmarks dodge it via
an explicit datum over short scenarios.

### [BUG] 3. `Tracker.cpp` (single-hypothesis) never prunes `recent_contributions`
`MhtTracker.cpp:595-601` prunes `contribution_history_` to a window;
`Tracker.cpp` only `push_back`s (140, 238, 262) and nothing clears it
(no `clear/erase/resize` anywhere). Long-mission unbounded memory growth,
and bias extractors re-scan an ever-growing vector each cycle → O(n²) CPU.
The `SourceTouch` comment claims "consumers are responsible for clearing,"
but no consumer does.

### [BUG] 4. JPDA soft-update path doesn't populate `contributing_sources` — ✅ FIXED 2026-06-18
In `Tracker::processBatch`, the hard-match branch updates
`tr.contributing_sources` (264-268) but the `soft` branch (204-244) does
not. Under JPDA the provenance list is never filled even in the
single-hypothesis pipeline.

### [MINOR] 5. Hard-batch path omits `covariance_is_default`
`Tracker.cpp:253-263` omits `touch.covariance_is_default = z.covariance_is_default;`
which both `process()` (139) and the soft path (237) set.

### [MINOR] 6. Relative-bearing builder drops heading uncertainty
`MeasurementBuilders` passes `sigma_heading_rad = 0.0` (line 63), zeroing
the random heading component of the projected position covariance.
Defensible if owned by `HeadingBiasEstimator`, but should be a real value
or an explicit comment.

### [MINOR] 7. `Cpa.cpp:37` uses bare `1e-12` instead of named `kEpsDv2`
The constant is defined (line 15) and used in `computeCpaWithUncertainty`
but not in `computeCpa`.

### [MINOR] 8. Cross-sensor extractor wasteful size gate
A track whose only two keys share a `source_id` (ARPA TTM+TLL) passes the
`latest.size() < 2` gate (line 176) but emits nothing (both rejected by
the same-`source_id` guard, line 215). Tighten gate to "≥2 distinct
source_ids."

### [PERF] 9. `TrackManager::index()` O(n) scan → O(n²) per cycle
Linear scan called from `recordHit/recordMiss/noteObservation/recordUpdated`,
plus the stale sweep calls `lastObservation(id)` per track. A
`TrackId→index` map fixes it if track counts grow.

### [PERF/VERIFY] 10. Hungarian BIG_M re-validation contract
`Hungarian.cpp` requires callers to re-validate infeasible (BIG_M)
assignments against the original cost. Confirm both `Murty.cpp` sites
(68, 96) do this. (Pending Murty read — see Part 2.)

### [OK] Notably correct
- `ImmEstimator`: dt-scaled TPM (`transitionFor`/`pi_.pow(dt)`) closes the
  "non-dt-scaled IMM TPM" gap; mixing, Joseph-form covariance, log-sum-exp
  mode update, V·P_D PDAF normalization all correct + well-documented.
- `bias/`: one-update-per-key/cycle de-correlation, same-hardware anchor
  guard, `std::map` determinism, Schmidt fold math all sound.
- `Wgs84` (Bowring), `MeasurementModels` Jacobians, `Cpa` uncertainty
  Jacobians correct.

---

## Part 2 — association + estimation internals

Deep-read: `association/Murty` + `Gating` + `JointEvents`, `estimation/Ukf`
+ `ParticleFilter` + `Ekf` + `CoordinatedTurn` + `BearingRangeGuard` +
`SigmaPoints`, `output/TrackOutput`, `bias/HeadingBiasEstimator`.

### [BUG] 11. ParticleFilterEstimator is hard-wired to a 4-state model — ✅ FIXED 2026-06-18
`initiate()` always builds an `Eigen::Vector4d`/`Matrix4d` ensemble
(lines 114-126), ignoring `motion_->stateDim()` — unlike `UkfEstimator`
which sizes from `stateDim()`. If a PF is constructed with a 5-state motion
model (CoordinatedTurn / ConstantVelocity5State), `predict()` builds a
5×5 `F`/`Q` and multiplies against a 4×N particle matrix → Eigen
dimension-mismatch assert/crash. Additionally `predict()` uses the linear
`motion_->transitionMatrix(dt)` rather than per-particle `propagate()`, so
even with a CT model the PF would only ever apply the CV-limit (no
nonlinear turn propagation). Net: PF is silently CV-only. Document the
constraint or size from `stateDim()` + propagate per particle.

### [MINOR] 12. SigmaPoints doesn't check Cholesky success — ✅ FIXED 2026-06-18
`computeSigmaPoints` ignores `llt.info()`; a non-PD `cov` yields a garbage
`L` and NaN sigma points that propagate through the whole UKF/IMM step.
EKF and IMM guard their determinants; UKF's sigma path does not. Add an
`info() == Success` check with a diagonal fallback.

### [MINOR] 13. Velocity `is_valid` criterion is weak
`TrackOutput.cpp:96` sets velocity valid whenever `v_cov.trace() > 0`,
which is true even for a freshly-initiated track whose velocity is pure
prior (no velocity observation yet). Consumers reading `is_valid` get a
COG derived entirely from the init prior. Consider gating on a velocity
σ threshold or an "observed velocity" flag.

### [MINOR] 14. HeadingBiasEstimator v1 AIS-ARPA path has no outlier gate
`observe(AisArpaPairObservation)` applies the KF update unconditionally
(after the range guard), while the v2 `BearingInnovation` and all v3
scalar paths apply an outlier-sigma rejection. A single bad AIS↔ARPA pair
goes straight into the bias state. Consider routing it through the same
`applyScalarUpdate` outlier gate.

### [OK] Correct
- `Murty`: re-validates BIG_M/∞ assignments against the original cost
  (`feasibleAgainst` + `totalCostAgainstOriginal`) — resolves Part 1 #10.
- `Ukf`/`Ekf` Joseph-form updates, `BearingRangeGuard` LOS-variance
  restoration, `CoordinatedTurn` closed-form F + Q, and `TrackOutput`
  polar (sog/cog) Jacobian all correct.

---

## Part 3 — adapters, metrics, pipeline plumbing

Deep-read: `adapters/ais` + `arpa` + `eoir` + `own_ship/OwnShipNmeaAdapter`,
`pipeline/ReorderBuffer`, `tracking/ClutterMapDetectionModel`,
`scenario/Gospa` + `Ospa`, `benchmark/Metrics`, `collision/CpaEvaluator`,
`projection/Projection`, `geo/Datum`, `MhtTracker` (predict/solve core).

### [BUG/GAP] 15. Live sensor adapters do no edge validation (invariant #6) — ✅ FIXED 2026-06-18
CLAUDE.md invariant #6 + the docstring require adapters to "validate
parsing/units/NaN/plausibility." The live adapters don't:
- `AisAdapter::ingest` converts `r.lat_deg/lon_deg` straight to ENU with
  no range check. AIS "position not available" sentinels (lat 91°,
  lon 181°) and NaN would produce garbage measurements / phantom tracks.
- `ArpaAdapter` (`parseDdmm`, range/bearing via `strtod`) silently maps a
  parse failure to `0.0` — a malformed TTM range becomes a target at
  own-ship position; lat/lon not range-checked.
- `EoIrAdapter::ingest` never checks `d.range_m`/`bearing` for sign/NaN.
By contrast `AisCsvReplayAdapter.cpp:132` *does* validate
(`|lat|>90 || |lon|>180 → skip`), and `OwnShipNmeaAdapter` guards NaN on
magnetic variation/deviation — so the discipline exists but is applied
inconsistently. Add a shared validate-at-edge helper and route all live
adapters through it.

### [MINOR] 16. ArpaAdapter magic numbers bypass config
TTM/TLL hardcode `sigma = 50.0 m` position std and `1.0°` bearing std
(lines 63-64, 103) instead of pulling from `ArpaAdapterConfig`. Makes
per-deployment noise tuning impossible without a recompile.

### [EVAL] 17. GOSPA/OSPA/id-switch metrics use greedy assignment — ✅ FIXED 2026-06-18 (intentional re-baseline)
`gospaGreedy`, `ospaGreedy`, and `assignPerStep` use greedy nearest-
neighbour, not the optimal (min-cost) assignment the true metrics require.
Greedy can over-count localization cost and, more importantly, flip
pairings between adjacent steps in close-crossing geometry → spurious
`id_switches` and OSPA spikes. This is the documented "Improve next" in
`Metrics.cpp` and aligns with the autoferry truth-fragmentation note. For
A/B estimator decisions this is a real confounder: reuse the existing
`hungarianAssignment` for the metric assignment so comparisons reflect the
tracker, not assignment churn. (The metric code is otherwise clean and
well-documented; continuity/RMSE correctly key by `truth_id`.)

### [OK] Correct
- `ReorderBuffer` determinism (ordered multimap; equal timestamps drain in
  insertion order — satisfies invariant #4).
- `ClutterMapDetectionModel` EWMA decay, bilinear/circular interpolation,
  weighted clutter evidence, deterministic `std::map` iteration.
- `CpaEvaluator` hysteresis + drop handling; `Projection` range/bearing
  Jacobian; `Datum` ENU↔geodetic; `OwnShipNmeaAdapter` velocity precedence
  and NaN guards.

---

## Part 4 — motion models, own-ship estimators, noise model, eval data path

Deep-read: `estimation/NoiseModels` + `ConstantVelocity5State`,
`own_ship/OwnShipVelocityEstimator`, `bias/AisArpaPairExtractor`,
`adapters/replay/AutoferryJsonReplay` (truth/velocity build).

### [OK] All clean
- `StudentTNoiseModel`: scale `s = (ν+δ²)/(ν+d)` correct; the
  `s < 1 → 1` clamp ("never down-trust below Gaussian") is a deliberate,
  documented conservatism, not a bug.
- `OwnShipVelocityEstimator`: per-axis LS slope, pooled residual variance
  `σ² = ss_res/(2(n−2))`, and the noise-aware two-halves maneuver gate are
  statistically sound.
- `ConstantVelocity5State` F/Q correct (turn-rate state carried but
  kinematically inert, as intended for the CV IMM mode).
- `AisArpaPairExtractor` correct (first AIS + first ARPA in window).
- `AutoferryJsonReplay`: **consolidates per-scan truth to one timestamp**
  (`scan_t = max target time`) — this is the fix for the truth-
  fragmentation issue noted in the autoferry eval findings; the data path
  is correct. Truth dedup + finite-difference velocity derivation are fine.

### [MINOR] 18. (carryover) HeadingBias v1 vs cross-sensor publish coupling — ✅ VERIFIED 2026-06-18 (no bug)
Not a defect, a design watch-item: `ArpaAdapter`/`EoIrAdapter` apply the
*heading* bias (`IHeadingBiasProvider`) but the per-sensor position bias
from `SensorBiasEstimator` is applied separately in the Tracker via
`applyBiasCorrection`. Confirm the two corrections are not both folding the
same physical offset on the ARPA/EO-IR path (double-debiasing). Worth a
targeted test; flagged because item 13 just added the second path.

---

## Coverage map

**Deep-read (≈50 files):** all of `core/types`, `core/bias` (incl.
AisArpaPairExtractor), `core/pipeline` (Tracker full; MhtTracker
predict/solve/readout/clutter; ReorderBuffer), `core/estimation` (Ekf,
Ukf, Imm, ParticleFilter, CoordinatedTurn, ConstantVelocity5State,
BearingRangeGuard, SigmaPoints, Resampling, MeasurementModels,
NoiseModels), `core/association` (Gating, Hungarian, Murty, JointEvents,
Jpda setup), `core/collision` (Cpa, CpaEvaluator), `core/geo` (Wgs84,
Datum), `core/output/TrackOutput`, `core/projection/Projection`,
`core/tracking` (TrackManager, DatumShift, ClutterMapDetectionModel,
TrackTree IPDA/VIMM recursion), `core/own_ship` (VelocityEstimator,
UereEstimator), `core/scenario` (Gospa, Ospa), `core/benchmark/Metrics`,
all live `adapters/` sensor parsers + `util/Nmea` +
`OwnShipProvider`/`OwnShipNmeaAdapter` + `AutoferryJsonReplay`.

**Not yet deep-read (lower-risk plumbing; no bugs asserted):** MhtTracker
association cost-build internals (lines 120-555) and TrackTree node
mechanics; `ConstantVelocity2D`, `PrescribedTurn`, `EstimatorDefaults`,
`SensorDefaults`; the rest of `core/benchmark` (Config, Sweep, BenchRunner,
Comparator, Consistency, Csv{Reader,Writer}, MarkdownRenderer, BenchSink)
and `core/scenario` (Builders, Harness*, Metrics, TruthResample); the other
replay adapters (PlotCsv, AisCsv, HaxrTruth, OwnshipCsv); `ports/*`
interface headers (pure abstractions). These are test-harness / I/O /
declaration files off the library's runtime path.

---

## Priorities (recommended order)

1. **#1 + #2** (MHT loses attributes; MHT no datum shift) — the canonical
   pipeline silently drops MMSI identity and breaks on recenter. Highest
   functional impact.
2. **#15** (adapter edge validation) — a single bad AIS fix can spawn a
   phantom track today; cheap to fix with a shared validator.
3. **#3 + #4** (Tracker.cpp unbounded `recent_contributions`; soft-path
   `contributing_sources`) — correctness/memory in the non-MHT pipeline.
4. **#17** (greedy metric assignment) — protects the integrity of your
   estimator A/B decisions.
5. **#11** (PF 4-state hard-wire) — guard or document before anyone wires
   PF to a 5-state model.
6. Everything else is minor / consistency.

