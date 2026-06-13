# NEES / NIS instrumentation + offline R calibration

**Backlog item:** `docs/algorithms/improvement-backlog.md` §12 (closes the
loop on §11's re-diagnosis). Predecessor design baseline: v0.5.0.

## Why now

Forensics on the sc5 churn (2026-06-12, item 11 re-diagnosis) measured
**mean position NEES 77.6** on near-truth confirmed tracks (consistent
≈ 2), 57 % of samples above the 99 % χ² bound, claimed σ 1.2–3.8 m vs
actual error 15.1 m mean. The filter is structurally overconfident on
real bearing-dominated data and every downstream symptom — duplicate-
birth conveyor, identity churn, gate misses on sparse radar — falls out
of that. Item 11's three opt-in gating knobs were measured: they trade
switches for lifetime/rmse because the underlying P stays wrong.

The fix is to make P honest. That requires (a) seeing per-sensor
innovation statistics in the live filter and (b) measuring posterior
covariance consistency against truth in the bench. This spec adds both.

## Scope (and what's explicitly out)

In:
- A new general-purpose innovation port emitted by `Tracker` /
  `MhtTracker` after each hard-match update, keyed by `(SensorKind,
  source_id)`.
- A bench-side aggregator that computes per-sensor **NIS** from
  innovations and per-(config, scenario) **NEES** from existing
  truth assignments.
- CSV + Markdown reporting columns.
- An offline R-calibration suggestion (`α̂` per sensor) printed
  alongside the consistency bands. Updating the per-sensor
  measurement-noise table is a separate, human-reviewed commit.

Out:
- Adaptive/online R inflation in the estimator. Deferred until the
  offline workflow demonstrably plateaus.
- The bearing range-variance guard (suspect b in item 12). Conditional
  on what NIS shows — designed only if (a) doesn't close the gap.
- JPDA soft-update emission. Same footnote `IBearingInnovationSink`
  carries today; revisit when item 8 wires JPDA into per-sensor
  (P_D, λ_C).
- Velocity / full-state NEES. Position-only first; AutoFerry truth SOG
  is derivative-quality and would blame the velocity block for a
  position-block bug.

## Math

### NIS (per-update, per-sensor)

After each successful hard-match update at time *t*:

- ν = z − h(x̂⁻)          (innovation in measurement space)
- S = H P⁻ Hᵀ + R          (predicted innovation covariance)
- εⁿⁱˢ = νᵀ S⁻¹ ν          (scalar NIS)

x̂⁻, P⁻ are the predicted state/covariance (pre-update). H is the
linearised measurement Jacobian, R the measurement noise used. The
sink receives ν, S, R already computed by the estimator — bench code
never reconstructs H/P internally.

Per `SourceKey k = (SensorKind, source_id)` the aggregator accumulates:

- N_k — sample count
- ε̄ⁿⁱˢ_k = (1/N_k) Σ εⁿⁱˢ_i
- coverage_{k, 0.95}, coverage_{k, 0.99} — fraction of samples below
  upper χ²_m bounds, m = dim(ν)
- α̂_k — fitted scaling (see below)

**95 % consistency band on ε̄ⁿⁱˢ.** εⁿⁱˢ ∼ χ²_m, so N·ε̄ⁿⁱˢ/m is
χ²_{Nm}/(Nm). Two-sided 95 % band is
[χ²_{Nm, 0.025}, χ²_{Nm, 0.975}] / N. Computed via Wilson-Hilferty
closed form; valid from N ≥ 30. Below that, emit ε̄ⁿⁱˢ, emit NaN for
the band, set a `low_sample` flag.

**Fitted-R suggestion.** Under the null R_true = α · R:
E[εⁿⁱˢ] = tr(S⁻¹ (HP⁻Hᵀ + α R)). In the common case R ≫ HP⁻Hᵀ
("overconfident in measurement"), E[εⁿⁱˢ] ≈ α · m, so
**α̂_k = ε̄ⁿⁱˢ_k / m**.

The α̂ report carries a caveat: when HP⁻Hᵀ dominates R, a high εⁿⁱˢ
is not necessarily a too-small-R problem — it can be a process-noise
mistuning (suspect c). The bench prints the trace ratio
tr(HP⁻Hᵀ) / tr(R) alongside α̂ so the human reading the report can
tell which regime they're in.

### NEES (per truth-tick, per assigned track, position only)

At every existing bench truth-tick t, the existing greedy 100 m
`assignPerStep` matches each truth_id to at most one track_id. For
each assigned pair:

- e = p_truth^ENU − p̂^ENU       (2-vector)
- εⁿᵉᵉˢ = eᵀ P_xy⁻¹ e            (P_xy is the 2×2 position block)

Both truth and track positions are projected through the same datum
in force at t (re-uses the existing bench projection path; remains
valid across `OwnShipProvider` datum shifts).

Per (config, scenario) the aggregator accumulates ε̄ⁿᵉᵉˢ, coverage,
p95, and **β̂ = ε̄ⁿᵉᵉˢ / 2** (the position-covariance scaling factor;
consistent ≈ 1; the sc5 measurement implies β̂ ≈ 39).

**Confirmed vs Tentative split.** Two separate column pairs:
`nees_mean_confirmed`, `nees_mean_tentative` (and `_p95`). Item 12's
diagnosis is about confirmed-track overconfidence; tentative-track
samples are reported but kept out of the lever.

**Default-covariance gating.** When `TrackOutput::covariance_is_default`
is true (a freshly initiated track before its second update), skip
the NEES sample and count it in `default_cov_skipped`. Otherwise
those samples would inject the per-sensor R / default initialiser
into the posterior-honesty measurement.

### Numerical conventions

All consistency math is unit-free (ν, S, e, P_xy carry their native
SI units; the inverse cancels in the quadratic form). LDLT solve via
Eigen, not explicit inverse; LDLT failure or smallest-pivot floor
< 1e-12 drops the sample and increments a counter (`dropped_singular`).

## Architecture

Two independent additions, both pure-add with nullable defaults.

### 1. `ports/IInnovationSink.hpp` (new)

```cpp
struct InnovationEvent {
  Timestamp time;
  TrackId track_id;
  DetectionParams::SourceKey source;   // (SensorKind, source_id)
  MeasurementModel model;              // Position2D / RangeBearing2D / Bearing2D
  Eigen::VectorXd residual;            // ν, in measurement space
  Eigen::MatrixXd S;                   // HPHᵀ + R (predicted innovation cov)
  Eigen::MatrixXd R;                   // measurement noise used
  std::size_t dim;                     // ν.size()
};

class IInnovationSink {
 public:
  virtual ~IInnovationSink() = default;
  virtual void onInnovation(const InnovationEvent& e) = 0;
};
```

Emitted from `Tracker::process` and `MhtTracker::processBatch` after
a successful hard-match `estimator.update`, at the same site where
`IBearingInnovationSink` already fires today. Wired via
`setInnovationSink(IInnovationSink*)` on each tracker; nullptr →
today's behaviour, zero overhead. MhtTracker emits **once per
measurement on the surviving leaf**, after the global solve — not on
pruned branches.

`IBearingInnovationSink` stays untouched. Its contract carries
pre-update predicted-state-variance fields that the bias estimator
needs and that aren't general. A follow-up can collapse the two
ports once the bias module is adapted.

### 2. `core/benchmark/Consistency.{hpp,cpp}` (new)

Pure post-processor. Public surface:

```cpp
struct NisStats {
  std::size_t n{0};
  double mean{0.0};
  double coverage_95{0.0};
  double coverage_99{0.0};
  double alpha_hat{0.0};
  double trace_ratio_HPH_over_R{0.0};
  double band_lo{NaN}, band_hi{NaN};
  bool low_sample{false};
  std::size_t dropped_singular{0};
};

struct NeesStats {
  std::size_t n_confirmed{0}, n_tentative{0};
  double mean_confirmed{NaN}, mean_tentative{NaN};
  double p95_confirmed{NaN}, p95_tentative{NaN};
  double coverage_95_confirmed{NaN};
  double beta_hat{NaN};
  std::size_t default_cov_skipped{0};
  std::size_t dropped_singular{0};
};

struct ConsistencyResult {
  std::map<DetectionParams::SourceKey, NisStats> per_source;
  NeesStats nees;
};
```

`NisCollector : public IInnovationSink` — streams ν/S/R into per-key
accumulators (Welford-stable mean; sorted insertion for coverage is
O(N log N) at finalize). `NeesCollector` — snapshots `(track_id, t,
p̂_ENU, P_xy, status, covariance_is_default)` on `onTrackUpdated`,
then at finalize walks `BenchResult.steps + assignPerStep` outputs
to compute per-step εⁿᵉᵉˢ on assigned pairs.

### 3. Wiring

```
core/benchmark/BenchRunner.cpp
  - constructs NisCollector + NeesCollector
  - tracker->setInnovationSink(&nis)
  - manager->setTrackSink(&fanout{bench_sink, nees})

core/benchmark/Metrics.{hpp,cpp}
  - ConsistencyResult travels alongside MetricsResult on the per-run record

core/benchmark/CsvWriter.{hpp,cpp}
  - new right-edge columns:
      nis_mean_<key>, nis_cov95_<key>, nis_alpha_<key>,
        nis_trace_ratio_<key>, nis_n_<key>, nis_low_sample_<key>,
      nees_mean_confirmed, nees_p95_confirmed,
        nees_cov95_confirmed, nees_beta_hat, nees_n_confirmed,
      nees_mean_tentative, nees_n_tentative,
      nees_default_cov_skipped

bench/render_markdown.cpp
  - per-(config, scenario) NEES column in the main table
  - new per-sensor NIS table appended to the report (one row per
    source_key, columns: N, ε̄, 95%-band, coverage_95, α̂, trace_ratio)
```

### File layout

```
ports/
  IInnovationSink.hpp                  NEW
core/pipeline/Tracker.{hpp,cpp}        + setInnovationSink, emission site
core/tracking/MhtTracker.{hpp,cpp}     + setInnovationSink, surviving-leaf emission
core/benchmark/
  Consistency.{hpp,cpp}                NEW (stats + collectors)
  Metrics.{hpp,cpp}                    + ConsistencyResult parallel return
  BenchRunner.{hpp,cpp}                wire both collectors
  CsvWriter.{hpp,cpp}                  + new columns
bench/render_markdown.cpp              + NEES column + NIS table
docs/algorithms/consistency.md         NEW (Math / Assumptions / Rationale /
                                       Ways to improve, per CLAUDE.md doc rule)
tests/benchmark/test_consistency.cpp           NEW (unit math)
tests/tracking/test_innovation_sink_emission.cpp NEW (emission contract)
tests/replay/test_consistency_e2e.cpp          NEW (one synthetic scenario)
tests/benchmark/test_bench_determinism.cpp     EXTEND with new columns
```

## Data flow per replay

```
sensor frame
   │
   ▼
Tracker / MhtTracker.process(z)
   │
   ├──► estimator.update(z) ──► onTrackUpdated   (ITrackSink)
   │                              │
   │                              └──► NeesCollector buffers (id, t, p̂, P_xy, status)
   │
   └──► onInnovation(ν, S, R, key, model)   (IInnovationSink)
                              │
                              └──► NisCollector accumulates per SourceKey

at each truth-tick t (existing bench step):
   assignPerStep gate matches truth_id → track_id
   NeesCollector + assignment → per-(truth_id, t) εⁿᵉᵉˢ
   stream into per-(config, scenario) accumulators

at run end:
   Consistency::finalize(...) → ConsistencyResult
     • NIS table per (config, scenario, source_key)
     • NEES summary per (config, scenario)
   → CSV (new columns) + Markdown (new tables)
```

## Error handling

- **S or P_xy ill-conditioned.** LDLT solve; on `info() != Success` or
  smallest pivot < 1e-12, drop sample and increment `dropped_singular`.
  Never substitute a regularised matrix — that would mask zero-R bugs
  in adapters or degenerate H.
- **Default-covariance samples.** Skip NEES for tracks where
  `TrackOutput::covariance_is_default == true`; counted separately.
- **No-assignment steps.** Truth without a track in 100 m gate adds
  nothing to NEES. That's a lifetime signal, already covered.
- **Tentative vs Confirmed.** NIS counts everything (legitimate filter
  behaviour). NEES partitions Tentative from Confirmed.
- **Datum re-centring.** Both sides project through the current datum
  at t; consistency math survives shifts.
- **Stale measurements.** Item 1's guard drops them before
  `estimator.update`; never reach the sink. No special handling.
- **Sink throw safety.** Same contract as `IBearingInnovationSink` /
  `ITrackSink`: must not throw across tracker boundary.
- **Low sample size.** N < 30 → ε̄ emitted, band NaN, `low_sample = true`.

## Testing

Unit `tests/benchmark/test_consistency.cpp`:
- `nisFromIidGaussianResiduals` — N=5000 samples from known S; ε̄
  inside Wilson-Hilferty band for m ∈ {1, 2}.
- `nisFittedAlpha` — synthetic R_true = α·R_claimed for α ∈ {0.5, 1, 4};
  α̂ within 5 % at N=5000.
- `neesFromKnownFilter` — single track + truth pair, P_xy = I, errors
  e ∼ N(0, I) → ε̄ⁿᵉᵉˢ ≈ 2.
- `bandSuppressedBelowMin` — N < 30 → band NaN, `low_sample = true`.
- `singularSDropped` — zero R passes through → counter increments,
  sample dropped, ε̄ unaffected.

Emission contract `tests/tracking/test_innovation_sink_emission.cpp`:
- Single-track `Tracker` with recording sink; one Position2D, one
  RangeBearing2D, one Bearing2D measurement. Assert one event per
  measurement; correct `SourceKey`, `model`, `ν.size()`, S/R values
  match the estimator-internal H/P/R via a stub estimator fixture.
- `nullSinkZeroOverhead` — run a synthetic scenario with / without a
  sink; assert OSPA + other existing metrics bit-identical.
- `mhtFiresOnceOnSurvivingLeaf` — MhtTracker with branching factor >1
  fires exactly one event per measurement after the solve.
- `staleMeasurementSilent` — guard drops measurement → no emission.

Bench integration `tests/replay/test_consistency_e2e.cpp`:
- One existing synthetic scenario (`crossing`) under
  `imm_cv_ct_mht_ipda_vimm`. Assert
  `nees_mean_confirmed ∈ [1.5, 3.0]` and NIS `coverage_95 ≥ 0.85` for
  the synthetic Position2D source. Pins the "consistent on clean
  synthetics" floor.
- AutoFerry rows are reported but not asserted — that's the point of
  running them.

Determinism: extend `tests/benchmark/test_bench_determinism.cpp` to
cover the new columns. Two back-to-back runs over the same scenarios
→ byte-identical rows.

## Acceptance

1. Bench produces NIS per (config, scenario, source_key) and NEES per
   (config, scenario) on the v0.5.0 scenario set, with no regressions
   in existing OSPA / lifetime / RMSE / id_switches columns.
2. Clean synthetics (crossing, etc.) satisfy
   `nees_mean_confirmed ∈ [1.5, 3.0]` under the canonical
   `imm_cv_ct_mht_ipda_vimm` config — pinning the consistency floor.
3. AutoFerry scenario report column shows sc5
   `nees_mean_confirmed` ≫ 2 — reproducing the item-12 diagnosis from
   a clean rebuild and confirming the instrumentation is wired.
4. Determinism preserved (replay → bit-identical CSV).
5. `docs/algorithms/consistency.md` written with the four required
   sections (Math, Assumptions, Rationale, Ways to improve / what to
   test next), per the project doc standard.

## Rationale (why this shape, not the alternatives considered)

- **General `IInnovationSink` vs. estimator-level instrumentation
  port:** estimators don't know SourceKey; the tracker does. Tracker
  emission keeps the inward-dependency rule and gives us per-sensor
  keying for free off item 4's work.
- **Bench-side NEES vs. core-side:** NEES needs truth, and truth is
  a bench-only concept. The core stays I/O-free, sensor-format-free,
  and truth-free.
- **Offline R suggestion vs. adaptive R:** adaptive R is a one-way
  door — once it's in the hot path, replays stop being deterministic
  in the same way and the per-sensor table loses its role as the
  audit surface. Keep R changes human-reviewed until the data
  demands otherwise.
- **Position-only NEES first:** AutoFerry truth SOG is derivative-
  quality; full-state NEES would blame the velocity block for a
  position bug.
- **Defer bearing range-variance guard:** item 12 lists it as "if (b)
  confirmed." The honest sequence is measure first, fix second.

## Ways to improve / what to test next

After this lands and the per-sensor R table is updated, the queued
experiments are:

1. Re-run the item-11 knob sweeps (`share_ambiguous_bearings`,
   `gate_threshold`, `gate_recapture_*`) with the honest R. Expectation
   per the backlog: conveyor births vanish at the base gate, sc5/sc6
   switches collapse without lifetime loss.
2. If sc5 NEES stays hot post-R-calibration, design the bearing
   range-variance guard (suspect b) as its own spec.
3. If both (a) and (b) close the gap and IMM process noise remains
   suspect, calibrate Q the same way (NEES at scenario level, no
   per-measurement port needed — just bench-side).
4. Generalise `IBearingInnovationSink` away once the bias estimator's
   v1 pair flow is migrated, leaving a single innovation port.
