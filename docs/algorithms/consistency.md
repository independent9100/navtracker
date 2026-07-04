# Filter consistency: NEES + NIS instrumentation

Per `CLAUDE.md`: every non-trivial algorithmic component gets the four
sections **Math / Assumptions / Rationale / Ways to improve**. This
document covers the per-update innovation port (`IInnovationSink`) and
the bench-side aggregator (`core/benchmark/Consistency.{hpp,cpp}`)
that land for backlog item 12.

Cross-references:
- Plain-English introduction: [16 — NEES & NIS](../learning/16-nees-nis.md)
- Spec: `docs/superpowers/specs/2026-06-13-nees-r-calibration-design.md`
- Backlog: `docs/algorithms/improvement-backlog.md` §12 (and the §11
  re-diagnosis that hands off to §12)
- Port: `ports/IInnovationSink.hpp`
- Sibling port: `ports/IBearingInnovationSink.hpp` (stays — bias-estimator
  specific)

## Math

### NIS — Normalized Innovation Squared

After each successful hard-match `estimator.update` the tracker emits

```
ν = z − h(x̂⁻)                        (model-correct residual; angle-wrapped)
S = H P⁻ Hᵀ + R                        (predicted innovation covariance)
εⁿⁱˢ = νᵀ S⁻¹ ν                       (scalar NIS, dimensionless)
```

`x̂⁻, P⁻` are the predicted single-Gaussian state at the measurement
time (Tracker uses the moment-matched state the EKF Jacobian path takes;
MhtTracker reconstructs the parent leaf's state via `estimator.predict`
to bit-exactly reproduce what `TrackTree::branch` passed to
`estimator.update`).

Per source key `k = (SensorKind, MeasurementModel, source_id)` the
aggregator accumulates:

```
N_k                                   sample count
ε̄ⁿⁱˢ_k = (1/N_k) Σ εⁿⁱˢ_i              Welford running mean
coverage_{k, 0.95}                    fraction below upper χ²_m at p = 0.95
coverage_{k, 0.99}                    fraction below upper χ²_m at p = 0.99
α̂_k = ε̄ⁿⁱˢ_k / m                      fitted-R scaling (interpretation below)
trace_ratio_k = mean(tr(HPHᵀ) / tr(R))   diagnostic regime flag
```

For a consistent filter `εⁿⁱˢ ∼ χ²_m` with `m = dim(ν)`. The
**95 % consistency band on ε̄ⁿⁱˢ** uses the fact that `N·ε̄ⁿⁱˢ/m ∼
χ²_{Nm}/(Nm)`, so the band is

```
[χ²_{Nm, 0.025}, χ²_{Nm, 0.975}] / N
```

computed via the Wilson-Hilferty closed form (valid for `Nm ≫ 30`; even
at modest N the W-H error is below `±2 %` for `m ≥ 2`). For `N < 30`
samples per key the band collapses to NaN and a `low_sample` flag rides
along.

**Per-sample upper quantiles** use the exact closed forms for the two
common cases — `χ²_1` upper-p = `(Φ⁻¹((1+p)/2))²` and
`χ²_2` upper-p = `−2·ln(1 − p)` — rather than Wilson-Hilferty, which
overshoots by ~20 % at `m = 2` and would bias the coverage_95 column
upward (caught by the unit test
`Consistency.NisFromIidGaussianResiduals_m2`).

**Fitted-R suggestion.** Under the null `R_true = α · R`,

```
E[εⁿⁱˢ] = tr(S⁻¹ (HP⁻Hᵀ + α R))
```

In the common case `R ≫ HPHᵀ` (the regime where the filter is
overconfident in the measurement), `E[εⁿⁱˢ] ≈ α · m`, so

```
α̂_k = ε̄ⁿⁱˢ_k / m
```

is a direct estimate of how much `R` should be inflated for source `k`.
The aggregator also publishes `trace_ratio_HPH_over_R`; when it's
`≫ 1`, `α̂` is unreliable (a high NIS is then driven by overconfident
`P`, not under-sized `R`) and the operator should treat the trace ratio
as a regime tell.

### NEES — Normalized Estimation Error Squared

At every bench truth-tick `t`, the existing greedy 100 m
`assignPerStep` maps each truth slot to at most one track id. For each
assigned `(truth_id, track_id)` pair the aggregator computes
**position-only** NEES against the WGS84 truth, after the bench's
existing ENU projection (same datum at the same `t`, so the math
survives `OwnShipProvider` datum shifts):

```
e = p_truth^ENU − p̂^ENU               (2-vector)
εⁿᵉᵉˢ = eᵀ P_xy⁻¹ e
```

`P_xy` is the 2×2 position block of the snapshot covariance. Per
`(config, scenario)` the aggregator publishes `N`, `ε̄ⁿᵉᵉˢ`,
`coverage_95`, `p95`, the Wilson-Hilferty band, and
`β̂ = ε̄ⁿᵉᵉˢ / 2` — the gross position-cov scaling factor (consistent
filter: `β̂ ≈ 1`; sc5 measured `β̂ ≈ 39`).

## Assumptions

1. The tracker emits `(ν, S, R)` from the pre-update predicted state.
   The port's contract spells this out; both `Tracker` and
   `MhtTracker` compute it from a single source (the same
   `predictMeasurement` helper the existing bearing port uses).
2. `S = HPHᵀ + R` with the same `R` the update applied. The emission
   path passes `z.covariance` (post-adapter validation) — adapters
   that leave covariance empty fail upstream by validation, so the
   sink never sees a zero `R` in production. (The unit test
   `Consistency.SingularSDropped` exercises the defensive path: a
   zero `R` produces a singular `S`, the LDLT fails, the sample is
   dropped and counted in `dropped_singular`.)
3. JPDA soft-update emission is out of scope. The general port,
   like `IBearingInnovationSink` today, fires on hard matches only.
4. NEES is **Confirmed-only**, implicitly: `BenchRunner::snapshotAt`
   already filters to Confirmed tracks, so the snapshot at each
   truth-tick contains only Confirmed candidates. No separate
   Tentative arm at the aggregator layer.
5. NEES is **position-only**. AutoFerry truth SOG is derivative-
   quality (the existing RMSE columns flag the issue); full-state
   NEES would blame the velocity covariance for a position-cov bug.
6. Track snapshots with a zero or sub-2×2 covariance are dropped
   into `default_cov_skipped` — a freshly-initiated track before its
   second update has no honest `P_xy` to test.
7. Datum re-centring during a replay shifts both truth and track
   projections through the current datum at `t`; the math survives.

## Rationale

- **Generic `IInnovationSink` over an estimator-level instrumentation
  port.** The `(SensorKind, source_id)` keying lives at the tracker
  layer (introduced by item 4's `DetectionParams::SourceKey` work),
  not the estimator. Tracker-side emission keeps the inward
  dependency rule (estimators don't import any sink port) and
  re-uses the existing `predictMeasurement` infrastructure. The
  alternative — passing a sink down to every estimator — would
  duplicate per-estimator wiring with no benefit.
- **Pre-computed `(ν, S, R)` fields rather than raw `H, P, R`.**
  Consumers (the bench aggregator today, future bias estimators
  tomorrow) don't reimplement Jacobians; the port is self-
  contained.
- **MhtTracker emission post-solve, surviving leaves only.** The MHT
  branches every leaf during `TrackTree::branch`; emitting at branch
  time would deliver innovations from counterfactual hypotheses the
  global solve never selected. The spec calls for "innovations of
  the filter the world actually saw," so emission lives in
  `MhtTracker::processBatch` after the global solve, walking
  `chosen_leaf[ti]` and reconstructing the parent's pre-update
  predicted state by re-running `estimator.predict`. Deterministic,
  bit-exact, and costs one extra predict per surviving tree per
  scan.
- **Bench-side NEES (not core-side).** NEES needs truth; truth is a
  bench-only concept by the hexagonal architecture. The core stays
  truth-free.
- **Long-format CSV columns rather than fixed per-sensor columns.**
  The existing bench CSV is already long-format
  `(run_id, config, scenario, seed, metric, value, unit)`; per-
  source NIS metrics encode the source key as a suffix on the
  metric name (`nis_mean:ais_pos2d`,
  `nis_alpha_hat:eoir_b2d_cam0`, …). Adds zero schema churn and any
  sensor mix flows through. **Per-target metrics use the same
  pattern**: `pos_rmse_m:truth_42`, `lifetime_ratio:truth_3`, …
  one row per (truth_id, metric). The truth_id is the canonical
  identifier of the truth slot in the scenario (synthetic scenarios
  assign sequential ids; replays use the dataset's ground-truth
  track id). Surfaces "which specific target is dragging the
  scenario mean" without re-running the bench.
- **Offline R-calibration report over an online adaptive R.** Adaptive
  `R` is a one-way door: replays stop being deterministic in the
  same way, and the per-sensor measurement-noise table loses its
  role as the audit surface. The aggregator publishes `α̂`; updating
  the noise model is a separate, human-reviewed commit.
- **Position-only NEES first.** See assumption 5.
- **Wilson-Hilferty for the band, exact closed forms for per-sample
  thresholds.** WH is fine for `Nm ≫ 30` (the band on the mean) but
  overshoots by ~20 % at `m = 2` for per-sample thresholds; using WH
  there would bias `coverage_95` upward. The exact closed forms for
  `m = 1` and `m = 2` are trivial, and only those two cases (and
  `m = 4` from `PositionVelocity2D`, which uses WH) appear in our
  sensor mix.

## Ways to improve / what to test next

1. **R-table calibration follow-up commit.** After this lands, run
   the bench on AutoFerry and read `α̂` per `(SensorKind, source_id)`
   from the consistency rows. Update the per-sensor measurement-
   noise model in the affected adapters; expect the camera Bearing2D
   entries to dominate the corrections (sc5 forensics put `β̂ ≈ 39`
   on the posterior position cov, and the camera is the high-rate
   source). Acceptance: per-sensor NIS bands include the consistent
   range, AutoFerry sc5 `β̂ → ~1`.

2. **Re-run item 11 knob sweeps with the calibrated R.** The
   backlog says the conveyor-belt symptoms should vanish once `P`
   is honest. Expectation: `share_ambiguous_bearings`,
   `gate_threshold`, `gate_recapture_*` either become unnecessary or
   default-able without the lifetime regression.

3. **Bearing range-variance guard (suspect b).** Item 12 lists this
   as conditional on calibrated-R NEES staying hot. If `β̂` does
   not converge to ~1 after step 1, design the guard as its own
   spec (range-direction variance must be non-decreasing under a
   Bearing2D update — classic BOT pathology).

4. **Process-noise (Q) calibration.** If both R and the bearing
   guard close the gap and NEES still ≫ 1, IMM `Q` is the next
   suspect. Bench-side only (no per-measurement port needed): walk
   the same NEES samples and back out the `Q` scaling that brings
   the band to consistent.

5. **Velocity-block NEES.** Once truth SOG/COG comes from a source
   with honest uncertainty (or a synthetic scenario built for the
   purpose), extend `NeesStats` with a 2-d velocity block and a
   `gamma_hat` analogous to `beta_hat`.

6. **JPDA soft-update emission.** When item 8 wires JPDA into
   per-sensor `(P_D, λ_C)`, add a `β`-weighted innovation variant
   to `IInnovationSink` so the JPDA path also reports NIS. The
   weighted residual + inflated `S/β` shape is the same trick
   `IBearingInnovationSink` uses today.

7. **Generalise `IBearingInnovationSink` away.** Once the
   heading-bias estimator's v1 pair flow migrates to the general
   port (carrying `predicted_state_var_rad2` as a derived stat from
   `S/R`), the bearing-specific port can retire.

8. **Per-(truth_id, step) timeseries dump.** A future debugging
   workflow may want a CSV of `(t, truth_id, track_id, εⁿᵉᵉˢ)`
   alongside the aggregates so spikes can be located in time.
   Trivial to add as a sidecar; out of scope here.
