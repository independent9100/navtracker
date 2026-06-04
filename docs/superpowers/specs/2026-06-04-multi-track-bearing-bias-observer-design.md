# Multi-Track Bearing-Innovation Heading-Bias Observer Design

**Date:** 2026-06-04
**Status:** Approved
**Predecessor:** `2026-06-03-heading-bias-estimator-design.md` (v1, AIS↔ARPA pair observation). This spec adds the v2 observer the v1 spec deferred (§11.1).

## 1. Problem

The v1 heading-bias estimator (`HeadingBiasEstimator`) requires AIS↔ARPA pairs to update its scalar `b̂`. In non-cooperative scenes (no AIS), the estimator's published estimate goes stale and the `IHeadingBiasProvider` gate flips to "unknown" — exactly when consumers (especially the bearing-only filters in `BearingOnlyMoving` traces) would benefit most from a corrected bearing.

This spec adds a second observation kind: bearing innovations from established tracks under the existing `Tracker`. The bias observer no longer needs AIS — every Bearing2D / RangeBearing2D measurement processed by the Tracker is a candidate update for `b̂`, gated by observability checks.

## 2. Architecture (path A — single fused estimator)

```
                +-------------+ AisArpaPair       observe(...)
sensor adapters | application |-------------------+
                +-------------+                   v
                                    +-------------------------+
        +---------+                 | HeadingBiasEstimator    |
        | Tracker |--BearingInnov.->|  - predict random walk  |---> IHeadingBiasProvider
        +---------+ (via sink)      |  - observe AIS-pair (v1)|
                                    |  - observe BI (v2 NEW)  |
                                    +-------------------------+
```

- New port: `IBearingInnovationSink` — Tracker emits one event per accepted bearing-model update.
- `HeadingBiasEstimator` implements that sink. Its existing `predictTo` and random-walk model are shared between the two observation kinds; both are scalar KF updates on the same hidden state `b`.
- Composition root wires the sink into the Tracker; if not wired, Tracker behaves as today (zero-emit, no overhead beyond a null check).

Forward-compatibility intact: existing v1 consumers see no API change. The `IHeadingBiasProvider.current()` contract is unchanged.

## 3. Math

### 3.1 Innovation as a measurement of `b`

For one Bearing2D / RangeBearing2D measurement on an established track:

- Predicted bearing from the track's *pre-update* state `x` (after predict step):
  `β̂ = atan2(x_y − s_y, x_x − s_x)` where `s` is `Measurement.sensor_position_enu`.
- Observed bearing `β` is `Measurement.value` (or its bearing component for RangeBearing2D).
- Wrapped innovation `r = wrap(β − β̂)`.

Decompose the observed bearing:
```
β = β_true + b + ν_state + ν_meas
```
- `b` = heading bias (common-mode across all tracks at one timestamp, slow random walk over time),
- `ν_state` = projection of the track's *state error* onto the bearing direction. Independent across tracks under standard tracker assumptions (independent CV process noise, independent associations).
- `ν_meas` ~ N(0, R) sensor bearing noise.

Innovation: `r = (β_true − β̂) + b + ν_state + ν_meas`. Under the linearization the EKF already uses, `(β_true − β̂) ≈ H · (x_true − x_pred)` whose expectation is zero and whose variance equals `H · P · Hᵀ`. So:
```
E[r]  = b
Var[r] = H · P · Hᵀ + R   ≡  S
```
This is the predicted innovation variance the EKF would compute. Treating `r` as a scalar measurement of `b` with variance `S` is the standard linear-Gaussian fusion.

### 3.2 Scalar KF update

Same form as the v1 AIS-pair observation:
```
K   = P_b / (P_b + S)
b̂   ← b̂ + K · r
P_b ← (1 − K) · P_b
```

### 3.3 Multi-track sequential fusion

Process innovations from N tracks within one cycle sequentially. Mathematically valid because `ν_state_i` are independent across tracks (different CV processes, different histories). Sequential scalar updates are equivalent to a single batch update under independence.

### 3.4 Why this isolates `b` and not state error

In bulk: the common-mode component `b` enters all `r_i` identically, while `ν_state_i` are zero-mean independent. After N updates with similar S, posterior variance shrinks like `1/N` *only* when the `ν_state_i` are well below `R`. The observability gate (§4) enforces that condition; if it isn't met, the innovation is rejected as state-error-dominated rather than letting it corrupt `b̂`.

## 4. Observability gates

Three gates applied inside `HeadingBiasEstimator::observe(BearingInnovation)`. An innovation passing all three is fused; otherwise it is silently rejected and counted in a diagnostic.

| Gate | Condition | Default | Rationale |
|---|---|---|---|
| **G1 — Minimum range** | `obs.range_m ≥ min_range_m` | 50 m | Bearing Jacobian `1/r`-singular at short range; innovation noise blows up. |
| **G2 — State error not dominant** | `obs.predicted_state_var_rad2 ≤ k · R` (with `S = state_var + R`) | `k = 1.0` | If state error variance exceeds sensor noise, the innovation is mostly tracking noise, not bias. |
| **G3 — Outlier rejection** | `|obs.innovation_rad| ≤ N · sqrt(obs.variance_rad2)` | `N = 5` | Safety net against association errors, sensor glitches, or wrap-around bugs. |

Defaults live in `HeadingBiasEstimatorConfig` so consumers can tune.

## 5. Wrap handling

The Tracker wraps the innovation to `[-π, π]` at the emit site using the existing `wrapAngle()` utility from `core/estimation/MeasurementModels.hpp`. The estimator treats the wrapped value as a linear quantity — valid as long as `|b|` stays small (the v1 spec already constrains this; expected `|b|` is below 5°).

## 6. Wiring — what the Tracker emits

### 6.1 New port

`ports/IBearingInnovationSink.hpp`:

```cpp
#pragma once
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

struct BearingInnovation {
  Timestamp time;
  TrackId track_id;             // diagnostic only
  double innovation_rad;        // wrap(β_obs − β_pred)
  double variance_rad2;         // H·P·Hᵀ + R
  double predicted_state_var_rad2;  // H·P·Hᵀ only, for gate G2
  double range_m;               // for gate G1
};

class IBearingInnovationSink {
 public:
  virtual ~IBearingInnovationSink() = default;
  virtual void onBearingInnovation(const BearingInnovation& obs) = 0;
};

}  // namespace navtracker
```

### 6.2 Tracker changes

`Tracker` grows a single setter (member, not constructor — optional, can be set after construction):

```cpp
void setBearingInnovationSink(IBearingInnovationSink* sink);  // nullptr disables
```

Default is `nullptr`. Emit point is **after** the associator returns a match and **before** the estimator update — so the track state used to compute `β̂` is the predicted (pre-update) state, which is what the math requires. The emit code:

```cpp
if (bearing_innov_sink_ != nullptr &&
    (z.model == MeasurementModel::Bearing2D ||
     z.model == MeasurementModel::RangeBearing2D)) {
  const Track& tr_pred = manager_.tracks()[ti];  // pre-update
  const auto pred = predictMeasurement(z.model, tr_pred.state,
                                       z.sensor_position_enu);
  // For RangeBearing2D, bearing is component 1; for Bearing2D, component 0.
  const int bidx = (z.model == MeasurementModel::Bearing2D) ? 0 : 1;
  const double beta_pred = pred.z_pred(bidx);
  const double beta_obs  = z.value(bidx);
  const double r = wrapAngle(beta_obs - beta_pred);
  // Project P onto bearing: σ²_β,state = H_row · P · H_rowᵀ.
  Eigen::RowVectorXd Hb = pred.H.row(bidx);
  const double state_var = (Hb * tr_pred.covariance * Hb.transpose())(0, 0);
  const double R_bb = (z.covariance.rows() > bidx && z.covariance.cols() > bidx)
                         ? z.covariance(bidx, bidx) : 0.0;
  const double S = state_var + R_bb;
  const double dx = tr_pred.state(0) - z.sensor_position_enu.x();
  const double dy = tr_pred.state(1) - z.sensor_position_enu.y();
  BearingInnovation obs;
  obs.time = z.time;
  obs.track_id = tr_pred.id;
  obs.innovation_rad = r;
  obs.variance_rad2 = S;
  obs.predicted_state_var_rad2 = state_var;
  obs.range_m = std::hypot(dx, dy);
  bearing_innov_sink_->onBearingInnovation(obs);
}
```

Applied identically in `process()` and in the matched branch of `processBatch()` (the soft/JPDA branch we leave untouched in v2 — see §9).

## 7. Estimator changes

### 7.1 New observation method

```cpp
void HeadingBiasEstimator::observe(const BearingInnovation& obs);
```

Implements: predict to `obs.time`, apply gates (§4), if all gates pass do the scalar KF update (§3.2). On gate rejection, increment per-gate diagnostic counters (not part of `IHeadingBiasProvider`):

```cpp
std::size_t rejectedByRange()       const;
std::size_t rejectedByStateVar()    const;
std::size_t rejectedByOutlier()     const;
std::size_t acceptedBearingObs()    const;
```

### 7.2 Sink implementation

`HeadingBiasEstimator` implements `IBearingInnovationSink`:

```cpp
class HeadingBiasEstimator : public IHeadingBiasProvider,
                             public IBearingInnovationSink {
 public:
  void onBearingInnovation(const BearingInnovation& obs) override {
    observe(obs);
  }
  // ... existing API ...
};
```

Multiple inheritance is fine here — both bases are pure interfaces with no state.

### 7.3 Config additions

`HeadingBiasEstimatorConfig` grows three fields with defaults:

```cpp
double bi_min_range_m{50.0};
double bi_state_var_ratio_max{1.0};   // gate G2 ratio k
double bi_outlier_sigma{5.0};         // gate G3 multiplier N
```

`bi_` prefix to distinguish from any v1 fields.

## 8. Assumptions

1. The Tracker's `manager_.predictAll(estimator, z.time)` is called before the emit, so `Track.state` and `.covariance` reflect the predicted (pre-update) state at `z.time`.
2. Bearing innovations from different tracks are conditionally independent given `b`. Holds under independent CV motion models and independent associations.
3. The bearing measurement model `R_bb` is populated correctly by the adapter. Adapters that omit it get a zero R, which fails gate G2 immediately (state_var ≥ 0 = k·R) and so the observation is harmless — but consumers should populate it.
4. `|b|` stays inside the linearization regime (`|b| < ~30°`). The v1 spec already constrains expected magnitude to a few degrees.

## 9. Out of scope (deferred)

- **JPDA / soft-update emit.** The soft branch of `processBatch` blends multiple measurements per track via betas; computing a single representative innovation is ambiguous (weighted mean? max-beta?). Out of scope for v2; the hard-match branch covers GNN tracking which is the default.
- **Cross-sensor bias.** Radar vs camera could have different mechanical/optical biases. Out of scope; assume one common platform bias.
- **Track-quality weighting beyond gates.** Could later add a confidence weight based on consecutive successful updates.
- **Bias rate (`db/dt`).** Could augment the state to `[b, ḃ]` if drift is large. Random walk is sufficient for current targets.

## 10. Rationale (decisions vs. alternatives)

| Decision | Considered | Chosen | Why |
|---|---|---|---|
| Single fused estimator | Sibling observer + fusion at consumer | Single | One random-walk model, no parallel drift; v1 §202 explicitly promised this path |
| Tracker emits, not estimator | EkfEstimator emits | Tracker emits | EkfEstimator interface stays clean; predict state is reachable from Tracker via `manager_.tracks()[ti]`; supports non-EKF estimators if they preserve the predicted state |
| Sink port (not callback / inject) | `std::function`, ctor-injected dep | `IBearingInnovationSink*` setter | Matches existing `IDatumChangeSink` pattern, nullable, lifetime managed at composition root |
| Three gates (range / state-var / outlier) | No gates; weight-only; just G3 | Three | G1 protects against Jacobian singularity, G2 protects against state-error contamination, G3 against sensor/association glitches. Each addresses a distinct failure mode |
| `k = 1.0` for G2 | 0.25, 4.0 | 1.0 | Requires `Var[ν_state] ≤ Var[ν_meas]`, i.e. the bias signal-to-state-noise ratio is at least 1. Strong enough to keep updates from chasing tracking error; loose enough that mature tracks usually pass |
| `N = 5σ` for G3 | 3σ, 7σ | 5σ | Standard chi-squared gate at ~3 × 10⁻⁷ rejection rate for innovation magnitude — safe against wrap-around and association swaps without rejecting legitimate transients |
| Emit before update | Emit after | Before | Math requires pre-update predicted state for `H` and `P` |

## 11. Validation

### 11.1 Estimator unit tests (`tests/bias/test_heading_bias_bearing_innovation.cpp`)

- **U-EST-1: Single observation drives `b̂` toward `b_true`.** Construct `BearingInnovation` with known `r`, `S`, mature range. Apply once. Verify `b̂` moves by exactly `K · r` and `P_b` shrinks by `(1 − K)`.
- **U-EST-2: Sequence of independent innovations converges.** Feed 100 innovations drawn from `N(b_true, S)` with seeded RNG. Verify `|b̂ − b_true| < 3 · sqrt(P_b)` (3σ envelope holds) and `P_b` is below initial.
- **U-EST-3: Range gate rejects short-range obs.** Set `range_m = 10`, default `bi_min_range_m = 50`. Verify no state change and `rejectedByRange()` increments.
- **U-EST-4: State-var gate rejects state-noise-dominated obs.** Set `predicted_state_var_rad2 = 10 · variance_rad2`. Verify rejection and counter.
- **U-EST-5: Outlier gate rejects 10σ innovations.** Set `innovation_rad = 10 · sqrt(variance_rad2)`. Verify rejection and counter.
- **U-EST-6: Wrap-around doesn't corrupt b̂.** Feed `r = π − ε` (already wrapped) — verify update applies the linear math without further wrapping.

### 11.2 Tracker emission test (`tests/pipeline/test_tracker_bearing_innovation_emit.cpp`)

- **U-TRK-1: Position2D doesn't emit.** Recording sink wired; feed a Position2D measurement; assert no events.
- **U-TRK-2: Bearing2D emits with correct `r, S, range`.** Construct a track at known predicted state, feed a Bearing2D measurement at the same timestamp (post-`predictAll`). Recompute `r, S, range` by hand and assert equality (within 1e-9).
- **U-TRK-3: RangeBearing2D emits the bearing component only.** Same as U-TRK-2 with the range-bearing model; assert the bearing component is what's reported.
- **U-TRK-4: Null sink is a no-op.** Default Tracker (no sink) processes a Bearing2D measurement with no crash.
- **U-TRK-5: Initiation does not emit.** Feed a Bearing2D measurement with no existing tracks → goes to initiate path → no emit.

### 11.3 Headline scenario (`tests/scenario/test_bearing_bias_convergence.cpp`)

**Important caveat surfaced during implementation:** pure single-target bearing-only sensing is structurally bias-state degenerate — the EKF state has enough freedom to absorb a uniform angular shift, so subsequent innovations collapse to zero regardless of the true bias. The v2 estimator therefore still needs *some* unbiased anchor source in the scene (Position2D from any sensor that isn't affected by the heading bias: GPS, lidar, AIS, anchor radar with range). The eval-log "zero-AIS" gap closes whenever *any* such source is present; pure bearing-only remains observationally unobservable without joint bias-state augmentation (deferred — see v1 spec §11 "Joint bias-state EKF augmentation").

Test as actually written:

- Stationary target at (800, 200). Stationary biased EO/IR sensor at origin. Stationary unbiased positioning source (`Ais` sensor kind, stand-in for any non-bearing-affected source) emits Position2D measurements at every cycle with σ = 1 m.
- Bias `b_true = +2° = 0.0349 rad` is injected into each EO/IR `Bearing2D` measurement.
- Wire `HeadingBiasEstimator` as the Tracker's `IBearingInnovationSink`.
- Run 200 cycles, alternating Position2D (anchor) and Bearing2D (biased) per cycle.
- Assert: `|estimator.biasRad() − 0.0349| < 0.5° = 0.00873 rad` after the scenario.
- Assert: `estimator.acceptedBearingObs() > 10`.
- Assert: `estimator.current().is_published == true`.

## 12. Files

| Action | Path |
|---|---|
| Create | `ports/IBearingInnovationSink.hpp` |
| Modify | `core/pipeline/Tracker.hpp` — sink setter + member |
| Modify | `core/pipeline/Tracker.cpp` — emit point in `process()` and `processBatch()` hard branch |
| Modify | `core/bias/HeadingBiasEstimator.hpp` — new `observe()`, new gates, multiple inheritance, diagnostics, config fields |
| Modify | `core/bias/HeadingBiasEstimator.cpp` — implement new `observe()` and `onBearingInnovation()` |
| Modify | `core/bias/HeadingBiasEstimator.hpp` doc block — add v2 math to existing four-part doc |
| Create | `tests/bias/test_heading_bias_bearing_innovation.cpp` |
| Create | `tests/pipeline/test_tracker_bearing_innovation_emit.cpp` |
| Create | `tests/scenario/test_bearing_bias_convergence.cpp` |
| Modify | `CMakeLists.txt` — register new test sources |

No changes to adapters, `Measurement`, `IEstimator`, or any consumer of `IHeadingBiasProvider`.

## 13. Documentation deliverables (CLAUDE.md four-part)

- `IBearingInnovationSink.hpp` — short header doc: math (what fields mean, how `S` is composed), assumptions (pre-update state, bearing wrap), rationale (port vs callback), ways-to-improve (JPDA emit, soft betas).
- `HeadingBiasEstimator.hpp` — append a "v2 observation kind" block to the existing four-part doc: math (§3 of this spec), assumptions (§8), rationale (§10 row "single fused estimator"), ways-to-improve (cross-sensor bias, bias rate state).
- This spec — referenced from the v1 spec's §11.1.

## 14. Acceptance

- Suite green (currently 354/354 → target 354 + ~15 new across the three new test files).
- The three scenario assertions in §11.3 pass with default gates and configs.
- `IHeadingBiasProvider.current().status == Available` in a pure bearing-only scene by the end of `buildBearingOnlyMovingSensorScenario` (zero AIS).
- No changes to `IEstimator`, adapters, or `Measurement`.
- Both header docs updated to the four-part standard.
