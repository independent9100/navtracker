# Adaptive UERE (Own-Ship Motion-Consistency) — Design

**Date:** 2026-06-03
**Status:** Approved, ready for plan

## 1. Motivation

Today's `pose.position_std_m` is driven by `HDOP × UERE_static` (G3 work, commit `f6c762c`) with a configured UERE default of 5 m. That's correct for a fixed receiver class in a known environment, but real σ_pos varies considerably with surroundings (open ocean vs. coastal multipath vs. urban canyon) and with constellation geometry over time. A static UERE either over-budgets in good conditions (false uncertainty → lazy filter) or under-budgets in degraded conditions (over-confident filter → spurious associations).

This spec adds a small online observer that watches successive GGA-derived ENU positions and infers σ_pos from how well they're fit by a constant-velocity model. When the observer has enough samples and the window isn't dominated by maneuvering, it overrides the static `HDOP × UERE` value. When not, the static path stays in effect. The result is a self-calibrating σ_pos that tracks the environment without operator tuning.

## 2. Scope

In scope:
- New `core/own_ship/UereEstimator` — sliding-window constant-velocity residual variance.
- `OwnShipNmeaAdapter` owns the estimator and chooses between adaptive and static σ when populating `pose.position_std_m`.
- Maneuver gating via two-halves velocity comparison.
- Unit tests for convergence, maneuver suppression, and small-N gating.
- Validation sweep that confirms the estimator tracks sim-injected σ.
- Eval-log section.

Out of scope:
- **RMC-based velocity cross-check.** We don't parse RMC today; could become a follow-up signal if motion-consistency alone proves insufficient.
- **Per-axis σ** (lat vs. lon). Stays isotropic, matching the rest of the GPS work.
- **Adaptive UERE coefficient** (vs. adaptive σ_pos directly). We publish σ_pos and let the consumer divide by HDOP if it cares about the coefficient.
- **AIS-vs-ARPA residual variance observer.** Considered and rejected in brainstorming — requires AIS-cooperative scenes and has the same gap the bias estimator showed on BearingOnlyMoving.

## 3. Architecture

```
GGA → datum.toEnu → (x_enu, y_enu, t) ──┐
                                        │
                                        ▼
                            UereEstimator::observe(t, x, y)
                                        │
                                        ▼
                            current() → (σ_pos, is_published)
                                        │
              ┌─────────────────────────┴─────────────────────────┐
              │                                                     │
   is_published → pose.position_std_m = σ_pos              is_published=false →
                                                            pose.position_std_m = HDOP × UERE_static
```

No new port. The estimator is a private member of `OwnShipNmeaAdapter`. Adapters downstream (ARPA, EOIR) consume `pose.position_std_m` exactly as today — the adaptive logic is transparent to them.

## 4. Math

### 4.1 Sliding window

Maintain `std::deque<Sample>` of up to `N` most recent observations, where `Sample = (t, x, y)`. Push on each `observe()`, pop front when size > N. Default N = 8.

### 4.2 Constant-velocity fit

Per axis (independently), fit `p(t) = a + b · (t − t_0)` where `t_0 = window[0].t`. Closed-form least squares over the window:

```
dt_i = t_i − t_0
b = Σ((dt_i − dt_mean)(p_i − p_mean)) / Σ((dt_i − dt_mean)²)
a = p_mean − b · dt_mean
```

Residuals: `r_i = p_i − (a + b · dt_i)`.

### 4.3 Variance estimate

Combine both axes assuming isotropic noise:

```
σ̂²_pos = (Σ r_xi² + Σ r_yi²) / (2(N − 2))
```

The `(N − 2)` correction is the per-axis residual degrees of freedom after fitting two parameters; the `2(N − 2)` denominator combines both axes' DOF for a single isotropic estimate.

### 4.4 Maneuver gating

Split the window into halves: `H1 = window[0..N/2)`, `H2 = window[N/2..N)`. Fit velocity in each half independently:

```
v_1 = (b_x_H1, b_y_H1)
v_2 = (b_x_H2, b_y_H2)
Δv = ||v_2 − v_1||
```

A fixed `Δv > Δv_threshold` rule misfires under noisy windows: the LS slope of each half itself has standard error `σ_v_half = σ̂_pos / √Σ(dt_i − dt̄)²`, so at σ̂_pos = 2 m and N/2 = 4 samples at 1 Hz spacing, Δv can hit ~1.5 m/s from noise alone even with no true maneuver. The gate must be noise-aware.

Define the velocity-difference noise envelope as:

```
σ_Δv = √2 · σ_v_half        (independent halves)
gate = Δv_threshold + 3·σ_Δv
```

Suppress publication when `Δv > gate`. The configurable `Δv_threshold` (default 0.5 m/s) is then the velocity-change-over-noise margin that distinguishes a real maneuver from a noisy steady-state, not an absolute Δv cutoff.

Publication resumes once a window passes the noise-aware gate. σ̂_pos used in the gate computation is the same as the value the estimator publishes when the gate is open — no chicken-and-egg, since the LS fit is well-defined whether or not we eventually publish.

### 4.5 Publish rule

`is_published == true` iff all of:
1. Window is full: `samples.size() == N`.
2. Maneuver check passed: `Δv ≤ Δv_threshold`.
3. Estimate is sane: `σ̂_pos > 0` and finite.

## 5. Configuration

```cpp
struct UereEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  // Optional floor on published σ; useful so the estimator never reports
  // an unrealistically small value if the window happens to land flat.
  double min_sigma_m{0.05};
};
```

`OwnShipNmeaAdapterConfig` already carries `uere_m{5.0}` (G3). Add one more field:

```cpp
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
  bool enable_adaptive_uere{false};  // NEW — default off, opt-in
  UereEstimatorConfig uere_estimator_cfg{};
};
```

Default off so existing tests are unaffected. New tests opt in.

## 6. `OwnShipNmeaAdapter` changes

In the GGA branch, after computing the ENU position via `datum.toEnu(...)`:

```cpp
if (cfg_.enable_adaptive_uere) {
  uere_estimator_.observe(t, enu.x(), enu.y());
}

const UereEstimate est = uere_estimator_.current();
const bool use_adaptive = cfg_.enable_adaptive_uere && est.is_published;

if (use_adaptive) {
  pose.position_std_m = est.sigma_m;
} else if (hdop > 0.0) {
  pose.position_std_m = hdop * cfg_.uere_m;  // existing G3 path
} else {
  pose.position_std_m = position_std_m_;     // sticky setter (sim path)
}
```

Existing precedence rule from G3 (HDOP wins over sticky setter) is preserved when adaptive is off or unpublished; when adaptive is published, it wins over both.

## 7. Assumptions

- Own-ship motion is approximately constant-velocity over the window duration (default 8 seconds at 1 Hz GGA). Holds for steady cruising; breaks during turns. Maneuver gating handles the break.
- GPS noise is approximately white over the window. Multipath persistence beyond a few seconds would bias the estimate downward; documented limitation for v1.
- GGA arrives at roughly regular intervals. The fit handles irregular spacing correctly but the maneuver-detector assumes the halves are comparable in duration.
- The N=8 default at 1 Hz gives an 8-second window — long enough to suppress per-fix jitter, short enough to track environment changes.

## 8. Rationale

**Why motion-consistency over AIS-vs-ARPA?** Self-contained on own-ship data; works whether the scene has cooperative targets or not. The AIS-vs-ARPA path has the same gap the bias estimator showed on BearingOnlyMoving (no AIS in scene → no observation). Motion-consistency works on the empty ocean.

**Why isotropic σ?** Matches what the rest of the GPS work uses (G1's `+σ²·I`). The error ellipse from constellation geometry is real but not exposed by NMEA GGA, and a per-axis split would propagate through G1's projection math non-trivially. Stay isotropic for v1.

**Why publish raw σ rather than UERE coefficient?** Consumers (ARPA/EOIR adapters) want σ_pos; they don't care whether it came from HDOP × UERE or from the online estimator. Publishing σ directly avoids a recombination step and matches how `pose.position_std_m` is already structured.

**Why fixed window size and not a Kalman-style recursive filter?** The fixed window has a clean closed-form solution per update, gives an unbiased variance estimate, and makes the maneuver detector trivial. A recursive filter would amortize the work but tangles the variance estimation with a stateful estimator that has its own tuning knobs.

**Why maneuver gating by velocity-change rather than acceleration thresholding?** Maritime own-ships rarely accelerate sharply — speed changes are slow but heading changes (turns) reproject the velocity vector quickly. The velocity-difference check captures both speed and heading changes in a single metric without needing to fit a turn-rate model.

**Why default `enable_adaptive_uere = false`?** Preserves the deterministic guarantees of the current pipeline. Existing tests stay byte-identical. Sim tests like the §14.9 sweep and the GPS sweep continue to use known static σ. New tests and production wiring opt in.

## 9. Test plan

### Unit (`tests/own_ship/test_uere_estimator.cpp`)

1. **`UnpublishedWhenWindowEmpty`**: fresh estimator → `is_published == false`.
2. **`UnpublishedBelowWindowSize`**: feed N-1 samples → `is_published == false`.
3. **`ConvergesOnSyntheticWhiteNoise`**: drive a constant-velocity ground truth at, say, 5 m/s east. Add white Gaussian noise σ=2 m. Feed N=8 samples at 1 Hz. Assert `is_published == true` and `0.5 × 2.0 ≤ σ̂ ≤ 1.5 × 2.0`.
4. **`TracksRangeOfSigmas`**: parametric over σ ∈ {0.5, 2, 5, 10} m. After 20 samples (refreshing window), running median of σ̂ falls within ±50 % of truth.
5. **`SuppressesDuringManeuver`**: first 4 samples at v = (5, 0), next 4 at v = (0, 5) → `is_published == false`.
6. **`ResumesAfterManeuver`**: maneuver followed by 8 steady samples → `is_published == true`.
7. **`MinSigmaFloor`**: feed an exact straight line (no noise) → reported σ ≥ `min_sigma_m`.

### `OwnShipNmeaAdapter` integration (`tests/adapters/own_ship/test_own_ship_nmea.cpp`)

8. **`AdaptiveDisabledMatchesStaticPath`**: with `enable_adaptive_uere = false`, GGA stream produces the same `pose.position_std_m` as before — regression guard for the existing G3 path.
9. **`AdaptivePublishesAfterWindowAndDominatesStatic`**: opt in adaptive; stream 10 GGA fixes with HDOP=2.0 and UERE_static=5.0 (would give σ=10 m) but with actual ENU position noise σ=1 m. After window fills, `pose.position_std_m` is close to 1 m, not 10 m.
10. **`AdaptiveFallsBackOnManeuver`**: inject a step velocity change → for the windows that include the change, `pose.position_std_m` reverts to `HDOP × UERE_static`.

### Sim/scenario validation (`tests/sim/test_bus_adaptive_uere.cpp`)

Two TESTs running 20 seeds each, mirroring `test_bus_gps_sweep.cpp` patterns:

11. **`AdaptiveTracksSimInjectedSigma`**: ClutterCrossing variant with σ_gps ∈ {0.1, 1, 5} m and adaptive enabled. Capture `pose.position_std_m` over time per run; assert mean published value within ±50 % of sim-injected σ.

12. **`AdaptiveSweepClutterCrossing`**: same ClutterCrossing cells as G8's GPS sweep, but with adaptive enabled in place of static UERE. Three rows per σ: (R-off, no estimator), (R-on, static), (R-on, adaptive). Expect adaptive to closely match static within statistical noise — the sweep already calibrates the static path to truth, so adaptive should land near it.

### Eval-log

Append "Adaptive UERE estimation (2026-06-03)" section: published-σ-vs-truth tracking table, sweep comparison vs. static UERE, verdict paragraph.

## 10. Ways to improve / what to test next

1. **RMC velocity cross-check.** Use NMEA RMC SOG/COG to provide a second velocity estimate; residual between the GGA-derived and RMC-derived velocity carries info on GPS noise structure. Doubles the observability when RMC is present.
2. **Multipath persistence model.** Replace white-noise assumption with first-order Gauss-Markov; estimate τ jointly. Useful for coastal / port operations where multipath dominates.
3. **Per-axis σ via GST sentence.** When NMEA GST is available, promote to a 2×2 covariance. Composes naturally with G1's projection if the projection gains a 2×2 GPS-cov argument.
4. **HDOP fusion.** Currently adaptive overrides HDOP-derived σ entirely. A future revision could fuse — use adaptive as the slow-varying baseline and HDOP × UERE_estimated as the per-fix scaling.
5. **Adaptive UERE _coefficient_ rather than σ.** Estimate UERE itself (σ̂ / HDOP) so the time-series carries info about receiver health independent of constellation geometry. Useful for diagnosis but doesn't change R-inflation behavior.
6. **Joint estimation with the heading-bias estimator.** Both observe AIS+ARPA residuals (one path); a single joint estimator with state `(b, σ_pos)` could exploit shared observations. Not high priority since the motion-consistency observer is fully decoupled from the bias channel.

## 11. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Observer source | Own-ship motion-consistency (LS over sliding window) | Self-contained; no AIS dependency; works on the empty ocean. |
| Window size | N=8 default | 8 s at 1 Hz GGA — balances jitter suppression vs. environmental tracking. |
| Variance estimator | `Σr² / 2(N−2)` (isotropic) | Standard unbiased variance under 2-parameter linear fit, matches G1's isotropic R-inflation. |
| Maneuver gating | Two-halves velocity check, threshold 0.5 m/s | Captures speed and heading changes in one metric; trivial to compute. |
| Adaptive vs static composition | Adaptive overrides static when published; static fallback otherwise | Backward compat; explicit opt-in via `enable_adaptive_uere`. |
| Publish output | Raw σ_pos (not UERE coefficient) | Consumers want σ_pos directly; avoids HDOP×UERE recombination. |
| Default opt-in | Off | Preserves deterministic guarantees of existing tests. |
