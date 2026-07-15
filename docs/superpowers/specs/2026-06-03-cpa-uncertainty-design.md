# CPA Uncertainty Quantification — Design

**Date:** 2026-06-03
**Status:** Approved, ready for plan

## 1. Motivation

`core/collision/Cpa.{hpp,cpp}` gives a deterministic point estimate of `(cpa_distance, tcpa, is_diverging)` between two tracks. The header explicitly flags "report a sigma bound on cpa_distance_m" as future work. This spec lands that — a Jacobian-based propagation of input track covariance through the closed-form CPA function, yielding `σ_cpa`, `σ_tcpa`, and `P(CPA < d_threshold)`.

The work in this session (§14.9 heading R-inflation, GPS position uncertainty, heading-bias estimator, adaptive UERE) has made `Track.covariance` meaningfully more correct. CPA was the only consumer that hadn't picked up the benefit. Closing this means every uncertainty improvement now propagates to a single operationally meaningful number: how likely is this pair to collide.

## 2. Scope

In scope:
- `CpaPrediction` struct alongside the existing `CpaResult`.
- `computeCpaWithUncertainty(track_a, track_b, t_ref, d_threshold_m)` — linear/Jacobian propagation through the closed-form CPA.
- `synthesizeOwnShipTrack(pose, velocity_enu, sigma_pos_m)` helper so own-ship can enter the same pair-CPA primitive.
- Singularity handling for parallel velocities and head-on / zero-CPA cases.
- Probability output via 1D Gaussian on `CPA` (standard collision-avoidance approximation).
- Unit tests for the math and singularity branches; one integration test that runs through the tracker on a known scenario.
- Eval-log section connecting prior covariance work to σ_cpa.

Out of scope (deferred to §11):
- **2D off-center Rician CDF** for accurate P near collision — the 1D Gaussian approximation under-counts probability when the CPA mean is small relative to σ.
- **Own-ship velocity uncertainty.** Caller provides own-ship velocity in v1; σ_v_own = 0. Future work parses NMEA RMC (SOG/COG) or finite-differences successive GGAs.
- **Track-to-track covariance** (joint Σ off-diagonals from shared GPS error, Approach 2 from the GPS spec). Linear v1 assumes track independence.
- **IMM-aware propagation** (mode-weighted means and covariances).
- **Target-vs-target CPA** at scale. The API supports it; v1 tests focus on own-ship-vs-target.

## 3. Architecture

```
Track a (state, cov)  ──┐
                        │   computeCpaWithUncertainty(a, b, t_ref, d)
Track b (state, cov)  ──┤   ────────────────────────────────────────►   CpaPrediction
                        │                                                {cpa, σ_cpa, tcpa,
                        │                                                 σ_tcpa, P(<d),
                        │                                                 is_diverging}
                        │
                        └──── synthesizeOwnShipTrack(pose, vel_enu, σ_pos_m)
                              builds a Track for the own-ship case
```

No new ports. No tracker changes. CPA stays a downstream utility consumed by composition roots, sinks, and tests. The existing `computeCpa` API stays exactly as it is.

## 4. Math

### 4.1 Inputs

Per the existing `computeCpa`:
- Each track is extrapolated from its `last_update` to `t_ref` along its CV velocity:
  ```
  p_a(t_ref) = state(0..1)_a + state(2..3)_a · (t_ref − last_update_a)
  p_b(t_ref) = state(0..1)_b + state(2..3)_b · (t_ref − last_update_b)
  ```
- `v_a = state(2..3)_a`, `v_b = state(2..3)_b`.
- `dp = p_a(t_ref) − p_b(t_ref)`, `dv = v_a − v_b`.

Joint input vector for the Jacobian:
```
x = [p_ax, p_ay, v_ax, v_ay, p_bx, p_by, v_bx, v_by] ∈ ℝ⁸
Σ = blockdiag(P_a, P_b) ∈ ℝ⁸×⁸  (v1: independent tracks)
```

(Note: `P_a`, `P_b` are taken at the tracks' `last_update`. Extrapolating to `t_ref` would require a process model. v1 uses the snapshot covariance directly — documented limitation; small for short extrapolation windows.)

### 4.2 Closed-form CPA recap

```
t_cpa_raw = −dp · dv / |dv|²
t_cpa     = max(t_cpa_raw, 0)
p_cpa     = dp + dv · t_cpa
cpa       = ||p_cpa||
```

`is_diverging = (t_cpa_raw < 0)`.

### 4.3 Jacobians

Define the partials of `dp` and `dv` w.r.t. `x`:

```
∂dp/∂p_a =  I_2   ∂dp/∂v_a =  dt_a · I_2
∂dp/∂p_b = −I_2   ∂dp/∂v_b = −dt_b · I_2
∂dv/∂v_a =  I_2   ∂dv/∂v_b = −I_2
∂dv/∂p_a = 0       ∂dv/∂p_b = 0
```

`dt_a = t_ref − last_update_a`, `dt_b = t_ref − last_update_b`.

**`∂t_cpa/∂x` (1×8).** From `t_cpa = −(dp · dv) / (dv · dv) = −N/D`. The quotient
rule gives `∂t_cpa/∂x = −N'/D + N·D'/D²` with `N' = (∂dp/∂x)ᵀ·dv + (∂dv/∂x)ᵀ·dp`
and `D' = 2·(∂dv/∂x)ᵀ·dv`. Substituting `N = −t_cpa·D` into the second term
(`N·D'/D² = −t_cpa·D'/D`) yields:

```
∂t_cpa/∂x = −[(∂dp/∂x)ᵀ · dv + (∂dv/∂x)ᵀ · dp] / |dv|²
            − 2 t_cpa · [(∂dv/∂x)ᵀ · dv] / |dv|²
```

This is a row vector built by stacking the eight partial-derivative contributions
per component. (W4.3 correction: the chain term is **subtracted**, not added —
the earlier `+ 2 t_cpa · […]` was a sign error that inflated `σ_tcpa` ~3× for
converging pairs and corrupted the head-on `σ_cpa`/probability fallback. The
error only surfaces where the target/own-ship velocity covariance is non-zero,
since the chain term lives entirely in the velocity columns.)

**`∂cpa/∂x` (1×8).** From `cpa = ||p_cpa|| = √(p_cpa · p_cpa)`:

```
∂cpa/∂x = (p_cpa / cpa) · ∂p_cpa/∂x
∂p_cpa/∂x = ∂dp/∂x + (∂dv/∂x) · t_cpa + dv · (∂t_cpa/∂x)
```

The first term in `∂p_cpa/∂x` is direct; the second has `t_cpa` as a scalar multiplying the matrix `∂dv/∂x`; the third has the column vector `dv` multiplying the row `∂t_cpa/∂x` to produce a 2×8 outer-product contribution.

### 4.4 Variances

```
σ²_tcpa = J_tcpa · Σ · J_tcpaᵀ      (scalar)
σ²_cpa  = J_cpa  · Σ · J_cpaᵀ        (scalar)
```

`σ_tcpa = √max(σ²_tcpa, 0)`, `σ_cpa = √max(σ²_cpa, 0)`.

### 4.5 Probability of CPA below threshold

```
P(CPA < d_threshold) = Φ((d_threshold − cpa) / σ_cpa)
```

where `Φ` is the standard normal CDF (`std::erfc` or `std::erf` based). Under the 1D-Gaussian approximation: if `σ_cpa` is zero (deterministic inputs), `P` is the step function {0 if cpa > d, 1 if cpa < d, 0.5 if equal}.

### 4.6 Singularity branches

The Jacobian denominator blows up at `|dv|² → 0` and `cpa → 0`. Handle each:

**Parallel velocities (`|dv|² < ε_dv`, default 10⁻⁶ m²/s²):**
- `is_diverging` false (constant distance).
- `tcpa = 0`, `σ_tcpa = +∞` (sentinel via `std::numeric_limits<double>::infinity()`).
- `cpa = ||dp||`, `σ_cpa = √(dp̂ · cov_dp · dp̂)` where `dp̂ = dp / ||dp||` and `cov_dp = P_a[0..1, 0..1] + P_b[0..1, 0..1]`.
- `P(CPA < d) = Φ((d − cpa) / σ_cpa)` (well-defined if `σ_cpa > 0`).

**Past CPA (`t_cpa_raw < 0`):**
- Same fallback as parallel velocities: `tcpa = 0`, σ_tcpa = ∞, `cpa = ||dp||`, σ_cpa from dp covariance.
- `is_diverging = true`.

**Head-on / near-zero CPA (`cpa < ε_cpa`, default 1 m):**
- `p_cpa / cpa` is undefined. Compute σ_cpa from the isotropic-Gaussian fallback:
  ```
  σ_cpa = √(tr(cov_p_cpa) / 2)
  ```
  where `cov_p_cpa = J_p_cpa · Σ · J_p_caᵀ` is the 2×2 covariance of `p_cpa`. This collapses to the right scalar when `p_cpa` is at the origin.

## 5. `synthesizeOwnShipTrack` helper

Per spec §3, own-ship enters as a synthesised Track so the API stays uniform:

```cpp
namespace navtracker {

Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum);
}
```

- Converts `pose.lat_deg, pose.lon_deg` to ENU via the supplied datum.
- Returns a `Track` with state `[ex, ey, vx, vy]` and covariance `diag(σ_pos², σ_pos², 0, 0)`. (Zero velocity covariance per v1 decision.)
- `id = TrackId{0}` (sentinel; not entered into TrackManager). `last_update = t`. `status = Confirmed`. Other Track fields default-initialised.
- σ_pos_m typically equals the latest `pose.position_std_m` from the GPS work, but the caller passes it explicitly so the helper is testable in isolation.

Lives in `core/collision/CpaOwnShip.{hpp,cpp}` or appended to `Cpa.cpp` — implementer's choice based on file size.

## 6. Assumptions

- Each track's covariance is well-defined and PSD at `t_ref`. No process-model extrapolation of P from `last_update` to `t_ref`; for short windows (default <10 s) this is a small under-counting.
- Track-to-track independence: `Σ = blockdiag(P_a, P_b)`. Deferred GPS Approach 2 would add off-diagonals; this spec does not.
- 1D-Gaussian on CPA is acceptable when `cpa > 2·σ_cpa`. Near collision (`cpa < σ_cpa`) the approximation under-counts P; documented limitation in §11.
- Own-ship velocity is known by the caller to sufficient accuracy that σ_v_own = 0 is acceptable. Typical GPS-derived SOG is <0.1 m/s standard error, dwarfed by tracker velocity uncertainty on targets.
- The Jacobian linearisation is valid when the input σ is small relative to the dynamic range over which CPA varies. For typical maritime σ_pos ~10 m and CPA ~100 m this holds; for σ_pos comparable to CPA (close-quarters with poor tracking), σ_cpa is approximate.

## 7. Rationale

**Why linear/Jacobian over sigma-points or MC?** Closed-form, deterministic, ~negligible runtime cost. Sigma-points (17 evaluations) and MC (hundreds of evaluations) buy robustness to nonlinearity that linearisation handles adequately for typical operating geometry. The singularities (parallel velocities, near-collision) require explicit branching in any approach — linear isn't structurally worse there. Pick linear, document the limitation; promote to sigma-points only if measured σ_cpa drift becomes operationally problematic.

**Why mean+σ+P instead of full 2×2 covariance over (CPA, TCPA)?** Operational consumers want one of: a band on the UI (mean+σ), an alarm threshold (P), or both. The cross-covariance term `Cov(CPA, TCPA)` enables joint queries ("P(CPA<200m AND TCPA<5min)") that no current consumer asks for. Adding it later is a non-breaking extension.

**Why synthesise own-ship as a Track rather than a separate overload?** One uniform pair-CPA primitive supports own-ship-vs-target *and* target-vs-target (deferred but enabled). Avoids API duplication. The synthesis helper is small and testable in isolation.

**Why σ_v_own = 0 in v1?** SOG from GPS RMC is <0.1 m/s standard error in steady cruise — small relative to tracker velocity σ on targets (typically 0.5–2 m/s). The dominant CPA uncertainty source is the target's tracking covariance, which the work-just-shipped now models more correctly. Adding own-ship velocity σ is a clean follow-up tied to RMC parsing.

**Why probability via Φ rather than Rice/chi for the 2D vector?** The 1D Gaussian is the standard collision-avoidance literature approximation and matches what operators expect. Documented as ways-to-improve for near-collision accuracy.

**Why no extrapolation of P from `last_update` to `t_ref`?** Would require a process-noise model coupled to each track's estimator. v1 keeps the snapshot covariance; for short extrapolation windows (the common case where `t_ref ≈ now`), the under-counting is small. Adding `P_extrap = F P Fᵀ + Q dt` is a clean two-line follow-up if measured σ_cpa proves over-confident at long horizons.

## 8. Test plan

### Unit (`tests/collision/test_cpa_uncertainty.cpp`)

1. **`ZeroCovGivesZeroSigma`** — both inputs with zero covariance → σ_cpa = σ_tcpa = 0; P is step function.
2. **`HeadOnPropagatesPositionSigma`** — collision at t_cpa=10 s, target σ_pos=10 m, own-ship σ_pos=0; expected σ_cpa ≈ 10 m (target's σ projects through the formula; velocity contribution damps to ~0 at the predicted t_cpa).
3. **`PerpendicularPassSigmaScalesLinearlyWithTargetSigmaPos`** — vary target σ_pos ∈ {1, 5, 10, 20} m; expected σ_cpa ≈ σ_pos within 20 %.
4. **`ParallelVelocitiesSigmaTcpaInfinite`** — set v_a = v_b; σ_tcpa = ∞; σ_cpa from current-dp covariance; tcpa = 0; is_diverging=false.
5. **`PastCpaUsesCurrentDistance`** — start with already-past-CPA geometry; is_diverging=true, σ_cpa from current dp σ.
6. **`HeadOnNearZeroCpaUsesIsotropicFallback`** — head-on at low σ; cpa < ε_cpa triggers the fallback; σ_cpa equals √(tr(cov_p_cpa)/2) within 1e-6.
7. **`ProbabilityMatchesGaussian`** — at d_threshold=cpa, P=0.5; at d much smaller, P→0; at d much larger, P→1; check three intermediate points against `std::erf`-based computation.
8. **`MonotonicityInSigmaCpa`** — for fixed mean cpa < d_threshold, P is monotonically decreasing in σ_cpa (more uncertainty → closer to 0.5); for cpa > d_threshold, P is monotonically increasing in σ_cpa.

### Integration (`tests/collision/test_cpa_synthesize_own_ship.cpp`)

9. **`SynthesizeOwnShipPlacesPoseAtCorrectEnu`** — feed a known lat/lon, datum at same point → state(0..1) within 1 mm of zero; state(2..3) = supplied velocity.
10. **`SynthesizeOwnShipCovarianceMatchesSigmaPos`** — supplied σ_pos = 5 m → state covariance has 25 on the position diagonal, 0 elsewhere.

### Scenario (`tests/scenario/test_cpa_scenario.cpp` or extend `tests/sim/`)

11. **`CpaInsideConfidenceBandOnKnownScenario`** — simple scenario where truth CPA is computable analytically (e.g., perpendicular pass, own-ship stationary, target moving at known speed). After 10 tracker updates, predict CPA via `computeCpaWithUncertainty`; assert `|cpa_predicted − cpa_truth| < 2·σ_cpa` (95 % band).

### Eval-log

Append "CPA uncertainty (2026-06-03)" section. For each of the three §14.9 sweep scenarios (ClutterCrossing, BearingOnlyMoving, Maneuvering), report at the scenario midpoint and final timestep:
- Mean CPA
- σ_cpa
- P(CPA < 200 m)
- σ_cpa contribution from σ_heading R-inflation and σ_GPS R-inflation (re-run with each off to show the delta)

Connects the past three sessions' work to a single operational number per scenario.

## 9. Files touched

- Modify: `core/collision/Cpa.hpp` (append `CpaPrediction` struct + `computeCpaWithUncertainty` decl).
- Modify: `core/collision/Cpa.cpp` (implement `computeCpaWithUncertainty`).
- Create: `core/collision/CpaOwnShip.hpp`, `core/collision/CpaOwnShip.cpp` (synthesizer helper).
- Create: `tests/collision/test_cpa_uncertainty.cpp`, `tests/collision/test_cpa_synthesize_own_ship.cpp`.
- Create or extend: `tests/scenario/test_cpa_scenario.cpp`.
- Modify: `CMakeLists.txt` (add new source + test files).
- Modify: `docs/algorithms/evaluation-log.md`.

## 10. Ways to improve / what to test next

1. **2D off-center Rician / chi distribution for P.** Replace the 1D-Gaussian approximation. Exact P from the 2D Gaussian over `p_cpa`. Needed near collision where the 1D approximation under-counts probability.
2. **Own-ship velocity uncertainty.** Parse NMEA RMC for SOG/COG (or finite-difference successive GGAs as the UereEstimator does). Add `velocity_enu_std_m_per_s` to `OwnShipPose`. Feed into `synthesizeOwnShipTrack`. Closes the σ_v_own = 0 simplification.
3. **Process-model extrapolation of P from `last_update` to `t_ref`.** `P_extrap = F P Fᵀ + Q (t_ref − last_update)`. Two-line addition; gives correct σ at longer horizons.
4. **Cross-track covariance (Approach 2 from GPS spec).** Σ gains off-diagonals when both tracks share own-ship GPS error. Tightens σ_cpa for relative geometry. Big architectural lift in EKF, but the CPA-uncertainty side is a one-line change once Σ is correct.
5. **IMM-aware propagation.** For tracks carrying mode probabilities, propagate each mode's `(state, cov)` separately and combine into a mixture distribution. Standard moment-matching gives mean + σ; the full mixture supports more accurate P queries.
6. **Sigma-point / unscented propagation.** If linearisation drift becomes operationally noticeable, switch to 2n+1 sigma points. The output shape stays the same.
7. **Target-vs-target CPA at scale.** The API already supports it; add a `CpaEvaluator` that runs over all track pairs each tick and exposes a `top-N` collision-risk ranking.

## 11. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Propagation method | Linear / Jacobian | Closed-form, fast, sufficient for typical operating geometry. |
| Output | mean, σ on CPA and TCPA, plus P(<d_threshold) | Diagnostics + alarm logic; matches operational consumers. |
| Own-ship integration | Synthesise a Track | Uniform pair-CPA primitive; reuses everything. |
| Own-ship velocity σ | 0 in v1 | SOG GPS noise is small relative to target tracker σ; clean follow-up. |
| Track-to-track Σ | Independent (blockdiag) | Approach 2 (cross-target) deferred; matches GPS spec scope. |
| P approximation | Φ((d − cpa) / σ_cpa) | Standard 1D-Gaussian; documented limitation near collision. |
| Singularities | Sentinel + explicit fallbacks (parallel, past-CPA, near-zero CPA) | Mirrors existing `computeCpa`'s branches. |
| API | New `computeCpaWithUncertainty` + `CpaPrediction` | Existing `computeCpa` / `CpaResult` unchanged. |
