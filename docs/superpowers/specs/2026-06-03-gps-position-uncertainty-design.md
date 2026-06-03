# GPS Position Uncertainty (Own-Ship) — Design

**Date:** 2026-06-03
**Status:** Approved, ready for plan

## 1. Motivation

Own-ship GPS noise enters every bearing-projected measurement: the projection adds `p_own_measured` to the relative range/bearing, so any error in `p_own_measured` propagates directly into the projected target ENU position. Today's tracker does not budget for it. The sim already injects GPS noise (`sim::OwnShipEmitter::gps_pos_std_m`, default 5 m), but adapters consume the noisy lat/lon as if exact — so the filter's R matrix is systematically too small at short range, where σ_GPS dominates bearing-cross-track error.

Scale check at typical ranges (σ_GPS = 5 m civilian, σ_β = 1°):

| Range | σ_β cross-track | σ_GPS | Variance share |
|---|---|---|---|
| 200 m  | 3.5 m | 5 m | **GPS dominant** |
| 500 m  | 8.7 m | 5 m | comparable |
| 1500 m | 26 m  | 5 m | GPS ~4 % |

GPS uncertainty matters most at **close range** — the opposite gradient from heading uncertainty, which dominated the §14.9 work. Together the two fixes cover the full operating envelope.

This spec adds the **per-target marginal correction**: each adapter-projected measurement gets `σ²_GPS · I` added to its covariance. The cross-target correlation that simultaneous measurements share `n_gps` is deliberately **not** modeled in v1; see §11.

## 2. Scope

In scope:
- New `position_std_m` field on `OwnShipPose`, propagated from both NMEA (HDOP × UERE) and sim (existing `gps_pos_std_m`).
- ARPA and EO/IR adapters read `pose.position_std_m` at projection time and pass it into `projectRangeBearingToEnu`, which adds `σ²_GPS · I` to the output covariance.
- Heading-bias estimator folds `(σ_GPS / r)²` into its measurement noise `σ²_v`, giving the bias state a realistic noise floor.
- `AisArpaPairExtractor` and `AisArpaPairObservation` carry the own-ship GPS std through.
- Validation: a close-range GPS sweep on ClutterCrossing plus a long-range sanity probe on BearingOnlyMoving.

Out of scope (deferred):
- **Cross-target correlation** of simultaneous own-ship error. Documented in §11.
- **State-augmented own-ship error** (Schmidt-Kalman / consider parameter).
- **GPS-jump coordinated update** when civilian GPS hops at differential handoff.
- **Per-axis GPS std** (lat vs lon, error-ellipse). Isotropic for v1.
- **Target-side AIS position uncertainty** in the AIS adapter beyond the existing fallback in the pair extractor.

## 3. Architecture

```
NMEA GGA → HDOP × UERE ───┐
                          ├──→ OwnShipPose.position_std_m ──┐
sim::OwnShipEmitter ──────┘                                 │
                                                            ▼
                              ArpaAdapter / EoIrAdapter at projection time:
                                projectRangeBearingToEnu(..., σ_gps=pose.position_std_m, ...)
                                                            │
                                                            ▼
                                   Measurement.covariance ← +σ²_gps · I
                                                            │
                                          ┌─────────────────┴─────────────────┐
                                          ▼                                   ▼
                                Tracker (consumes inflated R)   AisArpaPairExtractor
                                                                              │
                                                                              ▼
                                                                  HeadingBiasEstimator
                                                              folds (σ_gps / r)² into σ²_v
```

No new ports. No new tracker stages. Each touched file changes minimally.

## 4. Math

### 4.1 Projection covariance

`projectRangeBearingToEnu` currently composes:
```
J = [[sin β, r·cos β], [cos β, −r·sin β]]
R_meas = diag(σ²_r, σ²_β_eff)
cov_enu = J · R_meas · Jᵀ
```
where `σ²_β_eff = σ²_β + σ²_heading` from the §14.9 work. With this spec:
```
cov_enu = J · R_meas · Jᵀ + σ²_gps · I_2
```
The added term is range-independent — exactly the desired behavior since GPS error translates the entire bearing arc rigidly.

### 4.2 Bias-estimator noise floor

The AIS-vs-ARPA residual is `z = β_arpa_enu − β_truth_from_ais`. The angular contribution from own-ship GPS error at the ARPA target's range `r` is approximately `σ_gps / r` (small-angle). It enters σ²_v additively:
```
σ²_v = σ²_β_arpa + (σ_ais_pos / r_ais)² + (σ_own_gps / r_arpa)²
```
The AIS-position term already exists (Task 7 of the heading-bias work); the new third term is the own-ship GPS floor.

### 4.3 HDOP-to-σ conversion

NMEA GGA reports HDOP (Horizontal Dilution of Precision). The standard conversion is:
```
σ_pos = HDOP × UERE
```
where UERE (User Equivalent Range Error) is configured per adapter. Defaults: 5 m for civilian GPS, configurable down to 0.5–1 m for DGPS and 0.1 m for RTK. If HDOP is absent or invalid, `position_std_m` stays at its previous value or 0.

## 5. Configuration

### 5.1 `OwnShipPose` extension

```cpp
struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};   // NEW — isotropic 2D σ on ENU position
};
```

Backward compatible — default 0.0 means no R-inflation, identical to current behavior.

### 5.2 `OwnShipNmeaAdapterConfig`

```cpp
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};   // UERE used to convert HDOP → σ_pos
};
```
GGA parsing extracts HDOP from field 8 (per NMEA 0183) and sets `pose.position_std_m = hdop × cfg.uere_m`. If HDOP is missing or non-positive, the field defaults to 0.

### 5.3 `sim::OwnShipEmitter`

The existing `cfg.gps_pos_std_m` field is reused. Each emitted pose's `position_std_m` is set to the same value before being pushed through the adapter — so sim parity is exact (the truth knows what σ it injected).

### 5.4 `projectRangeBearingToEnu` signature

```cpp
PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       double sigma_gps_pos_m,        // NEW
                                       const Eigen::Vector2d& own_ship_pos_enu);
```
The new parameter goes right after `sigma_heading_rad`, matching its style. Default 0.0 in tests that don't care.

## 6. Adapter integration

### 6.1 ARPA

In the TTM branch of `ArpaAdapter`, after computing `bearing_true_rad_corrected` and `sigma_heading_eff`:

```cpp
const double sigma_gps_pos = own_opt->position_std_m;  // from OwnShipPose
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad_corrected,
                             50.0, 1.0 * kDeg2Rad,
                             sigma_heading_eff,
                             sigma_gps_pos,
                             own_xy);
```

The TLL branch (which gives absolute target lat/lon) is *not* affected by own-ship GPS — TLL is independent of own-ship pose. Leave it unchanged.

### 6.2 EO/IR

Same pattern as ARPA. EOIR detections are always projected via own-ship, so all detections get the GPS inflation.

### 6.3 AIS

AIS reports absolute target position derived from the *target's* GPS, independent of own-ship GPS. Leave `AisAdapter` unchanged.

## 7. Bias-estimator coupling

`AisArpaPairObservation` gains one field:

```cpp
struct AisArpaPairObservation {
  ...                                  // existing fields
  double own_position_std_m{0.0};      // NEW — own-ship GPS σ at observation time
};
```

`AisArpaPairExtractor` reads `recent_contributions` per track. The ARPA touch's `sensor_position_enu` is already the own-ship's reported ENU position; we need its uncertainty too. Two options:

**Chosen:** Extend `Track::SourceTouch` with an `own_position_std_m` field (default 0.0) populated by `Tracker::process` and `processBatch` from `z.sensor_position_std_m` — which itself is a new (optional) field on `Measurement` populated by the adapter when projecting. This carries the floor cleanly through the existing provenance pipeline.

**Alternative considered:** Pass the current OwnShipPose into `extractPairs(...)` separately. Rejected — pairs may be extracted at a cycle when own-ship has moved/changed, and we want the σ_GPS that was actually applied at the ARPA touch's time.

`HeadingBiasEstimator::observe` adds:
```cpp
const double sigma_own_gps_at_r = obs.own_position_std_m / obs.r_arpa;  // small-angle
σ²_v += sigma_own_gps_at_r * sigma_own_gps_at_r;
```
`r_arpa` is `(arpa_target_position_enu - own_position_enu).norm()` — already computed.

## 8. Assumptions

- GPS noise is approximately isotropic in the local ENU plane. Real HDOP collapses lat/lon error to a scalar, so this matches what the receiver tells us anyway.
- GPS noise at one cycle is **independent of measurement noise** (bearing/range) within the same cycle. True at the receiver level.
- GPS noise across **different cycles** is independent. Approximately true for white receiver noise; not true for multipath persistence — accepted limitation for v1.
- UERE is roughly constant for a given operating environment. Sufficient for v1; future work could make UERE adaptive.
- HDOP from NMEA GGA is trustworthy when present. We don't double-check by cross-validating from satellite geometry.

## 9. Rationale

**Why on `OwnShipPose` and not adapter config?** Uncertainty is a physical property of the GPS fix, not the consuming adapter. The fix-time σ varies with HDOP, satellite visibility, and differential-correction availability — putting it on the pose lets it vary per fix. The adapter pattern (matching σ_heading) would require the operator to know σ in advance, which doesn't match how real GPS receivers behave.

**Why isotropic σ rather than 2×2 covariance?** HDOP collapses lat/lon error to a scalar at the receiver. The full 2×2 error ellipse depends on satellite geometry, which most consumer-grade NMEA streams don't expose. Future work can promote to a 2×2 if a richer source (NMEA GST or proprietary) is available.

**Why add `(σ_GPS / r)²` to the bias estimator's σ²_v?** Without it, the estimator will believe its measurement noise is smaller than it actually is in poor-GPS conditions, leading to over-confident convergence on a wrong b̂. The fold-in is one line and gives the gating threshold a meaningful interpretation across GPS regimes.

**Why R-inflation (Approach 1) and not state-augmented (Approach 2)?** Approach 1 gives per-track marginal covariances that are exactly correct, which is what every current consumer of `Track` reads. Approach 2 captures the cross-target correlation that simultaneous measurements share `n_gps` — a real effect but invisible to today's downstream code. Worth doing only when relative-frame reasoning (multi-target CPA, fleet-level geometry) becomes a primary consumer. See §11 for the deferred path.

**Why leave TLL and AIS unchanged?** Neither projects through own-ship pose. TLL gives target lat/lon directly; AIS gives target absolute position from the target's GPS. Own-ship GPS error doesn't enter either.

## 10. Test plan

### Unit (existing files extended)

1. **`tests/adapters/util/test_projection.cpp`** — extend with two tests:
   - `GpsStdAddsToCovariance`: σ_GPS = 5 m, σ_β = 0, σ_r = 0 → output covariance equals `σ²_GPS · I` exactly.
   - `GpsStdComposesWithExistingNoise`: σ_GPS = 3 m, normal range/bearing noise → output covariance is the sum of the existing projection covariance and `σ²_GPS · I`.

2. **`tests/adapters/own_ship/test_own_ship_nmea.cpp`** — extend:
   - `ParsesHdopFromGga`: feed a GGA sentence with HDOP = 1.2, UERE = 5 → `pose.position_std_m == 6.0`.
   - `AbsentHdopLeavesStdAtZero`: GGA without HDOP field → `pose.position_std_m == 0`.

3. **`tests/sim/test_own_ship_emitter.cpp`** — extend:
   - `EmittedPoseCarriesGpsStd`: configure `gps_pos_std_m = 5` → emitted `pose.position_std_m == 5`.

4. **`tests/adapters/arpa/test_arpa_adapter.cpp`** — extend:
   - `InflatesCovarianceFromOwnShipGpsStd`: set `pose.position_std_m = 5`, feed TTM → output covariance has +25·I vs the baseline (with `position_std_m == 0`).
   - `TllUnaffectedByGpsStd`: same `pose.position_std_m = 5`, feed TLL → output covariance unchanged from baseline.

5. **`tests/adapters/eoir/test_eoir_adapter.cpp`** — extend with the same `InflatesCovarianceFromOwnShipGpsStd` test as ARPA.

6. **`tests/bias/test_heading_bias_estimator.cpp`** — extend with one test:
   - `GpsFloorPreventsOverConvergence`: feed pairs where `own_position_std_m / r ≈ 0.5°` → asymptotic `P_b` plateaus near `(0.5°)²` rather than collapsing to the AIS-σ-only floor.

7. **`tests/bias/test_ais_arpa_pair_extractor.cpp`** — extend:
   - `PropagatesOwnPositionStdFromTouch`: SourceTouch with `own_position_std_m = 3` → emitted observation carries `own_position_std_m == 3`.

### Scenario sweep (`tests/sim/test_bus_gps_sweep.cpp`)

ClutterCrossing at four σ_GPS ∈ {0, 0.1, 1, 5} m × 20 seeds. Three rows per σ_GPS: (σ_GPS injected, R-inflation off), (σ_GPS injected, R-inflation on), and an estimator-on row for completeness. Compare per-window OSPA and id-switch counts. Expected: row-2 vs row-1 shows clear OSPA improvement once σ_GPS ≥ 1 m, because R-on no longer over-trusts close-range projections.

### Long-range sanity probe (`tests/sim/test_bus_gps_sweep.cpp`)

BearingOnlyMoving at the same four σ_GPS levels. Expected: near-noise effect; documents that GPS inflation correctly does little at long range, complementing the §14.9 result.

### Eval-log

Append "GPS position uncertainty (2026-06-03)" section with the close-range table, the long-range probe, and a verdict paragraph summarizing the close-range improvement and confirming the lack of effect at long range.

## 11. Ways to improve / what to test next

1. **Cross-target correlation (Approach 2).** Today every simultaneous projected target is treated as having independent GPS error. The truth is they share `n_gps`. Three paths:
   - (a) **Schmidt-Kalman own-ship error**: augment each track's state with a 2-dim own-ship error term as a consider parameter — never updated but propagates correlation. Manageable change to `EkfEstimator`.
   - (b) **Joint-update EKF**: a single joint update across all active tracks each cycle. Cleanest math, biggest architectural change. Breaks per-track independence.
   - (c) **Coordinated-jump heuristic**: detect simultaneous innovations with similar direction across all tracks, attribute to own-ship, apply a shared offset. Cheap; captures GPS-jump events but no general math.

   Worth doing once relative-frame consumers (multi-target CPA, fleet-level geometry) land.

2. **Per-axis GPS std.** Promote `position_std_m` to a 2×2 covariance when NMEA GST (or proprietary error-ellipse) is available. Useful in environments with strong sky obstruction (urban canyons, dockside).

3. **Adaptive UERE.** Track innovation residuals against own-ship to estimate UERE online. Useful when GPS quality varies through the run.

4. **Target-side AIS GPS uncertainty.** AIS Class A reports include position accuracy bits; today we hard-code `pos_std_m = 10`. Extending `AisAdapter` to read the report's accuracy bits would tighten the bias-estimator's σ²_v in good-AIS scenes.

5. **Multipath persistence.** Real GPS noise has temporal correlation in multipath environments. Model `n_gps` as a Gauss-Markov process and add it to the own-ship state (related to §11.1 path a).

## 12. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Approach | R-inflation, Approach 1 | Fixes per-track marginal covariance; cheap; doesn't touch the EKF. |
| Config location | `OwnShipPose.position_std_m` | Uncertainty travels with the fix; varies per HDOP. |
| HDOP conversion | `σ = HDOP × UERE`, UERE per-adapter config (default 5 m) | Standard NMEA conversion; one knob the operator might tune. |
| Application point | Inside `projectRangeBearingToEnu` (+σ²·I on output) | One math change covers ARPA TTM and EO/IR uniformly. |
| TLL / AIS | Unchanged | Neither projects through own-ship; GPS error doesn't enter. |
| Bias-estimator coupling | Add `(σ_GPS / r)²` to σ²_v | Realistic noise floor; one line; preserves gating semantics. |
| Cross-target correlation | Deferred (Approach 2) | No relative-frame consumer today; large architectural lift. |
| Per-axis σ | Deferred | NMEA GGA gives only HDOP; not worth a 2×2 yet. |
