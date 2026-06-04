# RMC Parsing + Own-Ship Velocity — Design

**Date:** 2026-06-04
**Status:** Approved, ready for plan

## 1. Motivation

The CPA uncertainty work shipped with `σ_v_own = 0` as a documented v1 simplification — caller supplies own-ship velocity, no uncertainty on it. This spec closes that. NMEA RMC carries SOG/COG (Speed-Over-Ground, Course-Over-Ground) at the receiver; parsing it gives a direct authoritative velocity. As a fallback when only GGA is being parsed, a small GGA-finite-difference estimator runs alongside and fills in.

This also tightens `synthesizeOwnShipTrack` — its explicit `velocity_enu` and `sigma_pos_m` arguments collapse into reads from the pose. Downstream CPA σ now correctly incorporates own-ship velocity uncertainty through the existing Jacobian.

## 2. Scope

In scope:
- `OwnShipPose.velocity_enu`, `velocity_std_m_per_s`, `velocity_is_valid` fields.
- `OwnShipNmeaAdapter` parses RMC; extracts SOG/COG; populates pose velocity fields with σ_v derived from configurable σ_SOG and σ_COG.
- New `OwnShipVelocityEstimator` (sliding-window LS on successive GGAs, mirrors `UereEstimator`) as the GGA-derived fallback.
- Precedence: RMC wins when present and `prefer_rmc_velocity` is true; otherwise the GGA estimator's published value is used.
- `sim::OwnShipEmitter` learns to emit RMC sentences (it already has truth velocity); new `report_velocity{false}` flag for backward compat.
- `synthesizeOwnShipTrack` reads velocity and σ_v from pose; CPA σ now reflects own-ship velocity uncertainty.
- Validation: unit tests for RMC parse and the velocity estimator; integration tests confirming the precedence; CPA scenario test confirming σ_cpa changes with σ_v.
- Eval-log section.

Out of scope (deferred to §11):
- **Adaptive UERE consuming RMC velocity** as a second observer (would tighten σ_pos in some regimes). Documented as ways-to-improve.
- **Per-axis velocity σ** or 2×2 velocity covariance. Stays isotropic for v1.
- **Heading-rate** from RMC's secondary fields (NMEA doesn't standardize this; sentences like `$xxROT` are vendor-specific).
- **Magnetic variation** from RMC field 10. We use true bearings throughout; magnetic isn't needed.
- **RMC date field** parsing. The library's `Timestamp` is set by the caller; we don't currently convert NMEA date/time.

## 3. Architecture

```
NMEA RMC ────parse────► velocity_enu, σ_v ──┐
                                            │
NMEA GGA + UereEstimator                    ▼
   │                                  OwnShipPose ──► OwnShipProvider
   ▼                                            
OwnShipVelocityEstimator ─publish?─►──┘ (fallback when RMC absent)

OwnShipPose.velocity_enu + velocity_std_m_per_s
   │
   ▼
synthesizeOwnShipTrack(pose, datum)
   │
   ▼
Track with state = [px, py, vx, vy]
            cov   = diag(σ²_p, σ²_p, σ²_v, σ²_v)
   │
   ▼
computeCpaWithUncertainty(...)   ← Jacobian now propagates σ_v
```

No new ports. `OwnShipVelocityEstimator` is a small core component (sibling of `UereEstimator`). `OwnShipNmeaAdapter` owns it as a private member.

## 4. Math

### 4.1 RMC → ENU velocity

NMEA RMC SOG/COG to ENU vector (using compass-style COG: 0° = north, clockwise):

```
sog_m_per_s = sog_knots × 0.514444
cog_rad     = cog_deg × π/180
v_east   = sog_m_per_s × sin(cog_rad)
v_north  = sog_m_per_s × cos(cog_rad)
velocity_enu = [v_east, v_north]
```

### 4.2 RMC σ_v derivation

Scalar isotropic σ_v from configured σ_SOG and σ_COG_deg:

```
σ_v = √(σ²_SOG + (SOG · σ_COG_rad)²)
```

At rest (SOG = 0), σ_v collapses to σ_SOG — the COG-derived term vanishes, matching the physical intuition that bearing uncertainty doesn't matter when not moving.

### 4.3 GGA-derived velocity (fallback)

Same sliding-window LS pattern as `UereEstimator` (commit `b87925b`), reusing the same fit machinery:

- Window of N successive ENU positions `(t_i, x_i, y_i)`.
- LS fit `p_x(t) = a_x + v_x · t`, `p_y(t) = a_y + v_y · t`.
- `velocity_enu = [v_x, v_y]`.
- σ_v_per_axis from the LS slope standard error: `σ_v_x = σ_pos / √Σ(t_i − t̄)²` (same formula as the UereEstimator's gate already uses internally).
- Isotropic σ_v: `σ_v = √(σ²_v_x + σ²_v_y) / √2 ≈ σ_v_axis` (the per-axis σs are equal when residuals are isotropic).

The estimator is suppressed during maneuvers using the same two-halves velocity-difference gate as UereEstimator. (In fact the velocity that *triggers* the maneuver gate IS the slope we want — we can reuse the half-fits internally.)

### 4.4 Precedence

Per cycle, `OwnShipNmeaAdapter` produces a pose with velocity from:

```
if cfg.prefer_rmc_velocity AND rmc_velocity_seen_within_T_rmc_stale_s:
    velocity_enu, velocity_std = rmc_buffer
else if uere_estimator.current_velocity().is_published:
    velocity_enu, velocity_std = uere-estimator-derived
else:
    velocity_enu = (0, 0), velocity_std = 0, velocity_is_valid = false
```

`T_rmc_stale_s` defaults 5 s — if the RMC stream drops longer than that, fall to the GGA-derived path.

## 5. Configuration

### 5.1 `OwnShipPose` extension

```cpp
struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};   // NEW
  double velocity_std_m_per_s{0.0};                        // NEW
  bool velocity_is_valid{false};                           // NEW
};
```

Backward compatible — `velocity_is_valid` defaults `false` so existing tests that don't care see no behavior change.

### 5.2 `OwnShipNmeaAdapterConfig` extension

```cpp
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
  bool enable_adaptive_uere{false};
  UereEstimatorConfig uere_estimator_cfg{};

  // RMC velocity parsing.
  double sigma_sog_m_per_s{0.5};
  double sigma_cog_deg{1.0};
  bool prefer_rmc_velocity{true};
  double rmc_stale_seconds{5.0};

  // GGA-derived velocity estimator (fallback when RMC absent).
  bool enable_velocity_estimator{true};
  OwnShipVelocityEstimatorConfig velocity_estimator_cfg{};
};
```

Defaults chosen so the typical case (RMC present) works without operator tuning. Library users who set `OwnShipPose` directly bypass both paths.

### 5.3 `OwnShipVelocityEstimatorConfig`

```cpp
struct OwnShipVelocityEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  double min_sigma_v_m_per_s{0.05};
};
```

Mirrors `UereEstimatorConfig`. Some implementations may share the same window with `UereEstimator` — that's an implementer's call. Keeping them as separate classes preserves modularity; sharing the window is a follow-up if profiling shows duplicated work matters.

## 6. `synthesizeOwnShipTrack` signature change

Before (v1, CPA spec §5):
```cpp
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum);
```

After:
```cpp
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const geo::Datum& datum);
```

Reads `pose.position_std_m`, `pose.velocity_enu`, `pose.velocity_std_m_per_s`. Returns a Track with covariance:

```
covariance = diag(σ²_p, σ²_p, σ²_v, σ²_v)
```

When `pose.velocity_is_valid` is false, velocity contribution is zero (matches the v1 simplification). When true, the σ_v contribution flows through `computeCpaWithUncertainty`'s Jacobian into σ_cpa with no further changes.

## 7. `sim::OwnShipEmitter` extension

```cpp
struct OwnShipEmitterConfig {
  // ... existing fields ...
  bool report_velocity{false};           // NEW — default off; opt-in
  bool emit_rmc{true};                   // NEW — emit RMC sentences alongside GGA/HDT
};
```

When `report_velocity == true`:
- Sets `pose.velocity_enu` (computed from successive truth positions or directly from the trajectory).
- Sets `pose.velocity_std_m_per_s` to a small value (e.g., 0.1 m/s) representing the sim's truth-noise floor.
- Sets `pose.velocity_is_valid = true`.

When `emit_rmc == true` (default true so the RMC path is exercised by sim tests automatically):
- Composes a `$GPRMC` sentence per cycle with the truth SOG/COG plus configured noise.
- Pushes through the adapter alongside the GGA.

The default `report_velocity = false` preserves byte compat with existing tests (same pattern as `report_gps_std`).

## 8. Assumptions

- Receiver-reported SOG/COG accuracy is well-modeled by isotropic Gaussian with σ_SOG and σ_COG. True for civilian GPS in steady cruise; degrades at very low speed (COG becomes noisy and σ_COG should be bumped).
- RMC is broadcast at ≥ 0.2 Hz (typical: 1 Hz). 5-second stale window is generous.
- GGA-derived velocity is correct when own-ship motion is approximately constant-velocity over the window. Maneuver gating handles the non-cruise case.
- The two velocity-derivation paths agree to within their σ envelopes when both are active.

## 9. Rationale

**Why parse RMC instead of just relying on GGA-derivative?** RMC's SOG is computed at the GPS receiver from Doppler shift — vastly tighter than position differencing, especially at low speeds. Σ_SOG of 0.1 m/s for a civilian receiver compares to ~σ_pos × √2 / dt ≈ 7 m/s noise floor from 5 m GGA at 1 Hz finite-diff. Where RMC is available, prefer it; where it isn't, derive it.

**Why scalar isotropic σ_v?** Matches the rest of the work (σ_pos is scalar; σ_heading is scalar). Composes cleanly with CPA's Jacobian. Per-axis or full 2×2 covariance is a non-breaking future addition.

**Why `velocity_is_valid` instead of `std::optional`?** Keeps `OwnShipPose` a plain trivially-copyable struct (assuming no Eigen issues, which there aren't for fixed-size types). Library users explicitly opt in by setting the flag. Easier to default-construct than handling optional everywhere.

**Why default `prefer_rmc_velocity = true` and `enable_velocity_estimator = true`?** Most receivers broadcast RMC; getting velocity by default makes the library more useful out of the box. The estimator runs as a free safety net at trivial cost. Consumers that don't want either path explicitly disable them.

**Why does sim's `report_velocity` default `false`?** Backward compat. Existing tests built their assertions on a system that didn't model velocity; reporting it suddenly would shift σ_cpa numbers and possibly flip statistical comparisons (we saw exactly this with `report_gps_std` defaulting on, commit `ce61b65`). New tests opt in.

**Why drop `synthesizeOwnShipTrack`'s explicit args?** With velocity on the pose, the caller doesn't need to provide it separately — that was always a v1 shim. Removing the args clarifies that the pose is the authoritative source.

**Why no peer signal between `UereEstimator` and `OwnShipVelocityEstimator`?** They observe the same residuals (the same GGA stream produces both σ_pos and σ_v estimates). A joint estimator would be more efficient but requires careful joint distribution modeling. v1 keeps them separate; the cost is one extra window worth of state.

## 10. Test plan

### Unit

1. **`tests/adapters/own_ship/test_own_ship_nmea.cpp`** — extend:
   - `ParsesRmcSogCogIntoVelocityEnu`: GPRMC with SOG=10 knots, COG=045° → `velocity_enu ≈ (3.64, 3.64) m/s`. Use the standard 0.514444 conversion factor.
   - `ParsesRmcSigmaVFromConfig`: cfg.sigma_sog=0.5, cfg.sigma_cog=1° → `σ_v = √(0.25 + (5.14 · 0.017)²) ≈ 0.51 m/s` at SOG=10 knots.
   - `RmcZeroSogProducesSigmaSogAsSigmaV`: SOG=0 → σ_v = σ_SOG exactly.
   - `RmcAbsentTriggersEstimatorFallback`: only GGA fed → velocity_is_valid becomes true once estimator publishes; before then, velocity_is_valid = false.
   - `RmcStaleTriggersEstimatorFallback`: RMC fed once at t=0, then only GGA for 10 s → after 5 s the RMC is stale, velocity comes from estimator.

2. **`tests/own_ship/test_own_ship_velocity_estimator.cpp`** — new:
   - `UnpublishedWhenWindowEmpty`: fresh estimator → no published velocity.
   - `ConvergesOnConstantVelocityInput`: feed N=8 samples at v=(5, 3) m/s with σ=1 m noise; verify `|v_published - (5, 3)| < 1 m/s`.
   - `SuppressesDuringManeuver`: step velocity change; estimator does not publish for that window (same pattern as UereEstimator).
   - `SigmaVTracksNoise`: parametric σ_pos in {0.5, 2, 5} m → σ_v scales as ~σ_pos / √Σ(dt²).
   - `ResumesAfterManeuver`: maneuver + N clean samples → publishes again.

3. **`tests/sim/test_own_ship_emitter.cpp`** — extend:
   - `EmittedPoseCarriesVelocityWhenReportOn`: configure `report_velocity = true` → pose carries non-zero velocity_enu and velocity_is_valid = true.
   - `EmittedRmcParsesBackToVelocity`: emit RMC into adapter, verify the round-trip produces the same velocity (or close to it given parse / noise).
   - `DefaultDoesNotReportVelocity`: report_velocity = false (default) → velocity_is_valid stays false. Backward-compat regression guard.

4. **`tests/collision/test_cpa_synthesize_own_ship.cpp`** — modify:
   - Existing tests change to construct `OwnShipPose` with velocity fields and call new signature.
   - Add `SynthesizedTrackCarriesVelocityCovariance`: pose with σ_v = 1 → resulting Track covariance has 1 on the velocity diagonal.
   - Add `InvalidVelocityProducesZeroVelocityCovariance`: pose with velocity_is_valid = false → Track covariance has 0 on velocity diagonal (matches v1 behavior).

### Integration (scenario)

5. **`tests/scenario/test_cpa_scenario.cpp`** — extend the perpendicular-pass:
   - Two test variants: own-ship pose with σ_v = 0 (matches existing) vs σ_v = 1 m/s. Assert σ_cpa grows by a measurable amount in the second variant.

### Eval-log

Append "RMC velocity + CPA σ (2026-06-04)" section. Re-run the §14.9 sweep cells but with `report_velocity = true` and `σ_v` injected; report mean σ_cpa with and without σ_v contribution. Show that σ_cpa grows by `O(σ_v · TCPA)` as expected — and that CPA mean stays the same (velocity uncertainty doesn't bias the prediction, only widens its uncertainty band).

## 11. Ways to improve / what to test next

1. **Adaptive UERE consumes RMC velocity as a second observer.** Current adaptive UERE uses only motion-consistency (GGA-derived). Adding RMC velocity as a cross-check gives a second independent signal — composes the Allan-variance-style estimate. ~1 day.
2. **Per-axis σ_v or full 2×2 velocity covariance.** SOG and COG noise translate to anisotropic σ_v_east, σ_v_north (especially when COG is near 0° or 90°). Promote `velocity_std_m_per_s` → `Eigen::Matrix2d`. Composes through CPA Jacobian.
3. **Heading-rate / yaw-rate parsing.** Some receivers emit `$xxROT` (rate of turn) — could populate `heading_rate_deg_per_s` for tracker process models that benefit (coordinated-turn, prescribed-turn IMM modes).
4. **Joint velocity / position estimator.** Today `UereEstimator` and `OwnShipVelocityEstimator` are sibling components observing the same residuals. A joint formulation halves the state and is slightly more efficient.
5. **Time / date from RMC.** Wire NMEA time/date through to `Timestamp` for systems where the caller doesn't have wall-clock time available. Requires defining an epoch convention.
6. **Magnetic-variation parsing.** Some downstream consumers want magnetic bearings; the field is in RMC. Trivial to expose if requested.

## 12. Decision summary

| Decision | Choice | Why (one line) |
|---|---|---|
| Velocity σ representation | Scalar isotropic | Matches existing scalar σ_p; clean Jacobian composition. |
| Primary source | RMC SOG/COG | Doppler-derived at receiver; tight σ at low speeds. |
| Fallback | GGA finite-difference (`OwnShipVelocityEstimator`) | Always available; mirrors UereEstimator's pattern. |
| Precedence | RMC > estimator, with 5 s RMC-stale window | Use the better signal when fresh; degrade gracefully. |
| `synthesizeOwnShipTrack` signature | Drop explicit velocity/σ args | Pose is authoritative; v1 shim removed. |
| Sim emitter | `report_velocity{false}` default | Backward compat with existing tests. |
| RMC defaults | σ_SOG=0.5 m/s, σ_COG=1° | Pessimistic vs. typical civilian GPS spec. |
