# Heading Errors: SimBus Injection + Adapter R-Inflation Design

**Date:** 2026-06-02
**Author:** Andreas (with Claude)
**Related:** `2026-05-28-maritime-sensor-fusion-design.md` §14.9 (heading errors and gyro drift in own-ship pose), `2026-06-01-simulated-sensor-bus-design.md` §5.1 (OwnShipEmitter).

## 1. Goal

Wire §14.9 end to end:

1. **SimBus** can inject own-ship heading bias, linear drift, and white noise into the HDT NMEA stream.
2. **Adapters** (ARPA TTM, EO/IR) account for heading uncertainty in their measurement covariance, so the tracker correctly down-weights long-range relative bearings when own-ship heading is uncertain.
3. **Investigation** quantifies tracker degradation as a function of σ_heading, with and without R-inflation, across the four existing bus comparison scenarios.

In scope: σ_heading injection on own-ship; R-inflation in the two adapters that consume own-ship heading; a multi-seed sweep recorded in `evaluation-log.md`.

Out of scope: heading bias as a tracker state (§14.9 point 2), sensor orientation as a measurement attribute (§14.9 point 3), per-platform heading sources for multi-platform scenarios, magnetic-compass models or COG-based heading.

## 2. Motivating math

A 1° heading error on a target at 5 km range creates an 87 m cross-track position bias on the projected measurement — larger than typical nav-radar bearing noise. Without R-inflation, the tracker treats this projection as if its only uncertainty were the sensor's intrinsic σ_bearing (≈1°), so it weights long-range observations as overly informative and gets pulled around by a slowly-rotating bias.

The fix is to add the angular variance contributions: when own-ship heading has noise σ_h, the bearing reaching the tracker has total angular variance `σ_bearing_sensor² + σ_h²`. Two independent sources of angular uncertainty combine in quadrature.

## 3. Architecture

Single point of change for the math:

```
adapters/util/Projection.cpp::projectRangeBearingToEnu(
    range_m, bearing_true_rad,
    range_std_m,
    bearing_std_rad,
    sigma_heading_rad,        // NEW; defaulted to 0.0
    own_ship_pos_enu)
```

Internally, R becomes `diag(range_std², bearing_std² + σ_h²)` before the Jacobian transform; the rest of the projection math is unchanged. The Jacobian still maps from `(range, angle)` to `(east, north)`; the only difference is that the angle variance carries both sensor and heading contributions.

Two adapters consume this helper:

- **`ArpaAdapter` (TTM path with `bearing_units == "R"`)** — gains a `Config { heading_std_deg }` and passes it through. The TLL path (absolute lat/lon) is unaffected.
- **`EoIrAdapter`** — gains a `Config { heading_std_deg }` and passes it through. All its measurements go through `projectRangeBearingToEnu`.

`AisAdapter` is untouched (AIS lat/lon is absolute, not relative to own-ship heading). `OwnShipNmeaAdapter` is untouched (it parses the HDT it receives; whether that heading is biased is the sim's problem).

SIM side: `OwnShipEmitter` already has `heading_bias_deg` and `heading_drift_deg_per_s`. We add `heading_noise_std_deg` and draw a Gaussian per HDT tick.

## 4. Components

### 4.1 `OwnShipEmitter` — heading noise

`OwnShipEmitterConfig` gains:

```cpp
double heading_noise_std_deg{0.0};   // per-tick Gaussian on HDT
```

In `emit()`, the heading written into HDT becomes:

```
hdg = cfg.heading_true_deg
    + cfg.heading_bias_deg
    + cfg.heading_drift_deg_per_s * (t - t0)
    + N(0, cfg.heading_noise_std_deg)      // NEW
```

Wrap to [0, 360) as today. Use the same `std::mt19937` substream as the GPS-position noise (no new seed sub-stream needed — the emission count is deterministic for a given config, so adding one draw per tick stays reproducible).

### 4.2 `projectRangeBearingToEnu` — σ_h parameter

Signature change (default 0 for backwards compatibility — but only one helper, both call sites updated):

```cpp
PointAndCov2D projectRangeBearingToEnu(
    double range_m,
    double bearing_true_rad,
    double range_std_m,
    double bearing_std_rad,
    double sigma_heading_rad,             // NEW, default 0.0
    const Eigen::Vector2d& own_ship_pos_enu);
```

Implementation:

```cpp
const double bearing_var =
    bearing_std_rad * bearing_std_rad +
    sigma_heading_rad * sigma_heading_rad;
Eigen::Matrix2d R;
R << range_std_m * range_std_m, 0.0,
     0.0, bearing_var;
// rest unchanged: out.cov = J * R * J.transpose();
```

### 4.3 `ArpaAdapter` — heading_std_deg config

Constructor currently `ArpaAdapter(geo::Datum, OwnShipProvider&)`. Add a third parameter — a config struct — defaulted to `{}` so existing callers compile unchanged:

```cpp
struct ArpaAdapterConfig {
  double heading_std_deg{0.0};
};

ArpaAdapter(geo::Datum datum,
            OwnShipProvider& own_ship,
            ArpaAdapterConfig cfg = {});
```

In the TTM `R` branch, pass `cfg_.heading_std_deg * kDeg2Rad` to `projectRangeBearingToEnu`. TLL path unchanged (absolute lat/lon — no heading rotation).

### 4.4 `EoIrAdapter` — heading_std_deg config

Same pattern as ARPA:

```cpp
struct EoIrAdapterConfig {
  double heading_std_deg{0.0};
};

EoIrAdapter(geo::Datum datum,
            OwnShipProvider& own_ship,
            EoIrAdapterConfig cfg = {});
```

Pass `cfg_.heading_std_deg * kDeg2Rad` to `projectRangeBearingToEnu` in `ingest()`.

### 4.5 Investigation harness — `BusHeadingSweep`

A new test in `tests/sim/test_bus_heading_sweep.cpp`. There are **three** underlying scenario factories in `tests/sim/BusComparisonHelpers.hpp` — `runBusClutterCrossing`, `runBusManeuvering`, `runBusBearingOnlyMoving` (the JPDA/MHT comparisons share `runBusClutterCrossing`). We extend each to accept a new knob:

```cpp
struct HeadingSweepKnob {
  double sigma_heading_deg{0.0};      // per-tick white noise on HDT
  double bias_deg{0.0};               // constant offset on HDT
  double drift_deg_per_s{0.0};        // linear-in-time offset on HDT
  bool   r_inflation_on{false};       // pass σ_h through to adapter cfg
};
```

so the same scenario can be instantiated with or without injected errors and with or without adapter R-inflation.

For the sweep, pick **one canonical tracker per scenario** (EKF+GNN for the crossings and maneuvering, EKF+GNN for bearing-only — match the "winning" choices from the post-fix bus pass) and vary only the knob. The intent is to characterise *the error model*, not to re-run the algorithm bake-off.

- σ_h ∈ {0°, 0.5°, 1°, 2°}. Bias and drift held at 0 for the sweep. 20 seeds (201..220).
- Three scenarios × 4 σ_h levels × 2 inflation settings = **24 cells**.
- **Headline experiment**: `runBusBearingOnlyMoving` — target at 1.5 km, so 1° heading error → ~26 m cross-track. The crossing scenarios (targets at ~200 m) and maneuvering (similar range) are largely insensitive; they serve as "doesn't hurt at short range" sanity checks.

Output: per-cell `mean_ospa ± σ` and `mean_id_sw`, printed to stderr in the same format as the existing bus comparisons. The eval-log section is hand-written from these prints; no automated table generation.

A separate, smaller test exercises bias-only (σ=0, bias ∈ {0°, 1°}) and drift-only (σ=0, drift ∈ {0, 0.01 deg/s}) to confirm those mechanisms are wired — no sweep, single seed, single scenario, just a "yes the bias propagates" check.

Soft assertions only — the goal is to record numbers, not gate on a verdict.

## 5. Data flow (end to end)

```
OwnShipEmitter.emit(t)
  hdg_noisy = nominal + bias + drift·(t-t0) + N(0, σ_h)
  encode $GPHDT,hdg_noisy
  OwnShipNmeaAdapter.ingest()
  OwnShipProvider holds heading_true_deg = hdg_noisy

ArpaEmitter.emit(t)        EoIrEmitter.emit(t)
  encode $RATTM (relative)   encode CameraDetection (relative)
  ArpaAdapter.ingest()       EoIrAdapter.ingest()
    own = provider.latest()    own = provider.latest()
    bearing_true = bearing_rel + own.heading_true_deg   ← carries bias
    projectRangeBearingToEnu(
        range, bearing_true,
        sensor_σ_range,
        sensor_σ_bearing,
        σ_heading,             ← from adapter cfg
        own_xy)
      bearing_var = σ_bearing² + σ_heading²
      R = diag(σ_range², bearing_var)
      cov_enu = J·R·Jᵀ
```

The error and the inflation use the **same** σ_h value when wired correctly. If they disagree (sim injects 1°, adapter is told 0.5°), the tracker is over-confident; that's the failure mode the sweep is designed to expose, even if it's not the headline experiment.

## 6. Assumptions

- The HDT-derived heading is the *only* heading source the tracker sees. We don't model a separate COG channel that could be used to detect bias.
- Heading noise is white (per-tick i.i.d. Gaussian). No process model (no Wiener walk, no Markov dynamics). White noise is the conservative choice for the R-inflation experiment because it shows up immediately in measurement statistics.
- Bias and drift are exact (no parameter noise). The sweep concentrates on σ because that's the term R-inflation is designed to absorb.
- The two adapters that consume heading don't share a `σ_h` source — each takes its own config. Operationally this means the sweep configures the same value into both, but the architecture allows divergence.

## 7. Rationale

- **Why one helper change for the math.** Both bearing-consuming adapters already use `projectRangeBearingToEnu`. Adding the parameter there centralises the "angles sum in quadrature" rule. The alternative — duplicating the inflation in each adapter — would risk drift.
- **Why default σ_h = 0.** All existing tests and callers continue to work without change. Only callers that configure σ_h see new behavior. This is critical: we have 216 tests passing today; this work shouldn't make any of them fail.
- **Why per-adapter config, not a global "own-ship heading uncertainty" source.** §14.9's note "a way to pass σ_heading from the adapter config" matches the existing pattern (each adapter owns its noise parameters). Threading it through `OwnShipPose` would require touching the NMEA parser, the provider, and every adapter — overkill for the same end result.
- **Why white noise instead of a bias process model.** A Wiener-walk gyro bias is more realistic but introduces a time-correlated state that's harder to reason about. The sweep wants a clean, reproducible answer to "how much does the metric degrade with σ_h?" — that's what white noise gives.
- **Why the four bus scenarios.** They span (close-range crossing with clutter, maneuvering, long-range bearing-only, hard data association under clutter). The bearing-only one in particular has 1500 m range, so 1° heading noise → ~26 m cross-track — large enough to expose the inflation effect cleanly.
- **Why one combined test file.** The four scenarios share helpers, the sweep loop is mechanical, and the eval-log entry is one narrative. Splitting per-scenario would multiply boilerplate.

## 8. What to test next (follow-ups)

- **Heading bias as a tracker state** (§14.9 point 2). Estimate gyro bias online from comparison of dead-reckoned vs GPS-COG / fix-derived heading. A one-state extension of the tracker (or a sibling own-ship navigation filter that feeds corrected heading downstream).
- **Coloured-noise gyro model.** Replace per-tick white noise with a Wiener walk or first-order Markov for σ_h, and confirm the white-noise R-inflation still helps (or doesn't).
- **Sensor orientation as a measurement attribute** (§14.9 point 3 / §14.1 follow-up). Pass the sensor's pointing direction at measurement time rather than assuming it equals own-ship heading.
- **Heading errors interacting with §14.5** (close-range precision sensors). At short range, heading error contributes little cross-track; the R-inflation correctly auto-scales by range, which is the right behavior.
- **Multi-platform** (each platform has its own gyro/heading), which depends on §14.1 multi-platform follow-ups.

## 9. Testing strategy

- **Unit tests for `projectRangeBearingToEnu`.** Two existing parameter combinations stay; one new test with `sigma_heading_rad > 0` confirms cross-track variance grows by exactly `(range·σ_h)²` (compare with σ_h=0 cov and check the off-diagonal pattern is preserved).
- **Unit tests for `OwnShipEmitter`.** Existing bias/drift tests + a new noise test that verifies the parsed heading mean ≈ nominal+bias and stddev ≈ σ_h over N samples.
- **Unit tests for `ArpaAdapter` / `EoIrAdapter`.** A "covariance grows when heading_std_deg > 0" test for each.
- **Scenario sweep** as described in §4.5.

## 10. Decision table

| Decision | Choice | Rationale |
|---|---|---|
| Where the angular variance combines | In `projectRangeBearingToEnu` | Single helper, both bearing-consuming adapters route through it |
| σ_h source | Per-adapter config | Matches existing noise-parameter pattern; no plumbing across OwnShip→adapter boundary |
| Sweep variable | σ_h (white noise) | Cleanest exposure of R-inflation effect; bias/drift get a smaller separate probe |
| Sweep levels | {0°, 0.5°, 1°, 2°} | Spans "neglected" to "obviously broken" gyro; 4 levels keeps the table readable |
| Scenarios used | Three bus scenario factories (canonical tracker per scenario) | Max reuse; bearing-only at 1.5 km is the headline experiment, crossings serve as short-range sanity checks |
| R-inflation enabled flag | Per-cell in the sweep | Each cell has both R-off and R-on so the eval-log table directly answers "did the fix help?" |
| Default σ_h in configs | 0.0 | All existing tests continue to pass without modification |

## 11. Open question (resolved during brainstorming)

> Does heading error affect range sensors?

Range alone — a scalar distance to the target — is **unaffected** by heading error. Heading rotation changes direction, not distance. But for sensors that produce *range + bearing* (ARPA TTM, EO/IR with range), the bearing carries the heading error and propagates to a cross-track position error when projected to ENU. The R-inflation goes on the bearing variance, never on the range variance. Math invariant: `σ_angle_total² = σ_bearing_sensor² + σ_heading²`.
