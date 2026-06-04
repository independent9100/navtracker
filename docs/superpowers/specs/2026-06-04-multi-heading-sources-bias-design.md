# Multi-Heading-Source Bias Observations Design

**Date:** 2026-06-04
**Status:** Approved
**Predecessors:**
- `2026-06-03-heading-bias-estimator-design.md` (v1: AIS↔ARPA pair observation)
- `2026-06-04-multi-track-bearing-bias-observer-design.md` (v2: bearing innovations from Tracker)

This spec adds v3: observations of the gyrocompass bias derived from *other* heading sources — GPS true-heading (multi-antenna), GPS course-over-ground, and magnetic compass. Each source is independent and optional; the estimator must work for any subset of the 2³ = 8 combinations of (none, any, all) of these sources being present, including hot-swap during a mission.

## 1. Problem

The v2 observer proved that bearing-only scenes without *any* anchor are structurally unobservable (the EKF state absorbs the bias). v2 closed the gap whenever any non-bearing-affected positioning source was present, but it still needs *something*. In open-sea transits with no targets at all (no AIS, no radar contacts), neither v1 nor v2 can observe `b`.

But the own-ship platform itself usually has 1–3 additional heading sources beyond the gyrocompass:

- **GPS multi-antenna heading** (often via NMEA `HDT` from a GPS talker, or via a non-NMEA proprietary path): truly tied to vessel orientation, no drift, no crab. Gold standard but not universally fitted.
- **GPS course-over-ground (COG)** from `RMC`: ubiquitous (any GPS produces it). Tied to *velocity vector*, so equals heading only with zero crab — which fails in current, wind, low SOG, and turning.
- **Magnetic compass heading** from `HDG`: ubiquitous on most vessels. Tied to magnetic north, requires correction for variation (geographic, well-known) and deviation (per-vessel, often uncalibrated).

Each source provides a candidate observation of `b`. Their availability and quality vary across fleets, sea states, and lifetime of a mission. The system must handle any subset, gracefully degrade when sources disappear, and not over-trust any one source.

## 2. Architecture

Same single-fused-estimator path (A) the v2 work established. `HeadingBiasEstimator` grows three new `observe()` overloads, one per source. Each overload performs a scalar KF update against the same hidden state `b`. No new state variables; no port-level breakage.

```
                                            +----------------------+
       (NMEA / proprietary feed)            |                      |
                                            |                      |
   RMC ───────►  GpsCogAdapter ─────────►   |  HeadingBiasEstimator|
   HDG ───────►  MagneticAdapter ──────►    |  - observe(AisPair)  |---> IHeadingBiasProvider
   HDT (GP*) ─►  GpsHeadingAdapter ────►    |  - observe(BI)       |
                                            |  - observe(GpsCog)   |
   ARPA/EOIR ───► Tracker ─BI─────────►     |  - observe(Mag)      |
                                            |  - observe(GpsHdg)   |
   AIS ─────────► AisArpaPair extractor ─►  |                      |
                                            +----------------------+
```

Each adapter calls `observe(...)` when it has a fresh reading. Adapter availability ↔ wired-or-not at composition root. Absence of a wiring is a no-op for the estimator — it just doesn't get that kind of observation.

## 3. The three new observation kinds

Each observation carries the gyro heading at the same timestamp (looked up via `OwnShipProvider.poseAtOrBefore(t)` in the adapter, so the estimator stays oblivious to the provider). All angles in radians, wrapped to `(-π, π]`.

### 3.1 GPS true heading (multi-antenna) — the gold standard

```cpp
struct GyroVsGpsHeadingObservation {
  Timestamp time;
  double gyro_rad;
  double gps_true_heading_rad;
  double gps_true_heading_std_rad;
};
```

**Math:**
```
r = wrap(gyro_rad - gps_true_heading_rad)
R = gps_true_heading_std_rad²
```
`r` is a direct measurement of `b` with no unmodeled offset. `R` is just the GPS heading sensor noise (typically 0.05–0.5°).

**Assumptions:**
- The multi-antenna baseline is correctly calibrated; receiver outputs heading, not COG.
- Sensor `std` reflects the current fix quality (RTK locked vs. float vs. lost).

**Gate:** outlier only: `|r − b̂| ≤ N · sqrt(R + P_b)` with default `N = 5`. No other gate — this source needs no environmental conditions to be valid.

### 3.2 GPS course-over-ground — needs gating against crab

```cpp
struct GyroVsGpsCogObservation {
  Timestamp time;
  double gyro_rad;
  double gps_cog_rad;
  double gps_cog_std_rad;     // sensor noise on COG itself
  double sog_mps;             // current speed-over-ground
  double gyro_rate_rad_per_s; // |dψ/dt| at observation time
};
```

**Math:**
```
r = wrap(gyro_rad - gps_cog_rad)
R = gps_cog_std_rad² + σ_crab²
```
The `σ_crab²` inflation budgets for unmodeled crab angle (vessel forward axis vs. velocity vector). `σ_crab` is a *config*, not a derived quantity — defaults to a pessimistic 5° to absorb realistic current/wind effects.

**Assumptions:**
- Crab is zero-mean over long time horizons in the absence of persistent unilateral conditions. Over many cycles the inflation absorbs the per-observation deviation; the random-walk model on `b` integrates the residual.
- When `sog_mps` is small, COG itself becomes noisy (atan2 of a small velocity vector). Captured implicitly: most COG receivers report inflated `gps_cog_std_rad` at low speed, and the SOG gate below excludes that regime entirely.

**Gates** (all must pass; tunable via config):
- **C1 — Minimum SOG.** `sog_mps ≥ cog_min_sog_mps` (default 3.0). At low speed COG is dominated by jitter rather than direction.
- **C2 — Maximum gyro rate.** `|gyro_rate_rad_per_s| ≤ cog_max_gyro_rate_rad_per_s` (default 0.5°/s). During turns, gyro reading lags behind body orientation and COG lags behind both; transient crab can be large.
- **C3 — Outlier.** Same as §3.1.

The "straight-line for N seconds" temporal gate (proposed in brainstorming) is **out of scope for MVP**; the instantaneous gyro-rate gate is a simpler proxy that catches most maneuvering. Adding a temporal smoother is in §10.

### 3.3 Magnetic compass — needs variation; deviation budget inflates R

```cpp
struct GyroVsMagneticObservation {
  Timestamp time;
  double gyro_rad;
  double magnetic_heading_rad;
  double magnetic_heading_std_rad;        // sensor noise on the magnetic reading
  std::optional<double> magnetic_variation_rad;  // geographic, signed (east positive)
};
```

**Math:**
```
r = wrap(gyro_rad - (magnetic_heading_rad + variation_rad))
R = magnetic_heading_std_rad² + σ_deviation²
```
The `σ_deviation²` inflation budgets for unmodeled per-vessel deviation (the magnetic compass's response to ship iron, varies with heading). `σ_deviation` is a config, default 3° (pessimistic for uncalibrated installations); operators with a swung deviation table can drop it to ~0.5°.

**Assumptions:**
- Variation is known. NMEA `HDG` sentences may carry it; if absent, the adapter falls back to an externally supplied value (operator config or WMM lookup). If neither is available, the observation is **rejected** at the adapter level — the estimator never sees it.
- Deviation is approximately zero-mean across the vessel's heading distribution. Held by the inflation budget over many cycles.

**Gates:**
- **M1 — Variation required.** `magnetic_variation_rad` must be present. Adapter responsibility; estimator asserts internally.
- **M2 — Outlier.** Same as §3.1.

## 4. Source-availability matrix — the central design constraint

The system **must** work in any of the 2³ × (no/some target observations) permutations of source availability. Specifically:

| Sources wired | Behavior |
|---|---|
| **None** | Estimator behaves as v1/v2 alone (AIS-pair, bearing-innovation). If neither v1 nor v2 inputs arrive either, `current().is_published == false` permanently. No crashes. |
| **One source** | That source drives convergence. Inverse-variance fusion is trivial. |
| **Multiple sources** | Inverse-variance fusion via sequential scalar KF (mathematically equivalent to batch fusion under source independence, which holds here). Tightest sensor dominates `P_b`. |
| **Source disappears mid-mission** | No `observe()` calls; random-walk model continues to inflate `P_b`. After `stale_seconds` the publisher gate flips to "not published" if no other source picks up the slack. No crash. |
| **Source returns mid-mission** | `observe()` resumes. Predict-from-last brings `P_b` to its random-walked level; KF update tightens it. |
| **Source goes intermittently flaky** | Outlier gate G3 (per source) catches bad readings without poisoning `b̂`. |

The estimator **does not track which sources are wired**. It just accepts `observe()` calls when they arrive. There is no register/deregister API. This is intentional: source availability is a property of the composition root, not of estimator state.

Implication for tests (§7): every permutation tested independently, with assertions on `b̂` convergence (or lack thereof) and on `current().is_published` matching the source's information-theoretic capability.

## 5. Config

`HeadingBiasEstimatorConfig` grows:

```cpp
// Multi-heading-source path (v3, spec 2026-06-04-multi-heading-sources-bias-design).
double cog_min_sog_mps{3.0};                          // C1
double cog_max_gyro_rate_rad_per_s{0.5 * M_PI / 180.0}; // C2
double cog_crab_budget_rad{5.0 * M_PI / 180.0};       // σ_crab default
double mag_deviation_budget_rad{3.0 * M_PI / 180.0};  // σ_deviation default
double mhs_outlier_sigma{5.0};                        // G3 across all three (shared with bi_outlier_sigma in spirit)
```

Existing `bi_outlier_sigma` from v2 is reused for bearing innovations; the new `mhs_outlier_sigma` controls the three v3 paths. (Separate so operators can tune independently.)

## 6. Adapter changes

The composition root is responsible for *some* adapter that reads the available NMEA sentences (or proprietary feeds) and dispatches to the right `observe()` overload. To keep adapters thin we ship one helper:

```cpp
namespace navtracker {

// Convenience: given the current OwnShipPose history and a fresh
// observation timestamp `t`, return the gyro heading at-or-before `t`.
// Used by each adapter to time-align the gyro read with the source.
std::optional<double> gyroHeadingAt(const OwnShipProvider& provider, Timestamp t);

}
```

NMEA-specific sentence parsing is already split across:
- **RMC** → SOG, COG, magnetic variation (today: parsed, only SOG/COG used elsewhere).
- **HDT** → true heading (today: parsed via `OwnShipNmeaAdapter`).
- **HDG** → magnetic heading + variation + deviation (today: **not parsed**, to be added).

For MVP this spec only changes the NMEA path enough to expose:
- `RMC.magnetic_variation_rad` as a fallback variation source.
- A new `HDG` sentence parser populating `magnetic_heading_rad`, `magnetic_heading_std_rad` (from a config default or quality flag), `magnetic_variation_rad`.
- Talker-ID-based routing of `HDT`: when the talker matches `gps_heading_talkers` (config, default `{"GP","GN","GL"}`), the sentence is routed as `gps_true_heading_rad`; otherwise as `gyro` (existing behavior).

These are additive — existing consumers see no change to `OwnShipPose` semantics.

## 7. Validation

### 7.1 Estimator unit tests (`tests/bias/test_heading_bias_multi_source.cpp`)

Per observation kind:

- **U-HDG-1**: GPS-heading single observation moves `b̂` by `K·r` and shrinks `P_b` by `(1−K)`. Direct check.
- **U-HDG-2**: GPS-heading sequence converges within 3σ over 50 obs from `N(b_true, R)`.
- **U-HDG-3**: GPS-heading outlier rejected at default 5σ.

- **U-COG-1**: COG observation passes when all gates met; `R` includes `σ_crab²`; updates `b̂`.
- **U-COG-2**: SOG below `cog_min_sog_mps` rejected; counter `rejectedCogBySog()` increments.
- **U-COG-3**: Gyro-rate above `cog_max_gyro_rate_rad_per_s` rejected; counter `rejectedCogByGyroRate()` increments.
- **U-COG-4**: Sequence with σ_crab = 5° at constant true bias converges within 3σ over ~200 obs (slower than GPS-heading due to inflated R).

- **U-MAG-1**: Mag observation with supplied variation updates `b̂`; `R` includes `σ_deviation²`.
- **U-MAG-2**: Mag with `std::nullopt` variation rejected at adapter level (estimator never sees it — covered in adapter test). For the estimator-side unit test we assert the contract: passing an obs with NaN variation is undefined behavior (caller violates precondition); no estimator-side test for this case.
- **U-MAG-3**: Mag outlier rejected at default 5σ.

### 7.2 Source-availability permutation tests (`tests/bias/test_heading_bias_permutations.cpp`)

Drive the estimator with a controlled sequence of observations representing each of the 8 single-source subsets (plus a few mixed cases). For each:

- Inject 50 cycles of synthetic observations consistent with `b_true = +2°`.
- After the sequence, assert:
  - **None wired**: `b̂` unchanged (still at `initial_bias_rad`); `is_published == false`.
  - **Only GPS-heading**: `|b̂ − b_true| < 0.1°`; `is_published == true`.
  - **Only COG**: `|b̂ − b_true| < 0.5°` (wider tolerance — inflated R); `is_published == true` after `P_b` shrinks below threshold.
  - **Only Mag**: `|b̂ − b_true| < 0.3°` (between the two — depends on deviation budget); `is_published == true`.
  - **All three**: `|b̂ − b_true| < 0.1°` (GPS-heading dominates); `is_published == true` faster than any single source.

### 7.3 Dynamic-availability test

One scenario, sources turn on/off mid-run:

- Cycles 1–50: only GPS-heading wired. Converge to truth.
- Cycles 51–100: GPS-heading silent (simulating receiver loss); COG continues. Bias should hold within tolerance, possibly drift slightly under the inflated R of COG-only updates.
- Cycles 101–150: GPS-heading returns. Convergence tightens again.

Asserts: no crashes, `current().is_published` flips correctly under the `stale_seconds` gate during the silent period (if the silent period exceeds `stale_seconds`).

### 7.4 Crab-realistic regression (`tests/bias/test_cog_crab_realistic.cpp`)

Generate COG observations where each sample has crab drawn from `N(0, σ_crab_true)` with `σ_crab_true = 3°` (matching the realistic budget). After 200 obs, verify `b̂` converges to `b_true` within `0.5°` — confirming that the inflation budget is adequate.

## 8. Out of scope (deferred, listed)

- **Per-source bias states.** Modeling separate `b_gyro`, `b_mag`, `b_gps_cog` as a vector state would let us *disambiguate* biases from offsets like variation+deviation+crab. Substantially more complex (multi-state KF, observability analysis). MVP estimates a single scalar `b` (the gyro bias) and treats all source-specific offsets as inflated noise.
- **WMM integration** for variation lookup. Operator/adapter supplies variation; the estimator just consumes it.
- **Deviation table** for magnetic compass. Operator-supplied σ_deviation budget covers it.
- **Crab angle modeling.** Could derive from current/wind sensors if available; today it's just a fixed budget.
- **Joint state-bias EKF augmentation** for the v2 bearing-only single-target scene. Already deferred in v1 spec §11.5.
- **Per-source temporal smoothers** (e.g., 5-second window with maneuver detection). MVP uses instantaneous gates.
- **Hot-swap of the gyrocompass itself.** We assume the gyro is the bearing reference; if it goes silent, the bearings are unusable independently of this estimator.

## 9. Files

| Action | Path |
|---|---|
| Create | `core/bias/HeadingBiasObservations.hpp` — three new observation structs |
| Modify | `core/bias/HeadingBiasEstimator.hpp` — three `observe()` overloads, new config fields, diagnostic counters |
| Modify | `core/bias/HeadingBiasEstimator.cpp` — implement the three overloads with gates |
| Create | `tests/bias/test_heading_bias_multi_source.cpp` — per-kind unit tests |
| Create | `tests/bias/test_heading_bias_permutations.cpp` — source-availability matrix |
| Create | `tests/bias/test_cog_crab_realistic.cpp` — crab-realistic regression |
| Modify | `docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md` — v3 landing note in "Landed" |
| Modify | `CMakeLists.txt` — register three new test sources |

NMEA-side adapter changes (HDG parser, talker-ID-based HDT routing, RMC variation forwarding) are **deferred to a follow-up spec**. The MVP delivers the estimator path with synthetic observations driving the unit / permutation tests. Real NMEA wiring is independent and can be done incrementally without changing the estimator surface.

## 10. Rationale (decisions vs. alternatives)

| Decision | Considered | Chosen | Why |
|---|---|---|---|
| Single scalar `b`, multi-source observations | Vector state `[b_gyro, b_mag, b_cog]` | Single | Operationally simpler; covers the dominant use case (correct ARPA/EOIR bearings); MVP scope. The vector version is a known extension when data is available to calibrate it. |
| Three separate observation kinds | One unified `HeadingObservation` with a kind enum | Three structs | Each has different gate metadata (SOG, gyro rate, variation); a tagged union forces every adapter to populate fields it doesn't have, defeating type safety. |
| Adapter-side time alignment of gyro vs. source | Estimator queries `OwnShipProvider` | Adapter | Keeps estimator decoupled from `OwnShipProvider`; mirrors the v1/v2 patterns. |
| Estimator-side gates (not adapter-side) | Adapter-side gates (fire only when ready) | Estimator-side | Gates are tied to noise-budget math (σ_crab depends on regime); config and diagnostic counters live with the estimator; centralizes the policy. |
| No register/deregister API for sources | `wireSource(kind, enabled)` | None | Composition root wiring already encodes presence; runtime-deregistration would add state machine complexity with no observed value. |
| σ_crab and σ_deviation as config (not derived) | Derive from environmental sensors (wind, current) | Config | We don't have those sensors. Defaults pessimistic; operators with calibrations override. |
| Talker-ID-based HDT routing | Receiver-type config | Talker ID (config-supplied set) | NMEA-native; the operator already knows their receiver inventory. Deferred from this MVP though — see §9 "NMEA-side adapter changes". |
| `mhs_outlier_sigma` separate from `bi_outlier_sigma` | Single shared σ | Separate | Bearing innovations and heading-source observations have different noise structures; operators may want to tune independently. |
| Reject observation at adapter when variation unavailable | Estimator-side check | Adapter (with estimator-side precondition assertion) | Variation unavailability is a *data presence* issue, not a noise-budget issue; cleanest to filter at the source. |

## 11. Acceptance

This work is done when:

- Full suite green.
- The three per-kind unit test groups pass with default config.
- The eight permutation cases in §7.2 pass with the stated tolerances.
- The dynamic-availability test in §7.3 demonstrates correct publish/un-publish transitions across source disappearance and return.
- The crab-realistic regression in §7.4 confirms σ_crab default is adequate.
- No changes to `IHeadingBiasProvider`, `Tracker`, `Measurement`, or any v1/v2 observation kind.
- Headers carry the four-part doc (math, assumptions, rationale, ways to improve) for each new struct/method.

## 12. What changes in operator behavior

- Operators with multi-antenna GPS heading get a near-perfect bias source at runtime, with no targets required.
- Operators with only NMEA-COG get a usable but slower-converging bias source in straight-line transits at speed.
- Operators with only a magnetic compass with a swung deviation table can get a usable bias source; without one they need to accept the conservative 3° σ_deviation and slow convergence.
- Operators with **none** of the three get exactly the v1/v2 behavior — no regression.
- Operators with multiple sources get inverse-variance-weighted fusion for free, no tuning.
