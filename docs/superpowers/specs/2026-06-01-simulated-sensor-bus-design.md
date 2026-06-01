# SimulatedSensorBus — End-to-End Multi-Sensor Harness Design

**Date:** 2026-06-01
**Status:** Approved for implementation
**Related:** `2026-05-28-maritime-sensor-fusion-design.md` §14.1 (sensor pose), §14.9 (heading errors — deferred)

## 1. Goal

Build an end-to-end harness that synthesises ground-truth target trajectories, drives them through the **real adapters** for all four day-one sensor types (OwnShip, AIS, ARPA, EO/IR), and produces a `Scenario` consumable by the existing `runScenario` / `runScenarioBatched` machinery.

The harness validates that:
1. The full adapter chain (raw record → NMEA/struct → adapter → `Measurement` → tracker) does not regress the OSPA achieved on existing winning scenarios where we already inject `Measurement`s directly.
2. The system remains deterministic under replay (same truth + same seed → identical `Scenario`).

Out of scope for v1: heading bias and gyro drift (§14.9 — deferred), realistic geo datum effects beyond a single tangent plane, encoded `!AIVDM` AIS bitstreams (`AisAdapter` already accepts a struct), rigid-body offsets between sensor mount points (§14.2 — deferred), close-range precision sensors (§14.5 — deferred).

## 2. Architecture invariants honoured

This is **test/composition layer** — it depends on adapters and on `core/`, but adapters and `core/` do not depend on `sim/`. Mirrors the role of `app/` (composition root).

- **Determinism (invariant 4):** every stochastic step pulls from a `std::mt19937` whose seed is derived from the bus seed via a fixed substream rule. Same `(truth, seed)` → byte-identical `Scenario`.
- **Validate at the edges (invariant 6):** the simulator generates *valid* raw records; the adapters do the validation. The bus is allowed to inject dropouts and out-of-FOV detections, but every emitted record is well-formed.
- **Time-driven (invariant 4):** the bus advances on an internal scheduler keyed on simulated time, not wall clock. No `sleep`s.

## 3. Module layout

New top-level directory `sim/` (parallel to `adapters/`, `core/`, `app/`):

```
sim/
  TruthTrajectory.hpp        // pure: ground-truth target (no adapter deps)
  TruthTrajectory.cpp
  NmeaEncode.hpp             // helpers: lat/lon DD.MMmmmm, $XXX*HH checksum
  NmeaEncode.cpp
  SensorEmitter.hpp          // abstract: emit due records over [t_prev, now]
  AisEmitter.hpp / .cpp      // → AisAdapter::ingest(AisDynamicReport)
  ArpaEmitter.hpp / .cpp     // → ArpaAdapter::ingest("$RATTM,...", t)
  EoIrEmitter.hpp / .cpp     // → EoIrAdapter::ingest(CameraDetection)
  OwnShipEmitter.hpp / .cpp  // → OwnShipNmeaAdapter::ingest("$GPGGA,...", t) + HDT
  SimulatedSensorBus.hpp     // orchestrator
  SimulatedSensorBus.cpp
```

CMake: add `sim/` as a library target `navtracker_sim`, link to `navtracker_core` and the existing adapter library. Tests link `navtracker_sim`.

### 3.1 File responsibilities

- **TruthTrajectory** — function-of-time over a target's true `(position, velocity)`. Implementations: `ConstantVelocityTrajectory`, `PiecewiseLinearTrajectory` (legs), `CoordinatedTurnTrajectory` (for IMM scenarios). Each accepts `eval(Timestamp t) → TruthState { pos, vel }`. No noise here.
- **NmeaEncode** — `formatLatDdmm(double deg)`, `formatLonDdmm(double deg)`, `nmeaChecksum(std::string_view body)`, `wrap(std::string body) → "$" + body + "*" + checksum`. Pure string helpers, no Eigen.
- **SensorEmitter** — abstract:
  ```cpp
  class SensorEmitter {
   public:
    virtual ~SensorEmitter() = default;
    // Emit any records due in (last_emit_, now]. Implementations update last_emit_.
    virtual void emit(Timestamp now,
                      const std::vector<TruthState>& targets_truth,
                      std::mt19937& rng) = 0;
  };
  ```
- **OwnShipEmitter** — borrows the own-ship `TruthTrajectory` (held by the bus). Emits `$GPGGA` + `$GPHDT` at 1 Hz nominal cadence. Calls `OwnShipNmeaAdapter::ingest`. Heading is exact (§14.9 deferred).
- **AisEmitter** — per cooperative target: cadence from speed (Class A SOTDMA). Skips during dropout windows. Constructs `AisDynamicReport` directly. Calls `AisAdapter::ingest`.
- **ArpaEmitter** — per target visible to radar: 3 s rotation cadence. Range 50 m σ, bearing 1° σ. Encodes `$RATTM` with **relative** bearing (so adapter rotates by own-ship heading). Calls `ArpaAdapter::ingest`.
- **EoIrEmitter** — per target visible to camera: 10 Hz cadence; bearing-only (`range = 0` mode reserved for future) or bearing+range from optical estimation. Applies FOV gate around own-ship heading. Constructs `CameraDetection`. Calls `EoIrAdapter::ingest`.
- **SimulatedSensorBus** — config, scheduler, time loop, ground-truth sampling at OSPA cadence, draining adapter `poll()` calls into time-ordered `Scenario.measurements`.

## 4. Public API

```cpp
// sim/SimulatedSensorBus.hpp
struct SimulatedSensorBusConfig {
  Timestamp t0;
  double duration_s{60.0};
  double dt_s{0.1};                         // bus tick (must divide cadences cleanly)
  double truth_sample_dt_s{1.0};            // OSPA truth cadence
  std::uint32_t seed{2026};
  geo::Datum datum;                         // for all geodetic adapters
};

class SimulatedSensorBus {
 public:
  explicit SimulatedSensorBus(SimulatedSensorBusConfig cfg);

  // Trajectory ownership.
  void setOwnShip(std::shared_ptr<TruthTrajectory> trajectory);
  void addTarget(std::uint64_t truth_id, std::shared_ptr<TruthTrajectory> trajectory);

  // Each call wires the bus to a real adapter chain and a per-sensor emitter
  // configured with that adapter. The bus does NOT own the adapters; the caller
  // does (so the caller can poll them and feed Measurements into a Tracker).
  void attachOwnShip(OwnShipNmeaAdapter& adapter, OwnShipEmitterConfig cfg);
  void attachAis    (AisAdapter& adapter,         AisEmitterConfig cfg);
  void attachArpa   (ArpaAdapter& adapter,        ArpaEmitterConfig cfg);
  void attachEoIr   (EoIrAdapter& adapter,        EoIrEmitterConfig cfg);

  // Run the simulation through `duration_s`, returning a Scenario whose
  // `.measurements` are the Measurements emitted by all attached adapters in
  // strict ascending time order, and whose `.truth` is the per-target truth
  // sampled at `truth_sample_dt_s`.
  Scenario run();

 private:
  // implementation
};
```

`run()` time loop:

```
t = t0
last_truth_sample = t0
while t <= t0 + duration_s:
  truth_states = [target_traj.eval(t) for each target]
  for each emitter: emitter.emit(t, truth_states, rng_substream)
  drain all attached adapters' poll(); append Measurements (with time = source time)
  if t - last_truth_sample >= truth_sample_dt_s:
    record TruthSamples for each target at t
    last_truth_sample = t
  t += dt_s
sort measurements by time (stable) ; return Scenario { measurements, truth }
```

The `sort` is a safety net for the case where multiple adapters drained in a single tick produced timestamps that interleave with later ticks (e.g., AIS dropout catch-up). With well-formed cadences emitting at the current tick, ascending order falls out naturally.

## 5. Per-sensor emitter specs

### 5.1 OwnShipEmitter

**Math.** GPS position emitted at 1 Hz. Truth lat/lon obtained by inverse-projection of `own_ship_traj.eval(t).pos` (ENU) through the configured datum. Position noise added in ENU **before** the inverse projection: `(lat_noisy, lon_noisy) = datum.fromEnu(truth_enu + n)` where `n ~ N(0, σ_gps² I_2)`, `σ_gps = 5.0 m` (default). Heading emitted exact (§14.9 deferred) at the same 1 Hz cadence as a separate `$GPHDT` sentence.

**Assumptions.** Heading provided by an unbiased true-north gyro (§14.9 deferral). 1 Hz fix rate.

**Rationale.** Real shipboard GPS receivers do ~1–10 Hz; 1 Hz is the floor and matches the IMO/SOLAS minimum for AIS-rated nav inputs. HDT is the simplest heading sentence and the OwnShip adapter already parses it.

**What to test next.**
- Inject §14.9 heading bias `+ b` and drift `+ b₀ t` and re-run all scenarios; measure cross-track OSPA degradation as a function of (range, b).
- Higher fix rate (5 Hz, 10 Hz) and confirm tracker doesn't over-trust.

**Config:**
```cpp
struct OwnShipEmitterConfig {
  double dt_s{1.0};
  double gps_pos_std_m{5.0};
  // §14.9 hooks (default zero):
  double heading_bias_deg{0.0};
  double heading_drift_deg_per_s{0.0};
};
```

### 5.2 AisEmitter

**Math.** Per cooperative target, sample `Δt` from a Class-A SOTDMA cadence table keyed on the target's instantaneous speed:

| speed (kn) | nominal Δt (s) |
|------------|----------------|
| 0–14       | 10             |
| 14–23      | 6              |
| 23+        | 2              |

The emitter holds a per-target `next_emit_t` initially set to `t0`. On each `emit(now, …)`, if `now >= next_emit_t` it samples a Gaussian position perturbation in ENU with σ = `cfg.pos_std_m` (default 10 m), inverse-projects to lat/lon, builds an `AisDynamicReport` (with `high_accuracy` set by config), calls `adapter.ingest(report)`, and advances `next_emit_t += Δt(speed)`. Dropout windows skip emission without advancing the cadence (so the next report fires at the natural next slot after the dropout).

**Assumptions.** All attached targets are AIS-cooperative — non-cooperative targets are simply not added to the AIS emitter's target list. We treat AIS lat/lon as already in the same horizontal datum (WGS-84) as `cfg.datum`.

**Rationale.** SOTDMA cadence is the closest thing to a "real" AIS arrival pattern without simulating the slot map itself. Per-target dropouts model crowded harbours or shadowing.

**What to test next.**
- Replace fixed-Δt SOTDMA with stochastic jitter `Δt + N(0, σ_τ²)` to model slot contention.
- Inject MMSI swaps to test the hint-vs-fusion-id invariant 5.

**Config:**
```cpp
struct AisEmitterConfig {
  struct TargetEntry {
    std::uint64_t truth_id;
    std::uint32_t mmsi;
    bool high_accuracy{true};
  };
  std::vector<TargetEntry> targets;
  double pos_std_m{10.0};
  std::vector<std::pair<double,double>> dropout_windows_s;  // [start, end) relative to t0
};
```

### 5.3 ArpaEmitter

**Math.** Per target visible to radar: every `cfg.rotation_dt_s` (default 3 s), compute relative `(range, bearing_rel)` from current own-ship pose and target truth:

```
dxy   = target.pos - ownship.pos_enu
range = ||dxy||
β_true_rad = atan2(dxy.y, dxy.x)
β_rel_deg  = (β_true_rad * 180/π) - ownship.heading_true_deg, wrapped to [0, 360)
```

Add noise: `range_obs = range + N(0, cfg.range_std_m²)`, `β_rel_obs = β_rel + N(0, cfg.bearing_std_deg²)` (defaults 50 m, 1°). Encode as `$RATTM,<num>,<range>,<β_rel>,R,...,N*HH` — exactly the field layout `ArpaAdapter::ingest` expects (`bearing_units = "R"`, `dist_units = "N"` → nautical miles).

Range gating: skip targets outside `[cfg.min_range_m, cfg.max_range_m]` (defaults 50 m, 12 NM).

**Assumptions.** Radar's antenna boresight aligns with own-ship heading; ARPA target numbers are stable per target (cfg-provided). True azimuth observability comes via the own-ship HDT (since adapter rotates by heading).

**Rationale.** `$RATTM` is the standard NMEA sentence for relative-bearing tracked targets. Using relative bearing exercises the ArpaAdapter's heading-aware projection path.

**What to test next.**
- Switch some emissions to `$RATLL` (absolute lat/lon) and confirm both adapter paths give comparable OSPA.
- Inject radar track-number swaps under crossing geometry.
- §14.9: inject heading bias and watch ARPA's cross-track OSPA blow up.

**Config:**
```cpp
struct ArpaEmitterConfig {
  struct TargetEntry { std::uint64_t truth_id; int arpa_track_num; };
  std::vector<TargetEntry> targets;
  double rotation_dt_s{3.0};
  double range_std_m{50.0};
  double bearing_std_deg{1.0};
  double min_range_m{50.0};
  double max_range_m{22224.0};  // 12 NM
};
```

### 5.4 EoIrEmitter

**Math.** Per target visible to the camera: every `cfg.dt_s` (default 0.1 s = 10 Hz), compute `(range, β_rel_deg)` exactly as in 5.3. FOV gate:

```
boresight_rel_deg = cfg.boresight_relative_deg  // default 0 = looking forward
half_fov_deg      = cfg.fov_deg / 2             // default 30 → ±30° from boresight
if |wrap_signed(β_rel_deg - boresight_rel_deg)| > half_fov_deg: skip
```

Emit a `CameraDetection { time, bearing_relative_deg = β_rel + N(0, σ_β²), range_m, bearing_std_deg, range_std_m }`. For bearing-only mode (`cfg.range_mode == BearingOnly`), set `range_m = 0` and `range_std_m = cfg.bearing_only_range_std_m` (a large value the existing projection helper still tolerates — `EoIrAdapter` ultimately produces a Position2D with very-elongated covariance along the bearing line). For bearing+range mode, use truth range plus Gaussian noise.

**Assumptions.** Camera is fixed-mount looking forward; FOV symmetric about boresight; detection rate independent of range out to `max_range_m`. Bearing-only mode produces a wide along-bearing covariance — the existing adapter projection already handles this via `projectRangeBearingToEnu`.

**Rationale.** Real EO/IR systems often emit bearing-only or bearing+stereo-range; we cover both modes via a config flag.

**What to test next.**
- Range-dependent detection probability (cubic falloff).
- Frame-to-frame detection dropouts at SNR boundary.
- Multiple cameras with different boresights (gimballed or rear).

**Config:**
```cpp
struct EoIrEmitterConfig {
  enum class RangeMode { BearingOnly, BearingAndRange };
  struct TargetEntry { std::uint64_t truth_id; int sensor_track_id; };
  std::vector<TargetEntry> targets;
  double dt_s{0.1};
  double fov_deg{60.0};
  double boresight_relative_deg{0.0};
  double max_range_m{5000.0};
  RangeMode range_mode{RangeMode::BearingAndRange};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  double bearing_only_range_std_m{1000.0};
};
```

## 6. RNG substreams

The bus stores one `std::mt19937` seeded from `cfg.seed`. Each emitter holds its own substream produced by hashing `(seed, emitter_id_string)` with a fixed mixing function (e.g., `std::seed_seq`). This means:

- Order of `attach...` calls does not affect noise realisation.
- Re-running with the same seed and same config produces byte-identical NMEA strings and Measurement values.

A `test_simulated_bus_determinism.cpp` test asserts two independent `run()` calls with the same config yield identical `Scenario.measurements` (same count, same times, same `value`, same `covariance`).

## 7. Validation strategy

For each existing winning scenario (crossing, overtaking, parallel/head-on, bearing-only moving sensor), we add **one regression test** that:

1. Constructs the same truth trajectories the existing `Builders.cpp` builder used.
2. Drives them through `SimulatedSensorBus` with v1 default configs.
3. Feeds the resulting `Scenario` into the same `Tracker` (estimator + associator + manager) and harness (`runScenario` or `runScenarioBatched`) as the existing test.
4. Asserts `mean_ospa <= baseline_mean_ospa * tolerance_multiplier` where `tolerance_multiplier` is per-scenario (initial estimate 2.0 — refined during execution as we measure actual deltas).

The assertion is "no catastrophic regression" not "match the baseline." The bus injects strictly more noise (multi-sensor cadence variation, FOV gating, dropout, real adapter chain) than the existing direct-Measurement builders, so OSPA is expected to be modestly worse.

A separate **determinism test** asserts identical output across two `run()` invocations with the same config.

Because every regression scenario attaches all four sensors against the same set of targets, multi-sensor fusion (cross-sensor ID stability, mixed cadences, FOV gating) is exercised implicitly by the existing scenarios — no new mixed-sensor scenarios in v1 (user-chosen scope).

## 8. Implementation order (preview — full plan in writing-plans skill output)

Roughly 10 tasks:

1. `TruthTrajectory` interface + `ConstantVelocityTrajectory`.
2. `NmeaEncode` helpers + tests.
3. `SensorEmitter` interface + `OwnShipEmitter` (simplest — no targets, just position+heading).
4. `AisEmitter` + tests.
5. `ArpaEmitter` (NMEA encoding path) + tests.
6. `EoIrEmitter` (FOV gating, bearing/range modes) + tests.
7. `SimulatedSensorBus` orchestrator: time loop, scheduler, drain, sort.
8. Determinism test.
9. Regression replay of one existing winning scenario (crossing) end-to-end.
10. Replay remaining winning scenarios (overtaking, parallel/head-on, bearing-only moving sensor).

## 9. Decisions recorded

| Decision | Choice | Why |
|---|---|---|
| New top-level `sim/` vs nested `core/scenario/sim/` | Top-level `sim/` | Sim depends on adapters → can't live under `core/` without breaking invariant 1 |
| AIS input format | `AisDynamicReport` struct | Adapter already accepts struct; `!AIVDM` encoding is a future exercise |
| Encoded NMEA for ARPA + OwnShip | Yes | Adapters parse NMEA; exercising the parse path is the *point* of "end-to-end" |
| Heading bias / drift (§14.9) | Deferred to follow-up | User-confirmed; v1 keeps the harness pure |
| Validation metric | OSPA tolerance multiplier vs baseline | Absolute bounds would be brittle as noise models tune |
| Determinism guarantee | Per-emitter `std::mt19937` substream | Order-independent + reproducible |
| Adapter ownership | Caller owns adapters; bus borrows | Caller needs to poll them anyway to feed tracker |
| Sensor scope v1 | All four (OwnShip, AIS, ARPA, EO/IR) | User choice |
| Scenarios v1 | Reuse existing winning scenarios + one minimal cross-cue test | User choice |
