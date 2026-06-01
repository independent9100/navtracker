# Sensor Pose on Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `sensor_position_enu` to `Measurement` and thread it through the measurement-model math, so range/bearing and bearing-only `h(x)` are computed relative to *where the sensor was at measurement time*, not the implicit ENU origin. Then build a bearing-only scenario with a *moving* sensor (own-ship motion → parallax → range becomes observable) and use it to finally run the particle filter on the geometry it was designed for.

**Architecture:** New field on `Measurement` defaults to `Vector2d::Zero()` so every existing test continues to compute `h(x)` against the origin and stays numerically identical. New parameter on `predictMeasurement` / `predictMeasurementValue` carries the sensor pose forward to the math; default `Zero()` preserves existing call sites. Call sites that already hold a `Measurement` (estimators' update paths, gating, JPDA, MHT) are updated to forward `z.sensor_position_enu`. Range/bearing/bearing-only h(x) and Jacobian become `r = ‖p_target − p_sensor‖`, `β = atan2(py − sy, px − sx)`, with `H` partials evaluated at the shifted position.

**Tech Stack:** C++17, Eigen 3.4, gtest. No new dependencies.

---

## Design Notes (read before implementing)

**Why default-zero.** Existing scenarios (`buildRangeBearingPassScenario`, `buildBearingOnlyScenario`) emit measurements without `sensor_position_enu` set; with the default of `Zero()`, the math reduces to the current "sensor at ENU origin" behavior. All current tests pass unchanged.

**Why parameter, not just `Measurement` arg.** Some call sites use `predictMeasurement` from a state vector without a `Measurement` (e.g. unit tests on the measurement model itself). Adding a parameter with a default lets both styles coexist. The convention going forward: if you have a `Measurement`, pass `z.sensor_position_enu`; if you're testing the model in isolation, the default is fine.

**What `sensor_position_enu` means.** The sensor's location in the ENU frame at the moment the measurement was taken. For a stationary shore radar this is set once at adapter construction. For a sensor mounted on own-ship this is the own-ship's ENU position at the measurement timestamp — the EO/IR and ARPA adapters in `adapters/` already have access to this via `OwnShipProvider` (they currently *project* relative measurements to ENU positions and emit `Position2D`; with this change, a passive-bearing sensor can keep its bearings as-is and attach the sensor pose for the tracker to use directly).

**What this does NOT do.** Sensor *orientation* (boresight, pan/tilt) is out of scope for this plan. Bearing measurements here are still in ENU (true bearing), not body-frame. Adding sensor orientation is captured as future work (§14.x extension).

**The scenario this unlocks.** `buildBearingOnlyMovingSensorScenario`: a sensor that itself moves at known velocity emits bearings to a static (or slowly-moving) target. Range becomes observable via parallax — the standard bearing-only-with-own-ship-motion problem. The Gaussian posterior on target position is genuinely banana-shaped during the convergence window; this is where the PF should finally win against EKF/UKF.

---

## File Structure

**Modify:**
- `core/types/Measurement.hpp` — add `sensor_position_enu` field
- `core/estimation/MeasurementModels.hpp` — add sensor-pose parameter to both `predictMeasurementValue` and `predictMeasurement`
- `core/estimation/MeasurementModels.cpp` — use sensor pose in range/bearing/bearing-only math
- `core/association/Gating.cpp` — forward `z.sensor_position_enu`
- `core/estimation/EkfEstimator.cpp` — forward `z.sensor_position_enu`
- `core/estimation/UkfEstimator.cpp` — forward `z.sensor_position_enu` per sigma point
- `core/estimation/ParticleFilterEstimator.cpp` — forward per particle
- `core/estimation/ImmEstimator.cpp` — forward per mode
- `core/association/JpdaAssociator.cpp` — forward in the likelihood loop
- `core/tracking/TrackTree.cpp` — forward in `branch`'s likelihood computation
- `core/scenario/Builders.hpp` / `.cpp` — add `buildBearingOnlyMovingSensorScenario`
- `tests/estimation/test_measurement_models.cpp` — add a sensor-pose math test
- `tests/scenario/test_builders.cpp` — add builder test
- `tests/scenario/test_filter_comparison.cpp` — add the new bearing-only-moving-sensor comparison
- `docs/algorithms/estimation.md` — document the measurement-model change
- `docs/algorithms/evaluation-log.md` — record the moving-sensor result

---

## Task 1: `sensor_position_enu` field on Measurement

**Files:**
- Modify: `core/types/Measurement.hpp`
- Modify: `tests/types/test_measurement.cpp` (append)

- [ ] **Step 1: Append failing test**

Append to `tests/types/test_measurement.cpp`:

```cpp
TEST(Measurement, DefaultSensorPositionIsZero) {
  navtracker::Measurement m;
  EXPECT_DOUBLE_EQ(m.sensor_position_enu.x(), 0.0);
  EXPECT_DOUBLE_EQ(m.sensor_position_enu.y(), 0.0);
}
```

- [ ] **Step 2: Run, verify fail**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=Measurement.DefaultSensorPositionIsZero`
Expected: FAIL (field not present).

- [ ] **Step 3: Add the field**

In `core/types/Measurement.hpp`, inside the `Measurement` struct (after `hints`):

```cpp
  // Where the sensor was at measurement time, in the ENU frame. Default
  // is the origin (stationary sensor at the datum). For a sensor mounted
  // on a moving platform this is the platform's ENU position at the
  // measurement timestamp.
  Eigen::Vector2d sensor_position_enu{Eigen::Vector2d::Zero()};
```

- [ ] **Step 4: Run, verify pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests 2>&1 | tail -3`
Expected: 161/161 PASS.

- [ ] **Step 5: Commit**

```bash
git add core/types/Measurement.hpp tests/types/test_measurement.cpp
git commit -m "types: Measurement carries sensor_position_enu (defaults to origin)"
```

---

## Task 2: Thread `sensor_position_enu` through `MeasurementModels`

**Files:**
- Modify: `core/estimation/MeasurementModels.hpp`
- Modify: `core/estimation/MeasurementModels.cpp`
- Modify: `tests/estimation/test_measurement_models.cpp`

- [ ] **Step 1: Append failing tests**

Append to `tests/estimation/test_measurement_models.cpp`:

```cpp
TEST(MeasurementModels, RangeBearingFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 100.0, 0.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(50.0, 0.0);  // sensor offset 50 m east of origin
  const Eigen::VectorXd z =
      navtracker::predictMeasurementValue(
          navtracker::MeasurementModel::RangeBearing2D, state, sensor);
  // Range = |target - sensor| = 50 m. Bearing = atan2(0, 50) = 0.
  EXPECT_NEAR(z(0), 50.0, 1e-9);
  EXPECT_NEAR(z(1), 0.0, 1e-9);
}

TEST(MeasurementModels, BearingOnlyFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 0.0, 100.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(0.0, 50.0);
  const Eigen::VectorXd z =
      navtracker::predictMeasurementValue(
          navtracker::MeasurementModel::Bearing2D, state, sensor);
  // From sensor at (0, 50), the target at (0, 100) is at bearing pi/2.
  EXPECT_NEAR(z(0), 3.14159265358979323846 / 2.0, 1e-9);
}

TEST(MeasurementModels, RangeBearingJacobianFromOffsetSensor) {
  Eigen::Vector4d state;
  state << 60.0, 80.0, 0.0, 0.0;
  const Eigen::Vector2d sensor(10.0, 20.0);
  // Shifted: dx = 50, dy = 60, r = sqrt(50^2 + 60^2) = sqrt(6100).
  const navtracker::MeasurementPrediction p =
      navtracker::predictMeasurement(
          navtracker::MeasurementModel::RangeBearing2D, state, sensor);
  const double r = std::sqrt(6100.0);
  EXPECT_NEAR(p.H(0, 0), 50.0 / r, 1e-9);
  EXPECT_NEAR(p.H(0, 1), 60.0 / r, 1e-9);
  EXPECT_NEAR(p.H(1, 0), -60.0 / (r * r), 1e-9);
  EXPECT_NEAR(p.H(1, 1),  50.0 / (r * r), 1e-9);
}
```

- [ ] **Step 2: Run, verify fail (compile error — extra parameter not yet accepted)**

- [ ] **Step 3: Update declarations**

In `core/estimation/MeasurementModels.hpp`, change both function signatures:

```cpp
// h(x) only. Optional `sensor_position_enu` shifts the implicit "sensor at
// origin" assumption to an arbitrary point in the ENU frame.
Eigen::VectorXd predictMeasurementValue(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());

// h(x) and H. Same sensor_position_enu semantics as above.
MeasurementPrediction predictMeasurement(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu = Eigen::Vector2d::Zero());
```

- [ ] **Step 4: Update implementations**

In `core/estimation/MeasurementModels.cpp`, replace the bodies of both functions to use shifted positions. The new implementations:

```cpp
Eigen::VectorXd predictMeasurementValue(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu) {
  const double dx = state(0) - sensor_position_enu.x();
  const double dy = state(1) - sensor_position_enu.y();
  switch (model) {
    case MeasurementModel::Position2D:
      return Eigen::Vector2d(state(0), state(1));
    case MeasurementModel::PositionVelocity2D:
      return state.head<4>();
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(dx, dy);
      if (r < 1e-6) r = 1e-6;
      return Eigen::Vector2d(r, std::atan2(dy, dx));
    }
    case MeasurementModel::Bearing2D: {
      double r = std::hypot(dx, dy);
      if (r < 1e-6) r = 1e-6;
      Eigen::VectorXd v(1);
      v(0) = std::atan2(dy, dx);
      return v;
    }
  }
  return Eigen::VectorXd();
}

MeasurementPrediction predictMeasurement(
    MeasurementModel model,
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& sensor_position_enu) {
  MeasurementPrediction out;
  out.z_pred = predictMeasurementValue(model, state, sensor_position_enu);
  const int n = static_cast<int>(state.size());
  const double dx = state(0) - sensor_position_enu.x();
  const double dy = state(1) - sensor_position_enu.y();
  switch (model) {
    case MeasurementModel::Position2D: {
      out.H = Eigen::MatrixXd::Zero(2, n);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      break;
    }
    case MeasurementModel::PositionVelocity2D: {
      out.H = Eigen::MatrixXd::Zero(4, n);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      out.H(2, 2) = 1.0;
      out.H(3, 3) = 1.0;
      break;
    }
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(dx, dy);
      if (r < 1e-6) r = 1e-6;
      out.H = Eigen::MatrixXd::Zero(2, n);
      out.H(0, 0) = dx / r;
      out.H(0, 1) = dy / r;
      out.H(1, 0) = -dy / (r * r);
      out.H(1, 1) = dx / (r * r);
      break;
    }
    case MeasurementModel::Bearing2D: {
      double r = std::hypot(dx, dy);
      if (r < 1e-6) r = 1e-6;
      out.H = Eigen::MatrixXd::Zero(1, n);
      out.H(0, 0) = -dy / (r * r);
      out.H(0, 1) =  dx / (r * r);
      break;
    }
  }
  return out;
}
```

- [ ] **Step 5: Build, verify pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests 2>&1 | tail -3`
Expected: 164/164 PASS. **Existing range/bearing and bearing-only tests must still pass** — they implicitly use the default (origin), so their results are unchanged.

- [ ] **Step 6: Commit**

```bash
git add core/estimation/MeasurementModels.hpp core/estimation/MeasurementModels.cpp tests/estimation/test_measurement_models.cpp
git commit -m "estimation: MeasurementModels accept sensor_position_enu (default origin)"
```

---

## Task 3: Forward `z.sensor_position_enu` at all call sites

**Files:**
- Modify: `core/association/Gating.cpp`
- Modify: `core/estimation/EkfEstimator.cpp`
- Modify: `core/estimation/UkfEstimator.cpp`
- Modify: `core/estimation/ParticleFilterEstimator.cpp`
- Modify: `core/estimation/ImmEstimator.cpp`
- Modify: `core/association/JpdaAssociator.cpp`
- Modify: `core/tracking/TrackTree.cpp`

These are mechanical updates: every existing call site that passes a `Measurement z` should now forward `z.sensor_position_enu` as the third argument to `predictMeasurement` / `predictMeasurementValue`.

- [ ] **Step 1: Gating**

In `core/association/Gating.cpp`, in `mahalanobisDistance`:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(z.model, track.state, z.sensor_position_enu);
```

- [ ] **Step 2: EkfEstimator**

In `core/estimation/EkfEstimator.cpp`, in both `update` and `softUpdate`:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(z.model, track.state, z.sensor_position_enu);
```

For `softUpdate`, the variable name is `z0` (from `gated_measurements[0]`). Use `z0.sensor_position_enu`.

- [ ] **Step 3: UkfEstimator**

In `core/estimation/UkfEstimator.cpp`, in the sigma-point measurement-prediction loop:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(z.model, sp.points.col(i), z.sensor_position_enu);
```

- [ ] **Step 4: ParticleFilterEstimator**

In `core/estimation/ParticleFilterEstimator.cpp`, in the per-particle likelihood loop:

```cpp
const Eigen::VectorXd z_pred = predictMeasurementValue(
    z.model, track.particles.col(i), z.sensor_position_enu);
```

- [ ] **Step 5: ImmEstimator**

In `core/estimation/ImmEstimator.cpp`, in the per-mode update loop:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(z.model, x_j, z.sensor_position_enu);
```

- [ ] **Step 6: JpdaAssociator**

In `core/association/JpdaAssociator.cpp`, in the validation-matrix construction loop:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(measurements[j].model, tracks[t].state,
                       measurements[j].sensor_position_enu);
```

- [ ] **Step 7: TrackTree::branch**

In `core/tracking/TrackTree.cpp` `TrackTree::branch`, in the likelihood block:

```cpp
const MeasurementPrediction pred =
    predictMeasurement(z.model, tmp_predicted.state, z.sensor_position_enu);
```

- [ ] **Step 8: Build full suite, verify all pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests 2>&1 | tail -3`
Expected: 164/164 PASS — all existing scenarios continue to use the default `Zero()` sensor pose, so numerics are unchanged.

- [ ] **Step 9: Commit**

```bash
git add core/association/Gating.cpp core/estimation/EkfEstimator.cpp core/estimation/UkfEstimator.cpp core/estimation/ParticleFilterEstimator.cpp core/estimation/ImmEstimator.cpp core/association/JpdaAssociator.cpp core/tracking/TrackTree.cpp
git commit -m "estimation+association: forward z.sensor_position_enu at all call sites"
```

---

## Task 4: `buildBearingOnlyMovingSensorScenario` builder

The new scenario that exercises the new field. Sensor moves at constant velocity; target is stationary (simplest geometry that demonstrates parallax). Each emitted `Measurement` carries the sensor's position at emit time.

**Files:**
- Modify: `core/scenario/Builders.hpp`
- Modify: `core/scenario/Builders.cpp`
- Modify: `tests/scenario/test_builders.cpp`

- [ ] **Step 1: Declaration**

Append to `core/scenario/Builders.hpp`:

```cpp
// Stationary target observed by a bearing-only sensor on a moving platform.
// The sensor starts at sensor_start and moves with sensor_velocity; the
// target is at target_position (no motion). Initial sample emits a wide
// Position2D measurement (track seed) at the *target's* position with
// large covariance; subsequent samples emit Bearing2D measurements whose
// sensor_position_enu is the sensor's ENU location at that timestamp.
// Range becomes observable from the sensor's parallax over the run.
Scenario buildBearingOnlyMovingSensorScenario(
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& sensor_start,
    const Eigen::Vector2d& sensor_velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);
```

- [ ] **Step 2: Implementation**

Append to `core/scenario/Builders.cpp`. Reuse the existing `makeBearingMeasurement` and `makeMeasurement` helpers; also set `m.sensor_position_enu` on the bearing measurement:

```cpp
Scenario buildBearingOnlyMovingSensorScenario(
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& sensor_start,
    const Eigen::Vector2d& sensor_velocity,
    const std::vector<double>& times,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed,
    std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> pos_noise(0.0, initial_position_std_m);
  std::normal_distribution<double> b_noise(0.0, bearing_std_rad);
  Scenario s;
  const Eigen::Vector2d zero_vel(0.0, 0.0);
  for (std::size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    s.truth.push_back(makeTruth(target_position, zero_vel, t, truth_id));
    const Eigen::Vector2d sensor_pos = sensor_start + sensor_velocity * t;
    if (i == 0) {
      const Eigen::Vector2d noisy(target_position.x() + pos_noise(rng),
                                  target_position.y() + pos_noise(rng));
      Measurement m = makeMeasurement(noisy, t, initial_position_std_m);
      m.sensor_position_enu = sensor_pos;
      s.measurements.push_back(std::move(m));
    } else {
      const double dx = target_position.x() - sensor_pos.x();
      const double dy = target_position.y() - sensor_pos.y();
      const double b = std::atan2(dy, dx);
      const double noisy_b = b + b_noise(rng);
      Measurement m = makeBearingMeasurement(noisy_b, t, bearing_std_rad);
      m.sensor_position_enu = sensor_pos;
      s.measurements.push_back(std::move(m));
    }
  }
  return s;
}
```

- [ ] **Step 3: Test**

Append to `tests/scenario/test_builders.cpp`:

```cpp
TEST(Builders, BearingOnlyMovingSensorAttachesSensorPose) {
  std::vector<double> times{0.0, 1.0, 2.0};
  const navtracker::Scenario s =
      navtracker::buildBearingOnlyMovingSensorScenario(
          Eigen::Vector2d(1000.0, 0.0),    // target
          Eigen::Vector2d(0.0, 0.0),       // sensor start
          Eigen::Vector2d(5.0, 0.0),       // sensor moves +x at 5 m/s
          times, 50.0, 0.05, /*seed*/ 99);
  ASSERT_EQ(s.measurements.size(), 3u);
  // First sample is Position2D seeded at the target with sensor at (0,0).
  EXPECT_EQ(s.measurements[0].model, navtracker::MeasurementModel::Position2D);
  EXPECT_DOUBLE_EQ(s.measurements[0].sensor_position_enu.x(), 0.0);
  // Second sample at t=1: sensor at (5, 0).
  EXPECT_EQ(s.measurements[1].model, navtracker::MeasurementModel::Bearing2D);
  EXPECT_NEAR(s.measurements[1].sensor_position_enu.x(), 5.0, 1e-9);
  // Third sample at t=2: sensor at (10, 0).
  EXPECT_NEAR(s.measurements[2].sensor_position_enu.x(), 10.0, 1e-9);
}
```

- [ ] **Step 4: Build, verify pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=Builders.BearingOnlyMovingSensor*`
Expected: 1/1 PASS.

Full: `./build/navtracker_tests 2>&1 | tail -3`
Expected: 165/165 PASS.

- [ ] **Step 5: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "scenario: buildBearingOnlyMovingSensorScenario (moving sensor, fixed target)"
```

---

## Task 5: Filter comparison on the moving-sensor scenario

The actual test of whether PF wins on a genuinely non-Gaussian posterior.

**Files:**
- Modify: `tests/scenario/test_filter_comparison.cpp`

- [ ] **Step 1: Append the test**

Append to `tests/scenario/test_filter_comparison.cpp`:

```cpp
TEST(FilterComparison, BearingOnlyMovingSensor) {
  // Sensor moves +x at 5 m/s for 60 seconds. Target sits at (1000, 100) — a
  // moderate broadside angle. Initial Position2D seed has wide covariance
  // (sigma = 100 m) so the prior on range is broad; subsequent bearings at
  // sigma = 3 deg refine the posterior. With pure bearing measurements from
  // a moving sensor, range is observable via parallax over time; the
  // intermediate posterior is banana-shaped, which is the geometry where the
  // PF can outperform the EKF/UKF Gaussian approximation.
  std::vector<double> times;
  for (int i = 0; i <= 60; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 3.0 * kPi / 180.0;
  const Scenario s = buildBearingOnlyMovingSensorScenario(
      Eigen::Vector2d(1000.0, 100.0),
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(5.0, 0.0),
      times, /*init_pos_std*/ 100.0, bearing_std, /*seed*/ 137);
  auto motion = std::make_shared<ConstantVelocity2D>(0.05);
  const EkfEstimator ekf(motion, 5.0);
  const UkfEstimator ukf(motion, 5.0);
  const ParticleFilterEstimator pf(motion, /*N*/ 2000, /*v_std*/ 5.0,
                                   /*ess_frac*/ 0.5, /*seed*/ 137);

  const RunOutput e = run(ekf, s, /*gate*/ 1500.0, /*cutoff*/ 300.0,
                          /*confirm*/ 1, /*delete*/ 8, /*miss_timeout*/ 90.0);
  const RunOutput u = run(ukf, s, 1500.0, 300.0, 1, 8, 90.0);
  const RunOutput p = run(pf,  s, 1500.0, 300.0, 1, 8, 90.0);

  std::fprintf(stderr,
               "\n[BearingOnlyMovingSensor] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[BearingOnlyMovingSensor] UKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[BearingOnlyMovingSensor] PF  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count,
               p.mean_ospa, p.id_switches, p.final_track_count);
}
```

- [ ] **Step 2: Build and run, capture numbers**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=FilterComparison.BearingOnlyMovingSensor 2>&1 | tail -15`

Expected: PASS with stderr printing three filters' (mean_ospa, id_switches, tracks).

**IMPORTANT — copy all three numbers into your report.** They go into Task 6's eval log.

- [ ] **Step 3: Commit**

```bash
git add tests/scenario/test_filter_comparison.cpp
git commit -m "scenario: EKF/UKF/PF comparison on bearing-only moving-sensor scenario"
```

---

## Task 6: Documentation update

**Files:**
- Modify: `docs/algorithms/estimation.md`
- Modify: `docs/algorithms/evaluation-log.md`

- [ ] **Step 1: Update `docs/algorithms/estimation.md`**

In the "Measurement models" section, add a paragraph at the end:

```markdown
**Sensor pose.** The `RangeBearing2D` and `Bearing2D` models accept an
optional `sensor_position_enu` parameter (default `(0, 0)`). When supplied,
`r` and `β` are computed as `‖p_target − p_sensor‖` and
`atan2(py − sy, px − sx)`; the Jacobian's position partials are evaluated
at the shifted position. This is what makes moving-sensor bearing-only
fusion tractable: the geometry that creates parallax is captured in
`h(x)` rather than baked into the adapter's projection layer. See
`Measurement.sensor_position_enu` for how adapters populate it.
```

- [ ] **Step 2: Append the eval log entry**

Substitute the three numbers from Task 5 into:

```markdown
## 2026-06-01 — Bearing-only with moving sensor (parallax)

`Measurement.sensor_position_enu` is now wired through every estimator and
associator's measurement-model call path. The new scenario builder
`buildBearingOnlyMovingSensorScenario` emits an initial wide-covariance
Position2D seed (σ = 100 m) followed by 60 s of `Bearing2D` measurements
(σ = 3°) from a sensor moving +x at 5 m/s, target stationary at
(1000, 100). Range is observable via the sensor's parallax over the run.

| Filter | mean OSPA (m) |
|--------|----------------|
| EKF (CV)        | <ekf> |
| UKF (CV)        | <ukf> |
| PF  (CV, N=2000)| <pf>  |

**Takeaway.** <One paragraph: where the PF lands relative to EKF/UKF.
Expected shape: the PF should match or beat the UKF here, with the gap
appearing during the early-convergence window when the posterior on range
is genuinely banana-shaped. If the PF still ties or loses, hypothesize
why — most likely candidates: (a) the prior is wide enough that range
converges within a few scans and the unimodal Gaussian approximation is
fine, (b) the bearing noise is too tight for the Monte Carlo variance to
matter, (c) the scenario doesn't sit in the convergence window long
enough to expose the non-Gaussian phase.>

**Methodology notes.** Single seed (137). Multi-seed sweep is the
straightforward next step. The scenario can be made harder by widening
the initial-position prior or tightening the bearing noise; both shift
the geometry deeper into the PF-favoring regime.

**Open follow-ups.** (1) Multi-seed sweep on this scenario. (2) Sweep
over sensor velocity (slower = less parallax = harder = PF should win
by more). (3) Add a slowly-moving target variant. (4) Use this scenario
to test IMM (CT modes with prescribed turn rates) — should be no
different from CV here since the target is stationary, but it
establishes the comparison baseline.
```

- [ ] **Step 3: Commit**

```bash
git add docs/algorithms/estimation.md docs/algorithms/evaluation-log.md
git commit -m "docs: sensor pose on measurements + bearing-only moving-sensor result"
```

---

## Task 7: Full sweep + plan commit

- [ ] **Step 1: Run the full suite**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests 2>&1 | tail -3`
Expected: ~167 tests pass.

- [ ] **Step 2: Commit the plan**

```bash
git add docs/superpowers/plans/2026-06-01-sensor-pose-on-measurement.md
git commit -m "plan: sensor pose on measurement (executed)"
```

Plan complete. With §14.1 in place, §14.5 (close-range precision sensors) and §14.4 (own-ship-motion bearing-only scenarios) become unblockable.
