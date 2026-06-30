# Synthetic Multi-Target + Shore-Clutter Bench Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add parametric multi-target geometry generators and a stationary shore-clutter injector with perfect ground truth, plus an in-memory coastline hook so a synthetic scenario's own land mask both seeds the clutter and drives the land model — enabling a clean `use_land_model` A/B on perfect truth.

**Architecture:** Pure scenario builders in `core/scenario/Builders.{hpp,cpp}` (deterministic, no I/O, seeded `std::mt19937`). A synthetic coastline is built in code as a `CoastlineGeometry` + `geo::Datum`; the same object seeds the fixed shore returns and is handed to the existing `CoastlineModel` via a new optional `ScenarioRun::syntheticCoastline()` hook, which `Sweep.cpp` prefers over the file path. Scenario registration lives in `adapters/benchmark/SimScenarioRun.cpp`.

**Tech Stack:** C++17, Eigen, GoogleTest, CMake/Conan. No new dependencies.

## Global Constraints

- C++17 only; no later standard; no new third-party dependency.
- Hexagonal: builders and the synthetic shore live in `core/` with zero I/O, zero wall-clock, zero unseeded RNG. Determinism: same seed → byte-identical measurements.
- All synthetic geometry is generated directly in ENU (metres, SI). Truth-emission order per scan is documented and deterministic.
- `core/scenario/Builders.cpp` and `core/land/CoastlineGeometry.cpp` are both already in the `navtracker_core` CMake target — the new builders may use `CoastlineGeometry` and `geo::Datum`/`geo::Geodetic` with no new link dependency.
- `LandPolygon::outer` vertices are stored as `Eigen::Vector2d(lon_deg, lat_deg)` — longitude first.
- Existing-scenario behaviour must not change: the new `ScenarioRun` hook defaults to "no coastline".
- Stationary shore clutter is a fixed-position point process (same nominal ENU positions every scan), distinct from the existing per-scan uniform-Poisson `buildClutterCrossingScenario`. Shore-clutter measurements use `source_id = "sim_shore"`, `SensorKind::ArpaTtm`, and create **no** `TruthSample`.

---

### Task 1: Parametric geometry builders (parallel lanes, crossing-angle, convoy)

**Files:**
- Modify: `core/scenario/Builders.hpp` (add three declarations)
- Modify: `core/scenario/Builders.cpp` (add three definitions; add `kPi` constant)
- Test: `tests/scenario/test_builders.cpp` (append tests)

**Interfaces:**
- Consumes: existing anonymous-namespace helpers `makeMeasurement`, `makeTruth` in `Builders.cpp`; `Scenario` (`core/scenario/Truth.hpp`).
- Produces:
  - `Scenario buildParallelLaneScenario(int n_targets, double lane_spacing_m, const Eigen::Vector2d& start, const Eigen::Vector2d& velocity, const std::vector<double>& sample_times_seconds, double pos_noise_std_m, std::uint32_t seed = 0);`
  - `Scenario buildCrossingAngleScenario(double crossing_angle_deg, double speed_mps, const Eigen::Vector2d& crossing_point, const std::vector<double>& sample_times_seconds, double pos_noise_std_m, std::uint32_t seed = 0);`
  - `Scenario buildConvoyScenario(int n_targets, double gap_m, double speed_mps, double overtaker_speed_mps, const std::vector<double>& sample_times_seconds, double pos_noise_std_m, std::uint32_t seed = 0);`

- [ ] **Step 1: Write the failing tests**

Append to `tests/scenario/test_builders.cpp`:

```cpp
#include <cmath>
using navtracker::buildConvoyScenario;
using navtracker::buildCrossingAngleScenario;
using navtracker::buildParallelLaneScenario;

TEST(Builders, ParallelLaneEmitsNTargetsPerScanSpacedPerpendicular) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario s = buildParallelLaneScenario(
      /*n_targets=*/4, /*lane_spacing_m=*/50.0,
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),  // heading +x
      times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 8u);          // 4 targets x 2 scans
  ASSERT_EQ(s.measurements.size(), 8u);
  // Lanes offset perpendicular to +x velocity => along +y, spacing 50 m.
  EXPECT_DOUBLE_EQ(s.truth[0].position.y(), 0.0);
  EXPECT_DOUBLE_EQ(s.truth[1].position.y(), 50.0);
  EXPECT_DOUBLE_EQ(s.truth[3].position.y(), 150.0);
  EXPECT_EQ(s.truth[0].truth_id, 1u);
  EXPECT_EQ(s.truth[3].truth_id, 4u);
}

TEST(Builders, CrossingAngleVelocitiesSubtendRequestedAngle) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildCrossingAngleScenario(
      /*crossing_angle_deg=*/60.0, /*speed_mps=*/20.0,
      Eigen::Vector2d(0.0, 0.0), times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 6u);          // 2 targets x 3 scans
  const Eigen::Vector2d va = s.truth[0].velocity;
  const Eigen::Vector2d vb = s.truth[1].velocity;
  const double ang = std::atan2(va.x() * vb.y() - va.y() * vb.x(),
                                va.dot(vb));
  EXPECT_NEAR(std::abs(ang) * 180.0 / M_PI, 60.0, 1e-6);
  EXPECT_NEAR(va.norm(), 20.0, 1e-9);
  EXPECT_NEAR(vb.norm(), 20.0, 1e-9);
}

TEST(Builders, ConvoyEmitsConvoyPlusOvertaker) {
  const std::vector<double> times{1.0};
  const Scenario s = buildConvoyScenario(
      /*n_targets=*/3, /*gap_m=*/80.0, /*speed_mps=*/5.0,
      /*overtaker_speed_mps=*/15.0, times, /*pos_noise_std_m=*/0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 4u);          // 3 convoy + 1 overtaker
  EXPECT_EQ(s.truth[3].truth_id, 4u);     // overtaker emitted last
  EXPECT_GT(s.truth[3].velocity.x(), s.truth[0].velocity.x());  // faster
}

TEST(Builders, GeometryBuildersDeterministicForSameSeed) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario a = buildConvoyScenario(2, 80.0, 5.0, 15.0, times, 4.0, 9);
  const Scenario b = buildConvoyScenario(2, 80.0, 5.0, 15.0, times, 4.0, 9);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.measurements[i].value(0), b.measurements[i].value(0));
    EXPECT_DOUBLE_EQ(a.measurements[i].value(1), b.measurements[i].value(1));
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -20`
Expected: compile error — `buildParallelLaneScenario` / `buildCrossingAngleScenario` / `buildConvoyScenario` not declared.

- [ ] **Step 3: Add the declarations to `Builders.hpp`**

Append before the closing `}  // namespace navtracker`:

```cpp
// N constant-velocity targets on parallel lanes, all sharing `velocity`.
// Lane i (i = 0..n_targets-1) is offset from `start` by i*lane_spacing_m along
// the unit vector perpendicular to `velocity` (rotated +90 deg: (-vy, vx)/|v|).
// Truth ids are 1..n_targets, emitted in lane order each scan. `velocity` must
// be non-zero. Stresses track resolution / merge as spacing shrinks.
Scenario buildParallelLaneScenario(
    int n_targets,
    double lane_spacing_m,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two CV targets that both pass through `crossing_point` at the mid-time of
// `sample_times_seconds`, subtending `crossing_angle_deg`. Target A heads +x
// at `speed_mps`; target B heads at `crossing_angle_deg` from +x at the same
// speed. Truth emitted in (A, B) order each scan (ids 1, 2). Sweep the angle
// externally (e.g. 30/60/90) to probe angle-dependent association.
Scenario buildCrossingAngleScenario(
    double crossing_angle_deg,
    double speed_mps,
    const Eigen::Vector2d& crossing_point,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// `n_targets` CV targets in a single lane along +x at `speed_mps`, spaced by
// `gap_m` (member i starts at x = -i*gap_m, y = 0), plus one faster overtaker
// at `overtaker_speed_mps` starting behind the convoy on a parallel track
// 25 m to the side. Truth emitted convoy-first (ids 1..n_targets) then the
// overtaker (id n_targets+1) each scan. Stresses in-line association +
// overtaking.
Scenario buildConvoyScenario(
    int n_targets,
    double gap_m,
    double speed_mps,
    double overtaker_speed_mps,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);
```

- [ ] **Step 4: Add the definitions to `Builders.cpp`**

Add `kPi` to the existing anonymous namespace (just after the `namespace {` on line 10):

```cpp
constexpr double kPi = 3.14159265358979323846;
```

Add the three definitions before the closing `}  // namespace navtracker` (after `buildBearingOnlyMovingSensorScenario`):

```cpp
Scenario buildParallelLaneScenario(int n_targets, double lane_spacing_m,
                                   const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& velocity,
                                   const std::vector<double>& times,
                                   double pos_noise_std_m, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const Eigen::Vector2d dir = velocity.normalized();
  const Eigen::Vector2d perp(-dir.y(), dir.x());
  Scenario s;
  for (double t : times) {
    for (int i = 0; i < n_targets; ++i) {
      const Eigen::Vector2d lane_start =
          start + perp * (static_cast<double>(i) * lane_spacing_m);
      const Eigen::Vector2d truth_pos = lane_start + velocity * t;
      s.truth.push_back(
          makeTruth(truth_pos, velocity, t, static_cast<std::uint64_t>(i + 1)));
      const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                  truth_pos.y() + noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
    }
  }
  return s;
}

Scenario buildCrossingAngleScenario(double crossing_angle_deg, double speed_mps,
                                    const Eigen::Vector2d& crossing_point,
                                    const std::vector<double>& times,
                                    double pos_noise_std_m, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const double th = crossing_angle_deg * kPi / 180.0;
  const Eigen::Vector2d vel_a(speed_mps, 0.0);
  const Eigen::Vector2d vel_b(speed_mps * std::cos(th), speed_mps * std::sin(th));
  const double t_mid =
      times.empty() ? 0.0 : 0.5 * (times.front() + times.back());
  const Eigen::Vector2d start_a = crossing_point - vel_a * t_mid;
  const Eigen::Vector2d start_b = crossing_point - vel_b * t_mid;
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d ta = start_a + vel_a * t;
    const Eigen::Vector2d tb = start_b + vel_b * t;
    s.truth.push_back(makeTruth(ta, vel_a, t, 1));
    s.truth.push_back(makeTruth(tb, vel_b, t, 2));
    const Eigen::Vector2d na(ta.x() + noise(rng), ta.y() + noise(rng));
    const Eigen::Vector2d nb(tb.x() + noise(rng), tb.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(na, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(nb, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildConvoyScenario(int n_targets, double gap_m, double speed_mps,
                             double overtaker_speed_mps,
                             const std::vector<double>& times,
                             double pos_noise_std_m, std::uint32_t seed) {
  constexpr double kOvertakerLateralOffsetM = 25.0;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  const Eigen::Vector2d vel(speed_mps, 0.0);
  const Eigen::Vector2d vel_ot(overtaker_speed_mps, 0.0);
  const Eigen::Vector2d ot_start(
      -static_cast<double>(n_targets) * gap_m - 100.0, kOvertakerLateralOffsetM);
  Scenario s;
  for (double t : times) {
    for (int i = 0; i < n_targets; ++i) {
      const Eigen::Vector2d start_i(-static_cast<double>(i) * gap_m, 0.0);
      const Eigen::Vector2d truth_pos = start_i + vel * t;
      s.truth.push_back(
          makeTruth(truth_pos, vel, t, static_cast<std::uint64_t>(i + 1)));
      const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                  truth_pos.y() + noise(rng));
      s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
    }
    const Eigen::Vector2d ot_pos = ot_start + vel_ot * t;
    s.truth.push_back(makeTruth(ot_pos, vel_ot, t,
                                static_cast<std::uint64_t>(n_targets + 1)));
    const Eigen::Vector2d ot_noisy(ot_pos.x() + noise(rng),
                                   ot_pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(ot_noisy, t, pos_noise_std_m));
  }
  return s;
}
```

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders --output-on-failure`
Expected: all `Builders.*` tests PASS.

- [ ] **Step 6: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "bench(E): parametric geometry builders (parallel lanes / crossing-angle / convoy)"
```

---

### Task 2: Synthetic shore + stationary shore-clutter injector

**Files:**
- Modify: `core/scenario/Builders.hpp` (add `SyntheticShore`, `buildSyntheticShore`, `addShoreClutter`; add includes)
- Modify: `core/scenario/Builders.cpp` (add `makeShoreMeasurement` helper + two definitions; add includes)
- Test: `tests/scenario/test_builders.cpp` (append tests, including ENU→geodetic round-trip via `CoastlineModel`)

**Interfaces:**
- Consumes: `CoastlineGeometry`, `LandPolygon`, `CoastlinePriorParams` (`core/land/CoastlineGeometry.hpp`); `geo::Datum`, `geo::Geodetic` (`core/geo/Datum.hpp`, `core/geo/Wgs84.hpp`); `CoastlineModel` (`core/land/CoastlineModel.hpp`, test only); `Scenario` (`core/scenario/Truth.hpp`).
- Produces:
  - `struct SyntheticShore { CoastlineGeometry geometry; geo::Datum datum; std::vector<Eigen::Vector2d> clutter_enu_points; };`
  - `SyntheticShore buildSyntheticShore(const geo::Geodetic& datum_origin, double shore_y_m, double extent_m, double land_depth_m, double pier_width_m, double pier_length_m, int n_clutter, const CoastlinePriorParams& params = {});`
  - `Scenario addShoreClutter(Scenario base, const geo::Datum& datum, const std::vector<Eigen::Vector2d>& clutter_enu_points, double detection_prob, double pos_noise_std_m, std::uint32_t seed = 0);`

- [ ] **Step 1: Write the failing tests**

Append to `tests/scenario/test_builders.cpp`:

```cpp
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineModel.hpp"
using navtracker::addShoreClutter;
using navtracker::buildSyntheticShore;
using navtracker::CoastlineModel;
using navtracker::SyntheticShore;

TEST(ShoreClutter, SyntheticShoreClutterPointsReadHardGatePrior) {
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0},
      /*shore_y_m=*/500.0, /*extent_m=*/1500.0, /*land_depth_m=*/400.0,
      /*pier_width_m=*/40.0, /*pier_length_m=*/150.0, /*n_clutter=*/30);
  ASSERT_EQ(shore.clutter_enu_points.size(), 30u);
  const CoastlineModel model(shore.geometry, shore.datum);
  // Every deep-inland clutter point sits in the hard-gate plateau (c ~ 1).
  for (const auto& p : shore.clutter_enu_points) {
    EXPECT_GE(model.clutterPrior(p), 0.95);
  }
  // A point far out to sea reads ~0 (open water).
  EXPECT_NEAR(model.clutterPrior(Eigen::Vector2d(0.0, -500.0)), 0.0, 1e-9);
}

TEST(ShoreClutter, InjectorAddsFixedReturnsWithoutTruth) {
  // Two-scan base scenario, one real target.
  const std::vector<double> times{1.0, 2.0};
  navtracker::Scenario base = navtracker::buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0), times, 0.0, 1, 1);
  const std::size_t base_truth = base.truth.size();
  const std::size_t base_meas = base.measurements.size();
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0}, 500.0, 1500.0, 400.0,
      40.0, 150.0, /*n_clutter=*/5);
  navtracker::Scenario s = addShoreClutter(
      base, shore.datum, shore.clutter_enu_points,
      /*detection_prob=*/1.0, /*pos_noise_std_m=*/0.0, /*seed=*/7);
  // P_D = 1 => 5 clutter returns x 2 scans added; truth unchanged.
  EXPECT_EQ(s.truth.size(), base_truth);
  EXPECT_EQ(s.measurements.size(), base_meas + 10u);
  EXPECT_TRUE(s.datum.has_value());
  std::size_t shore_count = 0;
  for (const auto& m : s.measurements) {
    if (m.source_id == "sim_shore") {
      ++shore_count;
      EXPECT_EQ(m.sensor, navtracker::SensorKind::ArpaTtm);
    }
  }
  EXPECT_EQ(shore_count, 10u);
  // Measurements remain time-sorted after injection.
  for (std::size_t i = 1; i < s.measurements.size(); ++i) {
    EXPECT_LE(s.measurements[i - 1].time.seconds(),
              s.measurements[i].time.seconds());
  }
}

TEST(ShoreClutter, ClutterPositionsRepeatAcrossScans) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  navtracker::Scenario base = navtracker::buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0), times, 0.0, 1, 1);
  const SyntheticShore shore = buildSyntheticShore(
      navtracker::geo::Geodetic{42.35, -71.05, 0.0}, 500.0, 1500.0, 400.0,
      40.0, 150.0, /*n_clutter=*/1);
  navtracker::Scenario s = addShoreClutter(base, shore.datum,
                                           shore.clutter_enu_points,
                                           /*detection_prob=*/1.0,
                                           /*pos_noise_std_m=*/0.0, /*seed=*/3);
  // With zero noise, the single shore point lands at the same ENU position
  // on every scan (fixed-position process, not redrawn).
  std::vector<Eigen::Vector2d> shore_pos;
  for (const auto& m : s.measurements)
    if (m.source_id == "sim_shore") shore_pos.push_back(m.value.head<2>());
  ASSERT_EQ(shore_pos.size(), 3u);
  EXPECT_DOUBLE_EQ(shore_pos[0].x(), shore_pos[1].x());
  EXPECT_DOUBLE_EQ(shore_pos[1].x(), shore_pos[2].x());
  EXPECT_DOUBLE_EQ(shore_pos[0].y(), shore_pos[2].y());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -20`
Expected: compile error — `SyntheticShore` / `buildSyntheticShore` / `addShoreClutter` not declared.

- [ ] **Step 3: Add declarations + includes to `Builders.hpp`**

At the top of `Builders.hpp`, add after the existing includes (`#include "core/scenario/Truth.hpp"`):

```cpp
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineGeometry.hpp"
```

Append before the closing `}  // namespace navtracker`:

```cpp
// A synthetic coastline plus the fixed shore-clutter positions derived from
// it. The same object seeds the stationary returns (addShoreClutter) AND is
// handed to the land model (ScenarioRun::syntheticCoastline) so an A/B of
// use_land_model runs against one shoreline. Deterministic: no RNG.
struct SyntheticShore {
  CoastlineGeometry geometry;
  geo::Datum datum;
  std::vector<Eigen::Vector2d> clutter_enu_points;  // ENU, deep inland
};

// Build a simple synthetic shoreline about `datum_origin`: land occupies
// y >= shore_y_m up to y = shore_y_m + land_depth_m, across x in
// [-extent_m, extent_m], with one rectangular pier of width pier_width_m
// protruding pier_length_m into the water at x = 0. The ENU outline is
// converted to geodetic via the datum and stored as one LandPolygon (outer
// ring, lon/lat). `n_clutter` stationary returns are placed deep inland
// (y = shore_y_m + 0.5*land_depth_m, spread across x) — the hard-gate region.
SyntheticShore buildSyntheticShore(
    const geo::Geodetic& datum_origin,
    double shore_y_m,
    double extent_m,
    double land_depth_m,
    double pier_width_m,
    double pier_length_m,
    int n_clutter,
    const CoastlinePriorParams& params = {});

// Add stationary shore clutter to `base`. For each distinct scan timestamp in
// base.measurements, each point in `clutter_enu_points` emits a Position2D
// measurement (SensorKind::ArpaTtm, source_id "sim_shore") at its fixed ENU
// position plus isotropic Gaussian noise, with probability `detection_prob`
// (seeded Bernoulli). NO TruthSample is created for clutter. Sets base.datum
// = datum and returns base with measurements re-sorted by time. The same
// nominal positions recur every scan — the defining property versus the
// uniform-Poisson buildClutterCrossingScenario.
Scenario addShoreClutter(
    Scenario base,
    const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& clutter_enu_points,
    double detection_prob,
    double pos_noise_std_m,
    std::uint32_t seed = 0);
```

- [ ] **Step 4: Add the helper + definitions to `Builders.cpp`**

Add `makeShoreMeasurement` to the anonymous namespace (after `makeBearingMeasurement`, before the closing `}  // namespace`):

```cpp
Measurement makeShoreMeasurement(const Eigen::Vector2d& noisy_pos,
                                 double t_seconds, double std_m) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::ArpaTtm;  // radar-like fixed clutter
  m.source_id = "sim_shore";
  m.model = MeasurementModel::Position2D;
  m.value = noisy_pos;
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}
```

Append before the closing `}  // namespace navtracker`:

```cpp
SyntheticShore buildSyntheticShore(const geo::Geodetic& datum_origin,
                                   double shore_y_m, double extent_m,
                                   double land_depth_m, double pier_width_m,
                                   double pier_length_m, int n_clutter,
                                   const CoastlinePriorParams& params) {
  const geo::Datum datum(datum_origin);
  const double S = shore_y_m;
  const double T = shore_y_m + land_depth_m;
  const double L = -extent_m, R = extent_m;
  const double pw = 0.5 * pier_width_m;
  const double pl = pier_length_m;
  // ENU outer ring: land interior above the shoreline with a pier notch
  // protruding into the water at x = 0.
  const std::vector<Eigen::Vector2d> ring_enu = {
      {L, S},      {-pw, S}, {-pw, S - pl}, {pw, S - pl},
      {pw, S},     {R, S},   {R, T},        {L, T}};
  LandPolygon poly;
  poly.outer.reserve(ring_enu.size());
  for (const auto& p : ring_enu) {
    const geo::Geodetic g =
        datum.toGeodetic(Eigen::Vector3d(p.x(), p.y(), 0.0));
    poly.outer.emplace_back(g.lon_deg, g.lat_deg);
  }
  CoastlineGeometry geometry({poly}, params);
  std::vector<Eigen::Vector2d> clutter;
  clutter.reserve(static_cast<std::size_t>(std::max(0, n_clutter)));
  const double y_inland = shore_y_m + 0.5 * land_depth_m;
  const double x0 = -0.8 * extent_m, x1 = 0.8 * extent_m;
  for (int i = 0; i < n_clutter; ++i) {
    const double frac =
        n_clutter <= 1 ? 0.5 : static_cast<double>(i) / (n_clutter - 1);
    clutter.emplace_back(x0 + frac * (x1 - x0), y_inland);
  }
  return SyntheticShore{std::move(geometry), datum, std::move(clutter)};
}

Scenario addShoreClutter(Scenario base, const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& clutter_enu_points,
                         double detection_prob, double pos_noise_std_m,
                         std::uint32_t seed) {
  // Distinct scan times in first-seen order (builders emit time-grouped).
  std::vector<double> scan_times;
  for (const auto& m : base.measurements) {
    const double t = m.time.seconds();
    if (scan_times.empty() || scan_times.back() != t) scan_times.push_back(t);
  }
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    for (const auto& pt : clutter_enu_points) {
      if (u(rng) < detection_prob) {
        const Eigen::Vector2d noisy(pt.x() + noise(rng), pt.y() + noise(rng));
        base.measurements.push_back(
            makeShoreMeasurement(noisy, t, pos_noise_std_m));
      }
    }
  }
  std::stable_sort(base.measurements.begin(), base.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time.seconds() < b.time.seconds();
                   });
  base.datum = datum;
  return base;
}
```

(`<algorithm>`, `<cmath>`, `<random>` are already included at the top of `Builders.cpp`.)

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R "Builders|ShoreClutter" --output-on-failure`
Expected: all `Builders.*` and `ShoreClutter.*` tests PASS.

- [ ] **Step 6: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "bench(E): synthetic shore geometry + stationary shore-clutter injector"
```

---

### Task 3: In-memory coastline hook on `ScenarioRun` + Sweep wiring

**Files:**
- Modify: `core/benchmark/ScenarioRun.hpp` (add `syntheticCoastline()` hook + includes)
- Modify: `core/benchmark/Sweep.cpp` (prefer in-memory coastline over file path)
- Test: `tests/benchmark/test_scenario_run_port.cpp` (append hook-default + override tests)

**Interfaces:**
- Consumes: `CoastlineGeometry` (`core/land/CoastlineGeometry.hpp`).
- Produces: `virtual std::optional<CoastlineGeometry> ScenarioRun::syntheticCoastline() const;` (default `std::nullopt`). Sweep builds `CoastlineModel(geometry, *scen.datum)` from it when present and `config.use_land_model`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/benchmark/test_scenario_run_port.cpp`:

```cpp
#include <optional>
#include "core/land/CoastlineGeometry.hpp"
using navtracker::CoastlineGeometry;
using navtracker::CoastlinePriorParams;
using navtracker::LandPolygon;

TEST(ScenarioRunPort, SyntheticCoastlineDefaultsToNone) {
  FakeScenarioRun run;
  EXPECT_FALSE(run.syntheticCoastline().has_value());
}

namespace {
class CoastlineScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override { return {"coast", false, 1}; }
  Scenario generate(std::uint64_t) override { return {}; }
  std::optional<CoastlineGeometry> syntheticCoastline() const override {
    LandPolygon poly;
    poly.outer = {{-71.0, 42.0}, {-70.0, 42.0}, {-70.0, 43.0}, {-71.0, 43.0}};
    return CoastlineGeometry({poly}, CoastlinePriorParams{});
  }
};
}  // namespace

TEST(ScenarioRunPort, SyntheticCoastlineOverrideReturnsGeometry) {
  CoastlineScenarioRun run;
  const auto geom = run.syntheticCoastline();
  ASSERT_TRUE(geom.has_value());
  EXPECT_FALSE(geom->empty());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -20`
Expected: compile error — `syntheticCoastline` is not a member of `ScenarioRun`.

- [ ] **Step 3: Add the hook to `ScenarioRun.hpp`**

Add includes near the top (with the other includes):

```cpp
#include <optional>

#include "core/land/CoastlineGeometry.hpp"
```

Add the virtual method to `class ScenarioRun` (after `seedSensorBiasEstimator`, before the closing `};`):

```cpp
  // Optional in-memory coastline for synthetic scenarios. Default = none, so
  // every existing scenario is untouched. When present AND config.use_land_model
  // AND Scenario.datum is set, Sweep builds a CoastlineModel from this geometry
  // (in preference to coastline_geojson_path) so the synthetic land mask that
  // seeds the shore clutter also drives the land model. Real-data replay
  // scenarios leave this null and keep using coastline_geojson_path.
  virtual std::optional<CoastlineGeometry> syntheticCoastline() const {
    return std::nullopt;
  }
```

- [ ] **Step 4: Update the Sweep land-wiring block**

In `core/benchmark/Sweep.cpp`, replace the existing land-wiring block (currently the `std::shared_ptr<CoastlineModel> land;` block at lines ~341–357) with:

```cpp
          // Task 6 (land) + Task E (synthetic): build and wire a CoastlineModel
          // when the config opts in. Prefer an in-memory synthetic coastline
          // from the ScenarioRun (synthetic shore-clutter scenarios); otherwise
          // fall back to a GeoJSON fixture path (real-data replays). The
          // shared_ptr outlives the synchronous runBenchPmbm call below (the
          // tracker holds only a raw pointer). The datum is fixed for the whole
          // run, so no datum-sink registration is needed.
          std::shared_ptr<CoastlineModel> land;
          if (config.use_land_model && scen.datum.has_value()) {
            std::optional<CoastlineGeometry> synth =
                scenario_ptr->syntheticCoastline();
            if (synth.has_value()) {
              land = std::make_shared<CoastlineModel>(std::move(*synth),
                                                      *scen.datum);
              tracker.setLandModel(land.get());
            } else if (!desc.coastline_geojson_path.empty()) {
              std::ifstream probe(desc.coastline_geojson_path);
              if (probe.good()) {
                try {
                  auto geom = loadCoastlineGeoJson(desc.coastline_geojson_path,
                                                   CoastlinePriorParams{});
                  land = std::make_shared<CoastlineModel>(std::move(geom),
                                                          *scen.datum);
                  tracker.setLandModel(land.get());
                } catch (const std::exception&) {
                  // GeoJSON parse failure — proceed without land model
                }
              }
            }
          }
```

Ensure `#include <optional>` is present in `Sweep.cpp` (add it with the other standard includes if missing).

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R ScenarioRunPort --output-on-failure`
Expected: all `ScenarioRunPort.*` tests PASS.

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/ScenarioRun.hpp core/benchmark/Sweep.cpp tests/benchmark/test_scenario_run_port.cpp
git commit -m "bench(E): in-memory coastline hook on ScenarioRun; Sweep prefers it over file path"
```

---

### Task 4: Register the new scenarios

**Files:**
- Modify: `adapters/benchmark/SimScenarioRun.cpp` (add detection tables, 7 `ScenarioRun` subclasses, register in `defaultSimScenarios`)
- Test: `tests/benchmark/test_sim_scenario_run.cpp` (update count 10→17; add label assertions; shore-clutter property assertions)

**Interfaces:**
- Consumes: Task 1 builders (`buildParallelLaneScenario`, `buildCrossingAngleScenario`, `buildConvoyScenario`), Task 2 (`buildSyntheticShore`, `addShoreClutter`, `SyntheticShore`), Task 3 hook (`syntheticCoastline`).
- Produces: 7 new scenario labels in `defaultSimScenarios()`: `parallel_lanes_dense`, `crossing_30`, `crossing_60`, `crossing_90`, `convoy_overtake`, `shore_clutter_open`, `shore_clutter_nearshore`.

- [ ] **Step 1: Update the failing tests**

In `tests/benchmark/test_sim_scenario_run.cpp`, change the count in `ProducesExpectedDefaultScenarios` from `10u` to `17u` and add label assertions:

```cpp
  ASSERT_EQ(scenarios.size(), 17u);
  // ... existing 10 EXPECT_EQ(labels.count(...)) lines stay ...
  EXPECT_EQ(labels.count("parallel_lanes_dense"), 1u);
  EXPECT_EQ(labels.count("crossing_30"), 1u);
  EXPECT_EQ(labels.count("crossing_60"), 1u);
  EXPECT_EQ(labels.count("crossing_90"), 1u);
  EXPECT_EQ(labels.count("convoy_overtake"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_open"), 1u);
  EXPECT_EQ(labels.count("shore_clutter_nearshore"), 1u);
```

Add a new test asserting the shore-clutter scenarios expose a coastline and a datum:

```cpp
TEST(SimScenarioRun, ShoreClutterScenariosExposeCoastlineAndDatum) {
  const auto scenarios = navtracker::benchmark::defaultSimScenarios();
  int checked = 0;
  for (const auto& s : scenarios) {
    const std::string label = s->descriptor().label;
    if (label != "shore_clutter_open" && label != "shore_clutter_nearshore")
      continue;
    ++checked;
    EXPECT_TRUE(s->syntheticCoastline().has_value()) << label;
    const auto scen = s->generate(0);
    EXPECT_TRUE(scen.datum.has_value()) << label;
    bool has_shore = false;
    for (const auto& m : scen.measurements)
      if (m.source_id == "sim_shore") has_shore = true;
    EXPECT_TRUE(has_shore) << label;
  }
  EXPECT_EQ(checked, 2);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R SimScenarioRun --output-on-failure 2>&1 | tail -20`
Expected: FAIL — `ProducesExpectedDefaultScenarios` expects 17 but gets 10; new labels absent.

- [ ] **Step 3: Add detection tables + subclasses to `SimScenarioRun.cpp`**

Add includes at the top (with the existing includes):

```cpp
#include "core/geo/Wgs84.hpp"
```

Add detection-table helpers to the anonymous namespace (after `bearingOnlyTable`):

```cpp
// Shore-clutter scenarios: AIS-cooperative targets + radar-like (ArpaTtm)
// stationary shore returns. λ_C for the radar channel reflects ~30 fixed
// returns over the ~2.7e6 m² scene (30/2.7e6 ≈ 1.1e-5 m⁻², declared 1e-5).
std::vector<SensorDetectionEntry> shoreClutterTable() {
  return {{SensorKind::Ais, MeasurementModel::Position2D,
           DetectionParams{0.95, 1e-6}},
          {SensorKind::ArpaTtm, MeasurementModel::Position2D,
           DetectionParams{0.95, 1e-5}}};
}

// Fixed synthetic shoreline shared by every shore-clutter scenario. Pure /
// deterministic, so generate() and syntheticCoastline() agree.
SyntheticShore makeBenchShore() {
  return buildSyntheticShore(navtracker::geo::Geodetic{42.35, -71.05, 0.0},
                             /*shore_y_m=*/500.0, /*extent_m=*/1500.0,
                             /*land_depth_m=*/400.0, /*pier_width_m=*/40.0,
                             /*pier_length_m=*/150.0, /*n_clutter=*/30);
}
```

Add the seven subclasses (after `NonCooperativeScenarioRun`, before the closing `}  // namespace`):

```cpp
class ParallelLanesDenseScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("parallel_lanes_dense", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // 4 lanes, 40 m apart, all heading +x at 10 m/s. Tight spacing stresses
    // track resolution / merge.
    return buildParallelLaneScenario(
        /*n_targets=*/4, /*lane_spacing_m=*/40.0, Eigen::Vector2d(-400.0, 0.0),
        Eigen::Vector2d(10.0, 0.0), linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0, static_cast<std::uint32_t>(seed));
  }
};

class CrossingAngleScenarioRun : public ScenarioRun {
 public:
  CrossingAngleScenarioRun(const char* label, double angle_deg)
      : label_(label), angle_deg_(angle_deg) {}
  ScenarioDescriptor descriptor() const override {
    return describe(label_, cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildCrossingAngleScenario(angle_deg_, /*speed_mps=*/20.0,
                                      Eigen::Vector2d(0.0, 0.0),
                                      linearSeconds(1, 40),
                                      /*pos_noise_std_m=*/8.0,
                                      static_cast<std::uint32_t>(seed));
  }

 private:
  const char* label_;
  double angle_deg_;
};

class ConvoyOvertakeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("convoy_overtake", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // 3 in-line targets 80 m apart at 5 m/s, plus a 15 m/s overtaker.
    return buildConvoyScenario(/*n_targets=*/3, /*gap_m=*/80.0,
                               /*speed_mps=*/5.0, /*overtaker_speed_mps=*/15.0,
                               linearSeconds(1, 60), /*pos_noise_std_m=*/5.0,
                               static_cast<std::uint32_t>(seed));
  }
};

class ShoreClutterOpenScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("shore_clutter_open", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Two real targets crossing in open water (y ~ 100, far from the shore at
    // y = 500 => land prior ~ 0, no suppression) + stationary shore clutter.
    const SyntheticShore shore = makeBenchShore();
    Scenario base = buildCrossingAngleScenario(
        /*crossing_angle_deg=*/90.0, /*speed_mps=*/15.0,
        Eigen::Vector2d(0.0, 100.0), linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0, static_cast<std::uint32_t>(seed));
    return addShoreClutter(std::move(base), shore.datum,
                           shore.clutter_enu_points, /*detection_prob=*/0.9,
                           /*pos_noise_std_m=*/8.0,
                           static_cast<std::uint32_t>(seed));
  }
  std::optional<CoastlineGeometry> syntheticCoastline() const override {
    return makeBenchShore().geometry;
  }
};

class ShoreClutterNearShoreScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("shore_clutter_nearshore", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // One slow AIS target 10 m offshore (y = 490, shore at y = 500 => land
    // prior c ≈ 0.4 => soft birth suppression, must NOT be deleted) +
    // stationary shore clutter. The "anchored ship near shore" validator.
    const SyntheticShore shore = makeBenchShore();
    Scenario base = buildStraightLineScenario(
        Eigen::Vector2d(-60.0, 490.0), Eigen::Vector2d(3.0, 0.0),
        linearSeconds(1, 40), /*pos_noise_std_m=*/8.0,
        static_cast<std::uint32_t>(seed), /*truth_id=*/1);
    return addShoreClutter(std::move(base), shore.datum,
                           shore.clutter_enu_points, /*detection_prob=*/0.9,
                           /*pos_noise_std_m=*/8.0,
                           static_cast<std::uint32_t>(seed));
  }
  std::optional<CoastlineGeometry> syntheticCoastline() const override {
    return makeBenchShore().geometry;
  }
};
```

Add the `#include <optional>` and the `CoastlineGeometry` symbol usage: `SimScenarioRun.cpp` already pulls `ScenarioRun.hpp` (which now includes `<optional>` and `CoastlineGeometry.hpp`) via `SimScenarioRun.hpp`, so no extra include is needed beyond `core/geo/Wgs84.hpp` and `core/scenario/Builders.hpp` (already included).

- [ ] **Step 4: Register the new scenarios in `defaultSimScenarios`**

Change `out.reserve(10);` to `out.reserve(17);` and append before `return out;`:

```cpp
  out.push_back(std::make_unique<ParallelLanesDenseScenarioRun>());
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_30", 30.0));
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_60", 60.0));
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_90", 90.0));
  out.push_back(std::make_unique<ConvoyOvertakeScenarioRun>());
  out.push_back(std::make_unique<ShoreClutterOpenScenarioRun>());
  out.push_back(std::make_unique<ShoreClutterNearShoreScenarioRun>());
```

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R SimScenarioRun --output-on-failure`
Expected: all `SimScenarioRun.*` tests PASS (count 17, new labels present, shore scenarios expose coastline + datum + `sim_shore` returns).

- [ ] **Step 6: Commit**

```bash
git add adapters/benchmark/SimScenarioRun.cpp tests/benchmark/test_sim_scenario_run.cpp
git commit -m "bench(E): register parametric geometry + shore-clutter scenarios"
```

---

### Task 5: Algorithm + learning documentation

**Files:**
- Create: `docs/algorithms/synthetic-clutter-bench.md` (four-section doc)
- Modify: `docs/learning/25-land-clutter-prior.md` (append a short "validated on synthetic perfect truth" note)

**Interfaces:** none (documentation only).

- [ ] **Step 1: Write the algorithm doc**

Create `docs/algorithms/synthetic-clutter-bench.md` with the four required sections (per CLAUDE.md): **Math** (the CV geometry generators and the fixed-position shore-clutter point process — for each scan, each fixed ENU point emits a detection with probability P_D, position = fixed + N(0, σ²I); contrast with the uniform-Poisson model where positions are redrawn each scan); **Assumptions** (perfect truth, seeded determinism, ENU generation with a per-scenario fictitious datum so the ENU→geodetic land query is consistent by construction, shore returns are on-land/hard-gate); **Rationale** (why fixed-position clutter is the right model for shore returns vs uniform-Poisson; why an in-memory shared land mask enables a clean A/B; why a synthetic shoreline rather than the philos GeoJSON — full control over target-vs-land geometry and perfect truth); **Ways to improve / what to test next** (RCS/multipath-modelled returns; range-bearing shore clutter; a near-shore-real-target sweep across offshore distance to map the soft-ramp survival boundary; multi-pier harbour geometry).

Cross-reference `docs/learning/25-land-clutter-prior.md` (intuitive intro) and `docs/superpowers/specs/2026-06-30-pmbm-synthetic-clutter-bench-design.md` (design).

- [ ] **Step 2: Append the learning-chapter note**

In `docs/learning/25-land-clutter-prior.md`, add a short subsection near the end: the land-clutter prior was first validated on philos **real** data (imperfect AIS-derived truth, anchored-ship ambiguity), and is now **also** validated on a synthetic scene with **perfect** truth, where we inject stationary shore clutter ourselves and switch the land model on/off. Plain English, one small diagram or none. Point to `docs/algorithms/synthetic-clutter-bench.md` for the precise reference.

- [ ] **Step 3: Commit**

```bash
git add docs/algorithms/synthetic-clutter-bench.md docs/learning/25-land-clutter-prior.md
git commit -m "docs(E): synthetic-clutter bench algorithm doc + learning-chapter note"
```

---

### Task 6: A/B measurement run + evaluation log

**Files:**
- Test: `tests/benchmark/test_synthetic_clutter_ab.cpp` (new; runs the A/B and asserts the success criterion as a regression guard)
- Modify: `CMakeLists.txt` (register the new test file in the `navtracker_tests` sources)
- Modify: `docs/algorithms/evaluation-log.md` (record results)
- Modify: `docs/algorithms/comparison-baselines.md` (record decision + Cl-2/Cl-3 placement)

**Interfaces:**
- Consumes: `defaultConfigs()` (select `imm_cv_ct_pmbm_coverage` and `imm_cv_ct_pmbm_coverage_land` by label), `defaultSimScenarios()`, `runSweep`, `MetricRow`.

- [ ] **Step 1: Write the A/B regression test**

Create `tests/benchmark/test_synthetic_clutter_ab.cpp`:

```cpp
#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
// Mean of a metric over seeds for (config, scenario).
double meanMetric(const std::vector<MetricRow>& rows, const std::string& config,
                  const std::string& scenario, const std::string& metric) {
  double sum = 0.0;
  int n = 0;
  for (const auto& r : rows) {
    if (r.config == config && r.scenario == scenario && r.metric == metric) {
      sum += r.value;
      ++n;
    }
  }
  return n == 0 ? 0.0 : sum / n;
}

std::vector<std::unique_ptr<ScenarioRun>> shoreScenariosOnly() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  for (auto& s : defaultSimScenarios()) {
    const std::string label = s->descriptor().label;
    if (label == "shore_clutter_open" || label == "shore_clutter_nearshore")
      out.push_back(std::move(s));
  }
  return out;
}
}  // namespace

TEST(SyntheticClutterAB, LandModelRemovesShoreOverCountKeepsRealTargets) {
  std::vector<Config> configs;
  for (const auto& c : defaultConfigs()) {
    if (c.label == "imm_cv_ct_pmbm_coverage" ||
        c.label == "imm_cv_ct_pmbm_coverage_land")
      configs.push_back(c);
  }
  ASSERT_EQ(configs.size(), 2u);

  SweepParams params;
  params.run_id = "synthetic_clutter_ab";
  params.synthetic_seeds = 5;
  const auto rows = runSweep(configs, shoreScenariosOnly(), params);

  for (const std::string scenario :
       {"shore_clutter_open", "shore_clutter_nearshore"}) {
    const double card_off = meanMetric(rows, "imm_cv_ct_pmbm_coverage",
                                       scenario, "card_err_mean");
    const double card_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "card_err_mean");
    // Land ON must reduce the (positive) over-count toward 0.
    EXPECT_LT(card_on, card_off) << scenario;
    EXPECT_LE(card_on, 1.0) << scenario;
    // Real targets are not dropped: lifetime_ratio stays healthy with land ON.
    const double life_on = meanMetric(rows, "imm_cv_ct_pmbm_coverage_land",
                                      scenario, "lifetime_ratio");
    EXPECT_GT(life_on, 0.5) << scenario;
  }
}
```

- [ ] **Step 2: Register the test in CMake**

In `CMakeLists.txt`, add `tests/benchmark/test_synthetic_clutter_ab.cpp` to the `navtracker_tests` source list (alongside the other `tests/benchmark/*.cpp` entries near line 300–306).

- [ ] **Step 3: Run the A/B test and verify it passes**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R SyntheticClutterAB --output-on-failure`
Expected: PASS. If `card_on` is not below `card_off`, STOP — the land model is not suppressing the synthetic shore clutter; investigate (likely the clutter is not landing in the hard-gate region, or the datum/geometry round-trip is off) before recording results.

- [ ] **Step 4: Capture the full metric table**

Run the full sweep across the new scenarios for both configs plus MHT for reference, and record the numbers. Easiest: temporarily widen the test's config filter to also include `imm_cv_ct_mht` (or the canonical MHT label from `defaultConfigs()`), and the geometry scenarios, and print `rows` to stdout (a `std::cerr` dump of `config,scenario,metric,value`), or read them from the test's `runSweep` return. Record GOSPA (`gospa_mean`), false mass (`gospa_false`), `card_err_mean`, `id_switches`, and `lifetime_ratio`.

- [ ] **Step 5: Record results in the evaluation log**

Append a dated section to `docs/algorithms/evaluation-log.md`: the geometry-breadth results (PMBM vs MHT across `parallel_lanes_dense`, `crossing_30/60/90`, `convoy_overtake` — does the breadth surface any regression?) and the shore-clutter A/B (`card_err_mean` and `gospa_false` land OFF vs ON for both shore scenarios, plus `lifetime_ratio` for the near-shore real target). State the verdict against the success criterion: land ON drives shore-clutter cardinality error to ~0 without dropping any real target.

- [ ] **Step 6: Record the decision in comparison-baselines**

Update `docs/algorithms/comparison-baselines.md`: note that the land-clutter prior is now confirmed on perfect-truth synthetic data (not only philos real data), and place this work on the Cl-2 (geometry breadth) and Cl-3 (clutter realism) axes.

- [ ] **Step 7: Commit**

```bash
git add tests/benchmark/test_synthetic_clutter_ab.cpp CMakeLists.txt docs/algorithms/evaluation-log.md docs/algorithms/comparison-baselines.md
git commit -m "bench(E): synthetic shore-clutter A/B test + evaluation-log + baselines decision"
```

---

## Self-Review

**1. Spec coverage:**
- §2 parametric parallel/crossing-angle/convoy + density → Task 1 (density = `n_targets`/`gap_m`/`lane_spacing_m`). ✓
- §2 stationary shore-clutter injector → Task 2 (`addShoreClutter`, fixed positions, `sim_shore`, no truth). ✓
- §2 near-shore real-target validator → Task 4 (`shore_clutter_nearshore`) + Task 6 assertion. ✓
- §2 in-memory coastline hook, one shoreline used twice → Task 3 (hook + Sweep) + Task 4 (`makeBenchShore` shared by `generate` and `syntheticCoastline`). ✓
- §4 coordinate frame (fictitious datum, ENU→geodetic consistency) → Task 2 `buildSyntheticShore` + Task 2 round-trip test. ✓
- §6 algorithm + learning docs → Task 5. ✓
- §7 tests (geometry invariants, determinism, fixed-position, source_id, no-truth, round-trip, wiring) → Tasks 1–4. ✓
- §8 measurement plan + success criterion → Task 6. ✓
- §9 decisions a–e all realized in Tasks 2–4. ✓

**2. Placeholder scan:** No TBD/TODO. Task 5 doc bodies are described by required content rather than verbatim prose (acceptable for a docs task — the four-section structure and required points are explicit). Task 6 Step 4 leaves the exact print mechanism to the implementer but names the exact metrics to capture.

**3. Type consistency:** `SyntheticShore{geometry, datum, clutter_enu_points}` consistent across Tasks 2/4; `buildSyntheticShore` signature identical in Task 2 decl/def and Task 4 `makeBenchShore` call; `addShoreClutter` signature identical across Task 2 and Task 4; `syntheticCoastline()` returns `std::optional<CoastlineGeometry>` consistently in Tasks 3/4; metric names (`card_err_mean`, `lifetime_ratio`, `gospa_false`, `gospa_mean`, `id_switches`) match `Sweep.cpp` emission. ✓

## Deviation from spec (flag for reviewer)

- Spec §5.3 mentions "one or two rectangular piers." This plan builds exactly **one** pier (single non-convex outer ring) — sufficient to make the shoreline non-trivial; a second pier adds no tested value (YAGNI).
- Spec §5.2 listed a `const CoastlineGeometry& shore` parameter on `addShoreClutter`. This plan drops it: the injector only needs the precomputed clutter points; the geometry is consumed by `buildSyntheticShore` (to make the points) and by the land model (via the hook). Cleaner, same behaviour.
