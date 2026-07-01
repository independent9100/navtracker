# Harbor Complete-Truth Scenario Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `harbor_complete_truth` — a synthetic benchmark scenario with fully-controlled ground truth (2 moving vessels + 3 anchored boats + 1 uncharted pier + uniform sea clutter) that serves as the honest yardstick for the live static-occupancy layer, plus a baseline measurement of today's PMBM on it.

**Architecture:** Reuse the existing `ScenarioRun` + `Builders` machinery. Add three reusable, deterministic scenario builders (`addFixedClutter`, `addAnchoredBoats`, `addUniformClutter`) that mirror the conventions of the existing `addShoreClutter`, then compose them in a new `HarborCompleteTruthScenarioRun`. The scenario is deliberately **chart-free** (no `syntheticCoastline()` override, no land model) so the pier stands as an *uncharted* open-water structure — isolating what a future live layer contributes.

**Tech Stack:** C++17, Eigen, GoogleTest, CMake. No new third-party dependencies.

## Global Constraints

- **C++17 only** — no later standard.
- **Coordinates: ENU local tangent plane, SI units** (metres, m/s). Internal math in ENU.
- **Determinism:** every builder is pure and seeded via `std::mt19937(seed)`; no wall-clock, no `Math.random`-equivalent. Same seed → byte-identical `measurements`.
- **Truth is a closed set (load-bearing):** injected clutter and the pier add **NO** `TruthSample`. The anchored boats **DO** add `TruthSample`s (one per boat per scan, **zero velocity**). This is the property that makes the scenario an honest yardstick — a track born on the pier/clutter scores as `gospa_false`; a dropped anchored boat scores as `missed` / low `lifetime_ratio`.
- **New builders mirror `addShoreClutter` conventions:** `SensorKind::ArpaTtm`, `MeasurementModel::Position2D`, seeded RNG, `stable_sort` by time at the end, set `base.datum = datum`.
- **CMake registration required:** every new `tests/**/*.cpp` file must be added to `CMakeLists.txt` (the block around lines 316–325).
- **Scenario registration:** add to `defaultSimScenarios()` and bump `out.reserve(17)` → `out.reserve(18)`; update `tests/benchmark/test_sim_scenario_run.cpp` (`ASSERT_EQ(..., 17u)` → `18u`; add the label; add the scenario to the size-2 detection-table branch).
- **Commit messages:** conventional style, imperative subject. **Do NOT add `Co-Authored-By` or other trailers.**
- **Frame origin for the scenario:** `geo::Datum(geo::Geodetic{42.35, -71.05, 0.0})` (the shared bench frame).

---

## Fixed Scenario Geometry (single source of truth — used by Task 4)

All ENU metres. 40 scans at 1 Hz (`t = 1..40 s`).

| element | truth? | placement | builder |
|---|---|---|---|
| Mover A (id 1) | ✅ | start `(-500,-150)`, vel `(20,0)` | `buildCrossingTargetsScenario` |
| Mover B (id 2) | ✅ | start `(500,150)`, vel `(-20,0)` | (same call) |
| Anchored boats (ids 3,4,5) | ✅ v=0 | `(100,300)`, `(250,320)`, `(-50,330)` | `addAnchoredBoats` |
| Pier (uncharted) | ❌ | 13 pts, `y=-350`, `x=-60..60` step 10 | `addFixedClutter` ("sim_pier") |
| Uniform sea clutter | ❌ | 5/scan in box `(-600,-450)`–`(600,450)` | `addUniformClutter` ("sim_clutter") |

- Movers noise 8 m; boats noise 5 m (compact watch circle), detect prob 0.95; pier noise 4 m, detect prob 0.9.
- Boats are ≥150 m apart (each a distinct compact region) and ≥150 m from any mover track; the pier is 120 m long (clearly *extended*) and ≥650 m from the boats.
- Per-stage seed offsets to decorrelate RNG streams: boats `seed`, pier `seed+7`, clutter `seed+12345`.
- Detection table: reuse `shoreClutterTable()` → `{AIS 1e-6, ArpaTtm 1e-5}`. The `ArpaTtm` λ_C=1e-5 models the diffuse ~5-FA/scan floor; the pier is **deliberately unmodeled** (that is the phenomenon under test).

---

## File Structure

- `core/scenario/Builders.hpp` — declare `addFixedClutter`, `addAnchoredBoats`, `addUniformClutter`.
- `core/scenario/Builders.cpp` — anon-namespace helpers (`distinctScanTimes`, `sortMeasurementsByTime`, `makeArpaPositionMeasurement`); define the three builders; refactor `addShoreClutter`/`makeShoreMeasurement` to delegate.
- `adapters/benchmark/SimScenarioRun.cpp` — `HarborCompleteTruthScenarioRun` + file-local `pierPoints()`; register in `defaultSimScenarios()`.
- `tests/scenario/test_builders.cpp` — unit tests for the three new builders.
- `tests/benchmark/test_sim_scenario_run.cpp` — update contract asserts.
- `tests/benchmark/test_harbor_complete_truth.cpp` — NEW ground-truth contract test.
- `tests/benchmark/test_harbor_complete_truth_baseline.cpp` — NEW baseline measurement (prints; light asserts).
- `docs/algorithms/synthetic-clutter-bench.md` — document the scenario.
- `docs/algorithms/evaluation-log.md` — record baseline numbers.
- `CMakeLists.txt` — register the two new test files.

---

### Task 1: Shared builder helpers + `addFixedClutter` (generalize `addShoreClutter`)

**Files:**
- Modify: `core/scenario/Builders.cpp` (anon namespace + `addShoreClutter`)
- Modify: `core/scenario/Builders.hpp` (declare `addFixedClutter`)
- Test: `tests/scenario/test_builders.cpp`

**Interfaces:**
- Consumes: existing `makeTruth`, `Timestamp::fromSeconds`, `Scenario`, `geo::Datum`.
- Produces:
  - anon: `std::vector<double> distinctScanTimes(const Scenario&)`; `void sortMeasurementsByTime(Scenario&)`; `Measurement makeArpaPositionMeasurement(const Eigen::Vector2d& noisy, double t_seconds, double std_m, const std::string& source_id)`.
  - public: `Scenario addFixedClutter(Scenario base, const geo::Datum& datum, const std::vector<Eigen::Vector2d>& points, const std::string& source_id, double detection_prob, double pos_noise_std_m, std::uint32_t seed)`.

- [ ] **Step 1: Write the failing test** — append to `tests/scenario/test_builders.cpp`:

```cpp
TEST(Builders, AddFixedClutterAddsRecurringReturnsNoTruth) {
  // Base: a single moving target over 5 scans (times 1..5), truth id 1.
  Scenario base = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      {1.0, 2.0, 3.0, 4.0, 5.0}, /*pos_noise_std_m=*/1.0, /*seed=*/7,
      /*truth_id=*/1);
  const std::size_t base_meas = base.measurements.size();
  const std::size_t base_truth = base.truth.size();

  const geo::Datum datum(geo::Geodetic{42.35, -71.05, 0.0});
  const std::vector<Eigen::Vector2d> pts = {{100.0, 0.0}, {110.0, 0.0}};
  Scenario s = addFixedClutter(base, datum, pts, "sim_pier",
                               /*detection_prob=*/1.0, /*pos_noise_std_m=*/2.0,
                               /*seed=*/3);

  // detection_prob = 1.0 -> exactly 2 fixed returns per scan x 5 scans.
  EXPECT_EQ(s.measurements.size(), base_meas + 2u * 5u);
  EXPECT_EQ(s.truth.size(), base_truth);  // NO truth added
  EXPECT_TRUE(s.datum.has_value());
  int pier = 0;
  for (const auto& m : s.measurements) {
    if (m.source_id == "sim_pier") {
      ++pier;
      EXPECT_EQ(m.sensor, SensorKind::ArpaTtm);
      EXPECT_EQ(m.model, MeasurementModel::Position2D);
    }
  }
  EXPECT_EQ(pier, 10);
  // measurements sorted by time
  for (std::size_t i = 1; i < s.measurements.size(); ++i)
    EXPECT_LE(s.measurements[i - 1].time.seconds(),
              s.measurements[i].time.seconds());
}

TEST(Builders, AddFixedClutterIsDeterministic) {
  Scenario base = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      {1.0, 2.0, 3.0}, 1.0, 7, 1);
  const geo::Datum datum(geo::Geodetic{42.35, -71.05, 0.0});
  const std::vector<Eigen::Vector2d> pts = {{100.0, 0.0}};
  Scenario a = addFixedClutter(base, datum, pts, "sim_pier", 0.7, 2.0, 5);
  Scenario b = addFixedClutter(base, datum, pts, "sim_pier", 0.7, 2.0, 5);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].value, b.measurements[i].value);
    EXPECT_EQ(a.measurements[i].source_id, b.measurements[i].source_id);
  }
}
```

Ensure the test file includes `#include "core/geo/Wgs84.hpp"` (for `geo::Geodetic`) and `#include "core/scenario/Builders.hpp"` if not already present.

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders -V`
Expected: compile error — `addFixedClutter` not declared.

- [ ] **Step 3: Add the anon-namespace helpers** in `core/scenario/Builders.cpp` (inside the existing `namespace { ... }`, after `makeShoreMeasurement`):

```cpp
std::vector<double> distinctScanTimes(const Scenario& s) {
  std::vector<double> scan_times;
  for (const auto& m : s.measurements) {
    const double t = m.time.seconds();
    if (scan_times.empty() || scan_times.back() != t) scan_times.push_back(t);
  }
  return scan_times;
}

void sortMeasurementsByTime(Scenario& s) {
  std::stable_sort(s.measurements.begin(), s.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time.seconds() < b.time.seconds();
                   });
}

Measurement makeArpaPositionMeasurement(const Eigen::Vector2d& noisy_pos,
                                        double t_seconds, double std_m,
                                        const std::string& source_id) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = source_id;
  m.model = MeasurementModel::Position2D;
  m.value = noisy_pos;
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}
```

Refactor the existing `makeShoreMeasurement` body to delegate (keep its signature):

```cpp
Measurement makeShoreMeasurement(const Eigen::Vector2d& noisy_pos,
                                 double t_seconds, double std_m) {
  return makeArpaPositionMeasurement(noisy_pos, t_seconds, std_m, "sim_shore");
}
```

- [ ] **Step 4: Add `addFixedClutter`** (public) in `Builders.cpp`, and refactor `addShoreClutter` to delegate. Place `addFixedClutter` just above `addShoreClutter`:

```cpp
Scenario addFixedClutter(Scenario base, const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& points,
                         const std::string& source_id, double detection_prob,
                         double pos_noise_std_m, std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    for (const auto& pt : points) {
      if (u(rng) < detection_prob) {
        const Eigen::Vector2d noisy(pt.x() + noise(rng), pt.y() + noise(rng));
        base.measurements.push_back(
            makeArpaPositionMeasurement(noisy, t, pos_noise_std_m, source_id));
      }
    }
  }
  sortMeasurementsByTime(base);
  base.datum = datum;
  return base;
}
```

Replace the body of the existing `addShoreClutter` with a delegating wrapper (behavior-preserving — same `"sim_shore"` tag, same RNG order):

```cpp
Scenario addShoreClutter(Scenario base, const geo::Datum& datum,
                         const std::vector<Eigen::Vector2d>& clutter_enu_points,
                         double detection_prob, double pos_noise_std_m,
                         std::uint32_t seed) {
  return addFixedClutter(std::move(base), datum, clutter_enu_points, "sim_shore",
                         detection_prob, pos_noise_std_m, seed);
}
```

- [ ] **Step 5: Declare `addFixedClutter`** in `core/scenario/Builders.hpp` (immediately before the `addShoreClutter` declaration), with a doc comment:

```cpp
// Add fixed, recurring returns at `points` (ENU) to `base`, tagged
// SensorKind::ArpaTtm / `source_id`. For each distinct scan timestamp in
// base.measurements, each point emits a Position2D measurement at its fixed
// position plus isotropic Gaussian noise, with probability `detection_prob`
// (seeded Bernoulli). NO TruthSample is created — these are environment /
// structure, not vessels. Sets base.datum = datum; returns re-sorted by time.
// addShoreClutter is a thin wrapper over this with source_id "sim_shore".
Scenario addFixedClutter(
    Scenario base, const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& points, const std::string& source_id,
    double detection_prob, double pos_noise_std_m, std::uint32_t seed = 0);
```

Add `#include <string>` to `Builders.hpp` if not already present.

- [ ] **Step 6: Run tests**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R "Builders|SimScenarioRun|BenchDeterminism|SyntheticClutter" -V`
Expected: PASS — new `Builders.AddFixedClutter*` pass; existing shore-clutter / determinism tests remain green (behavior preserved).

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "scenario: extract addFixedClutter + shared builder helpers"
```

---

### Task 2: `addAnchoredBoats` builder (fixed returns WITH zero-velocity truth)

**Files:**
- Modify: `core/scenario/Builders.hpp`, `core/scenario/Builders.cpp`
- Test: `tests/scenario/test_builders.cpp`

**Interfaces:**
- Consumes: `distinctScanTimes`, `sortMeasurementsByTime`, `makeArpaPositionMeasurement`, `makeTruth` (all from Task 1 / existing).
- Produces: `Scenario addAnchoredBoats(Scenario base, const geo::Datum& datum, const std::vector<Eigen::Vector2d>& boat_positions, std::uint64_t truth_id_start, double detection_prob, double pos_noise_std_m, std::uint32_t seed)`.

- [ ] **Step 1: Write the failing test** — append to `tests/scenario/test_builders.cpp`:

```cpp
TEST(Builders, AddAnchoredBoatsAddsZeroVelocityTruthAndRadarReturns) {
  Scenario base = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      {1.0, 2.0, 3.0, 4.0}, /*pos_noise_std_m=*/1.0, /*seed=*/7,
      /*truth_id=*/1);
  const std::size_t base_truth = base.truth.size();  // 4 (one per scan)
  const geo::Datum datum(geo::Geodetic{42.35, -71.05, 0.0});
  const std::vector<Eigen::Vector2d> boats = {{100.0, 50.0}, {200.0, 60.0}};

  Scenario s = addAnchoredBoats(base, datum, boats, /*truth_id_start=*/3,
                                /*detection_prob=*/1.0, /*pos_noise_std_m=*/2.0,
                                /*seed=*/9);

  // 2 boats x 4 scans = 8 truth samples added, all zero velocity, ids {3,4}.
  EXPECT_EQ(s.truth.size(), base_truth + 8u);
  std::set<std::uint64_t> boat_ids;
  int zero_vel = 0;
  for (const auto& ts : s.truth) {
    if (ts.truth_id >= 3) {
      boat_ids.insert(ts.truth_id);
      if (ts.velocity.norm() == 0.0) ++zero_vel;
      // truth position is the nominal anchor (noise-free)
      const bool at_b0 = (ts.position - Eigen::Vector2d(100.0, 50.0)).norm() == 0.0;
      const bool at_b1 = (ts.position - Eigen::Vector2d(200.0, 60.0)).norm() == 0.0;
      EXPECT_TRUE(at_b0 || at_b1);
    }
  }
  EXPECT_EQ(boat_ids, (std::set<std::uint64_t>{3u, 4u}));
  EXPECT_EQ(zero_vel, 8);

  // detection_prob 1.0 -> 2 radar returns per scan x 4 = 8, tagged sim_anchored
  int anchored = 0;
  for (const auto& m : s.measurements)
    if (m.source_id == "sim_anchored") {
      ++anchored;
      EXPECT_EQ(m.sensor, SensorKind::ArpaTtm);
    }
  EXPECT_EQ(anchored, 8);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders.AddAnchoredBoats -V`
Expected: compile error — `addAnchoredBoats` not declared.

- [ ] **Step 3: Implement** `addAnchoredBoats` in `Builders.cpp` (place just below `addFixedClutter`):

```cpp
Scenario addAnchoredBoats(Scenario base, const geo::Datum& datum,
                          const std::vector<Eigen::Vector2d>& boat_positions,
                          std::uint64_t truth_id_start, double detection_prob,
                          double pos_noise_std_m, std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (double t : scan_times) {
    for (std::size_t b = 0; b < boat_positions.size(); ++b) {
      const Eigen::Vector2d& pos = boat_positions[b];
      const std::uint64_t id = truth_id_start + static_cast<std::uint64_t>(b);
      // Truth exists every scan regardless of detection (the boat is there).
      base.truth.push_back(makeTruth(pos, Eigen::Vector2d::Zero(), t, id));
      if (u(rng) < detection_prob) {
        const Eigen::Vector2d noisy(pos.x() + noise(rng), pos.y() + noise(rng));
        base.measurements.push_back(makeArpaPositionMeasurement(
            noisy, t, pos_noise_std_m, "sim_anchored"));
      }
    }
  }
  sortMeasurementsByTime(base);
  base.datum = datum;
  return base;
}
```

- [ ] **Step 4: Declare** in `Builders.hpp` (after `addFixedClutter`):

```cpp
// Add `boat_positions.size()` stationary (anchored) vessels to `base`. Each
// boat is a REAL target: a zero-velocity TruthSample is emitted for it every
// scan (ids truth_id_start, +1, ...), AND — with probability detection_prob —
// a radar-like ArpaTtm / "sim_anchored" Position2D return with isotropic
// noise (a compact watch circle). Truth is emitted even on undetected scans
// (the boat is present). These are the "keep, never suppress" set for the
// live static-occupancy layer. Sets base.datum; returns re-sorted by time.
Scenario addAnchoredBoats(
    Scenario base, const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& boat_positions,
    std::uint64_t truth_id_start, double detection_prob, double pos_noise_std_m,
    std::uint32_t seed = 0);
```

- [ ] **Step 5: Run tests**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders -V`
Expected: PASS (all `Builders.*`).

- [ ] **Step 6: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "scenario: add addAnchoredBoats (zero-velocity truth + radar returns)"
```

---

### Task 3: `addUniformClutter` builder (transient uniform false alarms, no truth)

**Files:**
- Modify: `core/scenario/Builders.hpp`, `core/scenario/Builders.cpp`
- Test: `tests/scenario/test_builders.cpp`

**Interfaces:**
- Consumes: `distinctScanTimes`, `sortMeasurementsByTime`, `makeArpaPositionMeasurement`.
- Produces: `Scenario addUniformClutter(Scenario base, const geo::Datum& datum, const Eigen::Vector2d& box_min, const Eigen::Vector2d& box_max, int n_per_scan, std::uint32_t seed)`.

- [ ] **Step 1: Write the failing test** — append to `tests/scenario/test_builders.cpp`:

```cpp
TEST(Builders, AddUniformClutterAddsTransientReturnsNoTruth) {
  Scenario base = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      {1.0, 2.0, 3.0, 4.0, 5.0}, 1.0, 7, 1);
  const std::size_t base_truth = base.truth.size();
  const geo::Datum datum(geo::Geodetic{42.35, -71.05, 0.0});

  Scenario s = addUniformClutter(base, datum, Eigen::Vector2d(-100.0, -100.0),
                                 Eigen::Vector2d(100.0, 100.0),
                                 /*n_per_scan=*/4, /*seed=*/11);

  EXPECT_EQ(s.truth.size(), base_truth);  // NO truth
  int clut = 0;
  for (const auto& m : s.measurements)
    if (m.source_id == "sim_clutter") {
      ++clut;
      EXPECT_EQ(m.sensor, SensorKind::ArpaTtm);
      EXPECT_GE(m.value.x(), -100.0);
      EXPECT_LE(m.value.x(), 100.0);
      EXPECT_GE(m.value.y(), -100.0);
      EXPECT_LE(m.value.y(), 100.0);
    }
  EXPECT_EQ(clut, 4 * 5);  // 4 per scan x 5 scans
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders.AddUniformClutter -V`
Expected: compile error — `addUniformClutter` not declared.

- [ ] **Step 3: Implement** `addUniformClutter` in `Builders.cpp` (below `addAnchoredBoats`):

```cpp
Scenario addUniformClutter(Scenario base, const geo::Datum& datum,
                           const Eigen::Vector2d& box_min,
                           const Eigen::Vector2d& box_max, int n_per_scan,
                           std::uint32_t seed) {
  const std::vector<double> scan_times = distinctScanTimes(base);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(box_min.x(), box_max.x());
  std::uniform_real_distribution<double> uy(box_min.y(), box_max.y());
  for (double t : scan_times) {
    for (int i = 0; i < n_per_scan; ++i) {
      const Eigen::Vector2d fp(ux(rng), uy(rng));
      base.measurements.push_back(
          makeArpaPositionMeasurement(fp, t, /*std_m=*/1.0, "sim_clutter"));
    }
  }
  sortMeasurementsByTime(base);
  base.datum = datum;
  return base;
}
```

- [ ] **Step 4: Declare** in `Builders.hpp` (after `addAnchoredBoats`):

```cpp
// Add `n_per_scan` uniform-random false alarms per scan to `base`, drawn in
// the ENU box [box_min, box_max], tagged ArpaTtm / "sim_clutter". Positions
// are re-drawn every scan (transient — the defining contrast with the fixed
// recurring returns of addFixedClutter). NO TruthSample. Sets base.datum;
// returns re-sorted by time.
Scenario addUniformClutter(
    Scenario base, const geo::Datum& datum, const Eigen::Vector2d& box_min,
    const Eigen::Vector2d& box_max, int n_per_scan, std::uint32_t seed = 0);
```

- [ ] **Step 5: Run tests**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Builders -V`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp
git commit -m "scenario: add addUniformClutter (transient uniform false alarms)"
```

---

### Task 4: `HarborCompleteTruthScenarioRun` + registration + ground-truth contract test

**Files:**
- Modify: `adapters/benchmark/SimScenarioRun.cpp` (new class + `pierPoints()` + register + reserve)
- Modify: `tests/benchmark/test_sim_scenario_run.cpp` (count, label, detection-table branch)
- Create: `tests/benchmark/test_harbor_complete_truth.cpp`
- Modify: `CMakeLists.txt` (register new test file)

**Interfaces:**
- Consumes: `buildCrossingTargetsScenario`, `addAnchoredBoats`, `addFixedClutter`, `addUniformClutter`, `shoreClutterTable`, `describe`, `linearSeconds`.
- Produces: a scenario whose `descriptor().label == "harbor_complete_truth"`, with exactly 5 truth ids (1,2 moving; 3,4,5 zero-velocity), AIS + ArpaTtm measurements, and NO truth for pier/clutter.

- [ ] **Step 1: Write the failing contract test** — create `tests/benchmark/test_harbor_complete_truth.cpp`:

```cpp
// Ground-truth contract for the harbor_complete_truth yardstick: exactly 5
// truth targets (2 movers, 3 anchored boats with zero velocity), the pier and
// uniform clutter add NO truth, and the scenario is chart-free (no coastline).
#include <gtest/gtest.h>

#include <map>
#include <set>
#include <string>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
// unique_ptr::get() yields a non-const ScenarioRun* even from a const
// unique_ptr, so generate() (non-const) is directly callable.
ScenarioRun* findHarbor(const std::vector<std::unique_ptr<ScenarioRun>>& v) {
  for (const auto& s : v)
    if (s->descriptor().label == "harbor_complete_truth") return s.get();
  return nullptr;
}
}  // namespace

TEST(HarborCompleteTruth, TruthIsClosedFiveTargets) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  auto scen = h->generate(0);

  std::set<std::uint64_t> ids;
  std::map<std::uint64_t, double> max_speed;
  for (const auto& ts : scen.truth) {
    ids.insert(ts.truth_id);
    max_speed[ts.truth_id] =
        std::max(max_speed[ts.truth_id], ts.velocity.norm());
  }
  EXPECT_EQ(ids, (std::set<std::uint64_t>{1u, 2u, 3u, 4u, 5u}));
  // Movers move; anchored boats are stationary.
  EXPECT_GT(max_speed[1u], 1.0);
  EXPECT_GT(max_speed[2u], 1.0);
  EXPECT_EQ(max_speed[3u], 0.0);
  EXPECT_EQ(max_speed[4u], 0.0);
  EXPECT_EQ(max_speed[5u], 0.0);
}

TEST(HarborCompleteTruth, PierAndClutterAddNoTruth) {
  const auto scenarios = defaultSimScenarios();
  auto scen = findHarbor(scenarios)->generate(0);
  // 5 targets x 40 scans = 200 truth samples exactly.
  EXPECT_EQ(scen.truth.size(), 5u * 40u);
  bool has_pier = false, has_clutter = false, has_ais = false, has_anch = false;
  for (const auto& m : scen.measurements) {
    if (m.source_id == "sim_pier") has_pier = true;
    if (m.source_id == "sim_clutter") has_clutter = true;
    if (m.source_id == "sim_anchored") has_anch = true;
    if (m.sensor == SensorKind::Ais) has_ais = true;
  }
  EXPECT_TRUE(has_pier);
  EXPECT_TRUE(has_clutter);
  EXPECT_TRUE(has_anch);
  EXPECT_TRUE(has_ais);
  EXPECT_TRUE(scen.datum.has_value());
}

TEST(HarborCompleteTruth, ChartFreeNoCoastline) {
  const auto scenarios = defaultSimScenarios();
  ScenarioRun* h = findHarbor(scenarios);
  ASSERT_NE(h, nullptr);
  EXPECT_FALSE(h->syntheticCoastline().has_value());
}

TEST(HarborCompleteTruth, DeterministicForSameSeed) {
  const auto scenarios = defaultSimScenarios();
  auto a = findHarbor(scenarios)->generate(0);
  auto b = findHarbor(scenarios)->generate(0);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].value, b.measurements[i].value);
    EXPECT_EQ(a.measurements[i].source_id, b.measurements[i].source_id);
  }
}
```

Add `#include <algorithm>` and `#include <memory>` and `#include <vector>` and `#include <cstdint>` as needed.

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R HarborCompleteTruth -V`
Expected: FAIL — `findHarbor` returns null (scenario not registered) → `ASSERT_NE(h, nullptr)` fails. (After registering the test file in CMake — Step 5.)

- [ ] **Step 3: Add the scenario class** in `adapters/benchmark/SimScenarioRun.cpp`, just after `ShoreClutterNearShoreScenarioRun` (before the closing `}  // namespace`). First add a file-local `pierPoints()` helper near `makeBenchShore()`:

```cpp
// Uncharted pier: a 120 m line of fixed radar scatterers (13 points, 10 m
// apart) at y = -350, x in [-60, 60]. Deliberately NOT a charted obstacle and
// NOT in the truth set — the extended persistent structure the live
// static-occupancy layer must learn to suppress.
std::vector<Eigen::Vector2d> pierPoints() {
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i <= 12; ++i)
    pts.emplace_back(-60.0 + 10.0 * static_cast<double>(i), -350.0);
  return pts;
}
```

Then the scenario class:

```cpp
class HarborCompleteTruthScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("harbor_complete_truth", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Honest yardstick: complete, controlled truth. Two moving vessels
    // (AIS, ids 1-2) + three anchored non-AIS boats (zero velocity, ids 3-5,
    // the KEEP set) + one uncharted pier (extended fixed radar structure, NO
    // truth, the SUPPRESS set) + uniform sea clutter (transient, NO truth,
    // the IGNORE set). Chart-free on purpose (no syntheticCoastline) so the
    // pier is an *uncharted* open-water structure. See
    // docs/superpowers/specs/2026-07-01-honest-static-occupancy-stage1b-design.md.
    const auto times = linearSeconds(1, 40);
    const geo::Datum datum(navtracker::geo::Geodetic{42.35, -71.05, 0.0});
    const auto s32 = static_cast<std::uint32_t>(seed);

    Scenario base = buildCrossingTargetsScenario(
        Eigen::Vector2d(-500.0, -150.0), Eigen::Vector2d(20.0, 0.0),
        Eigen::Vector2d(500.0, 150.0), Eigen::Vector2d(-20.0, 0.0), times,
        /*pos_noise_std_m=*/8.0, s32);

    base = addAnchoredBoats(
        std::move(base), datum,
        {{100.0, 300.0}, {250.0, 320.0}, {-50.0, 330.0}},
        /*truth_id_start=*/3, /*detection_prob=*/0.95, /*pos_noise_std_m=*/5.0,
        s32);

    base = addFixedClutter(std::move(base), datum, pierPoints(), "sim_pier",
                           /*detection_prob=*/0.9, /*pos_noise_std_m=*/4.0,
                           s32 + 7u);

    base = addUniformClutter(std::move(base), datum,
                             Eigen::Vector2d(-600.0, -450.0),
                             Eigen::Vector2d(600.0, 450.0),
                             /*n_per_scan=*/5, s32 + 12345u);
    return base;
  }
  // No syntheticCoastline(): chart-free by design.
};
```

Ensure `#include "core/scenario/Builders.hpp"` is present (it is, line 12).

- [ ] **Step 4: Register** in `defaultSimScenarios()`: bump `out.reserve(17)` → `out.reserve(18)`, and add before `return out;`:

```cpp
  out.push_back(std::make_unique<HarborCompleteTruthScenarioRun>());
```

- [ ] **Step 5: Register the test file** in `CMakeLists.txt` — add next to the other benchmark tests (near line 316–325):

```cmake
  tests/benchmark/test_harbor_complete_truth.cpp
```

- [ ] **Step 6: Update the scenario contract test** `tests/benchmark/test_sim_scenario_run.cpp`:
  - Line 12: `ASSERT_EQ(scenarios.size(), 17u);` → `18u`.
  - After line 31 add: `EXPECT_EQ(labels.count("harbor_complete_truth"), 1u);`
  - In `ClutterFreeScenariosDeclareFloorDensity`, extend the shore branch condition (line 99–100) so the 2-entry-table assertion also covers the harbor scenario:

```cpp
    } else if (d.label == "shore_clutter_open" ||
               d.label == "shore_clutter_nearshore" ||
               d.label == "harbor_complete_truth") {
      ASSERT_EQ(d.detection_table.size(), 2u) << d.label;
```

- [ ] **Step 7: Run tests**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R "HarborCompleteTruth|SimScenarioRun" -V`
Expected: PASS — all contract asserts and all `SimScenarioRun.*` green.

- [ ] **Step 8: Commit**

```bash
git add adapters/benchmark/SimScenarioRun.cpp tests/benchmark/test_sim_scenario_run.cpp \
        tests/benchmark/test_harbor_complete_truth.cpp CMakeLists.txt
git commit -m "bench: add harbor_complete_truth honest-yardstick scenario"
```

---

### Task 5: Baseline measurement of today's PMBM + documentation

**Files:**
- Create: `tests/benchmark/test_harbor_complete_truth_baseline.cpp`
- Modify: `CMakeLists.txt`
- Modify: `docs/algorithms/synthetic-clutter-bench.md`
- Modify: `docs/algorithms/evaluation-log.md`

**Interfaces:**
- Consumes: `defaultConfigs()` (config `imm_cv_ct_pmbm`), `runSweep`, `MetricRow`, `defaultSimScenarios()`.

- [ ] **Step 1: Write the baseline measurement** — create `tests/benchmark/test_harbor_complete_truth_baseline.cpp` (measurement + light sanity gate; mirrors the structure of `test_philos_cluttermap_ab.cpp`):

```cpp
// Milestone-1 baseline: run TODAY's PMBM on the harbor_complete_truth
// yardstick and print every metric. This is the "before" number the honest
// static-occupancy layer must improve — NOT a tuning target. Light sanity
// asserts only (the movers are trackable; the scene produces tracks).
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
double meanMetric(const std::vector<MetricRow>& rows, const std::string& metric) {
  double s = 0.0;
  int n = 0;
  for (const auto& r : rows)
    if (r.scenario == "harbor_complete_truth" && r.metric == metric) {
      s += r.value;
      ++n;
    }
  return n ? s / n : 0.0;
}

std::vector<std::unique_ptr<navtracker::ScenarioRun>> harborOnly() {
  std::vector<std::unique_ptr<navtracker::ScenarioRun>> out;
  for (auto& s : defaultSimScenarios())
    if (s->descriptor().label == "harbor_complete_truth")
      out.push_back(std::move(s));
  return out;
}
}  // namespace

TEST(HarborCompleteTruthBaseline, TodaysPmbmMetrics) {
  const Config* base = nullptr;
  const auto all = defaultConfigs();
  for (const auto& c : all)
    if (c.label == "imm_cv_ct_pmbm") base = &c;
  ASSERT_NE(base, nullptr);

  SweepParams params;
  params.run_id = "harbor_complete_truth_baseline";
  params.synthetic_seeds = 5;
  const auto rows = runSweep({*base}, harborOnly(), params);
  ASSERT_FALSE(rows.empty());

  const char* keys[] = {"card_err_mean", "gospa_mean", "gospa_false",
                        "gospa_missed", "lifetime_ratio"};
  std::cout << "\n=== harbor_complete_truth  (baseline imm_cv_ct_pmbm) ===\n";
  for (const auto* m : keys)
    std::cout << "  " << m << " = " << meanMetric(rows, m) << "\n";
  std::cout << std::flush;

  // Sanity: the scene is trackable — some tracks live long enough to match
  // truth (movers + boats), so lifetime is clearly non-zero.
  EXPECT_GT(meanMetric(rows, "lifetime_ratio"), 0.3);
}
```

Note: if `runSweep`/`SweepParams` field names differ, match the exact signature already used in `tests/benchmark/test_philos_cluttermap_ab.cpp` (same header, same `runSweep(configs, scenarios, params)` form).

- [ ] **Step 2: Register the test file** in `CMakeLists.txt`:

```cmake
  tests/benchmark/test_harbor_complete_truth_baseline.cpp
```

- [ ] **Step 3: Run and capture the numbers**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R HarborCompleteTruthBaseline -V`
Expected: PASS; capture the printed `card_err_mean / gospa_mean / gospa_false / gospa_missed / lifetime_ratio` from the ctest `-V` output.

- [ ] **Step 4: Document the scenario** — add a section to `docs/algorithms/synthetic-clutter-bench.md` titled "harbor_complete_truth (honest yardstick)": describe the four return classes and their expected verdicts (movers→track, boats→keep, pier→suppress, clutter→ignore), the chart-free rationale, that truth is a closed set (pier/clutter carry no truth; boats are zero-velocity truth ids 3–5), and the exact geometry table from this plan. State plainly that this scenario — not philos gospa — is the promotion gate for the live static-occupancy layer.

- [ ] **Step 5: Record the baseline** — add a dated entry to `docs/algorithms/evaluation-log.md` (2026-07-01): the captured baseline numbers under `imm_cv_ct_pmbm`, interpreted against the expected verdicts (e.g. "card_err_mean = +N confirms today's PMBM over-counts on the pier; lifetime_ratio = M indicates the anchored boats are/are not currently kept"). This is the "before" the honest layer must beat.

- [ ] **Step 6: Commit**

```bash
git add tests/benchmark/test_harbor_complete_truth_baseline.cpp CMakeLists.txt \
        docs/algorithms/synthetic-clutter-bench.md docs/algorithms/evaluation-log.md
git commit -m "bench: baseline harbor_complete_truth under imm_cv_ct_pmbm + docs"
```

---

## Self-Review

- **Spec coverage:** the honest-static-occupancy spec §Testing names four gates — anchored-boat preservation (boats = zero-velocity truth ids 3–5, KEEP set ✓), dense/uniform clutter cleanliness (transient `addUniformClutter`, no truth ✓), structure suppression (pier `addFixedClutter`, no truth ✓), movers untouched (ids 1–2 ✓). Milestone-1 delivers the scenario + the baseline "before" (Task 5). The *layer* and its A/B are Milestone 2/3, out of scope here.
- **Type consistency:** `addFixedClutter`/`addAnchoredBoats`/`addUniformClutter` signatures are identical between their `.hpp` declarations and `.cpp` definitions and the call sites in Task 4. `pierPoints()` returns `std::vector<Eigen::Vector2d>`, consumed by `addFixedClutter`. Truth count `5*40=200` matches 40 scans (`linearSeconds(1,40)`) × (2 movers + 3 boats).
- **Placeholder scan:** every code step contains complete code; the one soft spot (exact `runSweep`/`SweepParams` field names in Task 5) is pinned to the existing `test_philos_cluttermap_ab.cpp` as the reference.
- **Determinism:** all three builders seed `std::mt19937`; per-stage seed offsets (`+7`, `+12345`) decorrelate streams; Task 4 asserts same-seed reproducibility.
