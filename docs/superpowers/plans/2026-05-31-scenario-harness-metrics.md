# Scenario Harness & Metrics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make algorithm alternatives *evaluable*. Provide synthetic ground-truth scenarios, an OSPA-style metric, and a harness that runs scenarios through a fully-composed `Tracker` and produces a quantitative score. This is what lets the deferred swaps (UKF/IMM/particle, JPDA/MHT, hint locking) be compared against the baseline with numbers.

**Architecture:** Adds a `core/scenario/` module (pure domain, no I/O): truth types, scenario builders, OSPA metric, harness. Two canonical scenarios as tests (single-target, parallel multi-target). All deterministic (fixed RNG seed) so re-runs reproduce the same metric.

**Tech Stack:** C++17 · Eigen 3.4 · GoogleTest. Builds on plans 1–5 (on `master`).

This is plan 6 of 6. Prereq: plans 1–5 merged — 59 tests pass.

**Documentation standard (CLAUDE.md):** Each algorithm carries Math / Assumptions / Rationale / Ways-to-improve. Concise notes per task; Task 5 writes `docs/algorithms/scenarios-and-metrics.md`.

---

## Design notes

- **Truth + measurements together.** A `Scenario` carries both ground-truth samples and the noisy `Measurement` stream they produced. The harness consumes the stream and compares the tracker output to the truth at matching timestamps.
- **OSPA (p=2) with greedy assignment.** Standard OSPA needs an optimal assignment (Hungarian) at each step. The baseline uses a greedy nearest-pair approximation, identical in spirit to the GNN data associator. Documented as an approximation; Hungarian is the documented next step.
- **Deterministic RNG.** Builders take a `seed`; tests use a fixed seed so the metric is reproducible across runs.
- **Truth time = measurement time.** For the baseline, truth samples are emitted at the same timestamps as measurements (no interpolation needed by the harness). Time-interpolated truth is a documented improvement.
- **Source-of-truth metric.** The harness samples *estimated* track positions at each measurement timestamp from `TrackManager::tracks()` (post-`process`), and *truth* positions matching that timestamp.

## File Structure

```
core/scenario/Truth.hpp                  TruthSample + Scenario types
core/scenario/Builders.hpp/.cpp          buildStraightLineScenario, buildParallelTargetsScenario
core/scenario/Ospa.hpp/.cpp              ospaGreedy(truth, est, cutoff)
core/scenario/Harness.hpp/.cpp           runScenario(...) -> ScenarioResult
docs/algorithms/scenarios-and-metrics.md
tests/scenario/test_builders.cpp
tests/scenario/test_ospa.cpp
tests/scenario/test_harness.cpp
tests/scenario/test_parallel_targets.cpp
```

## Build/test (from repo root)

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

If `~/.conan2`/"readonly database" error, re-run sandboxed off. Commits: `git -c commit.gpgsign=false commit -m "..."`. No pushes.

## Existing signatures this plan depends on

- `navtracker::Tracker(const IEstimator&, const IDataAssociator&, TrackManager&, double miss_timeout_seconds)` with `void process(const Measurement&)`.
- `navtracker::EkfEstimator(std::shared_ptr<const IMotionModel>, double init_speed_std)`.
- `navtracker::ConstantVelocity2D(double accel_psd)`.
- `navtracker::GnnAssociator(double gate_threshold)`.
- `navtracker::TrackManager(int confirm_hits, int delete_misses)` with `tracks() const`, `size()`.
- `navtracker::Measurement` Position2D layout; `navtracker::Timestamp` with `==`, `fromSeconds`, `seconds`, `nanos`.

---

## Task 1: Truth types + scenario builders

**Math/Logic.** A scenario is a deterministic synthetic dataset. The straight-line builder emits ground-truth `(pos, vel)` for one target moving as `pos(t) = start + velocity·t` at the supplied times; measurements are truth + Gaussian noise (std `pos_noise_std_m`, identity R). The parallel-targets builder emits two such tracks running in parallel and interleaves their measurements by time, deterministically.
**Assumptions.** Constant velocity per target; Gaussian zero-mean position noise; deterministic `std::mt19937` seeded from `seed`; truth and measurements share the same timestamps (no interpolation).
**Rationale.** Smallest scenarios that exercise (a) the EKF on a single target and (b) multi-target counting / association.
**Ways to improve.** Maneuvering trajectories; crossing/overtaking encounters; per-sensor noise / sensor mix; AIS dropout / non-cooperative scenarios; truth interpolation.

**Files:**
- Create: `core/scenario/Truth.hpp`
- Create: `core/scenario/Builders.hpp`, `core/scenario/Builders.cpp`
- Test: `tests/scenario/test_builders.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_builders.cpp`:
```cpp
#include <gtest/gtest.h>
#include "core/scenario/Builders.hpp"

using navtracker::buildParallelTargetsScenario;
using navtracker::buildStraightLineScenario;
using navtracker::Scenario;

TEST(Builders, StraightLineProducesTruthAndMeasurementsAtSameTimes) {
  const std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 0.0, /*seed=*/1, /*truth_id=*/42);
  ASSERT_EQ(s.truth.size(), 3u);
  ASSERT_EQ(s.measurements.size(), 3u);
  // No noise: truth at t=2 -> (20, 0); measurement matches.
  EXPECT_DOUBLE_EQ(s.truth[1].position.x(), 20.0);
  EXPECT_DOUBLE_EQ(s.truth[1].position.y(), 0.0);
  EXPECT_EQ(s.truth[1].truth_id, 42u);
  EXPECT_DOUBLE_EQ(s.measurements[1].value(0), 20.0);
  EXPECT_DOUBLE_EQ(s.measurements[1].time.seconds(), 2.0);
}

TEST(Builders, ParallelTargetsEmitsTwoTruthPerTime) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 500.0),
      Eigen::Vector2d(10.0, 0.0), times, 0.0, /*seed=*/1);
  ASSERT_EQ(s.truth.size(), 4u);
  ASSERT_EQ(s.measurements.size(), 4u);
  // Truth IDs must be distinct (two targets).
  EXPECT_NE(s.truth[0].truth_id, s.truth[1].truth_id);
}

TEST(Builders, DeterministicForSameSeed) {
  const std::vector<double> times{1.0, 2.0};
  const Scenario a = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  const Scenario b = buildStraightLineScenario(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(5.0, 0.0), times, 3.0, 7);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.measurements[i].value(0), b.measurements[i].value(0));
    EXPECT_DOUBLE_EQ(a.measurements[i].value(1), b.measurements[i].value(1));
  }
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `core/scenario/Builders.cpp` to `navtracker_core` (after the last existing source). Append `tests/scenario/test_builders.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/scenario/Builders.hpp` not found.

- [ ] **Step 4: Create `core/scenario/Truth.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// One sample of ground truth for a single target at a moment in time.
struct TruthSample {
  Timestamp time;
  std::uint64_t truth_id{0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // ENU meters
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};  // ENU m/s
};

// A deterministic synthetic dataset: ground truth + the measurement stream
// it produced. Truth and measurements share timestamps so the harness can
// compare without interpolation.
struct Scenario {
  std::vector<Measurement> measurements;  // chronological
  std::vector<TruthSample> truth;         // grouped by time, then by target
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/scenario/Builders.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/scenario/Truth.hpp"

namespace navtracker {

// One target moving at constant velocity from `start`. Measurements are
// Position2D with Gaussian noise (std = pos_noise_std_m, R = std^2 * I2).
Scenario buildStraightLineScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Two parallel constant-velocity targets. At each time both targets emit
// a Position2D measurement; measurements are interleaved (target A first,
// then target B per timestamp). truth_id_a = 1, truth_id_b = 2.
Scenario buildParallelTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

}  // namespace navtracker
```

- [ ] **Step 6: Create `core/scenario/Builders.cpp`**

```cpp
#include "core/scenario/Builders.hpp"

#include <random>

#include "core/types/Ids.hpp"

namespace navtracker {
namespace {

Measurement makeMeasurement(const Eigen::Vector2d& noisy_pos,
                            double t_seconds,
                            double std_m) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_seconds);
  m.sensor = SensorKind::Ais;  // arbitrary; scenarios are simulator-driven
  m.source_id = "sim";
  m.model = MeasurementModel::Position2D;
  m.value = noisy_pos;
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m + 1e-6);
  return m;
}

TruthSample makeTruth(const Eigen::Vector2d& pos,
                      const Eigen::Vector2d& vel,
                      double t_seconds,
                      std::uint64_t truth_id) {
  TruthSample ts;
  ts.time = Timestamp::fromSeconds(t_seconds);
  ts.truth_id = truth_id;
  ts.position = pos;
  ts.velocity = vel;
  return ts;
}

}  // namespace

Scenario buildStraightLineScenario(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& velocity,
                                   const std::vector<double>& times,
                                   double pos_noise_std_m,
                                   std::uint32_t seed,
                                   std::uint64_t truth_id) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_pos = start + velocity * t;
    s.truth.push_back(makeTruth(truth_pos, velocity, t, truth_id));
    const Eigen::Vector2d noisy(truth_pos.x() + noise(rng),
                                truth_pos.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy, t, pos_noise_std_m));
  }
  return s;
}

Scenario buildParallelTargetsScenario(const Eigen::Vector2d& start_a,
                                      const Eigen::Vector2d& start_b,
                                      const Eigen::Vector2d& velocity,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_a = start_a + velocity * t;
    const Eigen::Vector2d truth_b = start_b + velocity * t;
    s.truth.push_back(makeTruth(truth_a, velocity, t, 1));
    s.truth.push_back(makeTruth(truth_b, velocity, t, 2));
    const Eigen::Vector2d noisy_a(truth_a.x() + noise(rng),
                                  truth_a.y() + noise(rng));
    const Eigen::Vector2d noisy_b(truth_b.x() + noise(rng),
                                  truth_b.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_a, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_b, t, pos_noise_std_m));
  }
  return s;
}

}  // namespace navtracker
```

- [ ] **Step 7: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 8: Commit**

```bash
git add core/scenario/Truth.hpp core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_builders.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add Truth/Scenario types and deterministic builders"
```

---

## Task 2: OSPA metric (greedy nearest with cutoff, p=2)

**Math.** Given truth set `X` (size n_X) and estimate set `Y` (size n_Y); let `n = max(|X|,|Y|)`. For each greedy iteration, pair the closest remaining (x_i, y_j); record `d² = min(‖x_i − y_j‖, c)²`. Then `(n − pairs)` unmatched contribute `c²` each. Return `√(sum / n)`.
**Assumptions.** p = 2; cutoff `c` chosen per scenario (units = meters); greedy approximation in place of Hungarian.
**Rationale.** Standard MTT performance metric; mirrors the data-association cost we already use.
**Ways to improve.** Replace greedy with Hungarian (O(n³)) for optimal assignment; add OSPA-on-OSPA (OSPA²) over time windows.

**Files:**
- Create: `core/scenario/Ospa.hpp`, `core/scenario/Ospa.cpp`
- Test: `tests/scenario/test_ospa.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_ospa.cpp`:
```cpp
#include <gtest/gtest.h>
#include "core/scenario/Ospa.hpp"

using navtracker::ospaGreedy;

TEST(Ospa, ZeroWhenBothEmpty) {
  EXPECT_DOUBLE_EQ(
      ospaGreedy(std::vector<Eigen::Vector2d>{},
                 std::vector<Eigen::Vector2d>{}, 10.0),
      0.0);
}

TEST(Ospa, SinglePairDistance) {
  // One truth at (0,0), one est at (3,0), c=10 -> sqrt(9/1) = 3.
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(3.0, 0.0)};
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 3.0, 1e-12);
}

TEST(Ospa, UnmatchedCountsAsCutoff) {
  // One truth, no estimates, c=10 -> sqrt(100/1) = 10.
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{};
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 10.0, 1e-12);
}

TEST(Ospa, TwoPairsAverage) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0),
                                      Eigen::Vector2d(100.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(5.0, 0.0),
                                    Eigen::Vector2d(105.0, 0.0)};
  // Two pairs at d=5 -> sqrt((25+25)/2) = 5.
  EXPECT_NEAR(ospaGreedy(truth, est, 50.0), 5.0, 1e-12);
}

TEST(Ospa, DistancesAboveCutoffAreClipped) {
  std::vector<Eigen::Vector2d> truth{Eigen::Vector2d(0.0, 0.0)};
  std::vector<Eigen::Vector2d> est{Eigen::Vector2d(1000.0, 0.0)};
  // d=1000 > c=10 -> clipped to c. sqrt(100/1) = 10.
  EXPECT_NEAR(ospaGreedy(truth, est, 10.0), 10.0, 1e-12);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `core/scenario/Ospa.cpp` to `navtracker_core`. Append `tests/scenario/test_ospa.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `core/scenario/Ospa.hpp` not found.

- [ ] **Step 4: Create `core/scenario/Ospa.hpp`**

```cpp
#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

// OSPA (p=2) with cutoff c, using greedy nearest-pair assignment (an
// approximation; the optimal version uses Hungarian).
double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff);

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/scenario/Ospa.cpp`**

```cpp
#include "core/scenario/Ospa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {

double ospaGreedy(const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<Eigen::Vector2d>& est,
                  double cutoff) {
  const std::size_t n = std::max(truth.size(), est.size());
  if (n == 0) return 0.0;

  std::vector<bool> t_used(truth.size(), false);
  std::vector<bool> e_used(est.size(), false);
  double sum_sq = 0.0;
  std::size_t pairs = 0;

  while (true) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t bi = 0, bj = 0;
    bool found = false;
    for (std::size_t i = 0; i < truth.size(); ++i) {
      if (t_used[i]) continue;
      for (std::size_t j = 0; j < est.size(); ++j) {
        if (e_used[j]) continue;
        const double d = (truth[i] - est[j]).norm();
        if (d < best) {
          best = d;
          bi = i;
          bj = j;
          found = true;
        }
      }
    }
    if (!found) break;
    const double clipped = std::min(best, cutoff);
    sum_sq += clipped * clipped;
    t_used[bi] = true;
    e_used[bj] = true;
    ++pairs;
  }

  const std::size_t unmatched = n - pairs;
  sum_sq += static_cast<double>(unmatched) * cutoff * cutoff;
  return std::sqrt(sum_sq / static_cast<double>(n));
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Ospa.hpp core/scenario/Ospa.cpp tests/scenario/test_ospa.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add greedy OSPA (p=2, cutoff) metric"
```

---

## Task 3: Harness + single-target scenario test

**Math/Logic.** For each measurement in the scenario in order: `tracker.process(z)`; gather (a) all truth positions whose `time == z.time` and (b) all current track positions from `manager.tracks()` (first two state components are `[px, py]`); compute `ospaGreedy(truth, est, cutoff)`; append to `ospa_per_step`. Return `mean_ospa` over all steps.
**Assumptions.** First two state components are ENU position; truth share timestamps with measurements (true for the baseline builders).
**Rationale.** Single entry point that takes any composed tracker and a scenario, returns a scalar score → exactly what's needed to compare algorithm choices.
**Ways to improve.** Truth interpolation when timestamps don't align; cardinality + localization breakdown; ID-switch counting.

**Files:**
- Create: `core/scenario/Harness.hpp`, `core/scenario/Harness.cpp`
- Test: `tests/scenario/test_harness.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_harness.cpp`:
```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"

using navtracker::buildStraightLineScenario;
using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::runScenario;
using navtracker::Scenario;
using navtracker::ScenarioResult;
using navtracker::Tracker;
using navtracker::TrackManager;

TEST(Harness, SingleStraightTargetGetsLowOspa) {
  std::vector<double> times;
  for (int i = 1; i <= 20; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(100.0, 0.0), Eigen::Vector2d(5.0, 0.0),
      times, 5.0, /*seed=*/13);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(/*confirm=*/2, /*delete=*/3);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, /*ospa_cutoff=*/50.0);
  EXPECT_EQ(r.ospa_per_step.size(), 20u);
  EXPECT_LT(r.mean_ospa, 15.0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `core/scenario/Harness.cpp` to `navtracker_core`. Append `tests/scenario/test_harness.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `core/scenario/Harness.hpp` not found.

- [ ] **Step 4: Create `core/scenario/Harness.hpp`**

```cpp
#pragma once

#include <vector>

#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

struct ScenarioResult {
  std::vector<double> ospa_per_step;
  double mean_ospa{0.0};
};

// Drive a scenario through the supplied tracker. At each measurement time,
// compute OSPA between truth-at-that-time and the manager's current tracks
// (first two state components used as estimated position).
ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff);

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/scenario/Harness.cpp`**

```cpp
#include "core/scenario/Harness.hpp"

#include "core/scenario/Ospa.hpp"

namespace navtracker {

ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff) {
  ScenarioResult r;
  for (const Measurement& z : scenario.measurements) {
    tracker.process(z);

    std::vector<Eigen::Vector2d> truth_xy;
    for (const TruthSample& ts : scenario.truth) {
      if (ts.time == z.time) truth_xy.push_back(ts.position);
    }
    std::vector<Eigen::Vector2d> est_xy;
    for (const Track& tr : manager.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
      }
    }
    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));
  }
  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green; `mean_ospa < 15.0`.

If the assertion fails by a small margin, do NOT relax it without investigating: print the per-step values, ensure the tracker actually confirmed the track (M-of-N), and that the gate is wide enough. Tracker constructor and parameters in the test are deliberately chosen.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Harness.hpp core/scenario/Harness.cpp tests/scenario/test_harness.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add runScenario harness and baseline metric test"
```

---

## Task 4: Parallel-targets scenario test

Validates multi-target capability: two parallel tracks must both be confirmed, with low mean OSPA.

**Files:**
- Test: `tests/scenario/test_parallel_targets.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the test**

Create `tests/scenario/test_parallel_targets.cpp`:
```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"

using navtracker::buildParallelTargetsScenario;
using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::runScenario;
using navtracker::Scenario;
using navtracker::ScenarioResult;
using navtracker::Tracker;
using navtracker::TrackManager;

TEST(Scenarios, ParallelTargetsBothConfirmedAndLowOspa) {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 800.0),
      Eigen::Vector2d(5.0, 0.0),
      times, 5.0, /*seed=*/29);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(/*confirm=*/2, /*delete=*/4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, /*ospa_cutoff=*/50.0);
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_LT(r.mean_ospa, 20.0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `tests/scenario/test_parallel_targets.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it passes**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected green.

- [ ] **Step 4: Commit**

```bash
git add tests/scenario/test_parallel_targets.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "test(scenario): parallel multi-target scenario"
```

---

## Task 5: Documentation

Documentation only.

**Files:**
- Create: `docs/algorithms/scenarios-and-metrics.md`

- [ ] **Step 1: Create `docs/algorithms/scenarios-and-metrics.md`**

```markdown
# Scenarios & Metrics

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec §9 (testing).

## 1. Scenario types and builders

**Math/Logic.** Constant-velocity straight-line truth: `pos(t) = start + v·t`.
Measurements: truth + Gaussian noise (zero mean, std `pos_noise_std_m`,
identity R). Parallel-targets variant emits two independent CV truths at the
same timestamps, interleaving their measurements.

**Assumptions.** Constant velocity per target; Gaussian zero-mean position
noise; deterministic `std::mt19937` seeded from `seed`; truth and measurements
share timestamps.

**Rationale.** Smallest deterministic scenarios that exercise (a) the EKF on a
single target and (b) multi-target counting / association.

**Ways to improve / test next.** Maneuvering trajectories (turn rate, change
of speed); crossing / overtaking encounters; AIS dropout, non-cooperative
target (no MMSI); per-sensor noise models; truth interpolation for arbitrary
query times.

## 2. OSPA (greedy, p=2)

**Math.** Given truth set X, estimate set Y, cutoff `c`, `n = max(|X|,|Y|)`.
Greedily pair the closest remaining (x,y); record `min(‖x−y‖, c)²`. Unmatched
elements contribute `c²` each. `OSPA = √(sum / n)`.

**Assumptions.** p = 2; cutoff units = meters; greedy assignment in place of
the optimal Hungarian.

**Rationale.** Standard MTT performance metric. Greedy mirrors the GNN data
associator and is cheap; gives a meaningful scalar score for scenario
comparisons.

**Ways to improve / test next.** Hungarian assignment for optimal OSPA;
OSPA² over time windows; OSPA decomposition (localization vs cardinality).

## 3. Harness (`runScenario`)

**Math/Logic.** Drives the supplied `Tracker` over the scenario's measurement
stream in chronological order. At each measurement time, samples truth
positions where `truth.time == z.time` and estimated positions from
`manager.tracks()` (first two state components = ENU position), computes
`ospaGreedy`, and aggregates the mean.

**Assumptions.** State layout begins with `[px, py]`; truth and measurements
share timestamps (true for the baseline builders).

**Rationale.** Single entry point that takes any composed tracker
configuration (estimator + associator + manager) and a scenario, returns a
scalar score — the foundation for comparing algorithm choices (UKF/IMM
estimator, JPDA/MHT associator, hint locking) against the baseline.

**Ways to improve / test next.** Truth interpolation; cardinality /
localization breakdown; ID-switch counting; multi-seed runs with variance.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/scenarios-and-metrics.md
git -c commit.gpgsign=false commit -m "docs: add scenarios & metrics algorithm reference"
```

---

## Done criteria

- Full suite green.
- `core/scenario/` contains Truth + Scenario types, two deterministic builders, OSPA metric, and the harness.
- Two scenario tests pass (single-target and parallel multi-target) with sensible OSPA thresholds.
- `docs/algorithms/scenarios-and-metrics.md` documents math, assumptions, rationale, and improvement paths.

## End of Phase 1

This closes the six-plan Phase 1 sequence:

1. Foundation (build system + core types + geodesy).
2. Estimation (CV + EKF + measurement models).
3. Association + track management (gating + greedy GNN + lifecycle).
4. Pipeline + time (reorder buffer + Tracker + deterministic replay).
5. Sensor adapters (AIS + ARPA TTM/TLL + EO/IR + own-ship NMEA + end-to-end).
6. Scenario harness + metrics (this plan).

The deferred alternatives (Joseph-form covariance, UKF/IMM/particle, JPDA/MHT/Hungarian, MMSI/sensor-track-ID hint locking, bearing-only handling, OOSM retrodiction, etc.) are each documented in their respective algorithm docs under "Ways to improve / test next." Plan 6 gives you the numbers to choose which to pursue first.
