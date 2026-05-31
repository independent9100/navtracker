# Stress Scenarios Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Exercise the GNN+EKF baseline against the three classic data-association weakpoints — **crossing**, **overtaking**, and **AIS dropout** — and quantify the result. Adds the infrastructure (per-step harness snapshots + an ID-switch counter) and three scenario tests that lock in the *current* baseline performance as the regression threshold an alternative (IMM, JPDA, MHT, hint-aided association, …) will have to beat.

**Architecture:** Pure extension of `core/scenario/`. The harness gains a per-step snapshot, the metrics gain `countIdSwitches`, and `Builders` gains two new generators (`buildCrossingTargetsScenario`, `buildOvertakingScenario`). The AIS-dropout case needs no new builder — it reuses `buildStraightLineScenario` with sparse times.

**Tech Stack:** C++17 · Eigen · GoogleTest. Builds on plans 1–6 (on `master`).

**Documentation standard (CLAUDE.md):** Math / Assumptions / Rationale / Ways-to-improve per algorithm. Task 6 extends `docs/algorithms/scenarios-and-metrics.md`.

---

## Design notes

- **Snapshot the harness per step.** To reason about ID stability over time, the harness needs to record (truth positions, track id+positions) at every step. Done by adding a `steps` vector to `ScenarioResult`; existing fields stay.
- **ID switches via nearest-truth assignment.** For each truth index, find the nearest track within `cutoff` at every step; count transitions where the assigned `TrackId.value` changes from one non-zero id to a different non-zero id. Track loss (no track in gate) does NOT count as a switch — only "another track took over."
- **Truth ordering is index-stable per builder.** All builders emit truth in a fixed per-step order (target A first, target B second, …) so truth-by-index across steps refers to the same conceptual target. `countIdSwitches` relies on this.
- **Thresholds are baseline-realistic, not aspirational.** Crossing is likely to flip IDs; the assertion is `≤ 2` switches, not `0`. Overtaking with clean y-separation should stay clean (`0` switches expected). Dropout with gap < `miss_timeout` should coast and re-acquire with the same ID.

## File Structure

```
core/scenario/Truth.hpp                ScenarioStep + TrackSnapshot additions
core/scenario/Harness.hpp/.cpp         ScenarioResult.steps + populate
core/scenario/Metrics.hpp/.cpp         countIdSwitches
core/scenario/Builders.hpp/.cpp        buildCrossingTargetsScenario + buildOvertakingScenario
docs/algorithms/scenarios-and-metrics.md   extend with new builders + metric
tests/scenario/test_metrics.cpp
tests/scenario/test_crossing.cpp
tests/scenario/test_overtaking.cpp
tests/scenario/test_ais_dropout.cpp
```

## Build/test (from repo root)

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Commits: `git -c commit.gpgsign=false commit -m "..."`. No pushes.

## Current `core/scenario/Truth.hpp`

```cpp
struct TruthSample { Timestamp time; std::uint64_t truth_id; Eigen::Vector2d position; Eigen::Vector2d velocity; };
struct Scenario { std::vector<Measurement> measurements; std::vector<TruthSample> truth; };
```

## Current `core/scenario/Harness.hpp`

```cpp
struct ScenarioResult { std::vector<double> ospa_per_step; double mean_ospa{0.0}; };
ScenarioResult runScenario(const Scenario&, Tracker&, const TrackManager&, double ospa_cutoff);
```

## Current `core/scenario/Builders.hpp`

```cpp
Scenario buildStraightLineScenario(start, velocity, times, noise_std, seed=0, truth_id=1);
Scenario buildParallelTargetsScenario(start_a, start_b, velocity, times, noise_std, seed=0);
```

---

## Task 1: Per-step snapshots in harness

Add `ScenarioStep { Timestamp time; std::vector<Eigen::Vector2d> truth; std::vector<TrackSnapshot> tracks; }` and `TrackSnapshot { TrackId id; Eigen::Vector2d position; }`. Append one to `ScenarioResult.steps` per processed measurement. Existing tests continue to work (additive).

**Files:**
- Modify: `core/scenario/Truth.hpp`, `core/scenario/Harness.hpp`, `core/scenario/Harness.cpp`
- Test: append to `tests/scenario/test_harness.cpp`

- [ ] **Step 1: Write the failing test (append to `tests/scenario/test_harness.cpp`)**

```cpp
TEST(Harness, RecordsPerStepSnapshots) {
  std::vector<double> times{1.0, 2.0, 3.0};
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 0.0),
      times, 0.0, 7);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 3);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  ASSERT_EQ(r.steps.size(), 3u);
  EXPECT_DOUBLE_EQ(r.steps.front().time.seconds(), 1.0);
  EXPECT_EQ(r.steps.front().truth.size(), 1u);
  // After the first measurement at least one track exists.
  EXPECT_GE(r.steps.back().tracks.size(), 1u);
}
```

- [ ] **Step 2: Verify it fails**

`cmake --build build 2>&1 | head -20`. Expected: `ScenarioResult` has no member `steps`.

- [ ] **Step 3: Extend `core/scenario/Truth.hpp`**

Add after the existing definitions:

```cpp
// Snapshot of a single track at a particular processing step.
struct TrackSnapshot {
  TrackId id;
  Eigen::Vector2d position;
};

// What the harness observed at one processing step.
struct ScenarioStep {
  Timestamp time;
  std::vector<Eigen::Vector2d> truth;     // truth positions in stable per-step order
  std::vector<TrackSnapshot> tracks;      // current track set after processing
};
```

The file already includes `<vector>`, `<Eigen/Core>`, `core/types/Timestamp.hpp`. Also add `#include "core/types/Ids.hpp"` (for `TrackId`).

- [ ] **Step 4: Extend `core/scenario/Harness.hpp`**

Update the `ScenarioResult`:

```cpp
struct ScenarioResult {
  std::vector<double> ospa_per_step;
  double mean_ospa{0.0};
  std::vector<ScenarioStep> steps;
};
```

- [ ] **Step 5: Extend `core/scenario/Harness.cpp`**

Inside the per-measurement loop, after collecting `truth_xy` and `est_xy`, also build a step and emplace it into `r.steps`. Replace the loop body so it reads:

```cpp
  for (const Measurement& z : scenario.measurements) {
    tracker.process(z);

    std::vector<Eigen::Vector2d> truth_xy;
    for (const TruthSample& ts : scenario.truth) {
      if (ts.time == z.time) truth_xy.push_back(ts.position);
    }
    std::vector<Eigen::Vector2d> est_xy;
    std::vector<TrackSnapshot> snaps;
    for (const Track& tr : manager.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
        snaps.push_back(TrackSnapshot{tr.id, Eigen::Vector2d(tr.state(0), tr.state(1))});
      }
    }
    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

    ScenarioStep step;
    step.time = z.time;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));
  }
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green; all previous tests still pass.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Truth.hpp core/scenario/Harness.hpp core/scenario/Harness.cpp tests/scenario/test_harness.cpp
git -c commit.gpgsign=false commit -m "feat(scenario): record per-step snapshots in ScenarioResult"
```

---

## Task 2: `countIdSwitches` metric

**Math/Logic.** For each truth index i, maintain the last-assigned `TrackId.value`. At each step, nearest-truth-to-track within `cutoff`; if `last[i] ≠ 0 && now ≠ 0 && now ≠ last[i]`, count one switch; then `last[i] = now` if `now ≠ 0`.
**Assumptions.** Truth ordering is index-stable per builder (true for ours); cutoff in meters; a missing nearest (no track in gate) is a track-loss event, not an ID switch.
**Rationale.** Captures the canonical GNN failure mode at close passes.
**Ways to improve.** Per-truth identification across multi-target scenarios via Hungarian assignment; lifetime continuity weighting.

**Files:**
- Create: `core/scenario/Metrics.hpp`, `core/scenario/Metrics.cpp`
- Test: `tests/scenario/test_metrics.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_metrics.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/scenario/Metrics.hpp"

using navtracker::countIdSwitches;
using navtracker::ScenarioStep;
using navtracker::TrackId;
using navtracker::TrackSnapshot;

namespace {
ScenarioStep step(double t,
                  const std::vector<Eigen::Vector2d>& truth,
                  const std::vector<TrackSnapshot>& tracks) {
  ScenarioStep s;
  s.time = navtracker::Timestamp::fromSeconds(t);
  s.truth = truth;
  s.tracks = tracks;
  return s;
}
}  // namespace

TEST(CountIdSwitches, NoSwitchesWhenSameTrackTracks) {
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(1.1, 0.0)}}),
      step(2.0, {Eigen::Vector2d(2.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(2.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 0);
}

TEST(CountIdSwitches, OneSwitchWhenIdChangesOnce) {
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)},
           {TrackSnapshot{TrackId{9}, Eigen::Vector2d(1.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 1);
}

TEST(CountIdSwitches, TrackLossDoesNotCount) {
  // Step 1: track 7 visible. Step 2: no track in gate (track lost).
  // Step 3: track 7 returns. That's not a switch.
  std::vector<ScenarioStep> steps{
      step(0.0, {Eigen::Vector2d(0.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(0.1, 0.0)}}),
      step(1.0, {Eigen::Vector2d(1.0, 0.0)}, {}),
      step(2.0, {Eigen::Vector2d(2.0, 0.0)},
           {TrackSnapshot{TrackId{7}, Eigen::Vector2d(2.1, 0.0)}}),
  };
  EXPECT_EQ(countIdSwitches(steps, 10.0), 0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `core/scenario/Metrics.cpp` to `navtracker_core` (after `core/scenario/Harness.cpp`). Append `tests/scenario/test_metrics.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `core/scenario/Metrics.hpp` not found.

- [ ] **Step 4: Create `core/scenario/Metrics.hpp`**

```cpp
#pragma once

#include <vector>

#include "core/scenario/Truth.hpp"

namespace navtracker {

// Count truth-target ID switches across a sequence of harness steps.
// For each truth index i, the metric finds the nearest track within `cutoff`
// at each step and counts transitions where the assigned TrackId changes
// from one non-zero value to a different non-zero value.
int countIdSwitches(const std::vector<ScenarioStep>& steps, double cutoff);

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/scenario/Metrics.cpp`**

```cpp
#include "core/scenario/Metrics.hpp"

namespace navtracker {

int countIdSwitches(const std::vector<ScenarioStep>& steps, double cutoff) {
  if (steps.empty()) return 0;
  const std::size_t k = steps.front().truth.size();
  std::vector<std::uint64_t> last(k, 0);
  int switches = 0;
  for (const auto& s : steps) {
    for (std::size_t i = 0; i < s.truth.size() && i < k; ++i) {
      std::uint64_t best_id = 0;
      double best_d = cutoff;
      for (const auto& snap : s.tracks) {
        const double d = (s.truth[i] - snap.position).norm();
        if (d < best_d) {
          best_d = d;
          best_id = snap.id.value;
        }
      }
      if (last[i] != 0 && best_id != 0 && best_id != last[i]) ++switches;
      if (best_id != 0) last[i] = best_id;
    }
  }
  return switches;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Metrics.hpp core/scenario/Metrics.cpp tests/scenario/test_metrics.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add countIdSwitches metric"
```

---

## Task 3: Crossing-targets builder + scenario test

**Math/Logic.** Two CV targets on opposing courses with a small lateral offset. Target A: `start_a = (-500, +offset)`, `v_a = (+25, 0)`. Target B: `start_b = (+500, -offset)`, `v_b = (-25, 0)`. They cross `x = 0` at `t = 20 s` with `2·offset` meters of lateral separation. Sampled every 1 s for 40 s; noise `σ`.
**Assumptions.** Same time stream for both targets; truth emitted in (A, B) order per step; noise comparable to the offset stresses GNN at the crossing.
**Rationale.** The classic data-association stressor. With greedy GNN, the assignment at the crossing instant can swap.
**Ways to improve.** Configurable lateral offset / crossing angle; multiple crossing pairs.

**Test thresholds (baseline).** `mgr.size() == 2` end-state, `countIdSwitches ≤ 2`. Tighter thresholds become the regression bar for an IMM/JPDA upgrade.

**Files:**
- Modify: `core/scenario/Builders.hpp`, `core/scenario/Builders.cpp`
- Test: `tests/scenario/test_crossing.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_crossing.cpp`:

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
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

TEST(Stress, CrossingTargetsStayCountedAndIdsMostlyStable) {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      /*start_a=*/Eigen::Vector2d(-500.0, 10.0),
      /*velocity_a=*/Eigen::Vector2d(25.0, 0.0),
      /*start_b=*/Eigen::Vector2d(500.0, -10.0),
      /*velocity_b=*/Eigen::Vector2d(-25.0, 0.0),
      times, /*noise_std=*/8.0, /*seed=*/11);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_LE(countIdSwitches(r.steps, 30.0), 2);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `tests/scenario/test_crossing.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `buildCrossingTargetsScenario` not declared.

- [ ] **Step 4: Extend `core/scenario/Builders.hpp`**

Add the declaration:

```cpp
// Two CV targets on opposing courses, optionally laterally offset. Truth is
// emitted in (A, B) order per step.
Scenario buildCrossingTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& velocity_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity_b,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);
```

- [ ] **Step 5: Extend `core/scenario/Builders.cpp`**

Add the implementation (placing it after `buildParallelTargetsScenario`):

```cpp
Scenario buildCrossingTargetsScenario(const Eigen::Vector2d& start_a,
                                      const Eigen::Vector2d& velocity_a,
                                      const Eigen::Vector2d& start_b,
                                      const Eigen::Vector2d& velocity_b,
                                      const std::vector<double>& times,
                                      double pos_noise_std_m,
                                      std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_a = start_a + velocity_a * t;
    const Eigen::Vector2d truth_b = start_b + velocity_b * t;
    s.truth.push_back(makeTruth(truth_a, velocity_a, t, 1));
    s.truth.push_back(makeTruth(truth_b, velocity_b, t, 2));
    const Eigen::Vector2d noisy_a(truth_a.x() + noise(rng),
                                  truth_a.y() + noise(rng));
    const Eigen::Vector2d noisy_b(truth_b.x() + noise(rng),
                                  truth_b.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_a, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_b, t, pos_noise_std_m));
  }
  return s;
}
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green; baseline meets `mgr.size() == 2` and `countIdSwitches ≤ 2`. If the count exceeds 2, investigate before relaxing — print `countIdSwitches` output and the per-step OSPA, then either raise the assertion to a documented level or strengthen the scenario (e.g. wider lateral offset).

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_crossing.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add crossing-targets stress builder and test"
```

---

## Task 4: Overtaking builder + scenario test

**Math/Logic.** Same-direction CV targets with a y-separation; the faster one passes the slower one along x. With sufficient y-separation (≥ ~5σ) tracks should stay separate. Test asserts `mgr.size() == 2` end-state and `countIdSwitches == 0`.

**Files:**
- Modify: `core/scenario/Builders.hpp`, `core/scenario/Builders.cpp`
- Test: `tests/scenario/test_overtaking.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_overtaking.cpp`:

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
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

TEST(Stress, OvertakingKeepsBothTracksDistinct) {
  std::vector<double> times;
  for (int i = 1; i <= 60; ++i) times.push_back(static_cast<double>(i));
  // A slow at y=30, B faster at y=0; B overtakes longitudinally with 30 m
  // cross-track separation throughout.
  const Scenario s = buildOvertakingScenario(
      /*start_slow=*/Eigen::Vector2d(0.0, 30.0),
      /*velocity_slow=*/Eigen::Vector2d(10.0, 0.0),
      /*start_fast=*/Eigen::Vector2d(-500.0, 0.0),
      /*velocity_fast=*/Eigen::Vector2d(20.0, 0.0),
      times, /*noise_std=*/5.0, /*seed=*/23);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_EQ(countIdSwitches(r.steps, 30.0), 0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `tests/scenario/test_overtaking.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Reconfigure + build. Expected: `buildOvertakingScenario` not declared.

- [ ] **Step 4: Extend `core/scenario/Builders.hpp`**

Add the declaration:

```cpp
// Two same-direction CV targets with a lateral offset. Truth is emitted in
// (slow, fast) order per step.
Scenario buildOvertakingScenario(
    const Eigen::Vector2d& start_slow,
    const Eigen::Vector2d& velocity_slow,
    const Eigen::Vector2d& start_fast,
    const Eigen::Vector2d& velocity_fast,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);
```

- [ ] **Step 5: Extend `core/scenario/Builders.cpp`**

Add the implementation (placing it after `buildCrossingTargetsScenario`):

```cpp
Scenario buildOvertakingScenario(const Eigen::Vector2d& start_slow,
                                 const Eigen::Vector2d& velocity_slow,
                                 const Eigen::Vector2d& start_fast,
                                 const Eigen::Vector2d& velocity_fast,
                                 const std::vector<double>& times,
                                 double pos_noise_std_m,
                                 std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, pos_noise_std_m);
  Scenario s;
  for (double t : times) {
    const Eigen::Vector2d truth_slow = start_slow + velocity_slow * t;
    const Eigen::Vector2d truth_fast = start_fast + velocity_fast * t;
    s.truth.push_back(makeTruth(truth_slow, velocity_slow, t, 1));
    s.truth.push_back(makeTruth(truth_fast, velocity_fast, t, 2));
    const Eigen::Vector2d noisy_slow(truth_slow.x() + noise(rng),
                                     truth_slow.y() + noise(rng));
    const Eigen::Vector2d noisy_fast(truth_fast.x() + noise(rng),
                                     truth_fast.y() + noise(rng));
    s.measurements.push_back(makeMeasurement(noisy_slow, t, pos_noise_std_m));
    s.measurements.push_back(makeMeasurement(noisy_fast, t, pos_noise_std_m));
  }
  return s;
}
```

- [ ] **Step 6: Verify it passes**

`cmake --build build && ctest --test-dir build --output-on-failure`. Expected green.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Builders.hpp core/scenario/Builders.cpp tests/scenario/test_overtaking.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(scenario): add overtaking stress builder and test"
```

---

## Task 5: AIS dropout test (no new builder)

**Logic.** Reuse `buildStraightLineScenario` with sparse times. A single target measured at t ∈ {1..5}, dropped at 6..11, then resumes at 12..20. With `miss_timeout = 15 s`, the gap of 7 s should be absorbed: the track coasts (no maintenance fires because no measurements arrive during the gap), then re-associates the first post-gap measurement to the same track. Test asserts a single track with the *same* TrackId across the gap and zero ID switches.

**Files:**
- Test: `tests/scenario/test_ais_dropout.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the test**

Create `tests/scenario/test_ais_dropout.cpp`:

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
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

TEST(Stress, AisDropoutTrackSurvivesGapWithSameId) {
  // Present at 1..5, gap, present at 12..20: a 7-second dropout.
  std::vector<double> times;
  for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
  for (int i = 12; i <= 20; ++i) times.push_back(static_cast<double>(i));

  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, /*noise_std=*/5.0, /*seed=*/3);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(80.0);   // wider gate to bridge the gap
  TrackManager mgr(2, 5);
  Tracker tracker(est, assoc, mgr, /*miss_timeout=*/15.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 80.0);
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(countIdSwitches(r.steps, 80.0), 0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Append `tests/scenario/test_ais_dropout.cpp` to `navtracker_tests`.

- [ ] **Step 3: Build and run**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected green.

If `mgr.size() != 1` (the track was deleted or duplicated): the predicted position after the 7 s gap is `start + v · (t_resume)`. With `v=(10,0)` and noise drift, predicted position should still match the measurement within the gate. If it doesn't, widen the gate before relaxing the assertion.

- [ ] **Step 4: Commit**

```bash
git add tests/scenario/test_ais_dropout.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "test(scenario): AIS dropout — track survives gap with same id"
```

---

## Task 6: Docs

Extend `docs/algorithms/scenarios-and-metrics.md` with sections for the new builders and the ID-switch metric.

**Files:**
- Modify: `docs/algorithms/scenarios-and-metrics.md`

- [ ] **Step 1: Append the following sections to the doc**

````markdown
## 4. Stress scenarios

### Crossing (`buildCrossingTargetsScenario`)

**Math/Logic.** Two CV targets on opposing courses with a configurable
lateral offset. They cross `x = 0` at a known time with `2·offset` meters of
y-separation. Stress comes from the data-association decision at the
crossing instant when measurement noise overlaps the lateral separation.

**Assumptions.** Targets emit truth in (A, B) order per step; one
measurement per target per step.

**Rationale.** The canonical GNN failure mode (target swap).

**Ways to improve / test next.** Variable crossing angle and offset;
multiple simultaneous crossings; mixed-rate sensors at the crossing.

### Overtaking (`buildOvertakingScenario`)

**Math/Logic.** Same-direction CV targets with a y-offset; the faster one
passes the slower one along x. With sufficient lateral separation tracks
should stay distinct.

**Assumptions.** Cross-track separation ≥ several measurement σ to keep the
gate uncluttered.

**Rationale.** Close-pass without crossing; tests that proximity alone
doesn't trigger an ID switch.

**Ways to improve / test next.** Varying lateral offset; multi-sensor
contributions during the pass.

### AIS dropout (reuses `buildStraightLineScenario` with sparse times)

**Math/Logic.** Single straight-line target with a gap in the measurement
stream; the EKF predicts across the gap. With `gap < miss_timeout`, no
maintenance miss fires while no measurements arrive, so the track coasts and
re-associates the first post-gap measurement.

**Assumptions.** Gap is shorter than `miss_timeout`; gate wide enough that
the predicted post-gap position still includes the resumed measurement.

**Rationale.** Tests track survival across cooperative-sensor outages
(common in real AIS feeds).

**Ways to improve / test next.** Multi-target dropout; cross-sensor
hand-off during the gap (AIS drops while ARPA keeps reporting).

## 5. ID-switch metric (`countIdSwitches`)

**Math.** For each truth index i, maintain `last[i]` = last assigned
`TrackId.value`. At each step, find the nearest track within `cutoff` to
truth[i]; if `last[i] ≠ 0 ∧ now ≠ 0 ∧ now ≠ last[i]`, increment switches;
then update `last[i] = now` when `now ≠ 0`.

**Assumptions.** Truth ordering is index-stable across steps (true for the
builders); track-loss events (no track in gate) do NOT count as switches.

**Rationale.** Captures the canonical GNN failure (one track's id is
replaced by another's mid-scenario).

**Ways to improve / test next.** Hungarian-based truth-track matching for
sets where index correspondence is unclear; weighted by track-lifetime.
````

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/scenarios-and-metrics.md
git -c commit.gpgsign=false commit -m "docs: extend scenarios-and-metrics with stress cases and ID-switch metric"
```

---

## Done criteria

- Full suite green.
- Harness records per-step `ScenarioStep`s.
- `countIdSwitches` exists, with edge-case tests (no switch, one switch, track-loss not counted).
- Three stress scenarios (crossing, overtaking, AIS dropout) pass with documented baseline thresholds.
- `docs/algorithms/scenarios-and-metrics.md` documents builders and the ID-switch metric.
