# Truth-Tick-Driven OSPA Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the metric saturation discovered in the bus-driven comparisons by evaluating OSPA on the truth-sample clock instead of per-measurement, then re-run the four prior bus comparisons under the corrected metric.

**Architecture:** The three harness entry points (`runScenario`, `runScenarioBatched`, `runScenarioBatchedMht`) currently emit one `ospa_per_step` sample per measurement and match truth via `==` on `Timestamp`. Under the bus regime, truth is at 1 Hz while EO/IR fires at 10 Hz, so 93% of evaluation points have empty `truth_xy` and `ospaGreedy([], est, cutoff)` returns exactly the cutoff, pinning the reported mean near saturation. The fix is structural: walk the loop on unique truth timestamps; at each truth tick, process all pending measurements with `time <= tick`, then snapshot tracks and compute OSPA once. Direct-measurement scenarios (1:1 truth↔measurement) are numerically unaffected because every truth tick already has its measurements processed; only bus scenarios change.

**Tech Stack:** C++17, Eigen, gtest, CMake/Conan.

**Context for the implementer:**
- See `docs/superpowers/specs/2026-06-01-simulated-sensor-bus-design.md` for SimBus background.
- See `docs/algorithms/evaluation-log.md` (last section, "Bus-driven confirmation pass") for the prior — now suspect — verdicts that this fix re-tests.
- Saturation evidence (to be deleted in Task 10): `tests/sim/test_ospa_saturation_probe.cpp` shows 93%/92%/0% empty-truth ticks for the three saturating bus scenarios, with populated-truth means of 48.20/73.66 m vs cutoff 50/100.
- The metric refactor is **API-preserving**: signatures of `runScenario`, `runScenarioBatched`, `runScenarioBatchedMht` stay identical. Only the internal loop changes.
- All direct-measurement tests (`tests/scenario/*`) must continue to pass without re-baselining — verify this in Task 5 before touching bus baselines.

---

## File Structure

**Modify (core, source):**
- `core/scenario/Harness.cpp` — replace measurement-clock loop with truth-tick loop (Task 2)
- `core/scenario/HarnessBatched.cpp` — same refactor (Task 3)
- `core/scenario/HarnessBatchedMht.cpp` — same refactor (Task 4)

**Create (tests):**
- `tests/scenario/test_truth_tick_ospa.cpp` — gate test that verifies one OSPA sample per truth tick, regardless of measurement rate (Task 1)

**Modify (tests, re-baseline / re-run):**
- `tests/sim/test_bus_regression.cpp` — update OSPA tolerance baselines (Task 6)
- `tests/sim/test_bus_jpda_comparison.cpp` — record post-fix numbers in `stderr` print (Task 7)
- `tests/sim/test_bus_imm3_comparison.cpp` — same (Task 8)
- `tests/sim/test_bus_pf_comparison.cpp` — verify unchanged, update print (Task 9)
- `tests/sim/test_bus_mht_comparison.cpp` — same (Task 10)

**Modify (docs):**
- `docs/algorithms/evaluation-log.md` — append "Post-metric-fix bus pass" section (Task 11)

**Delete:**
- `tests/sim/test_ospa_saturation_probe.cpp` — diagnostic probe, no longer needed once fix lands (Task 12)
- Remove `tests/sim/test_ospa_saturation_probe.cpp` line from `CMakeLists.txt` (Task 12)

**CMakeLists.txt:** add `tests/scenario/test_truth_tick_ospa.cpp`, remove `tests/sim/test_ospa_saturation_probe.cpp`.

---

### Task 1: Gate test for truth-tick OSPA semantics

A failing-then-passing test that pins the new contract: one OSPA evaluation per unique truth timestamp, regardless of how many measurements fire between truth ticks. We cover all three harness variants with one fixture each.

**Files:**
- Create: `tests/scenario/test_truth_tick_ospa.cpp`
- Modify: `CMakeLists.txt` (line near 127, in the `add_executable(navtracker_tests …)` source list)

- [ ] **Step 1: Write the failing test**

Create `tests/scenario/test_truth_tick_ospa.cpp` with the following content. The test builds a tiny scenario by hand: 1 stationary target, truth sampled at 1 Hz (t = 0, 1, 2 → 3 samples), measurements at 10 Hz (t = 0.0, 0.1, …, 2.0 → 21 measurements). After the refactor, all three harnesses must emit exactly 3 OSPA samples (one per truth tick), each timestamp equal to a truth tick.

```cpp
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/mht/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

// Build a scenario with truth samples at 1 Hz (t = 0, 1, 2) and noiseless
// Position2D measurements at 10 Hz (t = 0.0, 0.1, ..., 2.0). One stationary
// target at the origin.
Scenario buildSparseTruthScenario() {
  Scenario s;
  for (int k = 0; k <= 2; ++k) {
    TruthSample ts;
    ts.time = Timestamp::fromSeconds(static_cast<double>(k));
    ts.truth_id = 1;
    ts.position = Eigen::Vector2d::Zero();
    ts.velocity = Eigen::Vector2d::Zero();
    s.truth.push_back(ts);
  }
  for (int k = 0; k <= 20; ++k) {
    Measurement m;
    m.time = Timestamp::fromSeconds(0.1 * static_cast<double>(k));
    m.type = Measurement::Type::Position2D;
    m.position = Eigen::Vector2d::Zero();
    m.position_covariance = Eigen::Matrix2d::Identity();
    s.measurements.push_back(m);
  }
  return s;
}

}  // namespace

TEST(TruthTickOspa, RunScenarioEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 5);
  Tracker tracker(ekf, gnn, mgr, 10.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}

TEST(TruthTickOspa, RunScenarioBatchedEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  JpdaAssociator jpda(20.0, 0.9, 1e-4);
  TrackManager mgr(1, 5);
  Tracker tracker(ekf, jpda, mgr, 10.0);

  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}

TEST(TruthTickOspa, RunScenarioBatchedMhtEvaluatesAtTruthTicks) {
  const Scenario s = buildSparseTruthScenario();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker tracker(ekf, cfg);

  const ScenarioResult r = runScenarioBatchedMht(s, tracker, 50.0);

  ASSERT_EQ(r.ospa_per_step.size(), 3u);
  ASSERT_EQ(r.steps.size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_DOUBLE_EQ(r.steps[k].time.seconds(),
                     static_cast<double>(k));
  }
}
```

- [ ] **Step 2: Register the test in `CMakeLists.txt`**

Add the new source file to the `add_executable(navtracker_tests …)` list. Open `CMakeLists.txt`, find the line:

```cmake
  tests/sim/test_bus_mht_comparison.cpp
  tests/sim/test_ospa_saturation_probe.cpp
)
```

Insert a new line under `tests/scenario/…` group (around line 100 — find an existing `tests/scenario/test_*.cpp` line and add immediately after):

```cmake
  tests/scenario/test_truth_tick_ospa.cpp
```

The exact placement inside the source list doesn't matter functionally; group it with the other `tests/scenario/` entries.

- [ ] **Step 3: Build and run the new test to verify it fails**

Run from the build directory:

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='TruthTickOspa.*' 2>&1 | tail -20
```

Expected: build succeeds; all three `TruthTickOspa.*` tests FAIL with assertion `r.ospa_per_step.size() == 3` failing (current implementation produces 21).

If the build fails because of a missing include, add the missing include (most likely `core/mht/MhtTracker.hpp`) and re-run. Do not modify the harness yet.

- [ ] **Step 4: Commit**

```bash
git add tests/scenario/test_truth_tick_ospa.cpp CMakeLists.txt
git commit -m "test(scenario): gate test for truth-tick-driven OSPA contract"
```

---

### Task 2: Refactor `runScenario` to truth-tick clock

Replace the measurement-clock loop with a truth-tick clock. For each unique truth timestamp, process all measurements `time <= tick`, then evaluate.

**Files:**
- Modify: `core/scenario/Harness.cpp`

- [ ] **Step 1: Replace the body of `runScenario`**

Open `core/scenario/Harness.cpp`. Replace lines 7-41 (entire function body) with:

```cpp
ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff) {
  ScenarioResult r;
  if (scenario.truth.empty()) return r;

  std::size_t mi = 0;  // next unprocessed measurement
  std::size_t ti = 0;  // first truth sample of the current tick group

  while (ti < scenario.truth.size()) {
    const Timestamp tick = scenario.truth[ti].time;

    // Process every measurement whose timestamp is <= the current truth tick.
    while (mi < scenario.measurements.size() &&
           !(tick < scenario.measurements[mi].time)) {
      tracker.process(scenario.measurements[mi]);
      ++mi;
    }

    // Collect all truth samples for this tick.
    std::vector<Eigen::Vector2d> truth_xy;
    std::size_t tj = ti;
    while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
      truth_xy.push_back(scenario.truth[tj].position);
      ++tj;
    }

    // Snapshot current tracks.
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
    step.time = tick;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));

    ti = tj;
  }

  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}
```

The `!(tick < measurement.time)` comparison rather than `<=` avoids requiring an `operator<=` on Timestamp (the codebase already exposes `operator<` and `operator==`). Verify by checking `core/types/Timestamp.hpp` if needed — if `operator<=` exists, you may use it for clarity.

- [ ] **Step 2: Build and run the `runScenario` gate test**

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='TruthTickOspa.RunScenarioEvaluatesAtTruthTicks' 2>&1 | tail -10
```

Expected: `[  PASSED  ] 1 test.`

- [ ] **Step 3: Run all existing direct-measurement scenario tests to confirm no regressions**

```bash
cd build && ./navtracker_tests --gtest_filter='-TruthTickOspa.*:OspaSaturationProbe.*:BusComparison.*:BusRegression.*' 2>&1 | tail -10
```

Expected: all tests PASS. Direct-measurement scenarios produce identical OSPA per truth tick because every truth timestamp has a corresponding measurement.

If any direct-measurement scenario test fails, STOP and report. The most likely cause is a scenario builder that produces measurements at timestamps where no truth sample exists — investigate before proceeding.

- [ ] **Step 4: Commit**

```bash
git add core/scenario/Harness.cpp
git commit -m "refactor(scenario): drive runScenario OSPA on truth clock"
```

---

### Task 3: Refactor `runScenarioBatched` to truth-tick clock

Same structural change, but with batched processing: at each truth tick, group pending measurements by timestamp and call `processBatch` per timestamp before evaluating OSPA.

**Files:**
- Modify: `core/scenario/HarnessBatched.cpp`

- [ ] **Step 1: Replace the body of `runScenarioBatched`**

Open `core/scenario/HarnessBatched.cpp`. Replace lines 7-49 (entire function body) with:

```cpp
ScenarioResult runScenarioBatched(const Scenario& scenario,
                                  Tracker& tracker,
                                  const TrackManager& manager,
                                  double ospa_cutoff) {
  ScenarioResult r;
  if (scenario.truth.empty()) return r;

  std::size_t mi = 0;
  std::size_t ti = 0;

  while (ti < scenario.truth.size()) {
    const Timestamp tick = scenario.truth[ti].time;

    // Process all measurements with time <= tick, grouped by timestamp.
    while (mi < scenario.measurements.size() &&
           !(tick < scenario.measurements[mi].time)) {
      const Timestamp t = scenario.measurements[mi].time;
      std::vector<Measurement> scan;
      while (mi < scenario.measurements.size() &&
             scenario.measurements[mi].time == t) {
        scan.push_back(scenario.measurements[mi]);
        ++mi;
      }
      tracker.processBatch(scan);
    }

    std::vector<Eigen::Vector2d> truth_xy;
    std::size_t tj = ti;
    while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
      truth_xy.push_back(scenario.truth[tj].position);
      ++tj;
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
    step.time = tick;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));

    ti = tj;
  }

  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}
```

- [ ] **Step 2: Build and run the batched gate test**

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='TruthTickOspa.RunScenarioBatchedEvaluatesAtTruthTicks' 2>&1 | tail -10
```

Expected: `[  PASSED  ] 1 test.`

- [ ] **Step 3: Run all existing direct-measurement batched scenario tests**

```bash
cd build && ./navtracker_tests --gtest_filter='HarnessBatched*:JpdaComparison*:MhtComparison*' 2>&1 | tail -10
```

Expected: all PASS. If any fail, STOP and report.

- [ ] **Step 4: Commit**

```bash
git add core/scenario/HarnessBatched.cpp
git commit -m "refactor(scenario): drive runScenarioBatched OSPA on truth clock"
```

---

### Task 4: Refactor `runScenarioBatchedMht` to truth-tick clock

Same as Task 3 but for the MHT variant.

**Files:**
- Modify: `core/scenario/HarnessBatchedMht.cpp`

- [ ] **Step 1: Replace the body of `runScenarioBatchedMht`**

Open `core/scenario/HarnessBatchedMht.cpp`. Replace lines 7-48 with:

```cpp
ScenarioResult runScenarioBatchedMht(const Scenario& scenario,
                                     MhtTracker& tracker,
                                     double ospa_cutoff) {
  ScenarioResult r;
  if (scenario.truth.empty()) return r;

  std::size_t mi = 0;
  std::size_t ti = 0;

  while (ti < scenario.truth.size()) {
    const Timestamp tick = scenario.truth[ti].time;

    while (mi < scenario.measurements.size() &&
           !(tick < scenario.measurements[mi].time)) {
      const Timestamp t = scenario.measurements[mi].time;
      std::vector<Measurement> scan;
      while (mi < scenario.measurements.size() &&
             scenario.measurements[mi].time == t) {
        scan.push_back(scenario.measurements[mi]);
        ++mi;
      }
      tracker.processBatch(scan);
    }

    std::vector<Eigen::Vector2d> truth_xy;
    std::size_t tj = ti;
    while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
      truth_xy.push_back(scenario.truth[tj].position);
      ++tj;
    }

    std::vector<Eigen::Vector2d> est_xy;
    std::vector<TrackSnapshot> snaps;
    for (const Track& tr : tracker.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
        snaps.push_back(TrackSnapshot{tr.id, Eigen::Vector2d(tr.state(0), tr.state(1))});
      }
    }

    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

    ScenarioStep step;
    step.time = tick;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));

    ti = tj;
  }

  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}
```

- [ ] **Step 2: Build and run the MHT gate test**

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='TruthTickOspa.RunScenarioBatchedMhtEvaluatesAtTruthTicks' 2>&1 | tail -10
```

Expected: `[  PASSED  ] 1 test.`

- [ ] **Step 3: Run direct-measurement MHT tests**

```bash
cd build && ./navtracker_tests --gtest_filter='MhtComparison*' 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add core/scenario/HarnessBatchedMht.cpp
git commit -m "refactor(scenario): drive runScenarioBatchedMht OSPA on truth clock"
```

---

### Task 5: Full test suite green check before re-baselining

Before touching any bus baselines, confirm the only failures are in the bus-specific tests we expect to update (`BusRegression*`, `BusComparison*`) and nothing else.

**Files:**
- None (read-only verification)

- [ ] **Step 1: Run full suite**

```bash
cd build && ./navtracker_tests 2>&1 | tail -40
```

- [ ] **Step 2: Verify the failure set**

The output should show failures ONLY in:
- `BusRegression.*` (bus regression baselines need updating — Task 6)
- Possibly `OspaSaturationProbe.*` if the populated-truth fraction assertion drifts (it has no assertions today, but if you added any while iterating, expect those to need updates)

Failures in any of these are acceptable: `BusComparison.*` (JPDA/IMM/MHT/PF). They use soft assertions but will print different numbers now; whether they still PASS depends on the post-fix winner.

Failures elsewhere — especially `TruthTickOspa`, `HarnessTest`, `Crossing`, `Overtaking`, `ParallelTargets`, `AisDropout`, `MultiSeedSweep`, `FilterComparison`, `JpdaComparison`, `MhtComparison` — would indicate a regression in the refactor. STOP and investigate before continuing.

- [ ] **Step 3: Commit (only if there is something to commit — usually skip)**

This task has no edits; no commit needed.

---

### Task 6: Re-baseline `test_bus_regression.cpp`

The bus regression tests pin OSPA tolerances against numbers captured under the broken metric. Re-capture against the corrected metric.

**Files:**
- Modify: `tests/sim/test_bus_regression.cpp`

- [ ] **Step 1: Run the bus regression tests and capture the new OSPA numbers**

```bash
cd build && ./navtracker_tests --gtest_filter='BusRegression.*' 2>&1
```

For each failing test, note the actual measured `mean_ospa` value reported in the failure message (or printed via `fprintf(stderr, …)` if the test uses that pattern).

- [ ] **Step 2: Read the current baselines**

Open `tests/sim/test_bus_regression.cpp`. The file contains several tests with hard-coded OSPA tolerances (e.g., `EXPECT_LT(mean_ospa, 7.0 * baseline_ospa)` or similar). Locate each tolerance and the comment/justification next to it.

- [ ] **Step 3: Update each baseline to the new measured value**

For each test that failed in Step 1, update the tolerance with the new measurement plus a comment explaining "post truth-tick-OSPA fix". The updated value should be the measured value plus ~10% headroom for cross-platform/build determinism.

Concretely: if a test currently asserts `EXPECT_LT(mean_ospa, 7.0 * X);` and the new measurement is `Y`, change to `EXPECT_LT(mean_ospa, Z);` where `Z = ceil(Y * 1.1 * 100) / 100` (round up to two decimal places). Add a `// Re-baselined 2026-06-02 under truth-tick OSPA; was N * X = …` comment.

If a regression test uses a *ratio* between direct-measurement and bus (the "6.66x regression ratio" mentioned in the eval-log history), the ratio should now be much closer to 1.0 under the corrected metric. Update the ratio bound accordingly with the same headroom rule.

- [ ] **Step 4: Re-run bus regression tests to verify they pass**

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='BusRegression.*' 2>&1 | tail -20
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/sim/test_bus_regression.cpp
git commit -m "test(sim): re-baseline bus regression under truth-tick OSPA"
```

---

### Task 7: Re-run JPDA vs GNN and capture post-fix numbers

The previous bus pass had JPDA tied with GNN at ~49.85 (right at the 50-m cutoff). With the metric fix, expect numbers in the 40–48 m range and a real discriminative signal.

**Files:**
- Modify: `tests/sim/test_bus_jpda_comparison.cpp` (only the printed line; no assertion change unless the soft assertion now fails differently)

- [ ] **Step 1: Run the JPDA bus comparison**

```bash
cd build && ./navtracker_tests --gtest_filter='BusComparison.JpdaVsGnnClutterCrossing' 2>&1 | tail -10
```

Note the four reported numbers (GNN OSPA mean ± stddev and id_sw mean; JPDA OSPA mean ± stddev and id_sw mean). Save them somewhere — they go in the eval-log in Task 11.

- [ ] **Step 2: Verify the soft assertion**

The test asserts `j.mean_ospa < g.mean_ospa || j.mean_id_sw < g.mean_id_sw`. If this fails, the soft assertion is now misaligned with reality. Decide:
- PASS as-is: nothing to do.
- FAIL: leave the failure visible. The point of this task is honest data capture, not forcing a verdict. The eval-log section in Task 11 will record whatever the data shows.

If the test fails and you want CI to remain green for the next round of work, change the assertion to a `SUCCEED();` plus `std::fprintf` (matching the saturation probe pattern) and add a comment: `// Soft observation only; verdict recorded in evaluation-log.md.` Do NOT just delete the assertion silently.

- [ ] **Step 3: Commit**

```bash
git add tests/sim/test_bus_jpda_comparison.cpp
git commit -m "test(sim): capture JPDA vs GNN bus comparison under truth-tick OSPA"
```

---

### Task 8: Re-run IMM-3 vs CV and capture post-fix numbers

Saturation probe showed populated-truth OSPA at 73.66 vs cutoff 100 — IMM-3 has the most headroom to express a margin here.

**Files:**
- Modify: `tests/sim/test_bus_imm3_comparison.cpp`

- [ ] **Step 1: Run**

```bash
cd build && ./navtracker_tests --gtest_filter='BusComparison.ImmVsCvManeuvering' 2>&1 | tail -10
```

(The exact test name may differ; check with `--gtest_list_tests` if needed.)

Capture the four numbers.

- [ ] **Step 2: Handle the soft assertion**

Same logic as Task 7 Step 2: PASS as-is or convert to print-only SUCCEED with comment.

- [ ] **Step 3: Commit**

```bash
git add tests/sim/test_bus_imm3_comparison.cpp
git commit -m "test(sim): capture IMM-3 vs CV bus comparison under truth-tick OSPA"
```

---

### Task 9: Re-run PF vs EKF and verify numerically unchanged

The PF scenario has truth_sample_dt_s = 1.0 and EO/IR at 1 Hz, so 0% empty-truth ticks pre-fix. Numbers should be identical (or within float noise) post-fix.

**Files:**
- Modify: `tests/sim/test_bus_pf_comparison.cpp` (only if numbers drift, which they shouldn't)

- [ ] **Step 1: Run**

```bash
cd build && ./navtracker_tests --gtest_filter='BusComparison.PfVsEkfBearingOnly' 2>&1 | tail -10
```

(Confirm exact test name via `--gtest_list_tests` if needed.)

- [ ] **Step 2: Verify numbers are within ±0.5 m of pre-fix**

Pre-fix: EKF 387.03 ± stddev, PF 380.41 ± stddev (from eval-log). New numbers should match within float-determinism noise. If they don't, investigate — that would indicate the truth/measurement timing relationship in `runBusBearingOnlyMoving` isn't what we modeled.

- [ ] **Step 3: Commit (if any change)**

```bash
git add tests/sim/test_bus_pf_comparison.cpp
git commit -m "test(sim): confirm PF vs EKF bus comparison unchanged by metric fix"
```

If no changes: no commit. Skip to Task 10.

---

### Task 10: Re-run MHT vs JPDA and capture post-fix numbers

Same scenario family as JPDA-vs-GNN; expect non-saturated numbers and a clearer signal.

**Files:**
- Modify: `tests/sim/test_bus_mht_comparison.cpp`

- [ ] **Step 1: Run**

```bash
cd build && ./navtracker_tests --gtest_filter='BusComparison.MhtVsJpdaClutterCrossing' 2>&1 | tail -10
```

(Check exact test name via `--gtest_list_tests`.)

Capture the numbers.

- [ ] **Step 2: Handle the soft assertion** (same logic as Task 7)

- [ ] **Step 3: Commit**

```bash
git add tests/sim/test_bus_mht_comparison.cpp
git commit -m "test(sim): capture MHT vs JPDA bus comparison under truth-tick OSPA"
```

---

### Task 11: Append eval-log "Post-metric-fix bus pass" section

Record the post-fix verdicts alongside the pre-fix ones so future readers can see how the saturation artifact distorted prior conclusions.

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

- [ ] **Step 1: Read the current bus-driven section**

Open `docs/algorithms/evaluation-log.md`. Locate the section titled "Bus-driven confirmation pass (2026-06-02)". Read it.

- [ ] **Step 2: Append a new section after it**

Add at the end of the file:

```markdown
## Post-metric-fix bus pass (2026-06-02)

The "Bus-driven confirmation pass" section above was contaminated by a metric
artifact: `runScenario` evaluated OSPA per-measurement and matched truth via
exact timestamp equality. With truth at 1 Hz and EO/IR at 10 Hz, ~93% of
evaluation points had empty `truth_xy` and `ospaGreedy([], est, cutoff)`
returned the cutoff — pinning the mean near saturation. See
`docs/superpowers/plans/2026-06-02-truth-tick-ospa.md` for the fix.

Verdicts re-run under truth-tick-driven OSPA (one evaluation per truth tick,
after all pending measurements processed):

| Comparison | Pre-fix (saturated) | Post-fix | Verdict |
|---|---|---|---|
| JPDA vs GNN | GNN 49.85/jpda 49.85 OSPA, id_sw 16.90 vs 18.55 | <fill in from Task 7> | <Confirm, Retract, or Unchanged> |
| IMM-3 vs CV | CV 96.95 / IMM 96.79 OSPA | <from Task 8> | <…> |
| PF vs EKF | EKF 387.03 / PF 380.41 OSPA | <from Task 9, expect ≈ unchanged> | Unchanged (no saturation in this scenario) |
| MHT vs JPDA | JPDA 49.85 / MHT 49.59 OSPA, id_sw 18.55 vs 25.55 | <from Task 10> | <…> |

Cross-cutting note: 3/4 prior verdicts were obscured by the metric saturation,
not by the algorithms. The PF verdict held because its scenario configured
EO/IR at 1 Hz (no cadence mismatch).
```

Replace each `<fill in from Task N>` with the actual recorded numbers from earlier tasks. For each verdict cell, write one of:
- **Confirms direct-measurement win** — if the post-fix numbers preserve the prior direct-measurement margin
- **Diminishes direct-measurement win** — if margin shrinks but direction matches
- **Retracts direct-measurement win** — if direction flips or margin disappears within stddev
- **Unchanged** — PF case

- [ ] **Step 3: Commit**

```bash
git add docs/algorithms/evaluation-log.md
git commit -m "docs(eval): record post-metric-fix bus verdicts"
```

---

### Task 12: Remove the saturation probe

The probe served its purpose. With the gate test (Task 1) and updated bus baselines (Tasks 6–10), there is no reason to keep it.

**Files:**
- Delete: `tests/sim/test_ospa_saturation_probe.cpp`
- Modify: `CMakeLists.txt` (remove the `tests/sim/test_ospa_saturation_probe.cpp` line)

- [ ] **Step 1: Delete the probe**

```bash
rm tests/sim/test_ospa_saturation_probe.cpp
```

- [ ] **Step 2: Remove from CMakeLists.txt**

Open `CMakeLists.txt`. Find the line:

```cmake
  tests/sim/test_ospa_saturation_probe.cpp
```

Delete it. Verify the parenthesis-closing line `)` following the source list is preserved.

- [ ] **Step 3: Rebuild and run the full suite**

```bash
cd build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests 2>&1 | tail -10
```

Expected: build succeeds; all tests PASS.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt tests/sim/test_ospa_saturation_probe.cpp
git commit -m "chore(test): remove saturation probe, replaced by truth-tick gate test"
```

---

## Self-review (controller, not subagent)

**Spec coverage check:**
- Metric refactor in all three harnesses → Tasks 2, 3, 4. ✓
- Gate test for new semantics → Task 1. ✓
- Bus regression baselines updated → Task 6. ✓
- Each prior comparison re-run → Tasks 7, 8, 9, 10. ✓
- Eval-log updated → Task 11. ✓
- Probe cleanup → Task 12. ✓

**Placeholder scan:** The eval-log section in Task 11 uses `<fill in from Task N>` markers — these are intentional placeholders to be replaced with measured numbers, not TBDs. The text directs the implementer to capture and substitute. Acceptable per the brief: "concrete commands and expected outputs" — the commands are concrete; the numbers come from running them.

**Type consistency:** `runScenario`, `runScenarioBatched`, `runScenarioBatchedMht` signatures unchanged across all tasks. `ScenarioResult.ospa_per_step` and `ScenarioResult.steps` still have lockstep size. `Timestamp::operator<` and `operator==` used consistently (not `<=` — see Task 2 Step 1 note).
