# Baseline Benchmark Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a benchmark harness that sweeps 5 named tracking configurations across 7 synthetic scenarios (10 seeds each) + 2 replay scenarios, emitting a single labelled CSV per run plus a Markdown companion — so baseline vs. improvement comparisons are joins over a fixed schema, not stdout grep.

**Architecture:** A new `core/benchmark/` library (pure domain, no I/O) holds sweep loop, metrics, and CSV writer. A new `adapters/benchmark/` holds sim and replay adapters that implement the `ScenarioRun` port. Three small executables under `bench/` (`baseline_matrix`, `render_markdown`, `compare`) wire the pieces. Existing `core/scenario/` types (`Scenario`, `TruthSample`, `ospaGreedy`, `countIdSwitches`) are reused — the harness adds an enriched runner that snapshots full track state via the existing `ITrackSink` push interface and direct `TrackManager` reads.

**Tech Stack:** C++17, CMake, Conan, gtest/gmock, Eigen. No new third-party deps.

**Spec:** `docs/superpowers/specs/2026-06-07-baseline-benchmark-harness-design.md`

---

## Reused existing infrastructure (do not modify)

- `core/scenario/Truth.hpp` — `TruthSample {time, truth_id, position, velocity}`, `Scenario {measurements, truth}`, `TrackSnapshot {id, position}`, `ScenarioStep {time, truth_positions, tracks}`.
- `core/scenario/Builders.hpp` — `buildCrossingTargetsScenario`, `buildOvertakingScenario`, `buildParallelTargetsScenario`, `buildCrossingDropoutScenario`, `buildManeuveringTargetScenario`, etc.
- `core/scenario/Ospa.hpp` — `ospaGreedy(truth, est, cutoff)`.
- `core/scenario/Metrics.hpp` — `countIdSwitches(steps, cutoff)`.
- `ports/ITrackSink.hpp` — `ITrackSink` push interface, registered via `TrackManager::setTrackSink`.
- `core/pipeline/Tracker.hpp`, `core/tracking/TrackManager.hpp` — the runtime pipeline.
- Existing replay loaders in `adapters/replay/` and the fixtures referenced by `tests/replay/test_philos_ospa.cpp` and `test_haxr_ospa.cpp`.

## File map

**New:**
- `core/benchmark/ScenarioRun.hpp` — `ScenarioRun` port + descriptor types.
- `core/benchmark/BenchSink.hpp/.cpp` — `ITrackSink` impl recording lifecycle events.
- `core/benchmark/BenchRunner.hpp/.cpp` — drives a `Scenario` through `Tracker`, captures rich per-step state via `BenchSink` + direct `TrackManager` reads, returns `BenchResult`.
- `core/benchmark/Metrics.hpp/.cpp` — `MetricsResult` struct; computes OSPA aggregates, per-step Hungarian assignment, continuity, ID switches, per-track RMSE — all from one `BenchResult`.
- `core/benchmark/Config.hpp/.cpp` — `Config` (label + estimator factory + associator factory) + `defaultConfigs()` returning the 5 baseline configs.
- `core/benchmark/Sweep.hpp/.cpp` — the matrix loop; produces `vector<MetricRow>`.
- `core/benchmark/CsvWriter.hpp/.cpp` — long-format CSV writer with `#`-comment header block.
- `adapters/benchmark/SimScenarioRun.hpp/.cpp` — wraps `Builders.hpp` for the 7 synthetic scenarios.
- `adapters/benchmark/ReplayScenarioRun.hpp/.cpp` — wraps philos + haxr loaders.
- `bench/baseline_matrix.cpp`, `bench/render_markdown.cpp`, `bench/compare.cpp`, `bench/CMakeLists.txt`.
- `tests/benchmark/test_metrics.cpp`, `test_bench_runner.cpp`, `test_csv_writer.cpp`, `test_config.cpp`, `test_sim_scenario_run.cpp`, `test_replay_scenario_run.cpp`, `test_sweep.cpp`, `test_render_markdown.cpp`, `test_compare.cpp`, `test_bench_determinism.cpp`.
- `docs/baselines/README.md` — how to run + how to read.

**Modified:**
- Top-level `CMakeLists.txt` — add `navtracker_benchmark` library, `navtracker_benchmark_adapters` library, `add_subdirectory(bench)`, wire new tests into `navtracker_tests`.

---

## Task 1: Skeleton — `core/benchmark/` library + smoke test

**Files:**
- Create: `core/benchmark/Placeholder.hpp`
- Modify: `CMakeLists.txt`
- Test: `tests/benchmark/test_smoke.cpp`

- [ ] **Step 1: Create placeholder header so the library has at least one TU**

`core/benchmark/Placeholder.hpp`:
```cpp
#pragma once

namespace navtracker {
namespace benchmark {

// Removed in a later task once real sources land.
inline int placeholder() { return 42; }

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Add the library to top-level `CMakeLists.txt`**

Find the line `add_library(navtracker_core ...)` and after the existing `target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)` block, insert:

```cmake
add_library(navtracker_benchmark
  core/benchmark/Placeholder.hpp  # header-only at this stage
)
set_target_properties(navtracker_benchmark PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(navtracker_benchmark PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark PUBLIC navtracker_core Eigen3::Eigen)
```

- [ ] **Step 3: Add a smoke test**

`tests/benchmark/test_smoke.cpp`:
```cpp
#include <gtest/gtest.h>

#include "core/benchmark/Placeholder.hpp"

TEST(BenchmarkSmoke, PlaceholderLinks) {
  EXPECT_EQ(navtracker::benchmark::placeholder(), 42);
}
```

- [ ] **Step 4: Wire the test into `navtracker_tests`**

Find the `add_executable(navtracker_tests ...)` list and add `tests/benchmark/test_smoke.cpp` to it. Find the matching `target_link_libraries(navtracker_tests ...)` block and add `navtracker_benchmark` to the linked libraries.

- [ ] **Step 5: Configure and build**

```bash
cmake --build build --target navtracker_tests -j
```

Expected: build succeeds.

- [ ] **Step 6: Run the smoke test**

```bash
./build/navtracker_tests --gtest_filter='BenchmarkSmoke.*'
```

Expected: 1 test, 1 passes.

- [ ] **Step 7: Commit**

```bash
git add core/benchmark/Placeholder.hpp tests/benchmark/test_smoke.cpp CMakeLists.txt
git commit -m "Bench: skeleton library wired into build"
```

---

## Task 2: `ScenarioRun` port + scenario descriptor types

**Files:**
- Create: `core/benchmark/ScenarioRun.hpp`
- Test: `tests/benchmark/test_scenario_run_port.cpp`

- [ ] **Step 1: Write the failing test**

`tests/benchmark/test_scenario_run_port.cpp`:
```cpp
#include <gtest/gtest.h>

#include "core/benchmark/ScenarioRun.hpp"
#include "core/scenario/Truth.hpp"

using navtracker::Scenario;
using navtracker::benchmark::ScenarioDescriptor;
using navtracker::benchmark::ScenarioRun;

namespace {
class FakeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"fake", /*is_multi_seed=*/true, /*seed_count=*/3};
  }
  Scenario generate(std::uint64_t seed) override {
    Scenario s;
    s.truth.push_back({{}, /*truth_id=*/seed + 1, {}, {}});
    return s;
  }
};
}  // namespace

TEST(ScenarioRunPort, DescriptorAndGenerateRoundtrip) {
  FakeScenarioRun run;
  const auto d = run.descriptor();
  EXPECT_EQ(d.label, "fake");
  EXPECT_TRUE(d.is_multi_seed);
  EXPECT_EQ(d.seed_count, 3u);

  const auto a = run.generate(0);
  const auto b = run.generate(1);
  ASSERT_EQ(a.truth.size(), 1u);
  ASSERT_EQ(b.truth.size(), 1u);
  EXPECT_EQ(a.truth[0].truth_id, 1u);
  EXPECT_EQ(b.truth[0].truth_id, 2u);
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --build build --target navtracker_tests -j
```

Expected: compile error — `core/benchmark/ScenarioRun.hpp` not found.

- [ ] **Step 3: Implement `ScenarioRun.hpp`**

`core/benchmark/ScenarioRun.hpp`:
```cpp
#pragma once

#include <cstdint>
#include <string>

#include "core/scenario/Truth.hpp"

namespace navtracker {
namespace benchmark {

struct ScenarioDescriptor {
  std::string label;
  bool is_multi_seed{false};
  std::uint32_t seed_count{1};
};

// Port: produces a Scenario (measurements + truth) for a given seed.
// Replays ignore the seed; synthetics use it for noise realisation.
class ScenarioRun {
 public:
  virtual ~ScenarioRun() = default;
  virtual ScenarioDescriptor descriptor() const = 0;
  virtual Scenario generate(std::uint64_t seed) = 0;
};

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 4: Wire test, build, run**

Add `tests/benchmark/test_scenario_run_port.cpp` to the `navtracker_tests` source list. Rebuild and run:
```bash
cmake --build build --target navtracker_tests -j
./build/navtracker_tests --gtest_filter='ScenarioRunPort.*'
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/benchmark/ScenarioRun.hpp tests/benchmark/test_scenario_run_port.cpp CMakeLists.txt
git commit -m "Bench: ScenarioRun port + scenario descriptor types"
```

---

## Task 3: `BenchSink` — `ITrackSink` impl that records lifecycle events

**Files:**
- Create: `core/benchmark/BenchSink.hpp`, `core/benchmark/BenchSink.cpp`
- Test: `tests/benchmark/test_bench_sink.cpp`

- [ ] **Step 1: Write the failing test**

`tests/benchmark/test_bench_sink.cpp`:
```cpp
#include <gtest/gtest.h>

#include "core/benchmark/BenchSink.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

using namespace navtracker;
using benchmark::BenchSink;

TEST(BenchSink, RecordsAllLifecycleEvents) {
  BenchSink sink;
  TrackLifecycleEvent e{TrackId{1}, Timestamp::fromSeconds(0.0), TrackStatus::Tentative};
  sink.onTrackInitiated(e);
  e.status = TrackStatus::Confirmed;
  e.time = Timestamp::fromSeconds(1.0);
  sink.onTrackConfirmed(e);
  e.time = Timestamp::fromSeconds(2.0);
  sink.onTrackUpdated(e);
  e.time = Timestamp::fromSeconds(3.0);
  sink.onTrackDeleted(e);

  const auto& events = sink.events();
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[0].kind, BenchSink::Kind::Initiated);
  EXPECT_EQ(events[1].kind, BenchSink::Kind::Confirmed);
  EXPECT_EQ(events[2].kind, BenchSink::Kind::Updated);
  EXPECT_EQ(events[3].kind, BenchSink::Kind::Deleted);
  EXPECT_EQ(events[0].id.value(), 1u);
}
```

- [ ] **Step 2: Run to confirm failure**

Build fails: header missing.

- [ ] **Step 3: Implement `BenchSink.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ITrackSink.hpp"

namespace navtracker {
namespace benchmark {

class BenchSink : public ITrackSink {
 public:
  enum class Kind { Initiated, Confirmed, Updated, Deleted };
  struct Event {
    Kind kind;
    TrackId id;
    Timestamp time;
    TrackStatus status;
  };

  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  void onTrackDeleted(const TrackLifecycleEvent& e) override;

  const std::vector<Event>& events() const { return events_; }
  void clear() { events_.clear(); }

 private:
  std::vector<Event> events_;
};

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 4: Implement `BenchSink.cpp`**

```cpp
#include "core/benchmark/BenchSink.hpp"

namespace navtracker {
namespace benchmark {

void BenchSink::onTrackInitiated(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Initiated, e.id, e.time, e.status});
}
void BenchSink::onTrackConfirmed(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Confirmed, e.id, e.time, e.status});
}
void BenchSink::onTrackUpdated(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Updated, e.id, e.time, e.status});
}
void BenchSink::onTrackDeleted(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Deleted, e.id, e.time, e.status});
}

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 5: Add `core/benchmark/BenchSink.cpp` to `navtracker_benchmark` sources in `CMakeLists.txt`. Add test to `navtracker_tests`. Build, run, expect PASS.**

```bash
cmake --build build --target navtracker_tests -j
./build/navtracker_tests --gtest_filter='BenchSink.*'
```

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/BenchSink.hpp core/benchmark/BenchSink.cpp \
        tests/benchmark/test_bench_sink.cpp CMakeLists.txt
git commit -m "Bench: BenchSink records ITrackSink lifecycle events"
```

---

## Task 4: `BenchRunner` — runs a `Scenario` through `Tracker`, captures full state

`BenchRunner` mirrors `runScenario` but additionally snapshots full track state (position + velocity) at every truth timestamp, so per-track RMSE for position/SOG/COG can be computed downstream. It uses `BenchSink` to observe lifecycle events and reads state directly from `TrackManager::tracks()` at evaluation points.

**Files:**
- Create: `core/benchmark/BenchRunner.hpp`, `core/benchmark/BenchRunner.cpp`
- Test: `tests/benchmark/test_bench_runner.cpp`

- [ ] **Step 1: Define result types in the header**

`core/benchmark/BenchRunner.hpp`:
```cpp
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/benchmark/BenchSink.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {
namespace benchmark {

// Per-track snapshot at a single evaluation timestamp.
struct TrackStateSnapshot {
  TrackId id;
  Eigen::Vector2d position;    // ENU metres
  Eigen::Vector2d velocity;    // ENU m/s
};

// Per truth snapshot at a single evaluation timestamp.
struct TruthStateSnapshot {
  std::uint64_t truth_id;
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
};

// One evaluation slice: all truth + all confirmed tracks at the same time.
struct BenchStep {
  Timestamp time;
  std::vector<TruthStateSnapshot> truth;
  std::vector<TrackStateSnapshot> tracks;
};

struct BenchResult {
  std::vector<BenchStep> steps;
  std::vector<BenchSink::Event> sink_events;  // full lifecycle stream
};

// Drive the scenario through the tracker. Measurements are injected in
// timestamp order. After each truth timestamp is reached, snapshot truth
// and confirmed tracks. The supplied BenchSink is registered with the
// manager for the duration of the run.
BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink);

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Write the test FIRST (TDD)**

`tests/benchmark/test_bench_runner.cpp`:
```cpp
#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/scenario/Builders.hpp"

using namespace navtracker;
using benchmark::BenchSink;

TEST(BenchRunner, ProducesPerTruthSnapshotsAndLifecycleEvents) {
  std::vector<double> times;
  for (int i = 1; i <= 10; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      times, 1.0, /*seed=*/7, /*truth_id=*/1);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  const GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  BenchSink sink;
  const auto result = benchmark::runBench(s, tracker, mgr, sink);

  // One step per truth timestamp.
  EXPECT_EQ(result.steps.size(), times.size());
  // Each step has exactly one truth and (after track confirms) one track.
  for (const auto& step : result.steps) {
    EXPECT_EQ(step.truth.size(), 1u);
  }
  // At least one lifecycle event captured.
  EXPECT_FALSE(result.sink_events.empty());
  // Last step should hold a confirmed track close to truth.
  const auto& last = result.steps.back();
  ASSERT_FALSE(last.tracks.empty());
  EXPECT_LT((last.tracks[0].position - last.truth[0].position).norm(), 25.0);
}
```

- [ ] **Step 3: Run the test to verify failure**

Build fails: `BenchRunner.hpp` exists but no implementation.

- [ ] **Step 4: Implement `BenchRunner.cpp`**

```cpp
#include "core/benchmark/BenchRunner.hpp"

#include <algorithm>

namespace navtracker {
namespace benchmark {
namespace {

// Group truth samples by timestamp (assumes input already sorted by time,
// which Builders honour).
struct TruthGroup {
  Timestamp time;
  std::vector<TruthStateSnapshot> snapshots;
};

std::vector<TruthGroup> groupTruth(const std::vector<TruthSample>& truth) {
  std::vector<TruthGroup> out;
  for (const auto& t : truth) {
    if (out.empty() || out.back().time != t.time) {
      out.push_back({t.time, {}});
    }
    out.back().snapshots.push_back({t.truth_id, t.position, t.velocity});
  }
  return out;
}

}  // namespace

BenchResult runBench(const Scenario& scenario,
                     Tracker& tracker,
                     TrackManager& manager,
                     BenchSink& sink) {
  manager.setTrackSink(&sink);

  BenchResult result;
  const auto truth_groups = groupTruth(scenario.truth);
  std::size_t next_truth = 0;

  for (const auto& m : scenario.measurements) {
    // Drain truth steps whose time has been crossed.
    while (next_truth < truth_groups.size() &&
           truth_groups[next_truth].time <= m.time) {
      BenchStep step;
      step.time = truth_groups[next_truth].time;
      step.truth = truth_groups[next_truth].snapshots;
      for (const auto& tr : manager.tracks()) {
        if (tr.status != TrackStatus::Confirmed) continue;
        step.tracks.push_back({tr.id,
                               tr.state.position(),
                               tr.state.velocity()});
      }
      result.steps.push_back(std::move(step));
      ++next_truth;
    }
    tracker.process(m);
  }

  // Drain remaining truth groups after last measurement.
  while (next_truth < truth_groups.size()) {
    BenchStep step;
    step.time = truth_groups[next_truth].time;
    step.truth = truth_groups[next_truth].snapshots;
    for (const auto& tr : manager.tracks()) {
      if (tr.status != TrackStatus::Confirmed) continue;
      step.tracks.push_back({tr.id,
                             tr.state.position(),
                             tr.state.velocity()});
    }
    result.steps.push_back(std::move(step));
    ++next_truth;
  }

  manager.setTrackSink(nullptr);
  result.sink_events = sink.events();
  return result;
}

}  // namespace benchmark
}  // namespace navtracker
```

**Note for implementer:** the exact accessors `tr.state.position()` / `velocity()` may differ from the actual Track API. Inspect `core/types/Track.hpp` and adjust. The intent is "read 2D ENU position and 2D ENU velocity from the live Track state".

- [ ] **Step 5: Wire `BenchRunner.cpp` into `navtracker_benchmark` library; add test to `navtracker_tests`. Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/BenchRunner.hpp core/benchmark/BenchRunner.cpp \
        tests/benchmark/test_bench_runner.cpp CMakeLists.txt
git commit -m "Bench: BenchRunner snapshots full state at truth timestamps"
```

---

## Task 5: Metrics — OSPA aggregates (`ospa_mean`, `ospa_p95`)

**Files:**
- Create: `core/benchmark/Metrics.hpp`, `core/benchmark/Metrics.cpp`
- Test: extend `tests/benchmark/test_metrics.cpp`

- [ ] **Step 1: Define result type + OSPA-aggregate functions in `Metrics.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "core/benchmark/BenchRunner.hpp"
#include "core/types/Ids.hpp"

namespace navtracker {
namespace benchmark {

struct MetricsResult {
  double ospa_mean{0.0};        // metres
  double ospa_p95{0.0};         // metres
  double lifetime_ratio{0.0};   // [0, 1]
  double track_breaks{0.0};     // count, mean across truth
  double id_switches{0.0};      // count, mean across truth
  double pos_rmse_m{0.0};       // metres
  double sog_rmse_mps{0.0};     // metres/second
  double cog_rmse_deg{0.0};     // degrees
};

struct MetricsParams {
  double ospa_cutoff_m{500.0};
  double assoc_gate_m{100.0};
};

// Per-step OSPA values across the run (one per step in result.steps).
std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m);

double mean(const std::vector<double>& v);
double percentile(std::vector<double> v, double q);  // q in [0,1]

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Write the failing test**

`tests/benchmark/test_metrics.cpp`:
```cpp
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "core/benchmark/Metrics.hpp"

using namespace navtracker;
using benchmark::BenchResult;
using benchmark::BenchStep;
using benchmark::TrackStateSnapshot;
using benchmark::TruthStateSnapshot;

namespace {
BenchStep makeStep(double t,
                   std::vector<Eigen::Vector2d> truth_pos,
                   std::vector<Eigen::Vector2d> track_pos) {
  BenchStep s;
  s.time = Timestamp::fromSeconds(t);
  for (std::size_t i = 0; i < truth_pos.size(); ++i) {
    s.truth.push_back({static_cast<std::uint64_t>(i + 1),
                       truth_pos[i],
                       Eigen::Vector2d::Zero()});
  }
  for (std::size_t i = 0; i < track_pos.size(); ++i) {
    s.tracks.push_back({TrackId{static_cast<std::uint64_t>(i + 1)},
                        track_pos[i],
                        Eigen::Vector2d::Zero()});
  }
  return s;
}
}  // namespace

TEST(Metrics, OspaPerStepMatchesHandComputed) {
  BenchResult r;
  // Step 0: truth (0,0) and track (3,4) -> distance 5
  r.steps.push_back(makeStep(0.0, {{0, 0}}, {{3, 4}}));
  // Step 1: truth (10,0) and track (10,0) -> 0
  r.steps.push_back(makeStep(1.0, {{10, 0}}, {{10, 0}}));

  const auto per_step = benchmark::computeOspaPerStep(r, /*cutoff=*/500.0);
  ASSERT_EQ(per_step.size(), 2u);
  EXPECT_NEAR(per_step[0], 5.0, 1e-9);
  EXPECT_NEAR(per_step[1], 0.0, 1e-9);
}

TEST(Metrics, MeanAndPercentile) {
  std::vector<double> v{1.0, 2.0, 3.0, 4.0, 5.0};
  EXPECT_NEAR(benchmark::mean(v), 3.0, 1e-9);
  EXPECT_NEAR(benchmark::percentile(v, 0.95), 5.0, 1e-9);
  EXPECT_NEAR(benchmark::percentile(v, 0.5), 3.0, 1e-9);
}
```

- [ ] **Step 3: Run to verify failure** (missing impl).

- [ ] **Step 4: Implement `Metrics.cpp` for OSPA aggregates**

```cpp
#include "core/benchmark/Metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "core/scenario/Ospa.hpp"

namespace navtracker {
namespace benchmark {

std::vector<double> computeOspaPerStep(const BenchResult& result,
                                       double cutoff_m) {
  std::vector<double> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    std::vector<Eigen::Vector2d> truth;
    truth.reserve(step.truth.size());
    for (const auto& t : step.truth) truth.push_back(t.position);
    std::vector<Eigen::Vector2d> est;
    est.reserve(step.tracks.size());
    for (const auto& tr : step.tracks) est.push_back(tr.position);
    out.push_back(ospaGreedy(truth, est, cutoff_m));
  }
  return out;
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = q * (v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  const double frac = idx - lo;
  return v[lo] * (1.0 - frac) + v[hi] * frac;
}

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 5: Build, run, expect PASS**

```bash
cmake --build build --target navtracker_tests -j
./build/navtracker_tests --gtest_filter='Metrics.*'
```

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Metrics.hpp core/benchmark/Metrics.cpp \
        tests/benchmark/test_metrics.cpp CMakeLists.txt
git commit -m "Bench: OSPA mean and p95 aggregates"
```

---

## Task 6: Per-step Hungarian assignment under gate

A single shared assignment function feeds continuity, ID-switches, and per-track RMSE — so all three metrics agree on which track represents which truth at each step.

**Files:**
- Modify: `core/benchmark/Metrics.hpp`, `core/benchmark/Metrics.cpp`
- Test: extend `tests/benchmark/test_metrics.cpp`

- [ ] **Step 1: Declare the function in `Metrics.hpp`**

Add inside `namespace benchmark {`:
```cpp
// Per-step assignment: for each truth index in step.truth, the assigned
// TrackId from step.tracks (or std::nullopt if no track within gate).
// Implementation: greedy nearest-neighbour under the gate. (Hungarian is
// equivalent for the small target counts in our scenarios; if profiling
// flags this hot, swap to true Hungarian.)
using StepAssignment = std::vector<std::optional<TrackId>>;
std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m);
```

- [ ] **Step 2: Write the failing test**

In `tests/benchmark/test_metrics.cpp`, add:
```cpp
TEST(Metrics, AssignPerStepGreedyWithinGate) {
  BenchResult r;
  // 2 truths, 2 tracks: track 1 next to truth 1, track 2 next to truth 2.
  r.steps.push_back(makeStep(0.0,
                             {{0, 0}, {100, 0}},
                             {{1, 0}, {101, 0}}));
  // Step 2: track 2 disappears (out of gate).
  r.steps.push_back(makeStep(1.0,
                             {{10, 0}, {110, 0}},
                             {{11, 0}, {999, 0}}));

  const auto assigns = benchmark::assignPerStep(r, /*gate=*/100.0);
  ASSERT_EQ(assigns.size(), 2u);
  ASSERT_EQ(assigns[0].size(), 2u);
  ASSERT_TRUE(assigns[0][0].has_value());
  ASSERT_TRUE(assigns[0][1].has_value());
  EXPECT_EQ(assigns[0][0]->value(), 1u);
  EXPECT_EQ(assigns[0][1]->value(), 2u);

  ASSERT_TRUE(assigns[1][0].has_value());
  EXPECT_FALSE(assigns[1][1].has_value());  // track 2 out of gate
}
```

- [ ] **Step 3: Run to confirm failure.**

- [ ] **Step 4: Implement greedy nearest-neighbour assignment**

In `Metrics.cpp`:
```cpp
#include <limits>
#include <unordered_set>

// ...

std::vector<StepAssignment> assignPerStep(const BenchResult& result,
                                          double gate_m) {
  std::vector<StepAssignment> out;
  out.reserve(result.steps.size());
  for (const auto& step : result.steps) {
    StepAssignment a(step.truth.size(), std::nullopt);
    std::unordered_set<std::uint64_t> claimed;
    for (std::size_t i = 0; i < step.truth.size(); ++i) {
      double best = gate_m;  // strict <; gate_m is the exclusive ceiling
      std::optional<TrackId> best_id;
      for (const auto& tr : step.tracks) {
        if (claimed.count(tr.id.value())) continue;
        const double d = (tr.position - step.truth[i].position).norm();
        if (d < best) {
          best = d;
          best_id = tr.id;
        }
      }
      if (best_id) {
        claimed.insert(best_id->value());
        a[i] = best_id;
      }
    }
    out.push_back(std::move(a));
  }
  return out;
}
```

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Metrics.hpp core/benchmark/Metrics.cpp tests/benchmark/test_metrics.cpp
git commit -m "Bench: per-step greedy assignment within gate"
```

---

## Task 7: Continuity & ID-switch metrics

Both consume `assignPerStep`. Computed once per truth track, then averaged across truths.

**Files:**
- Modify: `core/benchmark/Metrics.hpp`, `core/benchmark/Metrics.cpp`
- Test: extend `tests/benchmark/test_metrics.cpp`

- [ ] **Step 1: Declare in `Metrics.hpp`**

```cpp
struct ContinuityCounts {
  double lifetime_ratio;  // mean across truths in [0, 1]
  double track_breaks;    // mean count per truth
  double id_switches;     // mean count per truth
};
ContinuityCounts computeContinuity(const std::vector<StepAssignment>& assigns,
                                   std::size_t n_truths);
```

- [ ] **Step 2: Write the failing test**

```cpp
TEST(Metrics, ContinuityKnownPatterns) {
  // 1 truth, 6 steps, assignments: [1,1,_, _,1,2]
  std::vector<benchmark::StepAssignment> a;
  a.push_back({TrackId{1}});
  a.push_back({TrackId{1}});
  a.push_back({std::nullopt});
  a.push_back({std::nullopt});
  a.push_back({TrackId{1}});
  a.push_back({TrackId{2}});  // <- 1 id switch

  const auto c = benchmark::computeContinuity(a, /*n_truths=*/1);
  EXPECT_NEAR(c.lifetime_ratio, 4.0 / 6.0, 1e-9);
  EXPECT_NEAR(c.track_breaks, 1.0, 1e-9);  // one nullopt run
  EXPECT_NEAR(c.id_switches, 1.0, 1e-9);
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement**

```cpp
ContinuityCounts computeContinuity(const std::vector<StepAssignment>& assigns,
                                   std::size_t n_truths) {
  if (n_truths == 0 || assigns.empty()) return {0, 0, 0};
  std::vector<double> life(n_truths, 0.0);
  std::vector<double> breaks(n_truths, 0.0);
  std::vector<double> switches(n_truths, 0.0);
  std::vector<bool> in_gap(n_truths, true);
  std::vector<std::optional<TrackId>> prev(n_truths, std::nullopt);

  for (const auto& step : assigns) {
    for (std::size_t i = 0; i < n_truths && i < step.size(); ++i) {
      const auto& a = step[i];
      if (a.has_value()) {
        life[i] += 1.0;
        if (in_gap[i]) {
          // entering an assigned interval; only a break if we had been
          // assigned previously and dropped out. Track count of gap
          // intervals separately: increment break when we exit, not enter.
          in_gap[i] = false;
        }
        if (prev[i].has_value() && prev[i]->value() != a->value()) {
          switches[i] += 1.0;
        }
        prev[i] = a;
      } else {
        if (!in_gap[i]) {
          breaks[i] += 1.0;  // exited an assigned interval
          in_gap[i] = true;
        }
        prev[i] = std::nullopt;
      }
    }
  }

  ContinuityCounts c{};
  for (std::size_t i = 0; i < n_truths; ++i) {
    c.lifetime_ratio += life[i] / assigns.size();
    c.track_breaks += breaks[i];
    c.id_switches += switches[i];
  }
  c.lifetime_ratio /= n_truths;
  c.track_breaks /= n_truths;
  c.id_switches /= n_truths;
  return c;
}
```

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Metrics.hpp core/benchmark/Metrics.cpp tests/benchmark/test_metrics.cpp
git commit -m "Bench: continuity + id-switch metrics from shared assignment"
```

---

## Task 8: Per-track RMSE — position, SOG, COG

Uses the same `assignPerStep` output. Iterates assigned `(truth_idx, track_id)` pairs, computes position error, SOG error, and wrapped COG error. Aggregates per-truth mean-squared, then sqrt.

**Files:**
- Modify: `core/benchmark/Metrics.hpp`, `core/benchmark/Metrics.cpp`
- Test: extend `tests/benchmark/test_metrics.cpp`

- [ ] **Step 1: Declare in `Metrics.hpp`**

```cpp
struct RmseResult {
  double pos_rmse_m;
  double sog_rmse_mps;
  double cog_rmse_deg;
};

RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns);
```

- [ ] **Step 2: Write the failing test**

```cpp
TEST(Metrics, RmseLinearMotion) {
  BenchResult r;
  // 3 steps, truth at constant velocity (10, 0) m/s, no error in tracks.
  for (int k = 0; k < 3; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {10.0 * k, 0}, {10, 0}});
    s.tracks.push_back({TrackId{1}, {10.0 * k, 0}, {10, 0}});
    r.steps.push_back(s);
  }
  const auto a = benchmark::assignPerStep(r, 100.0);
  const auto rmse = benchmark::computeRmse(r, a);
  EXPECT_NEAR(rmse.pos_rmse_m, 0.0, 1e-9);
  EXPECT_NEAR(rmse.sog_rmse_mps, 0.0, 1e-9);
  EXPECT_NEAR(rmse.cog_rmse_deg, 0.0, 1e-9);
}

TEST(Metrics, RmseConstantOffsets) {
  BenchResult r;
  for (int k = 0; k < 4; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {0, 0}, {10, 0}});
    // 3m offset; SOG off by +1; COG rotated by 90 degrees (track velocity (0,11)).
    s.tracks.push_back({TrackId{1}, {3, 0}, {0, 11}});
    r.steps.push_back(s);
  }
  const auto a = benchmark::assignPerStep(r, 100.0);
  const auto rmse = benchmark::computeRmse(r, a);
  EXPECT_NEAR(rmse.pos_rmse_m, 3.0, 1e-6);
  EXPECT_NEAR(rmse.sog_rmse_mps, 1.0, 1e-6);
  EXPECT_NEAR(rmse.cog_rmse_deg, 90.0, 1e-6);
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement**

```cpp
namespace {
double wrapDeg(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}
double cogDeg(const Eigen::Vector2d& v) {
  // COG: angle measured clockwise from north (positive y).
  return wrapDeg(std::atan2(v.x(), v.y()) * 180.0 / M_PI);
}
}  // namespace

RmseResult computeRmse(const BenchResult& result,
                       const std::vector<StepAssignment>& assigns) {
  if (result.steps.empty() || assigns.empty()) return {0, 0, 0};
  const std::size_t n_truths = result.steps.front().truth.size();
  if (n_truths == 0) return {0, 0, 0};

  std::vector<double> pos_se(n_truths, 0.0);
  std::vector<double> sog_se(n_truths, 0.0);
  std::vector<double> cog_se(n_truths, 0.0);
  std::vector<std::size_t> n(n_truths, 0);

  for (std::size_t k = 0; k < result.steps.size(); ++k) {
    const auto& step = result.steps[k];
    const auto& assign = assigns[k];
    for (std::size_t i = 0; i < std::min(n_truths, assign.size()); ++i) {
      if (!assign[i].has_value()) continue;
      const TrackId tid = *assign[i];
      auto it = std::find_if(step.tracks.begin(), step.tracks.end(),
                             [&](const TrackStateSnapshot& s) {
                               return s.id.value() == tid.value();
                             });
      if (it == step.tracks.end()) continue;
      const Eigen::Vector2d dp = it->position - step.truth[i].position;
      pos_se[i] += dp.squaredNorm();
      const double ds = it->velocity.norm() - step.truth[i].velocity.norm();
      sog_se[i] += ds * ds;
      const double dc = wrapDeg(cogDeg(it->velocity) - cogDeg(step.truth[i].velocity));
      cog_se[i] += dc * dc;
      n[i] += 1;
    }
  }

  RmseResult out{0, 0, 0};
  std::size_t contributing = 0;
  for (std::size_t i = 0; i < n_truths; ++i) {
    if (n[i] == 0) continue;
    out.pos_rmse_m += std::sqrt(pos_se[i] / n[i]);
    out.sog_rmse_mps += std::sqrt(sog_se[i] / n[i]);
    out.cog_rmse_deg += std::sqrt(cog_se[i] / n[i]);
    ++contributing;
  }
  if (contributing == 0) return {0, 0, 0};
  out.pos_rmse_m /= contributing;
  out.sog_rmse_mps /= contributing;
  out.cog_rmse_deg /= contributing;
  return out;
}
```

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Metrics.hpp core/benchmark/Metrics.cpp tests/benchmark/test_metrics.cpp
git commit -m "Bench: per-track RMSE for position, SOG, COG"
```

---

## Task 9: `computeMetrics` — bundle all 8 metrics into `MetricsResult`

**Files:**
- Modify: `core/benchmark/Metrics.hpp`, `core/benchmark/Metrics.cpp`
- Test: extend `tests/benchmark/test_metrics.cpp`

- [ ] **Step 1: Declare**

```cpp
MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params);
```

- [ ] **Step 2: Test**

```cpp
TEST(Metrics, ComputeMetricsBundlesAll) {
  BenchResult r;
  for (int k = 0; k < 3; ++k) {
    BenchStep s;
    s.time = Timestamp::fromSeconds(k);
    s.truth.push_back({1, {10.0 * k, 0}, {10, 0}});
    s.tracks.push_back({TrackId{1}, {10.0 * k + 1, 0}, {10, 0}});
    r.steps.push_back(s);
  }
  const auto m = benchmark::computeMetrics(r, {});
  EXPECT_NEAR(m.ospa_mean, 1.0, 1e-6);
  EXPECT_NEAR(m.pos_rmse_m, 1.0, 1e-6);
  EXPECT_NEAR(m.lifetime_ratio, 1.0, 1e-9);
  EXPECT_NEAR(m.track_breaks, 0.0, 1e-9);
  EXPECT_NEAR(m.id_switches, 0.0, 1e-9);
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement**

```cpp
MetricsResult computeMetrics(const BenchResult& result,
                             const MetricsParams& params) {
  MetricsResult m{};
  const auto per_step = computeOspaPerStep(result, params.ospa_cutoff_m);
  m.ospa_mean = mean(per_step);
  m.ospa_p95 = percentile(per_step, 0.95);
  const auto assigns = assignPerStep(result, params.assoc_gate_m);
  const std::size_t n_truths =
      result.steps.empty() ? 0 : result.steps.front().truth.size();
  const auto cont = computeContinuity(assigns, n_truths);
  m.lifetime_ratio = cont.lifetime_ratio;
  m.track_breaks = cont.track_breaks;
  m.id_switches = cont.id_switches;
  const auto rmse = computeRmse(result, assigns);
  m.pos_rmse_m = rmse.pos_rmse_m;
  m.sog_rmse_mps = rmse.sog_rmse_mps;
  m.cog_rmse_deg = rmse.cog_rmse_deg;
  return m;
}
```

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Metrics.hpp core/benchmark/Metrics.cpp tests/benchmark/test_metrics.cpp
git commit -m "Bench: computeMetrics bundles all 8 metrics from one BenchResult"
```

---

## Task 10: `Config` + the 5 baseline configurations

**Files:**
- Create: `core/benchmark/Config.hpp`, `core/benchmark/Config.cpp`
- Test: `tests/benchmark/test_config.cpp`

The factory pattern: a `Config` owns label + two `std::function`s that *build* fresh tracker components per run (so multi-seed loops don't share mutable estimator state across runs).

- [ ] **Step 1: Define types in `Config.hpp`**

```cpp
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core/association/IDataAssociator.hpp"
#include "core/estimation/IEstimator.hpp"

namespace navtracker {
namespace benchmark {

using EstimatorFactory = std::function<std::shared_ptr<IEstimator>()>;
using AssociatorFactory = std::function<std::shared_ptr<IDataAssociator>()>;

struct Config {
  std::string label;
  EstimatorFactory build_estimator;
  AssociatorFactory build_associator;
};

// Returns the 5 baseline configs in fixed order:
//   ekf_cv_gnn, ekf_cv_jpda, ukf_cv_gnn, ukf_ct_gnn, imm_cv_ct_jpda
std::vector<Config> defaultConfigs();

}  // namespace benchmark
}  // namespace navtracker
```

**Note for implementer:** the actual port headers may be named differently (`ports/IEstimator.hpp` vs `core/estimation/...`). Check the existing scenario tests — they instantiate estimators via concrete classes (`EkfEstimator`, `UkfEstimator`, `ImmEstimator`) and associators (`GnnAssociator`, `JpdaAssociator`). Match what compiles.

- [ ] **Step 2: Write the failing test**

`tests/benchmark/test_config.cpp`:
```cpp
#include <gtest/gtest.h>

#include <set>

#include "core/benchmark/Config.hpp"

using namespace navtracker::benchmark;

TEST(Config, FiveDefaultConfigsWithUniqueLabels) {
  const auto configs = defaultConfigs();
  ASSERT_EQ(configs.size(), 5u);

  std::set<std::string> labels;
  for (const auto& c : configs) {
    labels.insert(c.label);
    EXPECT_NE(c.build_estimator, nullptr);
    EXPECT_NE(c.build_associator, nullptr);
  }
  EXPECT_EQ(labels.size(), 5u);
  EXPECT_EQ(labels.count("ekf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ekf_cv_jpda"), 1u);
  EXPECT_EQ(labels.count("ukf_cv_gnn"), 1u);
  EXPECT_EQ(labels.count("ukf_ct_gnn"), 1u);
  EXPECT_EQ(labels.count("imm_cv_ct_jpda"), 1u);
}

TEST(Config, FactoriesProduceUsableObjects) {
  for (const auto& c : defaultConfigs()) {
    auto est = c.build_estimator();
    auto asc = c.build_associator();
    EXPECT_NE(est, nullptr);
    EXPECT_NE(asc, nullptr);
  }
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement `Config.cpp`** — five factory entries. Reference an existing scenario test (e.g. `tests/scenario/test_crossing.cpp`) for the EKF+CV+GNN instantiation pattern, and other comparison tests for UKF/IMM/JPDA. The factories should return fresh instances each call.

Template:
```cpp
#include "core/benchmark/Config.hpp"

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/estimation/UkfEstimator.hpp"

namespace navtracker {
namespace benchmark {

std::vector<Config> defaultConfigs() {
  std::vector<Config> out;
  out.push_back({"ekf_cv_gnn",
                 [] {
                   auto motion = std::make_shared<ConstantVelocity2D>(0.1);
                   return std::make_shared<EkfEstimator>(motion, 5.0);
                 },
                 [] { return std::make_shared<GnnAssociator>(50.0); }});
  out.push_back({"ekf_cv_jpda",
                 [] {
                   auto motion = std::make_shared<ConstantVelocity2D>(0.1);
                   return std::make_shared<EkfEstimator>(motion, 5.0);
                 },
                 [] { return std::make_shared<JpdaAssociator>(/*params...*/); }});
  out.push_back({"ukf_cv_gnn",
                 [] {
                   auto motion = std::make_shared<ConstantVelocity2D>(0.1);
                   return std::make_shared<UkfEstimator>(motion, 5.0);
                 },
                 [] { return std::make_shared<GnnAssociator>(50.0); }});
  out.push_back({"ukf_ct_gnn",
                 [] {
                   auto motion = std::make_shared<CoordinatedTurn>(/*params*/);
                   return std::make_shared<UkfEstimator>(motion, 5.0);
                 },
                 [] { return std::make_shared<GnnAssociator>(50.0); }});
  out.push_back({"imm_cv_ct_jpda",
                 [] {
                   // ImmEstimator constructor takes a set of motion models +
                   // transition probability matrix. Check existing
                   // tests/estimation/test_imm_estimator.cpp for the actual
                   // ctor and replicate.
                   return std::make_shared<ImmEstimator>(/* ... */);
                 },
                 [] { return std::make_shared<JpdaAssociator>(/*params*/); }});
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
```

**Implementer:** Replace the `/* params */` placeholders by inspecting the existing tests for each estimator/associator combination. JPDA constructor args, IMM transition matrix, and CoordinatedTurn process-noise are all already chosen in existing comparison tests — copy those values so the baseline reflects what's currently considered "reasonable defaults" in this repo.

- [ ] **Step 5: Build, run tests, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Config.hpp core/benchmark/Config.cpp tests/benchmark/test_config.cpp CMakeLists.txt
git commit -m "Bench: 5 default configurations behind factory pattern"
```

---

## Task 11: `SimScenarioRun` adapter — 7 synthetic scenarios

**Files:**
- Create: `adapters/benchmark/SimScenarioRun.hpp`, `adapters/benchmark/SimScenarioRun.cpp`
- Modify: `CMakeLists.txt` (new `navtracker_benchmark_adapters` library)
- Test: `tests/benchmark/test_sim_scenario_run.cpp`

Each of the 7 synthetic scenarios from the spec gets a small `SimScenarioRun` subclass that wraps the appropriate `Builders.hpp` call and threads the seed through.

- [ ] **Step 1: Header**

`adapters/benchmark/SimScenarioRun.hpp`:
```cpp
#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

// Returns the 7 synthetic baseline scenarios. Each is multi-seed with
// seed_count = 10.
std::vector<std::unique_ptr<ScenarioRun>> defaultSimScenarios();

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Test**

```cpp
#include <gtest/gtest.h>

#include "adapters/benchmark/SimScenarioRun.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

TEST(SimScenarioRun, ProducesSevenScenariosWithExpectedLabels) {
  const auto scenarios = defaultSimScenarios();
  ASSERT_EQ(scenarios.size(), 7u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) labels.insert(s->descriptor().label);
  EXPECT_EQ(labels.count("crossing"), 1u);
  EXPECT_EQ(labels.count("overtaking"), 1u);
  EXPECT_EQ(labels.count("head_on"), 1u);
  EXPECT_EQ(labels.count("parallel_targets"), 1u);
  EXPECT_EQ(labels.count("ais_dropout"), 1u);
  EXPECT_EQ(labels.count("clock_skew"), 1u);
  EXPECT_EQ(labels.count("non_cooperative"), 1u);
}

TEST(SimScenarioRun, GenerateIsDeterministicForSameSeed) {
  const auto scenarios = defaultSimScenarios();
  const auto a = scenarios[0]->generate(0);
  const auto b = scenarios[0]->generate(0);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].time, b.measurements[i].time);
  }
}
```

- [ ] **Step 3: Run to confirm failure.**

- [ ] **Step 4: Implement `SimScenarioRun.cpp`**

For each of the 7 scenarios, define a `class XxxScenarioRun : public ScenarioRun` that returns the canonical descriptor and calls the matching `Builders.hpp` function with that seed. Example for crossing:

```cpp
class CrossingScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"crossing", true, 10};
  }
  Scenario generate(std::uint64_t seed) override {
    std::vector<double> times;
    for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
    return buildCrossingTargetsScenario(
        Eigen::Vector2d(-500.0, 10.0),
        Eigen::Vector2d(25.0, 0.0),
        Eigen::Vector2d(500.0, -10.0),
        Eigen::Vector2d(-25.0, 0.0),
        times, /*pos_noise_std_m=*/8.0,
        static_cast<std::uint32_t>(seed));
  }
};
```

Replicate for the remaining six. For scenarios without an exact `Builders.hpp` entry (`head_on`, `clock_skew`, `non_cooperative`, `ais_dropout`), look for the closest existing builder or compose from primitives. Specifically:
- `head_on` — use `buildCrossingTargetsScenario` with anti-parallel velocities, near-zero lateral offset.
- `ais_dropout` — use `buildCrossingDropoutScenario`.
- `clock_skew` — use `buildStraightLineScenario` and post-process via `sim/SkewInjector.cpp` to inject skew on a subset of timestamps.
- `non_cooperative` — use `buildBearingOnlyScenario` or `buildRangeBearingPassScenario`.

`defaultSimScenarios()` constructs one of each and returns them in fixed order.

- [ ] **Step 5: Wire `navtracker_benchmark_adapters` library into `CMakeLists.txt`**

```cmake
add_library(navtracker_benchmark_adapters
  adapters/benchmark/SimScenarioRun.cpp
)
target_include_directories(navtracker_benchmark_adapters PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark_adapters
  PUBLIC navtracker_benchmark navtracker_core navtracker_sim Eigen3::Eigen)
```

Add test source + link `navtracker_benchmark_adapters` into `navtracker_tests`.

- [ ] **Step 6: Build, run tests, expect PASS.**

- [ ] **Step 7: Commit**

```bash
git add adapters/benchmark/SimScenarioRun.hpp adapters/benchmark/SimScenarioRun.cpp \
        tests/benchmark/test_sim_scenario_run.cpp CMakeLists.txt
git commit -m "Bench: SimScenarioRun adapters for 7 synthetic scenarios"
```

---

## Task 12: `ReplayScenarioRun` adapter — philos + haxr

**Files:**
- Create: `adapters/benchmark/ReplayScenarioRun.hpp`, `adapters/benchmark/ReplayScenarioRun.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/benchmark/test_replay_scenario_run.cpp`

The replays produce a `Scenario` whose `truth` and `measurements` come from the loaders already used by `tests/replay/test_philos_ospa.cpp` and `test_haxr_ospa.cpp`. Open both tests and lift the loader sequences.

- [ ] **Step 1: Header**

```cpp
#pragma once

#include <memory>
#include <vector>

#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

// Two replays, single-seed (file-driven).
std::vector<std::unique_ptr<ScenarioRun>> defaultReplayScenarios();

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Test**

```cpp
#include <gtest/gtest.h>

#include "adapters/benchmark/ReplayScenarioRun.hpp"

using namespace navtracker::benchmark;

TEST(ReplayScenarioRun, TwoReplaysWithExpectedLabels) {
  const auto scenarios = defaultReplayScenarios();
  ASSERT_EQ(scenarios.size(), 2u);
  std::set<std::string> labels;
  for (const auto& s : scenarios) {
    labels.insert(s->descriptor().label);
    EXPECT_FALSE(s->descriptor().is_multi_seed);
    EXPECT_EQ(s->descriptor().seed_count, 1u);
  }
  EXPECT_EQ(labels.count("philos"), 1u);
  EXPECT_EQ(labels.count("haxr"), 1u);
}

TEST(ReplayScenarioRun, GenerateReturnsNonEmpty) {
  for (auto& s : defaultReplayScenarios()) {
    const auto data = s->generate(0);
    EXPECT_FALSE(data.measurements.empty())
        << "replay " << s->descriptor().label << " produced no measurements";
  }
}
```

- [ ] **Step 3: Run to confirm failure.**

- [ ] **Step 4: Implement** — one subclass per replay. Pull the loader calls from `tests/replay/test_philos_ospa.cpp` and `test_haxr_ospa.cpp`. Each `generate()` reloads the fixture from disk (the fixture path is constant; the test files name it). Link the replay library: `navtracker_replay`.

- [ ] **Step 5: Add `ReplayScenarioRun.cpp` to `navtracker_benchmark_adapters` sources. Add `navtracker_replay` to its link line. Add test, build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add adapters/benchmark/ReplayScenarioRun.hpp adapters/benchmark/ReplayScenarioRun.cpp \
        tests/benchmark/test_replay_scenario_run.cpp CMakeLists.txt
git commit -m "Bench: ReplayScenarioRun adapters for philos + haxr"
```

---

## Task 13: `Sweep` — the matrix loop

**Files:**
- Create: `core/benchmark/Sweep.hpp`, `core/benchmark/Sweep.cpp`
- Test: `tests/benchmark/test_sweep.cpp`

- [ ] **Step 1: Header**

```cpp
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/benchmark/Config.hpp"
#include "core/benchmark/Metrics.hpp"
#include "core/benchmark/ScenarioRun.hpp"

namespace navtracker {
namespace benchmark {

struct MetricRow {
  std::string run_id;
  std::string config;
  std::string scenario;
  std::uint64_t seed;
  std::string metric;
  double value;
  std::string unit;
};

struct SweepParams {
  std::string run_id;
  MetricsParams metrics;
  std::uint32_t synthetic_seeds{10};
  std::uint32_t track_manager_min_misses{2};
  std::uint32_t track_manager_max_misses{4};
  double tracker_init_gate_m{30.0};
};

std::vector<MetricRow> runSweep(
    const std::vector<Config>& configs,
    const std::vector<std::unique_ptr<ScenarioRun>>& scenarios,
    const SweepParams& params);

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Test**

```cpp
#include <gtest/gtest.h>

#include "core/benchmark/Sweep.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/scenario/Builders.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
class TinyStraightLine : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"tiny_line", true, 2};
  }
  Scenario generate(std::uint64_t seed) override {
    std::vector<double> times;
    for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
    return buildStraightLineScenario(
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(10, 0),
        times, 1.0,
        static_cast<std::uint32_t>(seed),
        1);
  }
};
}  // namespace

TEST(Sweep, RowCountMatchesMatrix) {
  std::vector<Config> configs = {defaultConfigs()[0]};  // ekf_cv_gnn
  std::vector<std::unique_ptr<ScenarioRun>> scenarios;
  scenarios.push_back(std::make_unique<TinyStraightLine>());

  SweepParams p;
  p.run_id = "test_run";
  p.synthetic_seeds = 2;

  const auto rows = runSweep(configs, scenarios, p);
  // 1 config * 1 scenario * 2 seeds * 8 metrics = 16 rows
  EXPECT_EQ(rows.size(), 16u);
  for (const auto& r : rows) {
    EXPECT_EQ(r.run_id, "test_run");
    EXPECT_EQ(r.config, "ekf_cv_gnn");
    EXPECT_EQ(r.scenario, "tiny_line");
  }
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement `Sweep.cpp`**

```cpp
#include "core/benchmark/Sweep.hpp"

#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/BenchSink.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {
namespace benchmark {

namespace {
void emit(std::vector<MetricRow>& out,
          const SweepParams& p,
          const std::string& config,
          const std::string& scenario,
          std::uint64_t seed,
          const MetricsResult& m) {
  out.push_back({p.run_id, config, scenario, seed, "ospa_mean", m.ospa_mean, "m"});
  out.push_back({p.run_id, config, scenario, seed, "ospa_p95", m.ospa_p95, "m"});
  out.push_back({p.run_id, config, scenario, seed, "lifetime_ratio", m.lifetime_ratio, "ratio"});
  out.push_back({p.run_id, config, scenario, seed, "track_breaks", m.track_breaks, "count"});
  out.push_back({p.run_id, config, scenario, seed, "id_switches", m.id_switches, "count"});
  out.push_back({p.run_id, config, scenario, seed, "pos_rmse_m", m.pos_rmse_m, "m"});
  out.push_back({p.run_id, config, scenario, seed, "sog_rmse_mps", m.sog_rmse_mps, "m/s"});
  out.push_back({p.run_id, config, scenario, seed, "cog_rmse_deg", m.cog_rmse_deg, "deg"});
}
}  // namespace

std::vector<MetricRow> runSweep(
    const std::vector<Config>& configs,
    const std::vector<std::unique_ptr<ScenarioRun>>& scenarios,
    const SweepParams& params) {
  std::vector<MetricRow> rows;
  for (const auto& config : configs) {
    for (const auto& scenario_ptr : scenarios) {
      const auto desc = scenario_ptr->descriptor();
      const std::uint32_t seeds =
          desc.is_multi_seed ? params.synthetic_seeds : 1u;
      for (std::uint32_t seed = 0; seed < seeds; ++seed) {
        const Scenario scen = scenario_ptr->generate(seed);
        auto est = config.build_estimator();
        auto asc = config.build_associator();
        TrackManager mgr(params.track_manager_min_misses,
                         params.track_manager_max_misses);
        Tracker tracker(*est, *asc, mgr, params.tracker_init_gate_m);
        BenchSink sink;
        const auto result = runBench(scen, tracker, mgr, sink);
        const auto m = computeMetrics(result, params.metrics);
        emit(rows, params, config.label, desc.label, seed, m);
      }
    }
  }
  return rows;
}

}  // namespace benchmark
}  // namespace navtracker
```

**Note:** `Tracker` and `TrackManager` constructor signatures vary across the codebase; cross-check `tests/scenario/test_crossing.cpp` for the canonical instantiation pattern and adapt.

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/Sweep.hpp core/benchmark/Sweep.cpp tests/benchmark/test_sweep.cpp CMakeLists.txt
git commit -m "Bench: matrix sweep over (config x scenario x seed)"
```

---

## Task 14: `CsvWriter` — long-format CSV with header block

**Files:**
- Create: `core/benchmark/CsvWriter.hpp`, `core/benchmark/CsvWriter.cpp`
- Test: `tests/benchmark/test_csv_writer.cpp`

- [ ] **Step 1: Header**

```cpp
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

struct CsvProvenance {
  std::string run_id;
  std::string started_at_utc;   // ISO 8601, e.g. "2026-06-07T10:14:22Z"
  std::string git_sha;          // includes "(clean)" or "(dirty)" suffix
  std::string build_type;       // "Release" / "Debug" / ...
  std::string compiler;         // "gcc 13.2.0"
  std::string host;             // "linux x86_64"
  std::vector<std::uint32_t> seeds;
  std::uint32_t config_count{0};
  std::uint32_t scenario_count{0};
  std::uint32_t total_runs{0};
  double elapsed_seconds{0.0};
};

void writeCsv(std::ostream& os,
              const CsvProvenance& prov,
              const std::vector<MetricRow>& rows);

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Test**

```cpp
#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/CsvWriter.hpp"

using namespace navtracker::benchmark;

TEST(CsvWriter, EmitsHeaderBlockAndRows) {
  CsvProvenance p;
  p.run_id = "2026-06-07_baseline";
  p.started_at_utc = "2026-06-07T10:14:22Z";
  p.git_sha = "abc1234 (clean)";
  p.build_type = "Release";
  p.compiler = "gcc 13.2.0";
  p.host = "linux x86_64";
  p.seeds = {0, 1};
  p.config_count = 1;
  p.scenario_count = 1;
  p.total_runs = 2;
  p.elapsed_seconds = 1.5;

  std::vector<MetricRow> rows = {
      {"2026-06-07_baseline", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 73.42, "m"},
  };

  std::ostringstream os;
  writeCsv(os, p, rows);
  const auto s = os.str();
  EXPECT_NE(s.find("# run_id: 2026-06-07_baseline"), std::string::npos);
  EXPECT_NE(s.find("# git_sha: abc1234 (clean)"), std::string::npos);
  EXPECT_NE(s.find("run_id,config,scenario,seed,metric,value,unit"), std::string::npos);
  EXPECT_NE(s.find("2026-06-07_baseline,ekf_cv_gnn,crossing,0,ospa_mean,73.42,m"),
            std::string::npos);
}
```

- [ ] **Step 3: Run to verify failure.**

- [ ] **Step 4: Implement**

```cpp
#include "core/benchmark/CsvWriter.hpp"

#include <ostream>
#include <sstream>

namespace navtracker {
namespace benchmark {

namespace {
std::string seedListJson(const std::vector<std::uint32_t>& v) {
  std::ostringstream os;
  os << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) os << ',';
    os << v[i];
  }
  os << ']';
  return os.str();
}
}  // namespace

void writeCsv(std::ostream& os,
              const CsvProvenance& p,
              const std::vector<MetricRow>& rows) {
  os << "# run_id: " << p.run_id << "\n"
     << "# started_at: " << p.started_at_utc << "\n"
     << "# git_sha: " << p.git_sha << "\n"
     << "# build_type: " << p.build_type << "\n"
     << "# compiler: " << p.compiler << "\n"
     << "# host: " << p.host << "\n"
     << "# seeds: " << seedListJson(p.seeds) << "\n"
     << "# configs: " << p.config_count << "\n"
     << "# scenarios: " << p.scenario_count << "\n"
     << "# total_runs: " << p.total_runs << "\n"
     << "# elapsed_seconds: " << p.elapsed_seconds << "\n";
  os << "run_id,config,scenario,seed,metric,value,unit\n";
  for (const auto& r : rows) {
    os << r.run_id << ',' << r.config << ',' << r.scenario << ','
       << r.seed << ',' << r.metric << ',' << r.value << ',' << r.unit << '\n';
  }
}

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 5: Build, run, expect PASS.**

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/CsvWriter.hpp core/benchmark/CsvWriter.cpp tests/benchmark/test_csv_writer.cpp CMakeLists.txt
git commit -m "Bench: long-format CSV writer with provenance header"
```

---

## Task 15: `bench/baseline_matrix` executable

**Files:**
- Create: `bench/baseline_matrix.cpp`, `bench/CMakeLists.txt`
- Modify: top-level `CMakeLists.txt` (`add_subdirectory(bench)`)

- [ ] **Step 1: Implement `bench/baseline_matrix.cpp`**

```cpp
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include "adapters/benchmark/ReplayScenarioRun.hpp"
#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
std::string nowUtcIso8601() {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::gmtime(&t);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string argv_str(int argc, char** argv, const std::string& flag) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) return argv[i + 1];
  }
  return {};
}
}  // namespace

int main(int argc, char** argv) {
  const std::string run_id =
      argv_str(argc, argv, "--run-id").empty()
          ? std::string("run_") + nowUtcIso8601()
          : argv_str(argc, argv, "--run-id");
  const std::string out_dir =
      argv_str(argc, argv, "--out").empty() ? std::string("docs/baselines/")
                                            : argv_str(argc, argv, "--out");

  const auto t0 = std::chrono::steady_clock::now();

  auto configs = defaultConfigs();
  auto sim_scenarios = defaultSimScenarios();
  auto replay_scenarios = defaultReplayScenarios();

  std::vector<std::unique_ptr<ScenarioRun>> all;
  for (auto& s : sim_scenarios) all.push_back(std::move(s));
  for (auto& s : replay_scenarios) all.push_back(std::move(s));

  SweepParams sp;
  sp.run_id = run_id;
  const auto rows = runSweep(configs, all, sp);

  const auto t1 = std::chrono::steady_clock::now();

  CsvProvenance prov;
  prov.run_id = run_id;
  prov.started_at_utc = nowUtcIso8601();
  // The CMake build can stamp the git SHA via a generated header; for
  // simplicity, the implementer should add an option to inject git SHA
  // and build metadata. For first cut, leave as "unknown".
  prov.git_sha = "unknown";
  prov.build_type = "Release";
  prov.compiler = "unknown";
  prov.host = "unknown";
  for (std::uint32_t s = 0; s < sp.synthetic_seeds; ++s) prov.seeds.push_back(s);
  prov.config_count = configs.size();
  prov.scenario_count = all.size();
  prov.total_runs =
      configs.size() *
      (sim_scenarios.size() * sp.synthetic_seeds + replay_scenarios.size());
  prov.elapsed_seconds =
      std::chrono::duration<double>(t1 - t0).count();

  const std::string out_path = out_dir + run_id + ".csv";
  std::ofstream os(out_path);
  if (!os) {
    std::cerr << "Cannot open output file: " << out_path << "\n";
    return 1;
  }
  writeCsv(os, prov, rows);
  std::cout << "Wrote " << rows.size() << " rows to " << out_path << "\n";
  return 0;
}
```

- [ ] **Step 2: `bench/CMakeLists.txt`**

```cmake
add_executable(navtracker_bench_baseline baseline_matrix.cpp)
target_link_libraries(navtracker_bench_baseline
  PRIVATE navtracker_benchmark navtracker_benchmark_adapters)
```

- [ ] **Step 3: Add to top-level `CMakeLists.txt`**

After all `add_library` calls, add `add_subdirectory(bench)`.

- [ ] **Step 4: Build and run with a tiny config (quick smoke)**

```bash
cmake --build build --target navtracker_bench_baseline -j
./build/bench/navtracker_bench_baseline --run-id smoke --out /tmp/
head -20 /tmp/smoke.csv
```
Expected: CSV emitted; first lines are the `#`-prefixed header block followed by `run_id,config,scenario,...`.

- [ ] **Step 5: Commit**

```bash
git add bench/baseline_matrix.cpp bench/CMakeLists.txt CMakeLists.txt
git commit -m "Bench: baseline_matrix executable wires sweep + CSV writer"
```

---

## Task 16: `bench/render_markdown` executable

Reads a CSV, groups rows by scenario, emits a Markdown file with one table per scenario. Cells are `mean ± stddev` across seeds.

**Files:**
- Create: `bench/render_markdown.cpp`
- Modify: `bench/CMakeLists.txt`
- Test: `tests/benchmark/test_render_markdown.cpp` (calls a library function)

To keep the executable tiny and testable, factor out the renderer into a `core/benchmark/MarkdownRenderer.hpp/.cpp` library function and have `render_markdown.cpp` be a 10-line shell around it.

- [ ] **Step 1: Library function in `core/benchmark/MarkdownRenderer.hpp`**

```cpp
#pragma once

#include <iosfwd>
#include <vector>

#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

// Reads MetricRows + provenance, writes one section per scenario,
// each section a table with rows = configs, columns = metrics. Cells
// are "mean ± stddev" aggregated across seeds.
void renderMarkdown(std::ostream& os,
                    const CsvProvenance& prov,
                    const std::vector<MetricRow>& rows);

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Failing test**

```cpp
#include <gtest/gtest.h>

#include <sstream>

#include "core/benchmark/MarkdownRenderer.hpp"

using namespace navtracker::benchmark;

TEST(MarkdownRenderer, EmitsScenarioSections) {
  CsvProvenance p;
  p.run_id = "test";
  std::vector<MetricRow> rows = {
      {"test", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 10.0, "m"},
      {"test", "ekf_cv_gnn", "crossing", 1, "ospa_mean", 12.0, "m"},
      {"test", "ekf_cv_gnn", "overtaking", 0, "ospa_mean", 5.0, "m"},
  };
  std::ostringstream os;
  renderMarkdown(os, p, rows);
  const auto s = os.str();
  EXPECT_NE(s.find("## crossing"), std::string::npos);
  EXPECT_NE(s.find("## overtaking"), std::string::npos);
  EXPECT_NE(s.find("ekf_cv_gnn"), std::string::npos);
  EXPECT_NE(s.find("11"), std::string::npos);  // mean of 10 and 12
}
```

- [ ] **Step 3: Implement** — straightforward: group by `(scenario, config, metric)`, compute mean & sample stddev across seeds, emit table per scenario. Include the provenance block at the top.

- [ ] **Step 4: Add `bench/render_markdown.cpp`**

```cpp
#include <fstream>
#include <iostream>

#include "core/benchmark/MarkdownRenderer.hpp"
// Reuse a small CSV parser — implement inline in render_markdown.cpp or
// add core/benchmark/CsvReader.hpp/.cpp if reused for compare too.
// (Adding CsvReader is recommended; it'll be reused in Task 17.)

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: render_markdown <input.csv> [output.md]\n";
    return 1;
  }
  // ... read provenance + rows from argv[1], compute output path, render
  // and write.
}
```

Also create `core/benchmark/CsvReader.hpp/.cpp` (mirror of `CsvWriter`): parses the `#`-prefixed header block into `CsvProvenance`, then reads rows into `std::vector<MetricRow>`. Write a small unit test for round-tripping `writeCsv` → `readCsv`.

- [ ] **Step 5: Build, run on a CSV produced in Task 15, verify the `.md` has scenario sections.**

```bash
cmake --build build --target navtracker_bench_render -j
./build/bench/navtracker_bench_render /tmp/smoke.csv
ls /tmp/smoke.md && head -40 /tmp/smoke.md
```

- [ ] **Step 6: Commit**

```bash
git add core/benchmark/MarkdownRenderer.hpp core/benchmark/MarkdownRenderer.cpp \
        core/benchmark/CsvReader.hpp core/benchmark/CsvReader.cpp \
        bench/render_markdown.cpp bench/CMakeLists.txt \
        tests/benchmark/test_render_markdown.cpp tests/benchmark/test_csv_reader.cpp \
        CMakeLists.txt
git commit -m "Bench: Markdown renderer + CSV reader"
```

---

## Task 17: `bench/compare` executable

Reads N CSVs, joins on `(config, scenario, metric)`, emits a Markdown diff where cells become `baseline → new (±Δ)` with `▲` / `▼` indicators.

**Files:**
- Create: `bench/compare.cpp`, `core/benchmark/Comparator.hpp/.cpp`
- Modify: `bench/CMakeLists.txt`
- Test: `tests/benchmark/test_compare.cpp`

- [ ] **Step 1: Library function**

```cpp
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "core/benchmark/CsvWriter.hpp"
#include "core/benchmark/Sweep.hpp"

namespace navtracker {
namespace benchmark {

struct ComparisonInput {
  CsvProvenance prov;
  std::vector<MetricRow> rows;
};

// Sign convention: for "lower-is-better" metrics (ospa_mean, ospa_p95,
// pos_rmse_m, sog_rmse_mps, cog_rmse_deg, track_breaks, id_switches), a
// negative delta is an improvement (▲); for "higher-is-better"
// (lifetime_ratio), a positive delta is improvement.
void renderComparison(std::ostream& os,
                      const std::vector<ComparisonInput>& inputs);

}  // namespace benchmark
}  // namespace navtracker
```

- [ ] **Step 2: Failing test**

```cpp
TEST(Comparator, MarksImprovementsAndRegressions) {
  // baseline ospa_mean = 100; improvement = 80 (lower is better -> ▲)
  ComparisonInput a{}, b{};
  a.prov.run_id = "baseline";
  b.prov.run_id = "improv";
  a.rows = {{"baseline", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 100.0, "m"}};
  b.rows = {{"improv", "ekf_cv_gnn", "crossing", 0, "ospa_mean", 80.0, "m"}};

  std::ostringstream os;
  renderComparison(os, {a, b});
  const std::string s = os.str();
  EXPECT_NE(s.find("▲"), std::string::npos);
  EXPECT_NE(s.find("100"), std::string::npos);
  EXPECT_NE(s.find("80"), std::string::npos);
}
```

- [ ] **Step 3: Implement** — straightforward join + cell formatting. The improvement-direction map is hardcoded by metric name.

- [ ] **Step 4: `bench/compare.cpp`** — small wrapper that loads N CSVs and calls `renderComparison`.

- [ ] **Step 5: Build, test, commit**

```bash
git add core/benchmark/Comparator.hpp core/benchmark/Comparator.cpp \
        bench/compare.cpp bench/CMakeLists.txt tests/benchmark/test_compare.cpp \
        CMakeLists.txt
git commit -m "Bench: compare executable joins N runs into one diff table"
```

---

## Task 18: Determinism integration test

**Files:**
- Create: `tests/benchmark/test_bench_determinism.cpp`

- [ ] **Step 1: Test**

```cpp
#include <gtest/gtest.h>

#include <functional>
#include <sstream>

#include "adapters/benchmark/SimScenarioRun.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"

using namespace navtracker::benchmark;

namespace {
std::size_t hashRows(const std::vector<MetricRow>& rows) {
  std::ostringstream os;
  for (const auto& r : rows) {
    os << r.run_id << ',' << r.config << ',' << r.scenario << ','
       << r.seed << ',' << r.metric << ',' << r.value << ',' << r.unit << '\n';
  }
  return std::hash<std::string>{}(os.str());
}
}  // namespace

TEST(BenchDeterminism, RepeatedSweepProducesIdenticalRows) {
  auto configs = defaultConfigs();
  auto scenarios = defaultSimScenarios();
  SweepParams p;
  p.run_id = "det";
  p.synthetic_seeds = 2;
  // Trim to first scenario + first config to keep the test fast.
  std::vector<Config> c1 = {configs.front()};
  std::vector<std::unique_ptr<ScenarioRun>> s1;
  s1.push_back(std::move(scenarios.front()));

  const auto rows1 = runSweep(c1, s1, p);
  // Rebuild fresh scenarios since the vector was moved-from.
  auto fresh = defaultSimScenarios();
  std::vector<std::unique_ptr<ScenarioRun>> s2;
  s2.push_back(std::move(fresh.front()));
  const auto rows2 = runSweep(c1, s2, p);

  EXPECT_EQ(hashRows(rows1), hashRows(rows2));
}
```

- [ ] **Step 2: Add to `navtracker_tests`, build, run, expect PASS.**

If it fails, the divergence is a determinism bug: either an estimator with un-seeded RNG, an associator with un-seeded sampling, or a `TrackManager` that depends on iteration order of an unordered container. Fix the root cause, don't paper over.

- [ ] **Step 3: Commit**

```bash
git add tests/benchmark/test_bench_determinism.cpp CMakeLists.txt
git commit -m "Bench: determinism test — repeated sweep is bit-identical"
```

---

## Task 19: `docs/baselines/README.md`

**Files:**
- Create: `docs/baselines/README.md`

- [ ] **Step 1: Content**

```markdown
# Baseline benchmarks

Each `*.csv` file in this directory is a single execution of the
benchmark harness; the matching `*.md` is its rendered companion;
any `*_vs_*.md` is a comparison between two or more runs.

## How to run a baseline

```bash
cmake --build build --target navtracker_bench_baseline -j
./build/bench/navtracker_bench_baseline \
    --run-id $(date +%Y-%m-%d)_baseline \
    --out docs/baselines/

./build/bench/navtracker_bench_render \
    docs/baselines/$(date +%Y-%m-%d)_baseline.csv

git add docs/baselines/$(date +%Y-%m-%d)_baseline.{csv,md}
git commit -m "Baseline benchmark: $(date +%Y-%m-%d)"
```

## How to compare runs

```bash
./build/bench/navtracker_bench_compare \
    docs/baselines/2026-06-07_baseline.csv \
    docs/baselines/2026-06-08_improv_a.csv \
    > docs/baselines/improv_a_vs_baseline.md
```

## What's in a CSV

One row per `(run_id, config, scenario, seed, metric)`. The first
lines are `#`-prefixed provenance (git SHA, build type, host, seeds,
total runs, elapsed). See `docs/superpowers/specs/2026-06-07-baseline-benchmark-harness-design.md`
for the full schema and the meaning of every metric.

## Configurations

- `ekf_cv_gnn` — EKF + ConstantVelocity2D + GNN (textbook baseline)
- `ekf_cv_jpda` — EKF + ConstantVelocity2D + JPDA (association upgrade)
- `ukf_cv_gnn` — UKF + ConstantVelocity2D + GNN (estimator upgrade)
- `ukf_ct_gnn` — UKF + CoordinatedTurn + GNN (non-linear motion model)
- `imm_cv_ct_jpda` — IMM(CV+CT) + JPDA (most expressive)

## Scenarios

Synthetic (10 seeds each): `crossing`, `overtaking`, `head_on`,
`parallel_targets`, `ais_dropout`, `clock_skew`, `non_cooperative`.

Replay (single seed): `philos`, `haxr`.

## Metrics

- `ospa_mean` / `ospa_p95` (m) — accuracy vs truth
- `lifetime_ratio` (ratio) — fraction of truth steps with an assigned track
- `track_breaks` (count) — mean number of gaps per truth track
- `id_switches` (count) — mean number of ID changes per truth track
- `pos_rmse_m` / `sog_rmse_mps` / `cog_rmse_deg` — decomposed state error

Full definitions and assumptions: see the design spec.
```

- [ ] **Step 2: Commit**

```bash
git add docs/baselines/README.md
git commit -m "Bench: docs/baselines README — how to run, what's measured"
```

---

## Self-review (run after writing all tasks)

- [x] **Spec coverage.**
  - 5 named configs → Task 10. ✓
  - 7 synthetic + 2 replay scenarios → Tasks 11, 12. ✓
  - OSPA aggregates → Task 5. ✓
  - Continuity + ID switches → Tasks 6, 7. ✓
  - Per-track RMSE → Task 8. ✓
  - CSV long format with `#` header block → Task 14. ✓
  - Markdown rendering → Task 16. ✓
  - Comparison tool → Task 17. ✓
  - Determinism check → Task 18. ✓
  - `docs/baselines/README.md` → Task 19. ✓
  - Hexagonal: `core/benchmark/` no I/O → Tasks 1–10, 13–14, 16, 17 all in `core/`. ✓
  - Existing tests untouched → no task modifies `tests/scenario/` or `tests/replay/`. ✓
- [x] **Placeholder scan.** A handful of "implementer: cross-check existing tests" notes are intentional pointers, not placeholders — they reference specific files. No "TBD/TODO" left.
- [x] **Type consistency.** `MetricRow` used identically in Tasks 13, 14, 16, 17. `BenchResult` / `BenchStep` consistent in Tasks 4, 5, 8, 9. `Config`, `ScenarioRun`, `ScenarioDescriptor`, `SweepParams`, `MetricsResult` consistent across uses.

Done. The plan is ready for execution.
