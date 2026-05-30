# Pipeline & Time Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire the existing pieces into a working tracker — a time-ordered reorder buffer plus an orchestrator that, for each released `Measurement`, predicts all tracks, runs association, updates the matched track or initiates a new one, and ages out tracks via a miss-timeout policy. Plus deterministic replay.

**Architecture:** Adds `core/pipeline/ReorderBuffer` and `core/pipeline/Tracker` (pure domain, no I/O). Tracker depends on `IEstimator`, `IDataAssociator`, `TrackManager` — only via their abstractions. Adds two edge ports: `ISensorAdapter` and `ITrackSink`. Extends `IEstimator` with `initiate` (already implemented in `EkfEstimator`) so the orchestrator can stay generic, and extends `TrackManager` with a `last_observation` timestamp per track so we can age tracks out by silence, not by per-message "misses."

**Tech Stack:** C++17 · Eigen 3.4 · GoogleTest. Builds on plans 1–3 (on `master`).

This is plan 4 of 6. Prereq: plans 1–3 merged — 36 tests pass.

**Documentation standard (CLAUDE.md):** Each algorithm carries Math / Assumptions / Rationale / Ways-to-improve. Concise notes per task here; Task 5 writes `docs/algorithms/pipeline.md`.

---

## Design notes

- **Single-measurement orchestration.** Measurements arrive asynchronously at very different rates (AIS ~10s, radar ~2s, camera ~10Hz). We process one at a time: predict all tracks to `z.time`, associate `{z}` against active tracks, update the matched track or initiate a new one.
- **Miss policy = timeout, not per-message.** Marking every non-matched track as "missed" on every single measurement would shred healthy tracks. Instead, each track carries `last_observation` (the last time it actually absorbed a measurement). On each `process`, tracks whose `(z.time − last_observation) > miss_timeout` get one `recordMiss`. This naturally produces Coasting and eventual Deletion when a target stops being observed.
- **Reorder buffer.** Holds incoming measurements for a bounded "reorder window" so late/out-of-order arrivals can still be released in time order. Anything arriving more than the window behind the latest observed time is dropped and counted (spec D4). The engine advances on these released timestamps — same code path for live and replay.
- **Sensor-ID hint association** remains deferred (per plan 3).

## File Structure

```
core/pipeline/ReorderBuffer.hpp/.cpp     time-ordered buffer with bounded reorder window
core/pipeline/Tracker.hpp/.cpp           orchestrator: predict, associate, update/initiate, maintain
ports/ISensorAdapter.hpp                 edge port: drain measurements from a source
ports/ITrackSink.hpp                     edge port: receive track-set updates
docs/algorithms/pipeline.md              math/assumptions/rationale/improve
tests/pipeline/test_reorder_buffer.cpp
tests/pipeline/test_tracker.cpp
tests/ports/test_edge_ports.cpp          compile-only smoke (implementable + virtual dispatch)
```

Touched existing files:
- `ports/IEstimator.hpp` — add pure virtual `Track initiate(const Measurement&) const`.
- `core/estimation/EkfEstimator.hpp` — mark existing `initiate` as `override`.
- `core/tracking/TrackManager.hpp/.cpp` — add `last_observation` per track + accessors + mutable `tracks()` overload + `predictAll`.
- Existing tests should remain green; one new test verifies the `IEstimator&`-dispatched `initiate`.

## Build/test (from repo root)

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

If a command fails with `~/.conan2`/"readonly database", re-run with the sandbox disabled. Commits: `git -c commit.gpgsign=false commit -m "..."`. No pushes.

## Current root CMakeLists.txt (post plan 3, for reference)

```cmake
cmake_minimum_required(VERSION 3.20)
project(navtracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

enable_testing()

add_library(navtracker_core
  core/geo/Wgs84.cpp
  core/geo/Datum.cpp
  core/estimation/ConstantVelocity2D.cpp
  core/estimation/MeasurementModels.cpp
  core/estimation/EkfEstimator.cpp
  core/association/Gating.cpp
  core/association/GnnAssociator.cpp
  core/tracking/TrackManager.cpp
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)

add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
  tests/types/test_ids.cpp
  tests/types/test_measurement.cpp
  tests/types/test_track.cpp
  tests/estimation/test_constant_velocity.cpp
  tests/estimation/test_measurement_models.cpp
  tests/estimation/test_ekf_estimator.cpp
  tests/association/test_gating.cpp
  tests/association/test_gnn_associator.cpp
  tests/tracking/test_track_manager.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
```

---

## Task 1: ReorderBuffer

**Math/Logic.** Track `latest_seen` = max timestamp pushed so far. On `push(m)`: if `m.time < latest_seen − window` → drop, increment `dropped_`, return false; else insert into a time-ordered multimap, update `latest_seen`, return true. On `drain()`: release in time order all entries with `time ≤ latest_seen − window`.
**Assumptions.** Source timestamps are trustworthy (engine drives on them); `window` chosen ≥ worst expected reorder skew; one-shot drain per processing cycle.
**Rationale.** Decouples message arrival from processing; gives live and replay the same release semantics → determinism (spec D2, D4).
**Ways to improve.** Per-source latency calibration; retrodiction/OOSM update for late data instead of drop; bounded total size with overflow drop.

**Files:**
- Create: `core/pipeline/ReorderBuffer.hpp`, `core/pipeline/ReorderBuffer.cpp`
- Test: `tests/pipeline/test_reorder_buffer.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/pipeline/test_reorder_buffer.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/pipeline/ReorderBuffer.hpp"

using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::ReorderBuffer;
using navtracker::Timestamp;

namespace {
Measurement at(double seconds) {
  Measurement m;
  m.time = Timestamp::fromSeconds(seconds);
  m.model = MeasurementModel::Position2D;
  return m;
}
}  // namespace

TEST(ReorderBuffer, ReleasesInTimeOrderAfterWindow) {
  ReorderBuffer buf(2.0);  // 2-second reorder window
  EXPECT_TRUE(buf.push(at(0.0)));
  EXPECT_TRUE(buf.push(at(3.0)));   // latest=3, cutoff=1
  auto out = buf.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 0.0);

  EXPECT_TRUE(buf.push(at(1.0)));   // accepted (1 >= cutoff 1)
  EXPECT_TRUE(buf.push(at(2.0)));   // accepted
  out = buf.drain();                // cutoff still 1
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 1.0);
}

TEST(ReorderBuffer, DropsLateMeasurements) {
  ReorderBuffer buf(2.0);
  buf.push(at(0.0));
  buf.push(at(5.0));                // latest=5, cutoff=3
  EXPECT_FALSE(buf.push(at(-1.0))); // too late
  EXPECT_EQ(buf.dropped(), 1u);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/pipeline/ReorderBuffer.cpp` to `navtracker_core` (after `core/tracking/TrackManager.cpp`). Add `tests/pipeline/test_reorder_buffer.cpp` to `navtracker_tests` (after `tests/tracking/test_track_manager.cpp`).

- [ ] **Step 3: Verify it fails**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/pipeline/ReorderBuffer.hpp` not found.

- [ ] **Step 4: Create `core/pipeline/ReorderBuffer.hpp`**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Time-ordered buffer that releases measurements once they are older than
// `window_seconds` behind the latest timestamp seen. Late arrivals (older
// than that cutoff) are dropped.
class ReorderBuffer {
 public:
  explicit ReorderBuffer(double window_seconds);

  // Returns true if the measurement was accepted; false if dropped as late.
  bool push(const Measurement& m);

  // Release all measurements with time <= (latest_seen - window), in
  // chronological order.
  std::vector<Measurement> drain();

  std::size_t pending() const { return queue_.size(); }
  std::size_t dropped() const { return dropped_; }

 private:
  std::int64_t window_nanos_;
  bool seen_{false};
  Timestamp latest_;
  std::multimap<Timestamp, Measurement> queue_;
  std::size_t dropped_{0};
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/pipeline/ReorderBuffer.cpp`**

```cpp
#include "core/pipeline/ReorderBuffer.hpp"

namespace navtracker {

ReorderBuffer::ReorderBuffer(double window_seconds)
    : window_nanos_(static_cast<std::int64_t>(window_seconds * 1e9)) {}

bool ReorderBuffer::push(const Measurement& m) {
  if (seen_) {
    const Timestamp cutoff{latest_.nanos() - window_nanos_};
    if (m.time < cutoff) {
      ++dropped_;
      return false;
    }
    if (latest_ < m.time) latest_ = m.time;
  } else {
    latest_ = m.time;
    seen_ = true;
  }
  queue_.emplace(m.time, m);
  return true;
}

std::vector<Measurement> ReorderBuffer::drain() {
  std::vector<Measurement> out;
  if (!seen_) return out;
  const Timestamp cutoff{latest_.nanos() - window_nanos_};
  auto it = queue_.begin();
  while (it != queue_.end() && !(cutoff < it->first)) {  // it->time <= cutoff
    out.push_back(it->second);
    it = queue_.erase(it);
  }
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `ReorderBuffer.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add core/pipeline/ReorderBuffer.hpp core/pipeline/ReorderBuffer.cpp tests/pipeline/test_reorder_buffer.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(pipeline): add time-ordered ReorderBuffer with bounded window"
```

---

## Task 2: Extend `IEstimator` with `initiate` (and mark EKF override)

**Why.** Plan 2 added `EkfEstimator::initiate` as a concrete method. The orchestrator (Task 4) needs to call `initiate` through the `IEstimator&` so the implementation stays pluggable. Promote `initiate` to a pure-virtual on the port.

**Files:**
- Modify: `ports/IEstimator.hpp`
- Modify: `core/estimation/EkfEstimator.hpp`
- Test: append to `tests/estimation/test_ekf_estimator.cpp`

- [ ] **Step 1: Append the failing test to `tests/estimation/test_ekf_estimator.cpp`**

```cpp
TEST(EkfEstimator, InitiateDispatchesViaIEstimatorBaseReference) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 5.0);
  const navtracker::IEstimator& base = ekf;
  Measurement z;
  z.time = Timestamp::fromSeconds(1.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "s";
  z.value = Eigen::Vector2d(7.0, -3.0);
  z.covariance = Eigen::Matrix2d::Identity();
  const Track t = base.initiate(z);
  EXPECT_DOUBLE_EQ(t.state(0), 7.0);
  EXPECT_DOUBLE_EQ(t.state(1), -3.0);
  EXPECT_EQ(t.status, navtracker::TrackStatus::Tentative);
}
```

(This requires `<memory>` / `make_shared` and the `IEstimator` header. The existing top-of-file `#include <memory>` and includes of `EkfEstimator.hpp`/`ConstantVelocity2D.hpp` are present from plan 2. Add `#include "ports/IEstimator.hpp"` near the other includes if not already pulled in transitively — it is pulled in via `EkfEstimator.hpp`, but adding it explicitly is harmless.)

- [ ] **Step 2: Verify it fails**

Run `cmake --build build 2>&1 | head -20`. Expected: FAIL — `base.initiate(z)` compile error: no member `initiate` in `IEstimator`.

- [ ] **Step 3: Edit `ports/IEstimator.hpp`**

Add the pure virtual method. The body becomes:
```cpp
class IEstimator {
 public:
  virtual ~IEstimator() = default;

  // Advance the track's state and covariance to time `to`.
  virtual void predict(Track& track, Timestamp to) const = 0;

  // Fold a measurement into the track. Assumes the track was already
  // predicted to z.time.
  virtual void update(Track& track, const Measurement& z) const = 0;

  // Create a new Tentative track seeded from a position-type measurement.
  virtual Track initiate(const Measurement& z) const = 0;
};
```

- [ ] **Step 4: Edit `core/estimation/EkfEstimator.hpp`**

Add `override` to the existing `initiate` declaration so the public block reads:
```cpp
 public:
  EkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

  // Create a new Tentative track seeded from a position-type measurement.
  Track initiate(const Measurement& z) const override;
```

- [ ] **Step 5: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: full suite green, including the new dispatch test.

- [ ] **Step 6: Commit**

```bash
git add ports/IEstimator.hpp core/estimation/EkfEstimator.hpp tests/estimation/test_ekf_estimator.cpp
git -c commit.gpgsign=false commit -m "feat(estimation): promote initiate to IEstimator port"
```

---

## Task 3: `TrackManager` — last_observation, mutable tracks, predictAll

Adds the bookkeeping the orchestrator needs: per-track `last_observation` timestamp, a mutable accessor (so the EKF can write back into stored tracks), and a `predictAll(estimator, to)` helper.

**Files:**
- Modify: `core/tracking/TrackManager.hpp`, `core/tracking/TrackManager.cpp`
- Test: append to `tests/tracking/test_track_manager.cpp`

- [ ] **Step 1: Append the failing test to `tests/tracking/test_track_manager.cpp`**

```cpp
TEST(TrackManager, TracksLastObservationAndPredictAll) {
  TrackManager mgr(3, 3);
  const TrackId id =
      mgr.add(Track{}, navtracker::Timestamp::fromSeconds(10.0));
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id).seconds(), 10.0);

  mgr.noteObservation(id, navtracker::Timestamp::fromSeconds(15.5));
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id).seconds(), 15.5);

  // mutableTracks gives writable access (the EKF will write through it).
  mgr.mutableTracks()[0].state = Eigen::VectorXd::Zero(4);
  EXPECT_EQ(mgr.tracks()[0].state.size(), 4);
}
```

You will also need `#include <Eigen/Core>` at the top of this test file (it isn't already there).

- [ ] **Step 2: Verify it fails**

Run `cmake --build build 2>&1 | head -20`. Expected: compile errors — no `add(Track,Timestamp)` overload, no `lastObservation`, no `noteObservation`, no `mutableTracks`.

- [ ] **Step 3: Edit `core/tracking/TrackManager.hpp`**

Add `#include "core/types/Timestamp.hpp"` (already pulled in transitively but make explicit). Update the public block to:

```cpp
 public:
  TrackManager(int confirm_hits, int delete_misses);

  // Register a new Tentative track; assigns and returns a fresh stable id.
  // `first_observation` seeds the lifecycle's "last observed" clock.
  TrackId add(const Track& track,
              Timestamp first_observation = Timestamp{});

  void recordHit(TrackId id);
  void recordMiss(TrackId id);

  // Record that this track absorbed a measurement at time `t`.
  void noteObservation(TrackId id, Timestamp t);
  Timestamp lastObservation(TrackId id) const;

  const std::vector<Track>& tracks() const { return tracks_; }
  std::vector<Track>& mutableTracks() { return tracks_; }
  std::size_t size() const { return tracks_.size(); }
```

And in the private section, add the parallel vector:

```cpp
  std::vector<Timestamp> last_observation_;  // parallel to tracks_
```

- [ ] **Step 4: Edit `core/tracking/TrackManager.cpp`**

Replace `add` and the erase paths to keep `last_observation_` in sync, and add the new methods:

```cpp
TrackId TrackManager::add(const Track& track, Timestamp first_observation) {
  Track t = track;
  t.id = TrackId{next_id_++};
  t.status = TrackStatus::Tentative;
  tracks_.push_back(t);
  counters_.push_back(Counters{1, 0});
  last_observation_.push_back(first_observation);
  return t.id;
}

void TrackManager::noteObservation(TrackId id, Timestamp t) {
  const int i = index(id);
  if (i < 0) return;
  last_observation_[i] = t;
}

Timestamp TrackManager::lastObservation(TrackId id) const {
  const int i = index(id);
  if (i < 0) return Timestamp{};
  return last_observation_[i];
}
```

And in `recordMiss`, when erasing also erase from `last_observation_`:
```cpp
  if (counters_[i].misses >= delete_misses_) {
    tracks_.erase(tracks_.begin() + i);
    counters_.erase(counters_.begin() + i);
    last_observation_.erase(last_observation_.begin() + i);
    return;
  }
```

- [ ] **Step 5: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: full suite green; existing TrackManager tests still pass (default arg keeps old `add(Track{})` calls valid).

- [ ] **Step 6: Commit**

```bash
git add core/tracking/TrackManager.hpp core/tracking/TrackManager.cpp tests/tracking/test_track_manager.cpp
git -c commit.gpgsign=false commit -m "feat(tracking): TrackManager learns last_observation and mutable access"
```

---

## Task 4: Edge ports — `ISensorAdapter`, `ITrackSink`

Pure-virtual headers that name the hexagon's edge ports. No behavior here; the adapters arrive in plan 5. Minimal compile-only smoke test to lock the signatures.

**Files:**
- Create: `ports/ISensorAdapter.hpp`, `ports/ITrackSink.hpp`
- Test: `tests/ports/test_edge_ports.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/ports/test_edge_ports.cpp`:

```cpp
#include <vector>

#include <gtest/gtest.h>
#include "ports/ISensorAdapter.hpp"
#include "ports/ITrackSink.hpp"

namespace {
class FakeAdapter : public navtracker::ISensorAdapter {
 public:
  std::vector<navtracker::Measurement> poll() override { return {}; }
};
class FakeSink : public navtracker::ITrackSink {
 public:
  void onTracks(const std::vector<navtracker::Track>&,
                navtracker::Timestamp) override {
    ++calls;
  }
  int calls = 0;
};
}  // namespace

TEST(EdgePorts, FakesImplementAndDispatch) {
  FakeAdapter a;
  navtracker::ISensorAdapter& ar = a;
  EXPECT_TRUE(ar.poll().empty());

  FakeSink s;
  navtracker::ITrackSink& sr = s;
  sr.onTracks({}, navtracker::Timestamp::fromSeconds(1.0));
  EXPECT_EQ(s.calls, 1);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `tests/ports/test_edge_ports.cpp` to `navtracker_tests` (after `tests/pipeline/test_reorder_buffer.cpp`).

- [ ] **Step 3: Verify it fails**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `ports/ISensorAdapter.hpp` / `ports/ITrackSink.hpp` not found.

- [ ] **Step 4: Create `ports/ISensorAdapter.hpp`**

```cpp
#pragma once

#include <vector>

#include "core/types/Measurement.hpp"

namespace navtracker {

// Driving-side edge port: produces normalized measurements pulled from a
// source (live sensor or a log replay). Concrete adapters live in `adapters/`.
class ISensorAdapter {
 public:
  virtual ~ISensorAdapter() = default;
  // Drain any measurements that have become available. Empty if none.
  virtual std::vector<Measurement> poll() = 0;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `ports/ITrackSink.hpp`**

```cpp
#pragma once

#include <vector>

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Driven-side edge port: receives the authoritative track set after each
// processing step (for display, logging, downstream consumers, etc).
class ITrackSink {
 public:
  virtual ~ITrackSink() = default;
  virtual void onTracks(const std::vector<Track>& tracks, Timestamp now) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `EdgePorts.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add ports/ISensorAdapter.hpp ports/ITrackSink.hpp tests/ports/test_edge_ports.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(ports): add ISensorAdapter and ITrackSink edge interfaces"
```

---

## Task 5: `Tracker` orchestrator

**Math/Logic.** For each `Measurement z` passed to `process`:
1. `manager.predictAll(estimator, z.time)` advances all tracks to z.time.
2. `result = associator.associate(manager.tracks(), {z})`.
3. If `result.matches` non-empty: take the (track_idx, 0) pair → `estimator.update(track, z)`; `manager.recordHit(id)`; `manager.noteObservation(id, z.time)`; add `z.source_id` to provenance if absent.
4. Else: `t = estimator.initiate(z); manager.add(t, z.time)`.
5. Maintenance: for each track, if `z.time − manager.lastObservation(id) > miss_timeout` then `manager.recordMiss(id)`.

**Assumptions.** One measurement per `process` call (the ReorderBuffer feeds them in order); ≤1 track matches a given measurement; `predictAll` is a no-op for `dt ≤ 0` (existing EKF semantics).
**Rationale.** Single-message processing matches the asynchronous, multi-rate sensor reality; timeout-based miss policy avoids destroying healthy tracks on every unrelated message.
**Ways to improve.** Batch-by-time-slice when multiple sensors land at the same instant; out-of-sequence (OOSM) retrodiction instead of drop; pre-association MMSI/sensor-track-ID hint locks.

**Files:**
- Create: `core/pipeline/Tracker.hpp`, `core/pipeline/Tracker.cpp`
- Modify: `core/tracking/TrackManager.hpp/.cpp` (add `predictAll`)
- Test: `tests/pipeline/test_tracker.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/pipeline/test_tracker.cpp`:

```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::Tracker;
using navtracker::TrackManager;
using navtracker::TrackStatus;

namespace {
Measurement positionAt(double t, double x, double y, const std::string& src) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity();
  return z;
}
}  // namespace

TEST(Tracker, InitiatesAndUpdatesSingleTarget) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(/*confirm=*/2, /*delete=*/3);
  Tracker tracker(estimator, associator, manager, /*miss_timeout=*/10.0);

  tracker.process(positionAt(0.0, 10.0, 0.0, "ais"));
  tracker.process(positionAt(1.0, 12.0, 0.0, "ais"));
  tracker.process(positionAt(2.0, 14.0, 0.0, "ais"));

  ASSERT_EQ(manager.size(), 1u);
  EXPECT_EQ(manager.tracks()[0].status, TrackStatus::Confirmed);
  EXPECT_NEAR(manager.tracks()[0].state(0), 14.0, 1.0);
  EXPECT_NEAR(manager.tracks()[0].state(1), 0.0, 1.0);
}

TEST(Tracker, StaleTrackTimesOutAndIsDeleted) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(1, 2);  // confirm immediately, delete after 2 misses
  Tracker tracker(estimator, associator, manager, /*miss_timeout=*/0.5);

  tracker.process(positionAt(0.0, 0.0, 0.0, "s"));         // initiate track A
  ASSERT_EQ(manager.size(), 1u);
  tracker.process(positionAt(1.0, 1000.0, 0.0, "s"));      // out of gate -> new track B; A misses (age 1 > 0.5)
  ASSERT_EQ(manager.size(), 2u);
  tracker.process(positionAt(2.0, 1000.5, 0.0, "s"));      // matches B; A age = 2 > 0.5, second miss -> deleted
  EXPECT_EQ(manager.size(), 1u);
}

TEST(Tracker, ReplayIsDeterministic) {
  const std::vector<Measurement> stream{
      positionAt(0.0, 0.0, 0.0, "a"),
      positionAt(1.0, 1.0, 0.0, "a"),
      positionAt(2.0, 2.0, 0.0, "a"),
      positionAt(3.0, 3.0, 0.0, "a"),
  };

  auto run = [&stream]() {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator estimator(motion, 10.0);
    GnnAssociator associator(20.0);
    TrackManager manager(2, 3);
    Tracker tracker(estimator, associator, manager, 10.0);
    for (const auto& z : stream) tracker.process(z);
    return manager.tracks();  // return snapshot copy
  };

  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].id.value, b[i].id.value);
    ASSERT_EQ(a[i].state.size(), b[i].state.size());
    for (int k = 0; k < a[i].state.size(); ++k) {
      EXPECT_DOUBLE_EQ(a[i].state(k), b[i].state(k));
    }
  }
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/pipeline/Tracker.cpp` to `navtracker_core` (after `core/pipeline/ReorderBuffer.cpp`). Add `tests/pipeline/test_tracker.cpp` to `navtracker_tests` (after `tests/ports/test_edge_ports.cpp`).

- [ ] **Step 3: Edit `core/tracking/TrackManager.hpp`**

Add a forward declaration of `IEstimator` at the top of the namespace (avoid circular includes), and a new method:
```cpp
class IEstimator;  // fwd

class TrackManager {
 public:
  // ... existing ...

  // Advance every active track to `to` via the estimator.
  void predictAll(const IEstimator& estimator, Timestamp to);

  // ... existing ...
};
```

- [ ] **Step 4: Edit `core/tracking/TrackManager.cpp`**

Add the include and the method:
```cpp
#include "ports/IEstimator.hpp"

// ... existing methods ...

void TrackManager::predictAll(const IEstimator& estimator, Timestamp to) {
  for (auto& t : tracks_) {
    estimator.predict(t, to);
  }
}
```

- [ ] **Step 5: Verify the new tests fail to compile**

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/pipeline/Tracker.hpp` not found.

- [ ] **Step 6: Create `core/pipeline/Tracker.hpp`**

```cpp
#pragma once

#include "core/types/Measurement.hpp"
#include "ports/IDataAssociator.hpp"
#include "ports/IEstimator.hpp"

namespace navtracker {

class TrackManager;

// Single-measurement orchestrator. Drives the predict-associate-update/initiate
// cycle, then ages unobserved tracks via a miss-timeout policy.
class Tracker {
 public:
  Tracker(const IEstimator& estimator,
          const IDataAssociator& associator,
          TrackManager& manager,
          double miss_timeout_seconds);

  void process(const Measurement& z);

 private:
  const IEstimator& estimator_;
  const IDataAssociator& associator_;
  TrackManager& manager_;
  double miss_timeout_seconds_;
};

}  // namespace navtracker
```

- [ ] **Step 7: Create `core/pipeline/Tracker.cpp`**

```cpp
#include "core/pipeline/Tracker.hpp"

#include <cstdint>
#include <vector>

#include "core/tracking/TrackManager.hpp"

namespace navtracker {

Tracker::Tracker(const IEstimator& estimator,
                 const IDataAssociator& associator,
                 TrackManager& manager,
                 double miss_timeout_seconds)
    : estimator_(estimator),
      associator_(associator),
      manager_(manager),
      miss_timeout_seconds_(miss_timeout_seconds) {}

void Tracker::process(const Measurement& z) {
  // 1. Predict every track to z.time.
  manager_.predictAll(estimator_, z.time);

  // 2. Associate the single measurement against active tracks.
  const std::vector<Measurement> batch{z};
  const AssociationResult result =
      associator_.associate(manager_.tracks(), batch);

  // 3. Match -> update, hit, note observation, provenance.
  if (!result.matches.empty()) {
    const std::size_t ti = result.matches.front().first;
    Track& tr = manager_.mutableTracks()[ti];
    estimator_.update(tr, z);
    bool has_src = false;
    for (const auto& s : tr.contributing_sources) {
      if (s == z.source_id) {
        has_src = true;
        break;
      }
    }
    if (!has_src) tr.contributing_sources.push_back(z.source_id);
    const TrackId id = tr.id;
    manager_.recordHit(id);
    manager_.noteObservation(id, z.time);
  } else {
    // 4. No match -> initiate a new Tentative track.
    Track seed = estimator_.initiate(z);
    manager_.add(seed, z.time);
  }

  // 5. Maintenance: miss-timeout on tracks not observed recently.
  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(miss_timeout_seconds_ * 1e9);
  std::vector<TrackId> stale;
  for (const auto& tr : manager_.tracks()) {
    const std::int64_t age =
        z.time.nanos() - manager_.lastObservation(tr.id).nanos();
    if (age > timeout_ns) stale.push_back(tr.id);
  }
  for (const TrackId id : stale) manager_.recordMiss(id);
}

}  // namespace navtracker
```

- [ ] **Step 8: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `Tracker.*` tests pass; full suite green.

- [ ] **Step 9: Commit**

```bash
git add core/pipeline/Tracker.hpp core/pipeline/Tracker.cpp core/tracking/TrackManager.hpp core/tracking/TrackManager.cpp tests/pipeline/test_tracker.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(pipeline): add Tracker orchestrator with miss-timeout and replay determinism"
```

---

## Task 6: Pipeline documentation

Documentation only.

**Files:**
- Create: `docs/algorithms/pipeline.md`

- [ ] **Step 1: Create `docs/algorithms/pipeline.md`**

```markdown
# Pipeline & Time

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec sections 6 and 7.

## 1. Reorder buffer (`ReorderBuffer`)

**Math/Logic.** Track `latest = max(time pushed)`. On `push(m)`: if
`m.time < latest − window` drop (count); else insert into a time-ordered
container, update `latest`. On `drain()`: pop in time order everything with
`time ≤ latest − window`.

**Assumptions.** Source timestamps are trusted; `window` ≥ worst expected
reorder skew; one-shot drain per cycle.

**Rationale.** Decouples message arrival from processing; gives live and replay
the same release semantics so the engine is deterministic (spec D2, D4).

**Ways to improve / test next.** Per-source latency calibration; OOSM /
retrodiction update instead of dropping; bounded total size with overflow
drop; multi-sensor scan grouping inside the window.

## 2. Single-measurement orchestration (`Tracker`)

**Math/Logic.** For each released `Measurement z`:
1. `predictAll(estimator, z.time)`.
2. `result = associator.associate(tracks, {z})`.
3. If matched: `estimator.update(track, z)`; `recordHit`; `noteObservation`;
   add `z.source_id` to provenance if absent.
4. Else: `estimator.initiate(z)` → `add` to manager.
5. Maintenance: for every track, if `z.time − last_observation > miss_timeout`
   then `recordMiss` (Coasting → Deleted via the lifecycle state machine).

**Assumptions.** One measurement per `process` call; ≤1 track matches a given
measurement; `predict` is a no-op for `dt ≤ 0`; the source-of-truth time base
is `Measurement.time`, never wall-clock.

**Rationale.** Single-message processing matches the asynchronous, multi-rate
reality of the sensor mix (spec §6). A timeout-based miss policy avoids the
classic mistake of marking every unrelated track as missed on every message,
which would shred healthy tracks.

**Ways to improve / test next.** Group simultaneous measurements into per-time
scans for joint association; OOSM retrodiction; pre-association MMSI/sensor-
track-ID hint locking; batch updates when multiple sensors agree.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/pipeline.md
git -c commit.gpgsign=false commit -m "docs: add pipeline & time algorithm reference"
```

---

## Done criteria

- Full suite green: `cmake --build build && ctest --test-dir build --output-on-failure`.
- `ReorderBuffer`, `Tracker` exist in `navtracker_core` (no I/O). `ISensorAdapter`/`ITrackSink` edge ports defined.
- `IEstimator::initiate` is a pure virtual; `EkfEstimator` overrides it.
- `TrackManager` exposes `last_observation`, mutable tracks, and `predictAll`.
- Deterministic-replay test passes — same input → identical output.
- `docs/algorithms/pipeline.md` documents math, assumptions, rationale, and improvement paths.

## Roadmap (remaining plans)

5. **Sensor adapters** — concrete `ISensorAdapter` for AIS, ARPA (TTM/TLL), EO/IR, own-ship; normalization/geo-projection into ENU with per-sensor R.
6. **Scenario harness + metrics** — synthetic ground-truth scenarios, OSPA/track-accuracy for evaluating estimator/association choices.
