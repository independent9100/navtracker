# Association & Track Management Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the data-association layer (Mahalanobis gating + greedy global-nearest-neighbor) and the track lifecycle manager (stable-ID allocation + M-of-N confirmation/deletion) that together turn measurements into a maintained track set.

**Architecture:** Adds a `core/association/` module (`Gating`, `GnnAssociator` behind the `IDataAssociator` port) and a `core/tracking/TrackManager`. Pure domain code, no I/O. Consumes the EKF/measurement-model pieces from plan 2. The per-cycle orchestration (predict→associate→update→manage) that wires these together lives in plan 4.

**Tech Stack:** C++17 · Eigen 3.4 · GoogleTest. Builds on plans 1–2 (on `master`).

This is plan 3 of 6. Prereq: plans 1–2 merged — `navtracker_core` with `core/types/*`, `core/geo/*`, `core/estimation/*`; 29 tests pass. Design reference: `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md` §11.3 and §7.

**Documentation standard (CLAUDE.md):** Each algorithm carries Math / Assumptions / Rationale / Ways-to-improve. Concise notes per task here; Task 4 writes `docs/algorithms/association.md`.

**Scope note — sensor-ID hints deferred:** spec D1 says MMSI / sensor-track-ID act as association *hints*. To keep the baseline bounded and well-tested, the GNN here is purely kinematic (gating + greedy). Hint-aided locking is documented as the next improvement, to be added with its own tests later.

---

## File Structure

```
core/association/Gating.hpp/.cpp          d2 = y^T S^-1 y between a track and a measurement
ports/IDataAssociator.hpp                 interface + AssociationResult struct
core/association/GnnAssociator.hpp/.cpp    greedy GNN over gated Mahalanobis distance
core/tracking/TrackManager.hpp/.cpp        stable IDs + M-of-N lifecycle state machine
docs/algorithms/association.md            math/assumptions/rationale/improve
tests/association/test_gating.cpp
tests/association/test_gnn_associator.cpp
tests/tracking/test_track_manager.cpp
```

## Build/test commands (from repo root)

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release   # after CMakeLists edits
cmake --build build
ctest --test-dir build --output-on-failure
```

If a command fails with a permission error mentioning `~/.conan2` / "readonly database", re-run with the sandbox disabled. Commit with `git -c commit.gpgsign=false commit -m "..."`; do not push.

## Existing signatures this plan depends on

- `MeasurementPrediction { Eigen::VectorXd z_pred; Eigen::MatrixXd H; }` and `MeasurementPrediction predictMeasurement(MeasurementModel, const Eigen::VectorXd& state)`, `Eigen::VectorXd measurementResidual(MeasurementModel, const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred)` in `core/estimation/MeasurementModels.hpp`.
- `Track { TrackId id; Timestamp last_update; TrackStatus status; Eigen::VectorXd state; Eigen::MatrixXd covariance; TrackAttributes attributes; std::vector<std::string> contributing_sources; }`.
- `TrackStatus { Tentative, Confirmed, Coasting, Deleted }`, `TrackId { std::uint64_t value; }` with `==`/`<`.
- `Measurement { Timestamp time; SensorKind sensor; std::string source_id; MeasurementModel model; Eigen::VectorXd value; Eigen::MatrixXd covariance; AssociationHints hints; }`.

## Current root CMakeLists.txt (post plan 2, for reference)

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
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
```

---

## Task 1: Gating (squared Mahalanobis distance)

**Math.** For track state `x`, covariance `P`, and measurement `z` with covariance `R`: predicted measurement `(ẑ,H)=h(x)`; innovation `y = z − ẑ` (bearing wrapped); innovation covariance `S = H P Hᵀ + R`; gating statistic `d² = yᵀ S⁻¹ y`.
**Assumptions.** Gaussian innovations; `S` invertible (positive-definite given `P,R` PD).
**Rationale.** `d²` is the standard χ²-distributed gating statistic; reused as association cost (spec §11.3).
**Ways to improve.** Cache `S`/inverse for reuse in the EKF update; log-likelihood cost including `|S|` for size-aware gating.

**Files:**
- Create: `core/association/Gating.hpp`, `core/association/Gating.cpp`
- Test: `tests/association/test_gating.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/association/test_gating.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/association/Gating.hpp"

using navtracker::mahalanobisDistance;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Track;

TEST(Gating, ZeroWhenMeasurementMatchesPrediction) {
  Track t;
  t.state = Eigen::Vector4d(10.0, 0.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  EXPECT_NEAR(mahalanobisDistance(t, z), 0.0, 1e-12);
}

TEST(Gating, KnownDistanceForOffsetMeasurement) {
  Track t;
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity();  // position cov = I
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(3.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();  // R = I
  // S = H P H^T + R = 2*I2; d2 = [3,0] (0.5*I2) [3,0]^T = 4.5
  EXPECT_NEAR(mahalanobisDistance(t, z), 4.5, 1e-12);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/association/Gating.cpp` to `navtracker_core` (after `core/estimation/EkfEstimator.cpp`). Add `tests/association/test_gating.cpp` to `navtracker_tests` (after `tests/estimation/test_ekf_estimator.cpp`).

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/association/Gating.hpp` not found.

- [ ] **Step 4: Create `core/association/Gating.hpp`**

```cpp
#pragma once

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Squared Mahalanobis distance between a measurement and a track's predicted
// measurement: d2 = y^T S^-1 y, with S = H P H^T + R.
double mahalanobisDistance(const Track& track, const Measurement& z);

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/association/Gating.cpp`**

```cpp
#include "core/association/Gating.hpp"

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

double mahalanobisDistance(const Track& track, const Measurement& z) {
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state);
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd& h = pred.H;
  const Eigen::MatrixXd s = h * track.covariance * h.transpose() + z.covariance;
  return y.dot(s.inverse() * y);
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `Gating.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add core/association/Gating.hpp core/association/Gating.cpp tests/association/test_gating.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(association): add Mahalanobis gating distance"
```

---

## Task 2: GNN associator

**Math.** Cost = gated `d²`. Greedy GNN: repeatedly pick the globally smallest in-gate (track, measurement) pair (`d² ≤ gate`), assign it, remove both, until no in-gate pair remains. Remaining tracks/measurements are reported unmatched.
**Assumptions.** ≤1 measurement per track per call; gate threshold is a χ² quantile for the measurement dimension (e.g. 9.21 ≈ χ²₂ at 0.99); costs computed independently per pair.
**Rationale.** Greedy GNN is deterministic, O(nt·nm·iterations), and adequate to validate the pipeline before investing in optimal assignment (spec §11.3, D5).
**Ways to improve.** Optimal 2D assignment (Hungarian/auction); JPDA/MHT; MMSI/sensor-track-ID hint locking; feature-aided (size/type) costs.

**Files:**
- Create: `ports/IDataAssociator.hpp`
- Create: `core/association/GnnAssociator.hpp`, `core/association/GnnAssociator.cpp`
- Test: `tests/association/test_gnn_associator.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/association/test_gnn_associator.cpp`:

```cpp
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"

using navtracker::GnnAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Track;

namespace {
Measurement positionMeas(double x, double y) {
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity();
  return z;
}
Track positionTrack(double x, double y) {
  Track t;
  t.state = Eigen::Vector4d(x, y, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  return t;
}
}  // namespace

TEST(GnnAssociator, MatchesNearestTracksAndMeasurements) {
  const GnnAssociator assoc(9.21);
  const std::vector<Track> tracks{positionTrack(0.0, 0.0), positionTrack(100.0, 0.0)};
  const std::vector<Measurement> meas{positionMeas(0.5, 0.0), positionMeas(100.5, 0.0)};
  const auto r = assoc.associate(tracks, meas);

  ASSERT_EQ(r.matches.size(), 2u);
  bool m00 = false, m11 = false;
  for (const auto& m : r.matches) {
    if (m.first == 0 && m.second == 0) m00 = true;
    if (m.first == 1 && m.second == 1) m11 = true;
  }
  EXPECT_TRUE(m00);
  EXPECT_TRUE(m11);
  EXPECT_TRUE(r.unmatched_tracks.empty());
  EXPECT_TRUE(r.unmatched_measurements.empty());
}

TEST(GnnAssociator, OutOfGateBecomesUnmatched) {
  const GnnAssociator assoc(9.21);
  const std::vector<Track> tracks{positionTrack(0.0, 0.0)};
  const std::vector<Measurement> meas{positionMeas(1000.0, 0.0)};
  const auto r = assoc.associate(tracks, meas);

  EXPECT_TRUE(r.matches.empty());
  ASSERT_EQ(r.unmatched_tracks.size(), 1u);
  EXPECT_EQ(r.unmatched_tracks[0], 0u);
  ASSERT_EQ(r.unmatched_measurements.size(), 1u);
  EXPECT_EQ(r.unmatched_measurements[0], 0u);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/association/GnnAssociator.cpp` to `navtracker_core` (after `core/association/Gating.cpp`). Add `tests/association/test_gnn_associator.cpp` to `navtracker_tests` (after `tests/association/test_gating.cpp`).

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/association/GnnAssociator.hpp` not found.

- [ ] **Step 4: Create `ports/IDataAssociator.hpp`**

```cpp
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Output of associating a batch of measurements to existing tracks. Indices
// refer into the input `tracks` and `measurements` vectors.
struct AssociationResult {
  std::vector<std::pair<std::size_t, std::size_t>> matches;  // (track_idx, meas_idx)
  std::vector<std::size_t> unmatched_tracks;
  std::vector<std::size_t> unmatched_measurements;
};

// Data-association strategy: assign measurements to tracks.
class IDataAssociator {
 public:
  virtual ~IDataAssociator() = default;
  virtual AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements) const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/association/GnnAssociator.hpp`**

```cpp
#pragma once

#include "ports/IDataAssociator.hpp"

namespace navtracker {

// Greedy global-nearest-neighbor association over gated squared Mahalanobis
// distance. Repeatedly assigns the globally smallest in-gate pair.
class GnnAssociator : public IDataAssociator {
 public:
  explicit GnnAssociator(double gate_threshold);

  AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements) const override;

 private:
  double gate_threshold_;  // chi-square gate on squared Mahalanobis distance
};

}  // namespace navtracker
```

- [ ] **Step 6: Create `core/association/GnnAssociator.cpp`**

```cpp
#include "core/association/GnnAssociator.hpp"

#include <limits>

#include "core/association/Gating.hpp"

namespace navtracker {

GnnAssociator::GnnAssociator(double gate_threshold)
    : gate_threshold_(gate_threshold) {}

AssociationResult GnnAssociator::associate(
    const std::vector<Track>& tracks,
    const std::vector<Measurement>& measurements) const {
  const std::size_t nt = tracks.size();
  const std::size_t nm = measurements.size();
  std::vector<bool> track_used(nt, false);
  std::vector<bool> meas_used(nm, false);
  AssociationResult result;

  while (true) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t best_t = 0;
    std::size_t best_m = 0;
    bool found = false;
    for (std::size_t ti = 0; ti < nt; ++ti) {
      if (track_used[ti]) continue;
      for (std::size_t mi = 0; mi < nm; ++mi) {
        if (meas_used[mi]) continue;
        const double d2 = mahalanobisDistance(tracks[ti], measurements[mi]);
        if (d2 <= gate_threshold_ && d2 < best) {
          best = d2;
          best_t = ti;
          best_m = mi;
          found = true;
        }
      }
    }
    if (!found) break;
    result.matches.emplace_back(best_t, best_m);
    track_used[best_t] = true;
    meas_used[best_m] = true;
  }

  for (std::size_t ti = 0; ti < nt; ++ti) {
    if (!track_used[ti]) result.unmatched_tracks.push_back(ti);
  }
  for (std::size_t mi = 0; mi < nm; ++mi) {
    if (!meas_used[mi]) result.unmatched_measurements.push_back(mi);
  }
  return result;
}

}  // namespace navtracker
```

- [ ] **Step 7: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `GnnAssociator.*` tests pass.

- [ ] **Step 8: Commit**

```bash
git add ports/IDataAssociator.hpp core/association/GnnAssociator.hpp core/association/GnnAssociator.cpp tests/association/test_gnn_associator.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(association): add greedy GNN associator with IDataAssociator port"
```

---

## Task 3: Track manager (stable IDs + M-of-N lifecycle)

**Math/Logic.** Lifecycle (spec §7): a new track is `Tentative` with `hits=1, misses=0`. `recordHit`: `hits++`, `misses=0`; when `hits ≥ confirm_hits` → `Confirmed`. `recordMiss`: `misses++`, `hits=0`; when `misses ≥ delete_misses` the track is deleted (removed); otherwise → `Coasting`. IDs are a monotonic `uint64` starting at 1, never reused.
**Assumptions.** "Hit" = associated this cycle, "miss" = not associated; consecutive-count policy (not sliding window); the orchestration that calls these (predict/associate/update) is plan 4.
**Rationale.** Simplest deterministic M-of-N state machine; stable-never-reused IDs satisfy the core invariant (spec D1) independent of external identity.
**Ways to improve.** Score-based (log-likelihood-ratio) confirmation/deletion; sliding-window M-of-N; promote Coasting back to Confirmed on re-acquisition.

**Files:**
- Create: `core/tracking/TrackManager.hpp`, `core/tracking/TrackManager.cpp`
- Test: `tests/tracking/test_track_manager.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/tracking/test_track_manager.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/tracking/TrackManager.hpp"

using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackManager;
using navtracker::TrackStatus;

TEST(TrackManager, AssignsMonotonicStableIds) {
  TrackManager mgr(3, 3);
  const TrackId id1 = mgr.add(Track{});
  const TrackId id2 = mgr.add(Track{});
  EXPECT_EQ(id1.value, 1u);
  EXPECT_EQ(id2.value, 2u);
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_EQ(mgr.tracks()[0].id, id1);
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
}

TEST(TrackManager, ConfirmsAfterConsecutiveHits) {
  TrackManager mgr(3, 3);  // confirm after 3 detections
  const TrackId id = mgr.add(Track{});  // detection #1 (hits=1)
  mgr.recordHit(id);                    // detection #2
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
  mgr.recordHit(id);                    // detection #3 -> Confirmed
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Confirmed);
}

TEST(TrackManager, CoastsThenDeletesAfterMisses) {
  TrackManager mgr(2, 2);  // confirm after 2 hits, delete after 2 misses
  const TrackId id = mgr.add(Track{});  // hits=1, Tentative
  mgr.recordHit(id);                    // hits=2 -> Confirmed
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Confirmed);
  mgr.recordMiss(id);                   // misses=1 -> Coasting
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Coasting);
  mgr.recordMiss(id);                   // misses=2 -> Deleted (removed)
  EXPECT_EQ(mgr.size(), 0u);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/tracking/TrackManager.cpp` to `navtracker_core` (after `core/association/GnnAssociator.cpp`). Add `tests/tracking/test_track_manager.cpp` to `navtracker_tests` (after `tests/association/test_gnn_associator.cpp`).

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/tracking/TrackManager.hpp` not found.

- [ ] **Step 4: Create `core/tracking/TrackManager.hpp`**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/types/Track.hpp"

namespace navtracker {

// Owns the active track set, allocates stable TrackIds, and runs the M-of-N
// lifecycle state machine. IDs are monotonic and never reused. The cycle that
// drives hits/misses (predict/associate/update) lives in the pipeline (plan 4).
class TrackManager {
 public:
  TrackManager(int confirm_hits, int delete_misses);

  // Register a new Tentative track; assigns and returns a fresh stable id.
  TrackId add(const Track& track);

  void recordHit(TrackId id);   // associated this cycle
  void recordMiss(TrackId id);  // not associated; may Coast or Delete (remove)

  const std::vector<Track>& tracks() const { return tracks_; }
  std::size_t size() const { return tracks_.size(); }

 private:
  struct Counters {
    int hits;
    int misses;
  };
  int index(TrackId id) const;  // position in tracks_, or -1 if absent

  int confirm_hits_;
  int delete_misses_;
  std::uint64_t next_id_{1};
  std::vector<Track> tracks_;
  std::vector<Counters> counters_;  // parallel to tracks_
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/tracking/TrackManager.cpp`**

```cpp
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

TrackManager::TrackManager(int confirm_hits, int delete_misses)
    : confirm_hits_(confirm_hits), delete_misses_(delete_misses) {}

TrackId TrackManager::add(const Track& track) {
  Track t = track;
  t.id = TrackId{next_id_++};
  t.status = TrackStatus::Tentative;
  tracks_.push_back(t);
  counters_.push_back(Counters{1, 0});
  return t.id;
}

int TrackManager::index(TrackId id) const {
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

void TrackManager::recordHit(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].hits += 1;
  counters_[i].misses = 0;
  if (counters_[i].hits >= confirm_hits_) {
    tracks_[i].status = TrackStatus::Confirmed;
  }
}

void TrackManager::recordMiss(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].misses += 1;
  counters_[i].hits = 0;
  if (counters_[i].misses >= delete_misses_) {
    tracks_.erase(tracks_.begin() + i);
    counters_.erase(counters_.begin() + i);
    return;
  }
  tracks_[i].status = TrackStatus::Coasting;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `TrackManager.*` tests pass; full suite green.

- [ ] **Step 7: Commit**

```bash
git add core/tracking/TrackManager.hpp core/tracking/TrackManager.cpp tests/tracking/test_track_manager.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(tracking): add TrackManager with stable IDs and M-of-N lifecycle"
```

---

## Task 4: Association & lifecycle documentation

Documentation only — no build/test.

**Files:**
- Create: `docs/algorithms/association.md`

- [ ] **Step 1: Create `docs/algorithms/association.md`**

```markdown
# Association & Track-Management Algorithms

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec sections 7 and 11.3.

## 1. Gating (`mahalanobisDistance`)

**Math.** `(ẑ,H)=h(x)`; innovation `y=z−ẑ` (bearing wrapped); innovation
covariance `S=H P Hᵀ + R`; gating statistic `d²=yᵀ S⁻¹ y` (χ²-distributed with
DOF = measurement dimension under the Gaussian hypothesis).

**Assumptions.** Gaussian innovations; `S` positive-definite (true when `P` and
`R` are PD).

**Rationale.** Standard χ² gate; `d²` doubles as the association cost.

**Ways to improve / test next.** Reuse the computed `S`/inverse in the EKF
update; add the `0.5·ln|S|` term for a proper log-likelihood cost.

## 2. Greedy GNN association (`GnnAssociator`)

**Math.** Cost between a track and a measurement is the gated `d²`. Greedy
assignment: repeatedly select the globally smallest in-gate pair (`d² ≤ gate`),
assign it, remove both from consideration, repeat until no in-gate pair remains.
Unselected tracks and measurements are returned as unmatched.

**Assumptions.** At most one measurement per track per call; the gate is a χ²
quantile for the measurement dimension (e.g. 9.21 ≈ χ²₂ at 0.99); pairwise
independent costs.

**Rationale.** Deterministic and simple; sufficient to validate the fusion
pipeline before investing in optimal assignment (spec D5).

**Ways to improve / test next.** Optimal 2D assignment (Hungarian/auction) so a
locally cheap greedy pick can't block a better global solution; JPDA for
ambiguous clutter; MHT for deferred decisions; MMSI / sensor-track-ID hint
locking before kinematic gating; feature-aided (size/type) costs.

## 3. Track lifecycle (`TrackManager`)

**Math/Logic.** New track: `Tentative`, `hits=1`, `misses=0`. `recordHit`:
`hits++`, `misses=0`; `hits ≥ confirm_hits` → `Confirmed`. `recordMiss`:
`misses++`, `hits=0`; `misses ≥ delete_misses` → deleted (removed); else
`Coasting`. IDs: monotonic `uint64` from 1, never reused.

**Assumptions.** Consecutive-count policy (not sliding window); one hit/miss per
track per cycle; orchestration lives in the pipeline (plan 4).

**Rationale.** Simplest deterministic M-of-N machine; stable never-reused IDs
satisfy the core identity invariant independent of MMSI (spec D1).

**Ways to improve / test next.** Score-based (log-likelihood-ratio)
confirmation/deletion; sliding-window M-of-N; re-promote Coasting → Confirmed on
re-acquisition.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/association.md
git -c commit.gpgsign=false commit -m "docs: add association & track-management algorithm reference"
```

---

## Done criteria

- Full suite green: `cmake --build build && ctest --test-dir build --output-on-failure`.
- `mahalanobisDistance`, `IDataAssociator` + `GnnAssociator`, and `TrackManager` exist in `navtracker_core` (no I/O).
- GNN matches in-gate pairs and reports unmatched tracks/measurements; TrackManager allocates stable monotonic IDs and runs the Tentative→Confirmed→Coasting→Deleted lifecycle.
- `docs/algorithms/association.md` documents math, assumptions, rationale, and improvement paths.

## Roadmap (remaining plans)

4. **Pipeline + time** — time-ordered reorder buffer; tracker orchestration wiring predict → `GnnAssociator::associate` → `EkfEstimator::update` / `initiate` → `TrackManager` lifecycle; `ISensorAdapter`/`ITrackSink` ports; deterministic replay test.
5. **Sensor adapters** — AIS, ARPA (TTM/TLL), EO/IR, own-ship; normalization/geo-projection into ENU with per-sensor R.
6. **Scenario harness + metrics** — synthetic ground-truth scenarios, OSPA/track-accuracy for comparing estimator/association choices.
```
