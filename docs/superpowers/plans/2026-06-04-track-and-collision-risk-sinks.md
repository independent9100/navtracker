# Track Lifecycle & Collision-Risk Sinks Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `ITrackSink` (lifecycle) and `ICollisionRiskSink` (CPA alerts) ports, plus a `CpaEvaluator` that walks own-ship × confirmed tracks each evaluation, applying hysteresis. Backward compatible — null sinks = today's behavior.

**Architecture:** `TrackManager` owns lifecycle events. `Tracker` proxies `Updated` via `TrackManager::recordUpdated`. `CpaEvaluator` is a standalone class wired by composition root; per-pair state tracks Risky/NotRisky.

**Spec:** `docs/superpowers/specs/2026-06-04-track-and-collision-risk-sinks-design.md`.

---

### Task 1 — `ITrackSink` port

**Files:**
- Create: `ports/ITrackSink.hpp`

- [ ] **Step 1: Write the port**

```cpp
#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Snapshot fired on a track-lifecycle transition. `status` reflects the
// status AT the moment the event fires. See spec §3.
struct TrackLifecycleEvent {
  TrackId id;
  Timestamp time;
  TrackStatus status;
};

class ITrackSink {
 public:
  virtual ~ITrackSink() = default;
  virtual void onTrackInitiated(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackConfirmed(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackUpdated(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackDeleted(const TrackLifecycleEvent& e) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Commit**

```bash
git add ports/ITrackSink.hpp
git commit -m "feat(ports): add ITrackSink for lifecycle events"
```

---

### Task 2 — `ICollisionRiskSink` port

**Files:**
- Create: `ports/ICollisionRiskSink.hpp`

- [ ] **Step 1: Write the port**

```cpp
#pragma once

#include "core/collision/Cpa.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

enum class CollisionRiskTransition {
  Entered,   // P crossed enter_probability from below
  Exited,    // P fell below exit_probability OR track was deleted
  Updated,   // still risky; per-cycle state refresh (gated by emit_updates)
};

struct CollisionRiskEvent {
  CollisionRiskTransition transition;
  TrackId other;             // the non-own-ship track in the pair
  Timestamp time;
  CpaPrediction prediction;  // full CPA at this moment
};

class ICollisionRiskSink {
 public:
  virtual ~ICollisionRiskSink() = default;
  virtual void onCollisionRisk(const CollisionRiskEvent& event) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Commit**

```bash
git add ports/ICollisionRiskSink.hpp
git commit -m "feat(ports): add ICollisionRiskSink for CPA-derived alerts"
```

---

### Task 3 — `TrackManager` lifecycle event firing

**Files:**
- Modify: `core/tracking/TrackManager.hpp`
- Modify: `core/tracking/TrackManager.cpp`

- [ ] **Step 1: Extend header**

In `core/tracking/TrackManager.hpp`, add `#include "ports/ITrackSink.hpp"` near the top.

In the public section after `predictAll`, add:

```cpp
  void setTrackSink(ITrackSink* sink) { sink_ = sink; }

  // Notify the sink (if any) that a track's kinematic state has changed.
  // Called by Tracker after a successful estimator.update. Pure event
  // fire — no state mutation here.
  void recordUpdated(TrackId id, Timestamp t);
```

In the private section after `last_observation_`, add:

```cpp
  ITrackSink* sink_{nullptr};
```

- [ ] **Step 2: Fire events from add / recordHit / recordMiss / recordUpdated**

In `core/tracking/TrackManager.cpp`, update each method:

`add`:
```cpp
TrackId TrackManager::add(const Track& track, Timestamp first_observation) {
  Track t = track;
  t.id = TrackId{next_id_++};
  t.status = TrackStatus::Tentative;
  tracks_.push_back(t);
  counters_.push_back(Counters{1, 0});
  last_observation_.push_back(first_observation);
  if (sink_ != nullptr) {
    sink_->onTrackInitiated({t.id, first_observation, t.status});
  }
  return t.id;
}
```

`recordHit` — fire confirm only on the transition (not on every hit while confirmed):
```cpp
void TrackManager::recordHit(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].hits += 1;
  counters_[i].misses = 0;
  const bool was_unconfirmed = tracks_[i].status != TrackStatus::Confirmed;
  if (counters_[i].hits >= confirm_hits_) {
    tracks_[i].status = TrackStatus::Confirmed;
    if (was_unconfirmed && sink_ != nullptr) {
      sink_->onTrackConfirmed({id, last_observation_[i], tracks_[i].status});
    }
  }
}
```

`recordMiss` — fire deletion BEFORE erasure:
```cpp
void TrackManager::recordMiss(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].misses += 1;
  counters_[i].hits = 0;
  if (counters_[i].misses >= delete_misses_) {
    if (sink_ != nullptr) {
      sink_->onTrackDeleted(
          {id, last_observation_[i], tracks_[i].status});
    }
    tracks_.erase(tracks_.begin() + i);
    counters_.erase(counters_.begin() + i);
    last_observation_.erase(last_observation_.begin() + i);
    return;
  }
  tracks_[i].status = TrackStatus::Coasting;
}
```

Add `recordUpdated` at the end of the file (before the closing namespace):
```cpp
void TrackManager::recordUpdated(TrackId id, Timestamp t) {
  if (sink_ == nullptr) return;
  const int i = index(id);
  if (i < 0) return;
  sink_->onTrackUpdated({id, t, tracks_[i].status});
}
```

- [ ] **Step 3: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_core 2>&1 | tail -3`
Expected: clean.

- [ ] **Step 4: Sanity — existing tracking tests still pass**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'TrackManager|Tracker(\.|$)' --output-on-failure 2>&1 | tail -10`
Expected: green.

- [ ] **Step 5: Commit**

```bash
git add core/tracking/TrackManager.hpp core/tracking/TrackManager.cpp
git commit -m "feat(tracking): TrackManager fires ITrackSink lifecycle events"
```

---

### Task 4 — `Tracker` calls `recordUpdated` after each update

**Files:**
- Modify: `core/pipeline/Tracker.cpp`

- [ ] **Step 1: Add the call in `process()`**

In `core/pipeline/Tracker.cpp`, find the block in `process()`:

```cpp
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
```

After `manager_.noteObservation(id, z.time);`, add:

```cpp
    manager_.recordUpdated(id, z.time);
```

- [ ] **Step 2: Add the call in `processBatch()` — both branches**

In `processBatch()`, in the soft branch, after:
```cpp
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
```
add `manager_.recordUpdated(id, t);`.

In the hard branch, after:
```cpp
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
      meas_used[mi] = true;
```
add `manager_.recordUpdated(id, t);` (just before `meas_used[mi] = true;` or just after).

- [ ] **Step 3: Build and run full pipeline tests**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'Tracker|Pipeline' --output-on-failure 2>&1 | tail -10`
Expected: green.

- [ ] **Step 4: Commit**

```bash
git add core/pipeline/Tracker.cpp
git commit -m "feat(tracker): notify TrackManager of updates for ITrackSink"
```

---

### Task 5 — `CpaEvaluator` core

**Files:**
- Create: `core/collision/CpaEvaluator.hpp`
- Create: `core/collision/CpaEvaluator.cpp`

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_set>

#include "core/collision/Cpa.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ICollisionRiskSink.hpp"

namespace navtracker {

class TrackManager;
class OwnShipProvider;

struct CpaEvaluatorConfig {
  double d_threshold_m{500.0};
  double enter_probability{0.5};
  double exit_probability{0.3};
  bool   evaluate_tentative{false};
  bool   emit_updates{false};
};

// Walks own-ship × each (Confirmed by default) track at every `evaluate(t)`
// call, computes CPA with uncertainty, and emits CollisionRiskEvents on
// per-pair Entered/Exited transitions (with hysteresis), plus optional
// Updated events when configured. See spec §5.
class CpaEvaluator {
 public:
  CpaEvaluator(const TrackManager& manager,
               const OwnShipProvider& provider,
               CpaEvaluatorConfig cfg = {});

  void setSink(ICollisionRiskSink* sink) { sink_ = sink; }

  // Run one evaluation pass. No-op if no own-ship pose is available.
  void evaluate(Timestamp t);

  // Diagnostics.
  std::size_t entered() const { return n_entered_; }
  std::size_t exited()  const { return n_exited_; }
  std::size_t updated() const { return n_updated_; }
  std::size_t riskyPairs() const { return state_.size(); }

 private:
  const TrackManager& manager_;
  const OwnShipProvider& provider_;
  CpaEvaluatorConfig cfg_;
  ICollisionRiskSink* sink_{nullptr};
  std::unordered_set<std::uint64_t> state_;
  std::size_t n_entered_{0};
  std::size_t n_exited_{0};
  std::size_t n_updated_{0};
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

```cpp
#include "core/collision/CpaEvaluator.hpp"

#include <algorithm>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/collision/CpaOwnShip.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

CpaEvaluator::CpaEvaluator(const TrackManager& manager,
                           const OwnShipProvider& provider,
                           CpaEvaluatorConfig cfg)
    : manager_(manager), provider_(provider), cfg_(cfg) {}

void CpaEvaluator::evaluate(Timestamp t) {
  const auto pose = provider_.latest();
  if (!pose.has_value()) return;
  const Track own = synthesizeOwnShipTrack(*pose, t, provider_);

  std::unordered_set<std::uint64_t> seen_this_cycle;

  for (const Track& tr : manager_.tracks()) {
    if (tr.id.value == 0) continue;
    const bool status_ok =
        cfg_.evaluate_tentative
        || tr.status == TrackStatus::Confirmed
        || tr.status == TrackStatus::Coasting;
    if (!status_ok) continue;

    const CpaPrediction pred =
        computeCpaWithUncertainty(own, tr, t, cfg_.d_threshold_m);
    const double p = pred.probability_below_threshold;
    const bool was_risky = state_.count(tr.id.value) > 0;
    const bool now_risky = was_risky
                               ? (p >= cfg_.exit_probability)
                               : (p >= cfg_.enter_probability);

    if (!was_risky && now_risky) {
      state_.insert(tr.id.value);
      ++n_entered_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Entered, tr.id, t, pred});
      }
    } else if (was_risky && !now_risky) {
      state_.erase(tr.id.value);
      ++n_exited_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Exited, tr.id, t, pred});
      }
    } else if (was_risky && now_risky && cfg_.emit_updates) {
      ++n_updated_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Updated, tr.id, t, pred});
      }
    }
    seen_this_cycle.insert(tr.id.value);
  }

  // For pairs in state_ not seen this cycle — track was deleted or no
  // longer matches the status gate. Fire Exited.
  std::vector<std::uint64_t> dropped;
  for (auto id : state_) {
    if (seen_this_cycle.count(id) == 0) dropped.push_back(id);
  }
  for (auto id : dropped) {
    state_.erase(id);
    ++n_exited_;
    if (sink_ != nullptr) {
      CpaPrediction empty{};
      empty.d_threshold_m = cfg_.d_threshold_m;
      sink_->onCollisionRisk(
          {CollisionRiskTransition::Exited, TrackId{id}, t, empty});
    }
  }
}

}  // namespace navtracker
```

- [ ] **Step 3: Register in CMake**

In `CMakeLists.txt`, find the `navtracker_core` source list (the section with `core/collision/Cpa.cpp` and `core/collision/CpaOwnShip.cpp`) and add `core/collision/CpaEvaluator.cpp` on the next line.

- [ ] **Step 4: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_core 2>&1 | tail -5`
Expected: clean.

- [ ] **Step 5: Commit**

```bash
git add core/collision/CpaEvaluator.hpp core/collision/CpaEvaluator.cpp CMakeLists.txt
git commit -m "feat(collision): CpaEvaluator with hysteresis-based risk transitions"
```

---

### Task 6 — `ITrackSink` lifecycle tests

**Files:**
- Create: `tests/tracking/test_track_sink.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ITrackSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public ITrackSink {
 public:
  std::vector<TrackLifecycleEvent> initiated;
  std::vector<TrackLifecycleEvent> confirmed;
  std::vector<TrackLifecycleEvent> updated;
  std::vector<TrackLifecycleEvent> deleted;
  void onTrackInitiated(const TrackLifecycleEvent& e) override { initiated.push_back(e); }
  void onTrackConfirmed(const TrackLifecycleEvent& e) override { confirmed.push_back(e); }
  void onTrackUpdated(const TrackLifecycleEvent& e) override { updated.push_back(e); }
  void onTrackDeleted(const TrackLifecycleEvent& e) override { deleted.push_back(e); }
};

Measurement positionAt(double x, double y, double t_s) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;
  m.source_id = "test";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

}  // namespace

TEST(TrackSink, AddFiresInitiated) {
  TrackManager mgr(2, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  Track t;
  t.state = Eigen::VectorXd::Zero(4);
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  mgr.add(t, Timestamp::fromSeconds(1.0));
  ASSERT_EQ(sink.initiated.size(), 1u);
  EXPECT_EQ(sink.initiated[0].status, TrackStatus::Tentative);
  EXPECT_DOUBLE_EQ(sink.initiated[0].time.seconds(), 1.0);
}

TEST(TrackSink, RecordHitFiresConfirmedOnce) {
  TrackManager mgr(2, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  Track t;
  t.state = Eigen::VectorXd::Zero(4);
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  const TrackId id = mgr.add(t, Timestamp::fromSeconds(1.0));
  // First hit was implicit in add (counters start at 1); need one more
  // to reach confirm_hits_=2.
  mgr.recordHit(id);
  ASSERT_EQ(sink.confirmed.size(), 1u);
  // Subsequent hits do not refire confirmed.
  mgr.recordHit(id);
  mgr.recordHit(id);
  EXPECT_EQ(sink.confirmed.size(), 1u);
}

TEST(TrackSink, RecordMissDeleteFiresDeletedBeforeErase) {
  TrackManager mgr(1, 2);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  Track t;
  t.state = Eigen::VectorXd::Zero(4);
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  const TrackId id = mgr.add(t, Timestamp::fromSeconds(1.0));
  mgr.recordMiss(id);
  EXPECT_EQ(sink.deleted.size(), 0u);  // not yet — needs delete_misses_=2
  mgr.recordMiss(id);
  ASSERT_EQ(sink.deleted.size(), 1u);
  EXPECT_EQ(sink.deleted[0].id, id);
}

TEST(TrackSink, TrackerProcessFiresUpdated) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(100.0);
  TrackManager mgr(1, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  Tracker tracker(est, assoc, mgr, 60.0);

  tracker.process(positionAt(100.0, 0.0, 1.0));
  tracker.process(positionAt(101.0, 0.0, 2.0));

  EXPECT_GT(sink.updated.size(), 0u);
}

TEST(TrackSink, NullSinkIsSafe) {
  TrackManager mgr(2, 4);
  Track t;
  t.state = Eigen::VectorXd::Zero(4);
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  const TrackId id = mgr.add(t, Timestamp::fromSeconds(1.0));
  mgr.recordHit(id);
  mgr.recordHit(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);  // delete triggers
  // No crash.
  SUCCEED();
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, find a tracking-related test (e.g., `tests/tracking/test_track_manager.cpp` if present, else any `tests/tracking/*.cpp`) and add `tests/tracking/test_track_sink.cpp` near it. If the directory has no tests yet, place it alongside `tests/pipeline/test_tracker.cpp`.

- [ ] **Step 3: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'TrackSink' --output-on-failure 2>&1 | tail -15
```
Expected: 5/5 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/tracking/test_track_sink.cpp CMakeLists.txt
git commit -m "test(tracking): ITrackSink lifecycle event coverage"
```

---

### Task 7 — `CpaEvaluator` tests

**Files:**
- Create: `tests/collision/test_cpa_evaluator.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/collision/CpaEvaluator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Track.hpp"
#include "ports/ICollisionRiskSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public ICollisionRiskSink {
 public:
  std::vector<CollisionRiskEvent> events;
  void onCollisionRisk(const CollisionRiskEvent& e) override {
    events.push_back(e);
  }
  std::size_t countOf(CollisionRiskTransition tr) const {
    std::size_t n = 0;
    for (const auto& e : events) if (e.transition == tr) ++n;
    return n;
  }
};

OwnShipPose makePose(double t_s, const Eigen::Vector2d& enu,
                     const Eigen::Vector2d& vel) {
  OwnShipPose p;
  p.time = Timestamp::fromSeconds(t_s);
  p.lat_deg = 0.0;
  p.lon_deg = 0.0;
  p.position_std_m = 1.0;
  p.velocity_enu = vel;
  p.velocity_std_m_per_s = 0.1;
  p.velocity_is_valid = true;
  // Hack: when datum is at the origin, ENU position derives from lat/lon
  // very close to zero. For simplicity, push ENU directly by setting
  // lat/lon to representable values. Use the geo::Datum inverse to set
  // lat/lon corresponding to the desired ENU.
  // (Tests build their own provider via the explicit datum constructor.)
  (void)enu;
  return p;
}

Track makeTrackAt(Eigen::Vector2d position, Eigen::Vector2d velocity,
                  double t_s, std::uint64_t id_value) {
  Track t;
  t.id = TrackId{id_value};
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::VectorXd(4);
  t.state << position.x(), position.y(), velocity.x(), velocity.y();
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  t.covariance(0, 0) = 4.0; t.covariance(1, 1) = 4.0;
  t.covariance(2, 2) = 0.04; t.covariance(3, 3) = 0.04;
  t.last_update = Timestamp::fromSeconds(t_s);
  return t;
}

}  // namespace

TEST(CpaEvaluator, NoOwnShipIsNoop) {
  OwnShipProvider provider;
  TrackManager mgr(1, 4);
  CpaEvaluator eval(mgr, provider);
  RecordingSink sink;
  eval.setSink(&sink);
  eval.evaluate(Timestamp::fromSeconds(1.0));
  EXPECT_TRUE(sink.events.empty());
}

TEST(CpaEvaluator, EnteredFiresWhenProbabilityCrossesThreshold) {
  // Own-ship at origin going east at 5 m/s; target 100m east of origin
  // going west at 5 m/s. Closing speed 10 m/s, will meet at t = 10 s.
  geo::Datum d({0.0, 0.0, 0.0});
  OwnShipProvider provider(d);
  TrackManager mgr(1, 4);
  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 50.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 0.0; pose.lon_deg = 0.0;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 0.0);
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;
  provider.update(pose);

  Track target = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                              Eigen::Vector2d(-5.0, 0.0), 0.0, 42);
  mgr.add(target, Timestamp::fromSeconds(0.0));
  // Force confirmed status (add sets to Tentative).
  mgr.recordHit(TrackId{1});

  eval.evaluate(Timestamp::fromSeconds(0.0));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Entered), 1u);
}

TEST(CpaEvaluator, NoRefireWhileRisky) {
  geo::Datum d({0.0, 0.0, 0.0});
  OwnShipProvider provider(d);
  TrackManager mgr(1, 4);
  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 50.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 0.0);
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;
  provider.update(pose);

  Track target = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                              Eigen::Vector2d(-5.0, 0.0), 0.0, 42);
  mgr.add(target, Timestamp::fromSeconds(0.0));
  mgr.recordHit(TrackId{1});

  eval.evaluate(Timestamp::fromSeconds(0.0));
  const std::size_t after_first = sink.countOf(CollisionRiskTransition::Entered);
  eval.evaluate(Timestamp::fromSeconds(0.01));
  eval.evaluate(Timestamp::fromSeconds(0.02));
  EXPECT_EQ(sink.countOf(CollisionRiskTransition::Entered), after_first);
}

TEST(CpaEvaluator, DeletedTrackFiresExited) {
  geo::Datum d({0.0, 0.0, 0.0});
  OwnShipProvider provider(d);
  TrackManager mgr(1, 1);  // delete after 1 miss
  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 50.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  OwnShipPose pose;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 0.0);
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;
  provider.update(pose);

  Track target = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                              Eigen::Vector2d(-5.0, 0.0), 0.0, 42);
  const TrackId tid = mgr.add(target, Timestamp::fromSeconds(0.0));
  mgr.recordHit(tid);
  eval.evaluate(Timestamp::fromSeconds(0.0));
  // Delete the track and evaluate again.
  mgr.recordMiss(tid);
  eval.evaluate(Timestamp::fromSeconds(1.0));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Exited), 1u);
}

TEST(CpaEvaluator, EmitUpdatesGated) {
  geo::Datum d({0.0, 0.0, 0.0});
  OwnShipProvider provider(d);
  TrackManager mgr(1, 4);
  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 50.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  cfg.emit_updates = true;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  OwnShipPose pose;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 0.0);
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;
  provider.update(pose);

  Track target = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                              Eigen::Vector2d(-5.0, 0.0), 0.0, 42);
  mgr.add(target, Timestamp::fromSeconds(0.0));
  mgr.recordHit(TrackId{1});

  eval.evaluate(Timestamp::fromSeconds(0.0));
  eval.evaluate(Timestamp::fromSeconds(0.01));
  eval.evaluate(Timestamp::fromSeconds(0.02));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Updated), 1u);
}

TEST(CpaEvaluator, TentativeSkippedByDefault) {
  geo::Datum d({0.0, 0.0, 0.0});
  OwnShipProvider provider(d);
  TrackManager mgr(10, 4);  // confirm threshold high so it stays Tentative
  CpaEvaluator eval(mgr, provider, {});
  RecordingSink sink;
  eval.setSink(&sink);

  OwnShipPose pose;
  pose.position_std_m = 1.0;
  pose.velocity_enu = Eigen::Vector2d(5.0, 0.0);
  pose.velocity_std_m_per_s = 0.1;
  pose.velocity_is_valid = true;
  provider.update(pose);

  Track target = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                              Eigen::Vector2d(-5.0, 0.0), 0.0, 42);
  target.status = TrackStatus::Tentative;
  mgr.add(target, Timestamp::fromSeconds(0.0));
  // No recordHit beyond the implicit one — track stays Tentative.
  eval.evaluate(Timestamp::fromSeconds(0.0));
  EXPECT_EQ(sink.events.size(), 0u);
}
```

- [ ] **Step 2: Register in CMake**

Add `tests/collision/test_cpa_evaluator.cpp` to `CMakeLists.txt` alongside existing collision tests (e.g., `tests/collision/test_cpa.cpp` if present, or `tests/scenario/test_cpa_scenario.cpp`).

- [ ] **Step 3: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'CpaEvaluator' --output-on-failure 2>&1 | tail -18
```
Expected: 6/6 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/collision/test_cpa_evaluator.cpp CMakeLists.txt
git commit -m "test(collision): CpaEvaluator hysteresis and lifecycle coverage"
```

---

### Task 8 — Final sweep

- [ ] **Step 1: Full suite**

Run: `cd /home/andreas/workspace/navtracker/build && ctest --output-on-failure 2>&1 | tail -3`
Expected: previous count + 11 new (5 lifecycle + 6 CPA evaluator).

- [ ] **Step 2: Acceptance (spec §13)**

Confirm:
- Lifecycle events fire on the right transitions; no double-fire.
- CPA hysteresis prevents Entered chatter; Exited fires on probability drop AND on track deletion.
- All prior tracking/pipeline/collision tests pass unchanged.
- No changes to existing public APIs beyond the additive setters and the new `recordUpdated` method.
