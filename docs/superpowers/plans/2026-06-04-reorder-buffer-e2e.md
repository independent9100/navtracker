# ReorderBuffer End-to-End Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove `ReorderBuffer`'s determinism, drop accounting, and accuracy-parity claims end-to-end with realistic per-sensor clock skew. No production-path changes.

**Architecture:** A new `sim/SkewInjector` reorders measurements by `arrival_time = truth_time + lag_k + jitter_k` (seeded RNG, stable sort, truth timestamps preserved). A new scenario test pipes sim → injector → `ReorderBuffer(W)` → `Tracker` and compares against the baseline un-skewed path.

**Tech Stack:** C++17, Eigen, GoogleTest, existing `core/scenario` harness, `core/pipeline/ReorderBuffer`.

**Spec:** `docs/superpowers/specs/2026-06-04-reorder-buffer-e2e-design.md`.

---

### Task 1 — SkewInjector header + four-part doc

**Files:**
- Create: `sim/SkewInjector.hpp`

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

// Per-sensor lag (s, constant) and jitter half-width J (s, uniform on [-J,+J]).
// Index by SensorKind cast to size_t.
struct SkewProfile {
  struct Entry { double lag_s{0.0}; double jitter_s{0.0}; };
  std::array<Entry, 7> by_kind{};  // size matches SensorKind enumerator count.

  Entry& at(SensorKind k) { return by_kind[static_cast<std::size_t>(k)]; }
  const Entry& at(SensorKind k) const { return by_kind[static_cast<std::size_t>(k)]; }
};

// Realistic maritime defaults — see spec §3.2.
SkewProfile defaultMaritimeSkewProfile();

// Re-orders `input` into arrival-time order produced by the SkewProfile.
// Measurement.time (truth timestamp) is NEVER modified — only the emission
// order changes. RNG is std::mt19937_64 seeded with `seed`; jitter is drawn
// from a 2049-tick integer ladder so that draws are bit-exact across STLs
// (std::uniform_real_distribution is not portable).
//
// === Math ===
// arrival_time(m) = m.time + lag_k + jitter_k
//   lag_k:    SkewProfile.at(m.sensor).lag_s
//   jitter_k: ((rng() % (2*N+1)) - N) * (J_k / N), N = 1024
// Output order: stable_sort by arrival_time, ties broken by ingestion index.
//
// === Assumptions ===
//   1. Truth timestamps (Measurement.time) are preserved unchanged downstream.
//   2. Skew is symmetric per sensor (Uniform(-J,+J)); burst-correlated lag is
//      out of scope (see "Ways to improve").
//   3. The injector is deterministic in (seed, input order). Same inputs +
//      same seed -> same arrival sequence on any STL/platform.
//
// === Rationale ===
//   - Stable sort on arrival_time: real packets arrive in jittered order; the
//     buffer is what decides drops, not the injector.
//   - Per-kind lag + uniform jitter: matches the observed envelope of
//     maritime feeds (VDL contention for AIS, frame-bus latency for EO/IR)
//     well enough to surface ordering bugs without overclaiming a calibrated
//     noise model.
//   - Integer jitter ladder: std::uniform_real_distribution differs across
//     libstdc++/libc++/MSVC; quantizing to 2049 ticks gives bit-exact draws.
//
// === Ways to improve / what to test next ===
//   - Burst-correlated lag (Markov idle/burst-of-K) for realistic AIS storms.
//   - Per-source-id keying (two AIS receivers with different lags).
//   - Calibrated profiles from sea-trial logs.
std::vector<Measurement> applySkew(const std::vector<Measurement>& input,
                                   const SkewProfile& profile,
                                   std::uint64_t seed);

}  // namespace navtracker
```

- [ ] **Step 2: Commit**

```bash
git add sim/SkewInjector.hpp
git commit -m "feat(sim): add SkewInjector header with four-part doc"
```

---

### Task 2 — SkewInjector implementation

**Files:**
- Create: `sim/SkewInjector.cpp`

- [ ] **Step 1: Write the .cpp**

```cpp
#include "sim/SkewInjector.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <random>

namespace navtracker {

SkewProfile defaultMaritimeSkewProfile() {
  SkewProfile p;
  p.at(SensorKind::Unknown) = {0.0, 0.0};
  p.at(SensorKind::Ais)     = {0.50, 1.00};
  p.at(SensorKind::ArpaTtm) = {0.05, 0.05};
  p.at(SensorKind::ArpaTll) = {0.05, 0.05};
  p.at(SensorKind::EoIr)    = {0.15, 0.05};
  p.at(SensorKind::OwnShip) = {0.00, 0.02};
  p.at(SensorKind::Lidar)   = {0.00, 0.00};
  return p;
}

namespace {
constexpr int kJitterTicks = 1024;  // half-range; full ladder is 2*N+1.

double drawJitter(std::mt19937_64& rng, double half_width_s) {
  if (half_width_s <= 0.0) return 0.0;
  const std::uint64_t span = 2ULL * kJitterTicks + 1ULL;
  const int tick = static_cast<int>(rng() % span) - kJitterTicks;
  return static_cast<double>(tick) * (half_width_s / static_cast<double>(kJitterTicks));
}
}  // namespace

std::vector<Measurement> applySkew(const std::vector<Measurement>& input,
                                   const SkewProfile& profile,
                                   std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  const std::size_t n = input.size();
  std::vector<std::int64_t> arrival_nanos(n);
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), std::size_t{0});

  for (std::size_t i = 0; i < n; ++i) {
    const auto& entry = profile.at(input[i].sensor);
    const double jitter = drawJitter(rng, entry.jitter_s);
    const double offset_s = entry.lag_s + jitter;
    arrival_nanos[i] =
        input[i].time.nanos() + static_cast<std::int64_t>(offset_s * 1e9);
  }

  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     return arrival_nanos[a] < arrival_nanos[b];
                   });

  std::vector<Measurement> out;
  out.reserve(n);
  for (std::size_t idx : order) out.push_back(input[idx]);
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, find the `navtracker_sim` target source list and add `sim/SkewInjector.cpp` alongside the existing emitter sources. Confirm by reading the list nearby — match the existing style exactly.

- [ ] **Step 3: Build core only**

Run: `cmake --build build --target navtracker_sim`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add sim/SkewInjector.cpp CMakeLists.txt
git commit -m "feat(sim): implement SkewInjector with deterministic integer-tick jitter"
```

---

### Task 3 — SkewInjector unit tests

**Files:**
- Create: `tests/sim/test_skew_injector.cpp`

- [ ] **Step 1: Write the tests**

```cpp
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "sim/SkewInjector.hpp"

using navtracker::applySkew;
using navtracker::defaultMaritimeSkewProfile;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::SkewProfile;
using navtracker::Timestamp;

namespace {
Measurement make(double t_s, SensorKind k, const std::string& src = "s") {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = k;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  return m;
}
}  // namespace

TEST(SkewInjector, ZeroProfileIsIdentity) {
  SkewProfile p;  // all zeros
  std::vector<Measurement> in = {
      make(0.0, SensorKind::Ais), make(1.0, SensorKind::Ais),
      make(2.0, SensorKind::OwnShip)};
  const auto out = applySkew(in, p, /*seed=*/1);
  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_EQ(out[i].time.nanos(), in[i].time.nanos());
  }
}

TEST(SkewInjector, TruthTimestampsPreserved) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in = {make(0.0, SensorKind::Ais),
                                 make(0.1, SensorKind::EoIr),
                                 make(0.2, SensorKind::ArpaTtm)};
  const auto out = applySkew(in, p, /*seed=*/42);
  // Every output's time field is one of the input times — no mutation.
  for (const auto& m : out) {
    bool found = false;
    for (const auto& src : in) {
      if (src.time.nanos() == m.time.nanos()) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found);
  }
}

TEST(SkewInjector, DeterministicWithSameSeed) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in;
  for (int i = 0; i < 50; ++i) {
    in.push_back(make(0.1 * i, i % 2 ? SensorKind::Ais : SensorKind::EoIr));
  }
  const auto a = applySkew(in, p, /*seed=*/1234);
  const auto b = applySkew(in, p, /*seed=*/1234);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].time.nanos(), b[i].time.nanos());
    EXPECT_EQ(a[i].sensor, b[i].sensor);
  }
}

TEST(SkewInjector, DifferentSeedsCanProduceDifferentOrder) {
  const auto p = defaultMaritimeSkewProfile();
  std::vector<Measurement> in;
  for (int i = 0; i < 50; ++i) {
    in.push_back(make(0.1 * i, SensorKind::Ais));  // big jitter
  }
  const auto a = applySkew(in, p, /*seed=*/1);
  const auto b = applySkew(in, p, /*seed=*/999);
  bool any_diff = false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].time.nanos() != b[i].time.nanos()) {
      any_diff = true;
      break;
    }
  }
  EXPECT_TRUE(any_diff);
}

TEST(SkewInjector, SingleSensorWithZeroJitterIsStableInOrder) {
  SkewProfile p;
  p.at(SensorKind::Ais) = {0.5, 0.0};  // constant lag, no jitter
  std::vector<Measurement> in = {make(0.0, SensorKind::Ais),
                                 make(1.0, SensorKind::Ais),
                                 make(2.0, SensorKind::Ais)};
  const auto out = applySkew(in, p, /*seed=*/7);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_DOUBLE_EQ(out[0].time.seconds(), 0.0);
  EXPECT_DOUBLE_EQ(out[1].time.seconds(), 1.0);
  EXPECT_DOUBLE_EQ(out[2].time.seconds(), 2.0);
}

TEST(SkewInjector, EmptyInputYieldsEmptyOutput) {
  const auto p = defaultMaritimeSkewProfile();
  const auto out = applySkew({}, p, /*seed=*/0);
  EXPECT_TRUE(out.empty());
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, add `tests/sim/test_skew_injector.cpp` to the `navtracker_tests` test source list (the same list that includes `tests/sim/test_own_ship_emitter.cpp`). Match the existing style.

- [ ] **Step 3: Build and run**

Run:
```
cmake --build build --target navtracker_tests
ctest --test-dir build -R SkewInjector --output-on-failure
```
Expected: 6/6 SkewInjector tests pass.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_skew_injector.cpp CMakeLists.txt
git commit -m "test(sim): SkewInjector unit tests (identity, determinism, order)"
```

---

### Task 4 — Expand ReorderBuffer header doc to four-part standard

**Files:**
- Modify: `core/pipeline/ReorderBuffer.hpp`

- [ ] **Step 1: Replace the existing one-line doc block above the class with this expanded block**

Find the comment lines 13–15 (currently a short two-sentence summary) and replace **only that comment block** with:

```cpp
// Time-ordered buffer that releases measurements once they are older than
// `window_seconds` behind the latest timestamp seen. Late arrivals (older
// than that cutoff) are dropped.
//
// === Math ===
//   Let t_latest = max time seen so far. On drain(), every queued
//   measurement m with m.time <= (t_latest - W) is released in
//   non-decreasing m.time order (multimap iteration). On push(m), if
//   m.time < (t_latest - W), the measurement is rejected as late and
//   counted in dropped().
//
// === Assumptions ===
//   1. Measurement.time carries the truth timestamp of the observation,
//      not the arrival time. The buffer corrects only ordering, not skew.
//   2. The user-chosen window W exceeds the maximum expected arrival skew
//      (lag + jitter). Otherwise late drops are by design.
//   3. drain() is called frequently enough that the queue size stays
//      bounded by the input rate * W.
//
// === Rationale ===
//   Fixed-window release was chosen over alternatives:
//     - Per-source ordering buffer: forces the consumer to know sensor
//       counts up front and complicates cross-sensor monotonicity.
//     - Statistical late-drop: requires a noise model per source and adds
//       state; deferred until we have calibrated lag distributions.
//     - Reorder-on-drain (sort the whole queue lazily): equivalent here
//       since the multimap is already sorted; we keep the eager form
//       because it lets drain() be a single forward sweep.
//
// === Ways to improve / what to test next ===
//   - Per-source windows: lets AIS use a larger W than ARPA without
//     paying the AIS latency on radar-only consumers.
//   - Statistical late-drop heuristic: drop on
//     P(late | observed history) > threshold instead of a hard window.
//   - Bounded-queue back-pressure for misuse (caller forgets to drain).
//   - Reorder-on-drain with std::vector + nth_element if the multimap
//     allocation cost ever shows up in profiling.
```

- [ ] **Step 2: Build and run existing buffer tests**

Run:
```
cmake --build build --target navtracker_tests
ctest --test-dir build -R ReorderBuffer --output-on-failure
```
Expected: existing 2 ReorderBuffer tests still pass; no behavior change.

- [ ] **Step 3: Commit**

```bash
git add core/pipeline/ReorderBuffer.hpp
git commit -m "docs: expand ReorderBuffer header to four-part standard"
```

---

### Task 5 — End-to-end scenario test (the headline deliverable)

**Files:**
- Create: `tests/scenario/test_reorder_buffer_e2e.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/ReorderBuffer.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"
#include "sim/SkewInjector.hpp"

using namespace navtracker;

namespace {

// Helper: run a scenario through a ReorderBuffer with the given window.
// Returns (ScenarioResult, dropped_count).
struct BufferedRun {
  ScenarioResult result;
  std::size_t dropped{0};
  std::size_t buffer_drains{0};
  bool all_drains_monotonic{true};
};

BufferedRun runBuffered(const Scenario& scenario_in_arrival_order,
                        double window_s,
                        double ospa_cutoff,
                        Tracker& tracker,
                        TrackManager& mgr) {
  BufferedRun out;
  ReorderBuffer buf(window_s);

  // We pipe measurements through the buffer in arrival order, draining
  // as we go, then hand the time-ordered drain output to the tracker.
  std::vector<Measurement> flushed_in_order;
  flushed_in_order.reserve(scenario_in_arrival_order.measurements.size());

  Timestamp last_drained{};
  bool last_drained_set = false;
  for (const auto& m : scenario_in_arrival_order.measurements) {
    buf.push(m);
    const auto drained = buf.drain();
    ++out.buffer_drains;
    for (const auto& dm : drained) {
      if (last_drained_set && dm.time < last_drained) {
        out.all_drains_monotonic = false;
      }
      last_drained = dm.time;
      last_drained_set = true;
      flushed_in_order.push_back(dm);
    }
  }
  // Drain the tail by pushing a sentinel — easier: just drain with a
  // zero-window buffer at the end. We instead flush via successive drain
  // calls; the buffer only releases items older than (latest - W), so to
  // get the rest out we reconstruct via the queue. The simplest correct
  // approach: use a wide pseudo-latest by pushing the last m again with
  // a far-future stamp? No — the cleanest is to just iterate the
  // remaining via a fresh wide drain by recreating a zero-window buffer
  // is wrong (it would not preserve ordering across the boundary).
  //
  // Instead: push a final synthetic "future" measurement to flush the
  // tail, then discard it from flushed_in_order.
  Measurement flush_marker;
  flush_marker.time = Timestamp{scenario_in_arrival_order.measurements.back().time.nanos()
                                + static_cast<std::int64_t>(window_s * 1e9) * 2};
  flush_marker.sensor = SensorKind::Unknown;
  flush_marker.model = MeasurementModel::Position2D;
  buf.push(flush_marker);
  const auto tail = buf.drain();
  for (const auto& dm : tail) {
    if (dm.time.nanos() == flush_marker.time.nanos()) continue;  // skip marker
    if (last_drained_set && dm.time < last_drained) {
      out.all_drains_monotonic = false;
    }
    last_drained = dm.time;
    last_drained_set = true;
    flushed_in_order.push_back(dm);
  }
  out.dropped = buf.dropped();

  // Now run a normal Scenario over the flushed sequence so we can reuse
  // runScenario for ScenarioResult parity with the baseline.
  Scenario s;
  s.measurements = std::move(flushed_in_order);
  s.truth = scenario_in_arrival_order.truth;
  out.result = runScenario(s, tracker, mgr, ospa_cutoff);
  return out;
}

// Count expected late drops by re-running the injector and tallying
// arrivals whose (arrival - truth) exceeds W.
std::size_t expectedLateCount(const std::vector<Measurement>& truth_ordered,
                              const SkewProfile& profile,
                              std::uint64_t seed,
                              double window_s) {
  // Reuse applySkew so the count matches the actual draws bit-for-bit.
  const auto arrival_ordered = applySkew(truth_ordered, profile, seed);
  // We need (arrival - truth) per message; reconstruct from the order.
  // Easiest: re-run the same draw sequence externally.
  // We mirror applySkew's RNG ladder exactly.
  std::mt19937_64 rng(seed);
  constexpr int N = 1024;
  std::size_t late = 0;
  const std::int64_t window_ns = static_cast<std::int64_t>(window_s * 1e9);
  for (const auto& m : truth_ordered) {
    const auto entry = profile.at(m.sensor);
    double jitter = 0.0;
    if (entry.jitter_s > 0.0) {
      const std::uint64_t span = 2ULL * N + 1ULL;
      const int tick = static_cast<int>(rng() % span) - N;
      jitter = static_cast<double>(tick) * (entry.jitter_s / static_cast<double>(N));
    }
    const double offset_s = entry.lag_s + jitter;
    const std::int64_t arrival_ns = m.time.nanos()
                                  + static_cast<std::int64_t>(offset_s * 1e9);
    if (arrival_ns - m.time.nanos() > window_ns) ++late;
  }
  (void)arrival_ordered;
  return late;
}

bool stepsEqual(const ScenarioResult& a, const ScenarioResult& b) {
  if (a.steps.size() != b.steps.size()) return false;
  for (std::size_t i = 0; i < a.steps.size(); ++i) {
    const auto& sa = a.steps[i];
    const auto& sb = b.steps[i];
    if (sa.time.nanos() != sb.time.nanos()) return false;
    if (sa.tracks.size() != sb.tracks.size()) return false;
    for (std::size_t j = 0; j < sa.tracks.size(); ++j) {
      if (sa.tracks[j].id != sb.tracks[j].id) return false;
      if (sa.tracks[j].position != sb.tracks[j].position) return false;
    }
  }
  return true;
}

Scenario buildCrossing() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  return buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0), Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0), Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, 11);
}

Scenario buildAisDropout() {
  std::vector<double> times;
  for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
  for (int i = 12; i <= 20; ++i) times.push_back(static_cast<double>(i));
  return buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 5.0, 3);
}

struct Pipeline {
  std::shared_ptr<ConstantVelocity2D> motion;
  EkfEstimator est;
  GnnAssociator assoc;
  TrackManager mgr;
  Tracker tracker;
  Pipeline(double assoc_gate, std::size_t min_hits, std::size_t miss_to_delete,
           double prune_age)
      : motion(std::make_shared<ConstantVelocity2D>(0.1)),
        est(motion, 5.0),
        assoc(assoc_gate),
        mgr(min_hits, miss_to_delete),
        tracker(est, assoc, mgr, prune_age) {}
};

constexpr double kComfortableWindow = 5.0;  // see spec §I-2
constexpr double kTightWindow = 0.25;       // deliberately under AIS lag
constexpr std::uint64_t kSeed = 42;

}  // namespace

// ===== Crossing =====

TEST(ReorderBufferE2E, CrossingDeterminismUnderSkew) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p1(50.0, 2, 4, 30.0);
  const auto r1 = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                              p1.tracker, p1.mgr);
  Pipeline p2(50.0, 2, 4, 30.0);
  const auto r2 = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                              p2.tracker, p2.mgr);

  EXPECT_TRUE(stepsEqual(r1.result, r2.result));  // I-1
  EXPECT_TRUE(r1.all_drains_monotonic);           // I-4
  EXPECT_TRUE(r2.all_drains_monotonic);
}

TEST(ReorderBufferE2E, CrossingAccuracyParityVsBaseline) {
  const Scenario truth_ordered = buildCrossing();
  Pipeline pb(50.0, 2, 4, 30.0);
  const auto baseline =
      runScenario(truth_ordered, pb.tracker, pb.mgr, 50.0);

  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);
  Pipeline ps(50.0, 2, 4, 30.0);
  const auto skewed = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                                  ps.tracker, ps.mgr);

  // I-3 accuracy parity, expressed via mean OSPA (proxy for position RMSE
  // here since both runs feed the same harness).
  const double rel_tol = 0.05;
  const double abs_tol = 0.5;
  const double diff = std::abs(skewed.result.mean_ospa - baseline.mean_ospa);
  const double allowed =
      std::max(abs_tol, rel_tol * std::abs(baseline.mean_ospa));
  EXPECT_LE(diff, allowed) << "mean_ospa baseline=" << baseline.mean_ospa
                           << " skewed=" << skewed.result.mean_ospa;

  EXPECT_EQ(ps.mgr.size(), pb.mgr.size());  // same number of confirmed tracks
}

TEST(ReorderBufferE2E, CrossingComfortableWindowDropsZero) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(50.0, 2, 4, 30.0);
  const auto r = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                             p.tracker, p.mgr);

  // I-2 (comfortable side): no drops expected.
  EXPECT_EQ(r.dropped, 0u);
}

TEST(ReorderBufferE2E, CrossingTightWindowDropsMatchGroundTruth) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(80.0, 2, 4, 30.0);
  const auto r = runBuffered(arrival_ordered, kTightWindow, 80.0,
                             p.tracker, p.mgr);

  const std::size_t expected =
      expectedLateCount(truth_ordered.measurements, profile, kSeed,
                        kTightWindow);

  // I-2 (tight side): drop count exactly matches ground truth.
  EXPECT_EQ(r.dropped, expected);
}

// ===== AIS dropout =====

TEST(ReorderBufferE2E, AisDropoutDeterminismUnderSkew) {
  const Scenario truth_ordered = buildAisDropout();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p1(80.0, 2, 5, 15.0);
  const auto r1 = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                              p1.tracker, p1.mgr);
  Pipeline p2(80.0, 2, 5, 15.0);
  const auto r2 = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                              p2.tracker, p2.mgr);

  EXPECT_TRUE(stepsEqual(r1.result, r2.result));
  EXPECT_TRUE(r1.all_drains_monotonic);
  EXPECT_TRUE(r2.all_drains_monotonic);
}

TEST(ReorderBufferE2E, AisDropoutTrackSurvivesSkew) {
  const Scenario truth_ordered = buildAisDropout();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(80.0, 2, 5, 15.0);
  const auto r = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                             p.tracker, p.mgr);

  EXPECT_EQ(p.mgr.size(), 1u);  // same as ais-dropout baseline assertion
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, add `tests/scenario/test_reorder_buffer_e2e.cpp` to the `navtracker_tests` test sources (the same list that includes `tests/scenario/test_crossing.cpp`).

- [ ] **Step 3: Build and run only the new tests**

Run:
```
cmake --build build --target navtracker_tests
ctest --test-dir build -R ReorderBufferE2E --output-on-failure
```
Expected: 6/6 ReorderBufferE2E tests pass. If an accuracy or drop-count test fails, investigate before adjusting tolerances — the spec's tolerances are intentionally tight to catch real regressions.

- [ ] **Step 4: Run the full suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: previous test count + 12 new (6 SkewInjector + 6 ReorderBufferE2E), all green.

- [ ] **Step 5: Commit**

```bash
git add tests/scenario/test_reorder_buffer_e2e.cpp CMakeLists.txt
git commit -m "test(scenario): end-to-end ReorderBuffer validation under realistic skew"
```

---

### Task 6 — Final sweep

- [ ] **Step 1: Re-read the spec acceptance section**

Open `docs/superpowers/specs/2026-06-04-reorder-buffer-e2e-design.md` §10 and confirm:

- Suite green: yes.
- I-1..I-4 all asserted: I-1 in `*DeterminismUnderSkew` tests, I-2 in `*ComfortableWindowDropsZero` and `*TightWindowDropsMatchGroundTruth`, I-3 in `*AccuracyParityVsBaseline`, I-4 in `all_drains_monotonic` checks in every buffered run.
- Four-part doc on `SkewInjector.hpp` and `ReorderBuffer.hpp`: yes (Tasks 1 and 4).
- No changes to `Tracker`, adapters, `Measurement`: confirm via `git diff --stat <base>..HEAD` showing only `sim/`, `tests/`, `CMakeLists.txt`, and the doc comment in `core/pipeline/ReorderBuffer.hpp`.

- [ ] **Step 2: No-op commit if needed**

No additional commit unless the sweep surfaces a gap.
