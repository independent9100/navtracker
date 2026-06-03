# Adaptive UERE Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an online observer that estimates own-ship GPS position σ from the residual variance of a constant-velocity fit over a sliding GGA window. Publish via `pose.position_std_m` whenever the window is full and no maneuver was detected; fall back to the static `HDOP × UERE_static` path otherwise. Default opt-out.

**Architecture:** New `core/own_ship/UereEstimator` (sliding-window LS). `OwnShipNmeaAdapter` owns the estimator as a private member and chooses adaptive vs. static when populating `pose.position_std_m`. No new port; downstream adapters (ARPA, EOIR) consume the pose field unchanged.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-03-adaptive-uere-design.md`.

---

## Task 1: `UereEstimator` core + unit tests

**Files:**
- Create: `core/own_ship/UereEstimator.hpp`
- Create: `core/own_ship/UereEstimator.cpp`
- Create: `tests/own_ship/test_uere_estimator.cpp`
- Modify: `CMakeLists.txt` (add to `navtracker_core` sources and `navtracker_tests` sources)

### Why

Per spec §3–§5 and §9 (Unit tests 1–7): one self-contained scalar observer with predictable closed-form math. All gating logic lives here so the adapter integration in Task 2 stays minimal.

### Steps

- [ ] **Step 1: Write the header**

Create `core/own_ship/UereEstimator.hpp`:

```cpp
#pragma once

#include <cstddef>
#include <deque>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct UereEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  double min_sigma_m{0.05};
};

struct UereEstimate {
  double sigma_m{0.0};
  bool is_published{false};
};

// Online observer for own-ship GPS position sigma from successive ENU
// fixes. Fits constant velocity over a sliding window and uses residual
// variance as a direct sigma_pos estimate. Suppresses publication during
// maneuvering windows (Δv between halves > threshold).
class UereEstimator {
 public:
  explicit UereEstimator(UereEstimatorConfig cfg = {});

  // Push one GGA-derived ENU sample. t/x/y in seconds and meters; the
  // implementation operates entirely on dt offsets internally.
  void observe(Timestamp t, double x_enu, double y_enu);

  UereEstimate current() const;

  // Diagnostics for tests.
  std::size_t windowSize() const { return samples_.size(); }

 private:
  struct Sample { double t; double x; double y; };

  UereEstimatorConfig cfg_;
  std::deque<Sample> samples_;
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

Create `core/own_ship/UereEstimator.cpp`:

```cpp
#include "core/own_ship/UereEstimator.hpp"

#include <cmath>

namespace navtracker {
namespace {

struct LinearFit {
  double a{0.0};     // intercept
  double b{0.0};     // slope (velocity component)
  double ss_res{0.0}; // Σ residual²
};

// Fit p(t) = a + b * (t - t0). Returns fit + SSR. Assumes samples.size() >= 2.
template <class Getter>
LinearFit fitAxis(const std::deque<UereEstimator::Sample>& samples,
                  double t0,
                  Getter get) {
  // samples is the public Sample type, but we get fed via a deque-of-Sample;
  // this overload uses raw fields via the Getter to avoid templating across
  // ranges.
  const double n = static_cast<double>(samples.size());
  double sum_dt = 0.0, sum_p = 0.0;
  for (const auto& s : samples) {
    sum_dt += (s.t - t0);
    sum_p  += get(s);
  }
  const double mean_dt = sum_dt / n;
  const double mean_p  = sum_p  / n;
  double sxy = 0.0, sxx = 0.0;
  for (const auto& s : samples) {
    const double dt = (s.t - t0) - mean_dt;
    const double dp = get(s) - mean_p;
    sxy += dt * dp;
    sxx += dt * dt;
  }
  LinearFit f;
  f.b = sxx > 0.0 ? sxy / sxx : 0.0;
  f.a = mean_p - f.b * mean_dt;
  double ss = 0.0;
  for (const auto& s : samples) {
    const double dt = s.t - t0;
    const double r  = get(s) - (f.a + f.b * dt);
    ss += r * r;
  }
  f.ss_res = ss;
  return f;
}

}  // namespace

UereEstimator::UereEstimator(UereEstimatorConfig cfg) : cfg_(cfg) {}

void UereEstimator::observe(Timestamp t, double x_enu, double y_enu) {
  samples_.push_back({static_cast<double>(t.nanos()) * 1e-9, x_enu, y_enu});
  while (samples_.size() > cfg_.window_size) samples_.pop_front();
}

UereEstimate UereEstimator::current() const {
  UereEstimate est;
  if (samples_.size() < cfg_.window_size) return est;
  if (cfg_.window_size < 4) return est;  // need at least 2 samples per half

  const double t0 = samples_.front().t;

  const auto fit_x = fitAxis(samples_, t0,
                             [](const Sample& s) { return s.x; });
  const auto fit_y = fitAxis(samples_, t0,
                             [](const Sample& s) { return s.y; });

  // Maneuver gating: split window in halves, fit velocity in each, reject
  // if |Δv| > threshold.
  const std::size_t half = samples_.size() / 2;
  std::deque<Sample> h1(samples_.begin(), samples_.begin() + half);
  std::deque<Sample> h2(samples_.begin() + half, samples_.end());
  if (h1.size() >= 2 && h2.size() >= 2) {
    const auto fx1 = fitAxis(h1, h1.front().t,
                             [](const Sample& s) { return s.x; });
    const auto fx2 = fitAxis(h2, h2.front().t,
                             [](const Sample& s) { return s.x; });
    const auto fy1 = fitAxis(h1, h1.front().t,
                             [](const Sample& s) { return s.y; });
    const auto fy2 = fitAxis(h2, h2.front().t,
                             [](const Sample& s) { return s.y; });
    const double dvx = fx2.b - fx1.b;
    const double dvy = fy2.b - fy1.b;
    const double dv  = std::sqrt(dvx * dvx + dvy * dvy);
    if (dv > cfg_.maneuver_dv_threshold_mps) return est;
  }

  const double n = static_cast<double>(samples_.size());
  if (n <= 2.0) return est;
  const double var = (fit_x.ss_res + fit_y.ss_res) / (2.0 * (n - 2.0));
  if (!(var > 0.0) || !std::isfinite(var)) return est;
  double sigma = std::sqrt(var);
  if (sigma < cfg_.min_sigma_m) sigma = cfg_.min_sigma_m;
  est.sigma_m = sigma;
  est.is_published = true;
  return est;
}

}  // namespace navtracker
```

- [ ] **Step 3: Wire into CMake**

In `CMakeLists.txt`, add `core/own_ship/UereEstimator.cpp` to the `navtracker_core` sources, alphabetically near `core/own_ship/` would-be path (note: no existing `core/own_ship/` folder; this is the first file there — pick a tidy position, e.g. after the `core/bias/` block).

- [ ] **Step 4: Write unit tests**

Create `tests/own_ship/test_uere_estimator.cpp` covering the seven tests from spec §9. Use a fixed seed `std::mt19937{42}` and `std::normal_distribution<>` so the tests are deterministic. Sketch:

```cpp
#include "core/own_ship/UereEstimator.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}
}  // namespace

TEST(UereEstimatorTest, UnpublishedWhenWindowEmpty) {
  UereEstimator est{};
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, UnpublishedBelowWindowSize) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  for (std::size_t i = 0; i < cfg.window_size - 1; ++i)
    est.observe(tAt(i * 1.0), 5.0 * i, 0.0);
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, ConvergesOnSyntheticWhiteNoise) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  std::mt19937 rng{42};
  const double sigma = 2.0;
  std::normal_distribution<double> n(0.0, sigma);
  const double v = 5.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), v * i + n(rng), 0.0 + n(rng));
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_GT(e.sigma_m, 0.5 * sigma);
  EXPECT_LT(e.sigma_m, 1.5 * sigma);
}

TEST(UereEstimatorTest, TracksRangeOfSigmas) {
  for (double sigma : {0.5, 2.0, 5.0, 10.0}) {
    UereEstimatorConfig cfg{};
    UereEstimator est{cfg};
    std::mt19937 rng{42};
    std::normal_distribution<double> n(0.0, sigma);
    // Stream 20 samples; on the last few, the window has settled.
    double last_sigma = -1.0;
    for (int i = 0; i < 20; ++i) {
      est.observe(tAt(i * 1.0), 5.0 * i + n(rng), n(rng));
      const auto e = est.current();
      if (e.is_published) last_sigma = e.sigma_m;
    }
    EXPECT_GT(last_sigma, 0.0);
    EXPECT_GT(last_sigma, 0.5 * sigma);
    EXPECT_LT(last_sigma, 1.5 * sigma);
  }
}

TEST(UereEstimatorTest, SuppressesDuringManeuver) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  // Window of 8: first 4 at v=(5,0), next 4 at v=(0,5). Δv = sqrt(50)≈7m/s
  // — well above threshold 0.5.
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    y += 5.0;
  }
  EXPECT_FALSE(est.current().is_published);
}

TEST(UereEstimatorTest, ResumesAfterManeuver) {
  UereEstimatorConfig cfg{};
  UereEstimator est{cfg};
  // Inject 8 maneuvering samples first.
  const std::size_t half = cfg.window_size / 2;
  double x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < half; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    x += 5.0;
  }
  for (std::size_t i = half; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), x, y);
    y += 5.0;
  }
  ASSERT_FALSE(est.current().is_published);
  // Then 8 steady samples — window slides over them.
  std::mt19937 rng{42};
  std::normal_distribution<double> n(0.0, 1.0);
  double v = 5.0;
  double t = cfg.window_size * 1.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i, t += 1.0) {
    x += v;
    est.observe(tAt(t), x + n(rng), y + n(rng));
  }
  EXPECT_TRUE(est.current().is_published);
}

TEST(UereEstimatorTest, MinSigmaFloor) {
  UereEstimatorConfig cfg{};
  cfg.min_sigma_m = 0.5;
  UereEstimator est{cfg};
  const double v = 5.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0), v * i, 0.0);  // perfect straight line
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_NEAR(e.sigma_m, cfg.min_sigma_m, 1e-9);
}

}  // namespace navtracker
```

- [ ] **Step 5: Wire test into CMake**

In `CMakeLists.txt`, add `tests/own_ship/test_uere_estimator.cpp` to the `navtracker_tests` source list.

- [ ] **Step 6: Build + run tests**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R UereEstimatorTest --output-on-failure
```
Expected: all 7 PASS.

- [ ] **Step 7: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 268/268 green (was 261/261; +7 new).

- [ ] **Step 8: Commit**

```
git add core/own_ship/UereEstimator.hpp core/own_ship/UereEstimator.cpp \
        tests/own_ship/test_uere_estimator.cpp CMakeLists.txt
git commit -m "own-ship: add UereEstimator (sliding-window LS, maneuver gating)"
```

---

## Task 2: `OwnShipNmeaAdapter` integration

**Files:**
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.hpp`
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.cpp`
- Modify: `tests/adapters/own_ship/test_own_ship_nmea.cpp` (extend with three new tests)

### Why

Per spec §6 and §9 (Integration tests 8–10): the adapter owns the estimator and chooses adaptive vs. static σ. Default off keeps existing tests deterministic.

### Steps

- [ ] **Step 1: Extend config + private state**

Modify `adapters/own_ship/OwnShipNmeaAdapter.hpp`:

```cpp
#include "core/own_ship/UereEstimator.hpp"

struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
  bool enable_adaptive_uere{false};
  UereEstimatorConfig uere_estimator_cfg{};
};
```

Add private member:

```cpp
UereEstimator uere_estimator_;
```

- [ ] **Step 2: Wire ctor**

Construct `uere_estimator_` from `cfg.uere_estimator_cfg` in the ctor initializer list. Keep the rest of construction unchanged.

- [ ] **Step 3: Update GGA handling**

In the GGA branch of `ingest(...)`, after datum conversion produces the ENU position (look for existing `datum_.toEnu(...)`-equivalent or the local equivalent — confirm the path the existing code uses to derive ENU; if currently lat/lon are stored without ENU conversion, do the conversion specifically for the estimator's input). Append:

```cpp
if (cfg_.enable_adaptive_uere) {
  uere_estimator_.observe(t, enu.x(), enu.y());
}

const UereEstimate est = uere_estimator_.current();
double sigma_pos = 0.0;
if (cfg_.enable_adaptive_uere && est.is_published) {
  sigma_pos = est.sigma_m;
} else if (hdop > 0.0) {
  sigma_pos = hdop * cfg_.uere_m;
} else {
  sigma_pos = position_std_m_;  // sticky setter (sim path)
}
pose.position_std_m = sigma_pos;
```

If the existing GGA branch already does the HDOP→sigma logic with its own variable names, splice into that block rather than duplicating it; the contract is: `pose.position_std_m` is set per the precedence rule above.

- [ ] **Step 4: Write the three integration tests**

Append to `tests/adapters/own_ship/test_own_ship_nmea.cpp`:

```cpp
TEST(OwnShipNmeaAdapterTest, AdaptiveDisabledMatchesStaticPath) {
  // With enable_adaptive_uere = false (default), the adapter behavior is
  // byte-identical to the G3 implementation. Feed a couple of GGA fixes
  // with HDOP=1.2 and assert pose.position_std_m == 1.2 * 5.0.
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});  // defaults
  // ... feed GGA with HDOP=1.2 ...
  EXPECT_NEAR(provider.latest()->position_std_m, 6.0, 1e-9);
}

TEST(OwnShipNmeaAdapterTest, AdaptivePublishesAfterWindowAndDominatesStatic) {
  // Opt in adaptive. Stream 10 GGA fixes:
  //   HDOP = 2.0 (static would give sigma = 10 m via UERE=5)
  //   Actual ENU position noise sigma = 1 m
  // After window fills, pose.position_std_m should track ~1 m, not 10.
  //
  // Implementation note: composing realistic GGA sentences requires the
  // emitter's lat/lon→DDMM helpers or use of the OwnShipEmitter path.
  // Simplest: use sim::OwnShipEmitter with report_gps_std=false (so HDOP
  // stays via the static sigma path) and configure it to inject sigma=1m.
  // Then enable adaptive on the adapter side.
}

TEST(OwnShipNmeaAdapterTest, AdaptiveFallsBackOnManeuver) {
  // Stream 4 steady samples then 4 in a different direction — same as the
  // SuppressesDuringManeuver unit test, but ingested through the adapter.
  // Assert pose.position_std_m falls back to HDOP * UERE_static for the
  // first few maneuvering windows.
}
```

Use existing fixtures where they exist. For tests 9 and 10, the cleanest path is to drive synthetic GGA strings — copy the GGA-string-construction pattern from existing tests in this file or from `sim::OwnShipEmitter::emit()`.

- [ ] **Step 5: Build + run targeted tests**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R OwnShipNmeaAdapterTest --output-on-failure
```

- [ ] **Step 6: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 271/271 green (was 268; +3 new).

- [ ] **Step 7: Commit**

```
git add adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp \
        tests/adapters/own_ship/test_own_ship_nmea.cpp
git commit -m "own-ship-nmea: adaptive UERE overrides static when published"
```

---

## Task 3: Sim validation sweep + eval-log

**Files:**
- Create: `tests/sim/test_bus_adaptive_uere.cpp`
- Modify: `tests/sim/BusComparisonHelpers.hpp` (small extension — add an `adaptive_uere` flag to the existing `GpsSweepKnob` and to `runBusClutterCrossingWithGps`)
- Modify: `CMakeLists.txt`
- Modify: `docs/algorithms/evaluation-log.md`

### Why

Per spec §9 (sweep tests 11 and 12): confirm the estimator tracks sim-injected σ across a range of conditions, and that adaptive σ in the bus closely matches static σ when both are calibrated to the same truth. Eval-log records the result.

### Steps

- [ ] **Step 1: Extend the existing helper**

In `tests/sim/BusComparisonHelpers.hpp`, append to `GpsSweepKnob`:

```cpp
bool adaptive_uere{false};
```

In `runBusClutterCrossingWithGps`, plumb the flag into the `OwnShipNmeaAdapterConfig`:

```cpp
OwnShipNmeaAdapterConfig own_adapter_cfg;
own_adapter_cfg.enable_adaptive_uere = knob.adaptive_uere;
OwnShipNmeaAdapter own_adapter(provider, own_adapter_cfg);
```

The existing builder constructs `OwnShipNmeaAdapter own_adapter(provider);` with no config — change that to pass the new config.

- [ ] **Step 2: Write the sweep test**

Create `tests/sim/test_bus_adaptive_uere.cpp` with two TESTs:

```cpp
// 11. AdaptiveTracksSimInjectedSigma:
//   For each sigma_injected in {0.1, 1.0, 5.0}:
//     20 seeds, adaptive on, R-on. Capture pose.position_std_m over time
//     via instrumentation. Assert mean published value is within ±50%
//     of sigma_injected.
//
// 12. AdaptiveSweepClutterCrossing:
//   Same cells as the G8 GPS sweep (sigma in {0.0, 0.1, 1.0, 5.0}, 20 seeds)
//   but with THREE rows per cell:
//     row_a: R-off (sigma_gps injected, no R inflation)
//     row_b: R-on static (sigma_gps injected, static UERE path)
//     row_c: R-on adaptive (sigma_gps injected, adaptive UERE)
//   Aggregate mean OSPA + id-switches per cell. Print formatted tables to
//   stderr. SUCCEED-only.
```

Mirror the printing format from `test_bus_gps_sweep.cpp` so the eval-log diffs cleanly.

For test 11, instrumentation: after each `bus.run()`, you don't have direct access to per-step published σ. The cleanest measurement is to capture `provider.latest()->position_std_m` after `bus.run()` completes — that's the final published value. Alternatively, add a small hook to `OwnShipNmeaAdapter` that records published σ over time and exposes a vector for the test. Probably the simpler path: track via a lambda or simple counter inside the test loop by re-running the bus with an instrumented adapter.

Pragmatic shortcut: since the constant-velocity own-ship in `runBusClutterCrossingWithGps` (look at the existing helper to confirm) is fully steady, the maneuver detector never triggers. So `provider.latest()->position_std_m` after `bus.run()` is the published value at end-of-run, which is a reasonable proxy for the mean. Verify this assumption in the helper code before writing the test — if the own-ship truth is actually stationary or has non-trivial motion, account for it.

- [ ] **Step 3: Wire into CMake**

Append `tests/sim/test_bus_adaptive_uere.cpp` to `navtracker_tests`.

- [ ] **Step 4: Run, capture tables**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R BusAdaptiveUere --output-on-failure 2>&1 | tee /tmp/adaptive_uere_sweep.txt
```

- [ ] **Step 5: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 273/273 green (was 271; +2 new TESTs).

- [ ] **Step 6: Append eval-log section**

In `docs/algorithms/evaluation-log.md`, append:

```markdown
## Adaptive UERE (2026-06-03)

**Setup.** Online σ_pos estimator runs over GGA-derived ENU positions in
a sliding 8-sample window; LS-fit residual variance gives σ̂; two-halves
velocity check suppresses publication during maneuvers. Adaptive mode is
default off; tests opt in via `OwnShipNmeaAdapterConfig::enable_adaptive_uere`.

### Tracking sigma across injected levels (ClutterCrossing, 20 seeds)

| sigma_injected (m) | mean published sigma (m) | within ±50%? |
|---|---|---|
| 0.10 | <fill from table 11> | <yes/no> |
| 1.00 | <fill> | <> |
| 5.00 | <fill> | <> |

### Sweep comparison (ClutterCrossing, 20 seeds, R-on path)

| sigma | row | OSPA | id_sw |
|---|---|---|---|
| 0.0 | R-off | <fill> | <> |
| 0.0 | R-on static | <> | <> |
| 0.0 | R-on adaptive | <> | <> |
| ... | ... | ... | ... |

### Verdict

<3–5 sentences. Confirm the estimator tracks σ within ±50% across the
range; confirm adaptive achieves OSPA and id-switch numbers close to the
static path (within statistical noise); note that adaptive's value is
*not* better numbers — the static path is calibrated to the sim's known
σ — but rather elimination of the static UERE knob in conditions where
σ is unknown a priori. Reference G8's GPS sweep result and the bias
estimator gating pattern for the broader self-calibration story.>
```

Fill `<fill>` from `/tmp/adaptive_uere_sweep.txt`; write the verdict in the same tone as prior sections.

- [ ] **Step 7: Commit**

```
git add tests/sim/test_bus_adaptive_uere.cpp tests/sim/BusComparisonHelpers.hpp \
        CMakeLists.txt docs/algorithms/evaluation-log.md
git commit -m "sim+eval: adaptive UERE sweep — tracks sim sigma, matches static path"
```

---

## Done criteria

- All 3 tasks committed.
- Full suite green (273/273).
- Eval-log section populated with concrete numbers and verdict.
- `OwnShipNmeaAdapterConfig::enable_adaptive_uere` defaults to false; no existing test is affected (byte-identical regression). Only tests that opt in see adaptive behavior.
