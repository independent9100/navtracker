# Heading Bias Estimator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a global scalar heading-bias state estimated from AIS-vs-ARPA position residuals and applied at the ARPA/EO/IR adapters, so long-range bearing accuracy improves once an AIS anchor is in scene.

**Architecture:** New `core/bias/HeadingBiasEstimator` implementing `ports/IHeadingBiasProvider`. Adapters take a nullable `const IHeadingBiasProvider*` constructor arg and subtract `b̂` from raw heading before bearing projection. `var(b̂)` composes in quadrature with the configured `heading_std_deg` from the §14.9 work. Per-cycle AIS+ARPA pair extraction happens in the composition root using a tiny `recent_contributions` field added to `Track`.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md`. The numbered sections in tasks below refer to that spec.

---

## Task 1: Track gains per-cycle source provenance

**Files:**
- Modify: `core/types/Track.hpp`
- Modify: `core/pipeline/Tracker.cpp` (write to it on every fusion)
- Test: `tests/pipeline/test_tracker.cpp` (extend)

### Why

`HeadingBiasEstimator` needs to identify which tracks were fused from both an AIS and an ARPA measurement *at the current cycle*. `Track::contributing_sources` is cumulative over the lifetime of the track, so an old AIS hit on a now-ARPA-only target would falsely qualify. We add a small per-cycle record that the composition root reads and clears.

### Steps

- [ ] **Step 1: Define `SourceTouch` and add the field**

Append to `core/types/Track.hpp` *inside* the `Track` struct (before the closing brace):

```cpp
  // Per-cycle provenance for downstream components (e.g. bias estimator).
  // Populated by Tracker when a measurement updates this track. Consumers
  // read after a cycle completes; they are responsible for clearing.
  struct SourceTouch {
    SensorKind sensor{SensorKind::Unknown};
    std::string source_id;
    Timestamp time;
    Eigen::Vector2d value_enu{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
    Eigen::Vector2d sensor_position_enu{Eigen::Vector2d::Zero()};
  };
  std::vector<SourceTouch> recent_contributions;
```

You'll also need to add `#include "core/types/Ids.hpp"` for `SensorKind` if it isn't already transitively included (it already is via the existing include of `Ids.hpp`). Verify `SensorKind` is visible.

- [ ] **Step 2: Write the failing test**

Add to `tests/pipeline/test_tracker.cpp` (use the existing TEST file's fixtures and helpers; mirror an existing simple test for setup):

```cpp
TEST(TrackerTest, RecordsRecentContributionsOnFusion) {
  // Build a minimal Tracker with a constant-velocity estimator + GNN
  // associator (use the same setup helpers other tests in this file use).
  // Construct two Position2D measurements at the same time on the same
  // location, one with sensor=SensorKind::Ais source_id="ais",
  // one with sensor=SensorKind::ArpaTtm source_id="arpa".
  //
  // Feed both via tracker.process(...) (so they end up fused into the same
  // track via gating). Verify the resulting track has at least two entries
  // in recent_contributions whose source_ids include "ais" and "arpa".
}
```

Use the test patterns already present in `tests/pipeline/test_tracker.cpp`. The point is: after feeding, `manager.tracks()[0].recent_contributions.size() >= 2` and the set of source_ids contains both.

- [ ] **Step 3: Run test, verify it fails**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R TrackerTest.RecordsRecentContributionsOnFusion --output-on-failure
```
Expected: fails because `recent_contributions` is never written.

- [ ] **Step 4: Implement the writes in `core/pipeline/Tracker.cpp`**

In `Tracker::process(const Measurement& z)`, immediately after `estimator_.update(tr, z);` (around line 29), append:

```cpp
Track::SourceTouch touch;
touch.sensor = z.sensor;
touch.source_id = z.source_id;
touch.time = z.time;
if (z.model == MeasurementModel::Position2D && z.value.size() >= 2) {
  touch.value_enu = z.value.head<2>();
  if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
    touch.covariance = z.covariance.topLeftCorner<2, 2>();
  }
}
touch.sensor_position_enu = z.sensor_position_enu;
tr.recent_contributions.push_back(std::move(touch));
```

Do the same in `Tracker::processBatch(...)` inside both the soft (JPDA) and hard branches, immediately after the corresponding `estimator_.update(tr, ...)` / `estimator_.softUpdate(tr, ...)` calls. For JPDA's `softUpdate` loop, append one `SourceTouch` per gated measurement (`gated[k]`), so a track that gets contributions from multiple sources in one cycle records all of them.

- [ ] **Step 5: Run test, verify it passes**

```
ctest --test-dir build -R TrackerTest.RecordsRecentContributionsOnFusion --output-on-failure
```
Expected: PASS.

- [ ] **Step 6: Run full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: all green. The new field is default-empty and zero-impact on every other test.

- [ ] **Step 7: Commit**

```
git add core/types/Track.hpp core/pipeline/Tracker.cpp tests/pipeline/test_tracker.cpp
git commit -m "tracker: record per-cycle source contributions on Track"
```

---

## Task 2: `IHeadingBiasProvider` port

**Files:**
- Create: `ports/IHeadingBiasProvider.hpp`
- Modify: `CMakeLists.txt` (no — header-only, but include path verification only)

### Why

The provider is the seam between the bias estimator and the adapters. Defining it as a port keeps adapters depending only on an interface, matching the architecture invariants.

### Steps

- [ ] **Step 1: Write the header**

Create `ports/IHeadingBiasProvider.hpp`:

```cpp
#pragma once

namespace navtracker {

// Snapshot of the heading-bias estimator's current published estimate.
// is_published is false when the estimator has not yet converged or
// has lost its anchor; consumers treat that as "use b = 0" (the
// pre-estimator behavior).
struct HeadingBiasEstimate {
  double bias_rad{0.0};
  double variance_rad2{0.0};
  bool is_published{false};
};

class IHeadingBiasProvider {
 public:
  virtual ~IHeadingBiasProvider() = default;
  virtual HeadingBiasEstimate current() const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Verify it compiles in isolation**

```
cmake --build build --target navtracker_core
```
Expected: success (no source file references it yet; header-only port).

- [ ] **Step 3: Commit**

```
git add ports/IHeadingBiasProvider.hpp
git commit -m "ports: add IHeadingBiasProvider"
```

---

## Task 3: `HeadingBiasEstimator` core implementation

**Files:**
- Create: `core/bias/HeadingBiasEstimator.hpp`
- Create: `core/bias/HeadingBiasEstimator.cpp`
- Modify: `CMakeLists.txt` (add `core/bias/HeadingBiasEstimator.cpp` to `navtracker_core` sources, in the `core/` block)

### Why

The estimator owns scalar state `(b̂, P_b)`, runs predict + sequential KF updates, gates publication, and implements `IHeadingBiasProvider`.

### Steps

- [ ] **Step 1: Write the header**

Create `core/bias/HeadingBiasEstimator.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"
#include "ports/IHeadingBiasProvider.hpp"

namespace navtracker {

struct HeadingBiasEstimatorConfig {
  // Initial estimate and variance (rad, rad^2).
  double initial_bias_rad{0.0};
  double initial_variance_rad2{(5.0 * 3.14159265358979323846 / 180.0)
                               * (5.0 * 3.14159265358979323846 / 180.0)};
  // Random-walk process noise (rad^2/s).
  double process_noise_var_per_sec{9.4e-11};  // ~2 deg/hr 1-sigma
  // Gating thresholds.
  double publish_variance_threshold_rad2{(0.3 * 3.14159265358979323846 / 180.0)
                                         * (0.3 * 3.14159265358979323846 / 180.0)};
  double stale_seconds{30.0};
};

// One pair observation: AIS-derived target position and ARPA-derived
// bearing-projected target position relative to the same own-ship
// origin at the same cycle. The estimator computes the angular
// disagreement and uses it as a scalar measurement of b.
struct AisArpaPairObservation {
  Timestamp time;
  Eigen::Vector2d own_position_enu;
  Eigen::Vector2d ais_target_position_enu;
  Eigen::Vector2d arpa_target_position_enu;
  // 1-sigma bearing noise contributed by the ARPA measurement, rad.
  double arpa_bearing_std_rad{1.0 * 3.14159265358979323846 / 180.0};
  // 1-sigma isotropic position noise on the AIS report, m.
  double ais_position_std_m{10.0};
};

class HeadingBiasEstimator : public IHeadingBiasProvider {
 public:
  explicit HeadingBiasEstimator(HeadingBiasEstimatorConfig cfg = {});

  // Time-only predict step. Safe to call repeatedly; idempotent if
  // `to` does not advance.
  void predictTo(Timestamp to);

  // Apply one AIS+ARPA pair observation. Internally calls predictTo
  // first, then performs a scalar KF update.
  void observe(const AisArpaPairObservation& obs);

  // IHeadingBiasProvider
  HeadingBiasEstimate current() const override;

  // Diagnostics (not part of IHeadingBiasProvider).
  double biasRad() const { return b_hat_; }
  double varianceRad2() const { return p_b_; }
  Timestamp lastUpdateTime() const { return last_update_; }

 private:
  HeadingBiasEstimatorConfig cfg_;
  double b_hat_;
  double p_b_;
  Timestamp last_predict_{};
  Timestamp last_update_{};
  bool has_any_update_{false};
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

Create `core/bias/HeadingBiasEstimator.cpp`:

```cpp
#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapToPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

double deltaSeconds(Timestamp a, Timestamp b) {
  return static_cast<double>(a.nanos() - b.nanos()) * 1e-9;
}

}  // namespace

HeadingBiasEstimator::HeadingBiasEstimator(HeadingBiasEstimatorConfig cfg)
    : cfg_(cfg),
      b_hat_(cfg.initial_bias_rad),
      p_b_(cfg.initial_variance_rad2) {}

void HeadingBiasEstimator::predictTo(Timestamp to) {
  if (last_predict_.nanos() == 0) {
    last_predict_ = to;
    return;
  }
  const double dt = deltaSeconds(to, last_predict_);
  if (dt <= 0.0) return;
  p_b_ += cfg_.process_noise_var_per_sec * dt;
  last_predict_ = to;
}

void HeadingBiasEstimator::observe(const AisArpaPairObservation& obs) {
  predictTo(obs.time);

  const Eigen::Vector2d ais_rel = obs.ais_target_position_enu - obs.own_position_enu;
  const Eigen::Vector2d arpa_rel = obs.arpa_target_position_enu - obs.own_position_enu;
  const double beta_truth = std::atan2(ais_rel.y(), ais_rel.x());
  const double beta_arpa = std::atan2(arpa_rel.y(), arpa_rel.x());

  const double z = wrapToPi(beta_arpa - beta_truth);
  const double r_ais = ais_rel.norm();
  // Guard tiny ranges; if AIS-reported target is essentially at own-ship
  // the bearing is undefined — skip.
  if (r_ais < 1.0) return;

  const double sigma_v2 =
      obs.arpa_bearing_std_rad * obs.arpa_bearing_std_rad
      + (obs.ais_position_std_m * obs.ais_position_std_m) / (r_ais * r_ais);

  const double y = wrapToPi(z - b_hat_);
  const double s = p_b_ + sigma_v2;
  const double k = p_b_ / s;
  b_hat_ += k * y;
  p_b_ = (1.0 - k) * p_b_;
  last_update_ = obs.time;
  has_any_update_ = true;
}

HeadingBiasEstimate HeadingBiasEstimator::current() const {
  HeadingBiasEstimate est;
  est.bias_rad = b_hat_;
  est.variance_rad2 = p_b_;
  if (!has_any_update_) {
    est.is_published = false;
    return est;
  }
  const bool tight = p_b_ <= cfg_.publish_variance_threshold_rad2;
  const double age_s = deltaSeconds(last_predict_, last_update_);
  const bool fresh = age_s <= cfg_.stale_seconds;
  est.is_published = tight && fresh;
  return est;
}

}  // namespace navtracker
```

- [ ] **Step 3: Add to CMake**

Modify `CMakeLists.txt` — inside `add_library(navtracker_core ...)`, add a new line under the existing `core/` block (alphabetically: between `core/association/` and `core/collision/` is fine, or after the `core/pipeline/` block):

```
  core/bias/HeadingBiasEstimator.cpp
```

- [ ] **Step 4: Verify it builds**

```
cmake --build build --target navtracker_core
```
Expected: success.

- [ ] **Step 5: Commit**

```
git add core/bias/HeadingBiasEstimator.hpp core/bias/HeadingBiasEstimator.cpp CMakeLists.txt
git commit -m "core: add HeadingBiasEstimator (random-walk scalar KF)"
```

---

## Task 4: Unit tests for `HeadingBiasEstimator`

**Files:**
- Create: `tests/bias/test_heading_bias_estimator.cpp`
- Modify: `CMakeLists.txt` (add to `navtracker_tests` sources)

### Why

Per spec §10 (Unit), seven tests covering construction, single update, convergence, drift tracking, anchor loss, gating, and wrap-around.

### Steps

- [ ] **Step 1: Write the test file**

Create `tests/bias/test_heading_bias_estimator.cpp`:

```cpp
#include "core/bias/HeadingBiasEstimator.hpp"

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace navtracker {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// Build a clean AIS+ARPA pair where the true ENU bearing is `beta_truth_rad`
// and the ARPA reports `beta_truth_rad + bias_rad`. Range = 1500 m.
AisArpaPairObservation makePair(Timestamp t, double beta_truth_rad,
                                double bias_rad, double range_m = 1500.0) {
  AisArpaPairObservation o;
  o.time = t;
  o.own_position_enu = Eigen::Vector2d::Zero();
  o.ais_target_position_enu =
      range_m * Eigen::Vector2d(std::cos(beta_truth_rad), std::sin(beta_truth_rad));
  const double beta_arpa = beta_truth_rad + bias_rad;
  o.arpa_target_position_enu =
      range_m * Eigen::Vector2d(std::cos(beta_arpa), std::sin(beta_arpa));
  o.arpa_bearing_std_rad = 0.5 * kDeg2Rad;
  o.ais_position_std_m = 5.0;
  return o;
}

Timestamp tAt(double seconds) {
  return Timestamp{static_cast<std::int64_t>(seconds * 1e9)};
}

}  // namespace

TEST(HeadingBiasEstimatorTest, InitialStateUnpublished) {
  HeadingBiasEstimator est{};
  const auto e = est.current();
  EXPECT_FALSE(e.is_published);
  EXPECT_NEAR(e.bias_rad, 0.0, 1e-12);
  EXPECT_GT(e.variance_rad2, 0.0);
}

TEST(HeadingBiasEstimatorTest, SinglePairMovesBiasTowardTruth) {
  HeadingBiasEstimator est{};
  const double true_bias = 1.0 * kDeg2Rad;
  est.observe(makePair(tAt(0.0), 0.0, true_bias));
  EXPECT_GT(est.biasRad(), 0.0);
  EXPECT_LT(est.biasRad(), true_bias + 0.01);
}

TEST(HeadingBiasEstimatorTest, ConvergesOverManyUpdates) {
  HeadingBiasEstimatorConfig cfg{};
  HeadingBiasEstimator est{cfg};
  const double true_bias = 2.0 * kDeg2Rad;
  for (int i = 0; i < 30; ++i) {
    // Vary bearing across pairs so the geometry isn't degenerate.
    const double beta = (i * 17.0) * kDeg2Rad;
    est.observe(makePair(tAt(i * 1.0), beta, true_bias));
  }
  const auto e = est.current();
  EXPECT_TRUE(e.is_published);
  EXPECT_NEAR(est.biasRad(), true_bias, 0.1 * kDeg2Rad);
}

TEST(HeadingBiasEstimatorTest, TracksDriftingBias) {
  HeadingBiasEstimatorConfig cfg{};
  cfg.process_noise_var_per_sec =
      std::pow(0.05 * kDeg2Rad, 2.0);  // ~3 deg/min, generous for the test
  HeadingBiasEstimator est{cfg};
  // Drive 0 deg -> 3 deg over 60 s, 1 Hz pairs.
  for (int i = 0; i <= 60; ++i) {
    const double true_bias = (3.0 * i / 60.0) * kDeg2Rad;
    const double beta = (i * 17.0) * kDeg2Rad;
    est.observe(makePair(tAt(i * 1.0), beta, true_bias));
  }
  EXPECT_NEAR(est.biasRad(), 3.0 * kDeg2Rad, 0.5 * kDeg2Rad);
}

TEST(HeadingBiasEstimatorTest, GatingClosesOnAnchorLoss) {
  HeadingBiasEstimatorConfig cfg{};
  cfg.stale_seconds = 10.0;
  HeadingBiasEstimator est{cfg};
  const double true_bias = 2.0 * kDeg2Rad;
  for (int i = 0; i < 30; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  EXPECT_TRUE(est.current().is_published);
  // Advance time past stale window without any updates.
  est.predictTo(tAt(60.0));
  EXPECT_FALSE(est.current().is_published);
}

TEST(HeadingBiasEstimatorTest, GatingDelaysPublicationUntilTightEnough) {
  HeadingBiasEstimatorConfig cfg{};
  // Make the threshold very tight so 1-2 updates aren't enough.
  cfg.publish_variance_threshold_rad2 =
      std::pow(0.05 * kDeg2Rad, 2.0);
  HeadingBiasEstimator est{cfg};
  const double true_bias = 1.0 * kDeg2Rad;
  est.observe(makePair(tAt(0.0), 0.0, true_bias));
  EXPECT_FALSE(est.current().is_published);
  for (int i = 1; i < 50; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  EXPECT_TRUE(est.current().is_published);
}

TEST(HeadingBiasEstimatorTest, HandlesWrapAroundInnovation) {
  HeadingBiasEstimator est{};
  // True bias near +pi - epsilon so beta_arpa wraps.
  const double true_bias = (kPi - 0.05);
  for (int i = 0; i < 40; ++i) {
    est.observe(makePair(tAt(i * 1.0), (i * 17.0) * kDeg2Rad, true_bias));
  }
  // Estimator should converge to true_bias (modulo wrap); check via cos
  // to side-step +pi/-pi sign issues.
  EXPECT_NEAR(std::cos(est.biasRad()), std::cos(true_bias), 0.05);
}

}  // namespace navtracker
```

- [ ] **Step 2: Add to CMake test sources**

Append to the `add_executable(navtracker_tests ...)` block in `CMakeLists.txt`, alphabetically near other `tests/`:

```
  tests/bias/test_heading_bias_estimator.cpp
```

- [ ] **Step 3: Run tests, expect green**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R HeadingBiasEstimatorTest --output-on-failure
```
Expected: all 7 PASS.

- [ ] **Step 4: Commit**

```
git add tests/bias/test_heading_bias_estimator.cpp CMakeLists.txt
git commit -m "test: HeadingBiasEstimator convergence, drift, gating, wrap"
```

---

## Task 5: ARPA adapter consumes `IHeadingBiasProvider`

**Files:**
- Modify: `adapters/arpa/ArpaAdapter.hpp`
- Modify: `adapters/arpa/ArpaAdapter.cpp`
- Test: `tests/adapters/arpa/test_arpa_adapter.cpp` (extend)

### Why

Per spec §5: subtract `b̂` from raw heading before computing the ENU bearing; compose `var(b̂)` into `sigma_heading_rad` so R-inflation shrinks as confidence grows. Nullable provider preserves backward compatibility.

### Steps

- [ ] **Step 1: Header — add the constructor parameter**

In `adapters/arpa/ArpaAdapter.hpp`, add include and parameter:

```cpp
#include "ports/IHeadingBiasProvider.hpp"
```

Change the constructor declaration to:

```cpp
ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
            ArpaAdapterConfig cfg = {},
            const IHeadingBiasProvider* bias_provider = nullptr);
```

Add to the private members:

```cpp
const IHeadingBiasProvider* bias_provider_;
```

- [ ] **Step 2: Implementation — store and use the provider**

In `adapters/arpa/ArpaAdapter.cpp`:

Update the constructor initializer:

```cpp
ArpaAdapter::ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
                         ArpaAdapterConfig cfg,
                         const IHeadingBiasProvider* bias_provider)
    : datum_(std::move(datum)),
      own_ship_(own_ship),
      cfg_(cfg),
      bias_provider_(bias_provider) {}
```

In the `TTM` branch, just before the `projectRangeBearingToEnu` call, compute corrected heading and effective σ:

```cpp
HeadingBiasEstimate bias_est = bias_provider_
                                   ? bias_provider_->current()
                                   : HeadingBiasEstimate{};
const double b_hat = bias_est.is_published ? bias_est.bias_rad : 0.0;
const double var_b_hat = bias_est.is_published ? bias_est.variance_rad2 : 0.0;

// Replace existing bearing_true_rad with bias-corrected form:
const double bearing_true_rad_corrected = bearing_true_rad - b_hat;

const double sigma_heading_cfg = cfg_.heading_std_deg * kDeg2Rad;
const double sigma_heading_eff =
    std::sqrt(sigma_heading_cfg * sigma_heading_cfg + var_b_hat);
```

Change the `projectRangeBearingToEnu` call to pass the corrected bearing and effective σ:

```cpp
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad_corrected,
                             50.0, 1.0 * kDeg2Rad,
                             sigma_heading_eff,
                             own_xy);
```

- [ ] **Step 3: Write the failing tests**

Add to `tests/adapters/arpa/test_arpa_adapter.cpp`:

```cpp
#include "ports/IHeadingBiasProvider.hpp"

namespace {

class StubBiasProvider : public navtracker::IHeadingBiasProvider {
 public:
  navtracker::HeadingBiasEstimate value;
  navtracker::HeadingBiasEstimate current() const override { return value; }
};

}  // namespace

TEST(ArpaAdapterTest, AppliesPublishedBiasToProjectedBearing) {
  // Set own-ship at origin, heading north (0 deg). Inject a TTM with
  // relative bearing 90 deg (east), range 1000 m. With no bias, the
  // projected ENU position is (1000, 0).
  //
  // Now wire a provider publishing b_hat = 5 deg. Expected projected
  // position is (1000 cos(-5 deg), 1000 sin(-5 deg)) relative to own
  // ship — i.e. a 5 deg rotation clockwise around own ship.
  //
  // Assert the projected enu position matches within tolerance.
}

TEST(ArpaAdapterTest, UnpublishedProviderActsIdentically) {
  // Same scenario but provider returns is_published=false.
  // Expected projected position identical to the no-provider path.
}

TEST(ArpaAdapterTest, ComposesVarianceWithConfiguredHeadingStd) {
  // cfg.heading_std_deg = 1.0; provider publishes var = (0.5 deg)^2.
  // Run the projection and verify the cross-track covariance matches
  // the value produced by projectRangeBearingToEnu with sigma_heading
  // = sqrt((1 deg)^2 + (0.5 deg)^2).
  //
  // Easiest assertion: build two adapters — one with provider, one
  // without but heading_std_deg = sqrt(1^2 + 0.5^2) — feed identical
  // TTMs, compare m.covariance.
}
```

Fill in each test using the existing test fixtures in this file for setting up own-ship pose, datum, and feeding TTM lines. Tolerances: positions to within 0.5 m, covariance entries to within 1 m².

- [ ] **Step 4: Run, expect them to pass**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R ArpaAdapterTest --output-on-failure
```
Expected: PASS. The implementation in Step 2 already covers the test logic; the tests confirm the wiring is correct.

- [ ] **Step 5: Run full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: green. Existing ARPA tests pass because the new ctor arg is defaulted to `nullptr`.

- [ ] **Step 6: Commit**

```
git add adapters/arpa/ArpaAdapter.hpp adapters/arpa/ArpaAdapter.cpp tests/adapters/arpa/test_arpa_adapter.cpp
git commit -m "arpa: apply IHeadingBiasProvider correction and compose sigma"
```

---

## Task 6: EO/IR adapter consumes `IHeadingBiasProvider`

**Files:**
- Modify: `adapters/eoir/EoIrAdapter.hpp`
- Modify: `adapters/eoir/EoIrAdapter.cpp`
- Test: `tests/adapters/eoir/test_eoir_adapter.cpp` (extend)

### Why

Mirror Task 5 for EO/IR. Spec §2 confirms EO/IR shares b with ARPA (same gyro), so it benefits from the same correction.

### Steps

- [ ] **Step 1: Header — add the constructor parameter**

In `adapters/eoir/EoIrAdapter.hpp`, add include:

```cpp
#include "ports/IHeadingBiasProvider.hpp"
```

Change ctor signature:

```cpp
EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship,
            EoIrAdapterConfig cfg = {},
            const IHeadingBiasProvider* bias_provider = nullptr);
```

Add private member:

```cpp
const IHeadingBiasProvider* bias_provider_;
```

- [ ] **Step 2: Implementation — same pattern as ARPA**

Update the ctor initializer list and modify the projection call site analogously to Task 5. Apply `b̂` to the corrected bearing; compose `sigma_heading_eff`. Use the same structure: query provider, branch on `is_published`, default `b_hat = 0`, `var_b_hat = 0`.

- [ ] **Step 3: Write the failing tests**

Add three tests to `tests/adapters/eoir/test_eoir_adapter.cpp` mirroring Task 5's three tests (`AppliesPublishedBiasToProjectedBearing`, `UnpublishedProviderActsIdentically`, `ComposesVarianceWithConfiguredHeadingStd`). Use `CameraDetection` inputs.

- [ ] **Step 4: Run, expect pass**

```
ctest --test-dir build -R EoIrAdapterTest --output-on-failure
```

- [ ] **Step 5: Full suite green**

```
ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```
git add adapters/eoir/EoIrAdapter.hpp adapters/eoir/EoIrAdapter.cpp tests/adapters/eoir/test_eoir_adapter.cpp
git commit -m "eoir: apply IHeadingBiasProvider correction and compose sigma"
```

---

## Task 7: AIS+ARPA pair extraction helper

**Files:**
- Create: `core/bias/AisArpaPairExtractor.hpp`
- Create: `core/bias/AisArpaPairExtractor.cpp`
- Test: `tests/bias/test_ais_arpa_pair_extractor.cpp`
- Modify: `CMakeLists.txt`

### Why

The composition root needs a small utility that takes the current track snapshot (after a tracker cycle) and emits `AisArpaPairObservation` records for any track whose `recent_contributions` contains both an AIS and an ARPA Position2D contribution from the current cycle. Encapsulating this in a tested helper keeps the composition root thin.

### Steps

- [ ] **Step 1: Write the header**

Create `core/bias/AisArpaPairExtractor.hpp`:

```cpp
#pragma once

#include <vector>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

struct AisArpaPairExtractorConfig {
  // Pairs from contributions older than this (relative to `cycle_time`)
  // are ignored. Default 0.5 s — anything in the current cycle.
  double cycle_window_seconds{0.5};
  // 1-sigma isotropic AIS position uncertainty assumed when the AIS
  // measurement covariance is degenerate; used as a fallback only.
  double ais_position_std_fallback_m{10.0};
  // 1-sigma ARPA bearing uncertainty fallback (rad).
  double arpa_bearing_std_fallback_rad{1.0 * 3.14159265358979323846 / 180.0};
};

// Extract AIS+ARPA pair observations from a fused-track snapshot. For
// each track in `tracks` whose recent_contributions (within the cycle
// window of cycle_time) include both an AIS Position2D contribution
// and an ARPA Position2D contribution, emit one observation that
// pairs them.
std::vector<AisArpaPairObservation> extractPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    AisArpaPairExtractorConfig cfg = {});

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

Create `core/bias/AisArpaPairExtractor.cpp`:

```cpp
#include "core/bias/AisArpaPairExtractor.hpp"

#include <algorithm>
#include <cmath>

#include "core/types/Ids.hpp"  // SensorKind

namespace navtracker {
namespace {

bool isAisKind(SensorKind k) {
  return k == SensorKind::Ais;
}

bool isArpaKind(SensorKind k) {
  return k == SensorKind::ArpaTtm || k == SensorKind::ArpaTll;
}

double sigmaFromCov2D(const Eigen::Matrix2d& c, double fallback) {
  // Use sqrt of mean eigenvalue (≈ isotropic std). Fall back if degenerate.
  const double tr = c.trace();
  if (!(tr > 0.0)) return fallback;
  return std::sqrt(tr * 0.5);
}

}  // namespace

std::vector<AisArpaPairObservation> extractPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    AisArpaPairExtractorConfig cfg) {
  std::vector<AisArpaPairObservation> out;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(cfg.cycle_window_seconds * 1e9);
  for (const Track& tr : tracks) {
    const Track::SourceTouch* ais = nullptr;
    const Track::SourceTouch* arpa = nullptr;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (!ais && isAisKind(t.sensor)) ais = &t;
      if (!arpa && isArpaKind(t.sensor)) arpa = &t;
      if (ais && arpa) break;
    }
    if (!ais || !arpa) continue;
    AisArpaPairObservation obs;
    obs.time = cycle_time;
    obs.own_position_enu = arpa->sensor_position_enu;
    obs.ais_target_position_enu = ais->value_enu;
    obs.arpa_target_position_enu = arpa->value_enu;
    obs.ais_position_std_m =
        sigmaFromCov2D(ais->covariance, cfg.ais_position_std_fallback_m);
    obs.arpa_bearing_std_rad = cfg.arpa_bearing_std_fallback_rad;
    out.push_back(std::move(obs));
  }
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 3: Write the failing tests**

Create `tests/bias/test_ais_arpa_pair_extractor.cpp`:

```cpp
#include "core/bias/AisArpaPairExtractor.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace navtracker {

namespace {

Track::SourceTouch makeTouch(SensorKind k, Timestamp t,
                             Eigen::Vector2d v,
                             Eigen::Vector2d own = Eigen::Vector2d::Zero()) {
  Track::SourceTouch s;
  s.sensor = k;
  s.time = t;
  s.value_enu = v;
  s.sensor_position_enu = own;
  s.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return s;
}

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

}  // namespace

TEST(AisArpaPairExtractorTest, EmitsOnePairWhenBothSourcesContributedThisCycle) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0)));
  const auto pairs = extractPairs({tr}, tAt(10.0));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_NEAR(pairs[0].ais_target_position_enu.x(), 1000.0, 1e-9);
  EXPECT_NEAR(pairs[0].arpa_target_position_enu.y(), 87.0, 1e-9);
}

TEST(AisArpaPairExtractorTest, SkipsWhenOnlyOneSourcePresent) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_TRUE(extractPairs({tr}, tAt(10.0)).empty());
}

TEST(AisArpaPairExtractorTest, IgnoresContributionsOutsideCycleWindow) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(0.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0)));
  EXPECT_TRUE(extractPairs({tr}, tAt(10.0)).empty());
}

TEST(AisArpaPairExtractorTest, MultipleTracksEmitMultiplePairs) {
  Track a;
  a.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(5.0), Eigen::Vector2d(1000.0, 0.0)));
  a.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(5.0), Eigen::Vector2d(990.0, 50.0)));
  Track b;
  b.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(5.0), Eigen::Vector2d(0.0, 2000.0)));
  b.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(5.0), Eigen::Vector2d(50.0, 1990.0)));
  EXPECT_EQ(extractPairs({a, b}, tAt(5.0)).size(), 2u);
}

}  // namespace navtracker
```

- [ ] **Step 4: Add to CMake**

Append to `navtracker_core` sources:

```
  core/bias/AisArpaPairExtractor.cpp
```

Append to `navtracker_tests` sources:

```
  tests/bias/test_ais_arpa_pair_extractor.cpp
```

- [ ] **Step 5: Run, expect pass**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R AisArpaPairExtractorTest --output-on-failure
```

- [ ] **Step 6: Commit**

```
git add core/bias/AisArpaPairExtractor.hpp core/bias/AisArpaPairExtractor.cpp \
        tests/bias/test_ais_arpa_pair_extractor.cpp CMakeLists.txt
git commit -m "core: AIS+ARPA pair extraction from track snapshot"
```

---

## Task 8: Bus-comparison harness can wire the estimator

**Files:**
- Modify: `tests/sim/BusComparisonHelpers.hpp`

### Why

The §14.9 sweep helpers build a tracker + sim bus and run scenarios. To validate the estimator end-to-end via the bus, we add a single helper that wires `HeadingBiasEstimator` + `AisArpaPairExtractor` into the existing per-cycle loop without changing existing helpers. New tests opt in; old tests untouched.

### Steps

- [ ] **Step 1: Locate the existing helper structure**

Read `tests/sim/BusComparisonHelpers.hpp` and find the entry point used by the §14.9 heading-sweep tests. There is a function that constructs adapters, tracker, sim bus, and runs the per-cycle loop, returning OSPA. Identify the smallest extension point.

- [ ] **Step 2: Add a knob and a parallel runner**

Append a `BiasEstimatorKnob` struct:

```cpp
struct BiasEstimatorKnob {
  bool enabled{false};
  HeadingBiasEstimatorConfig cfg{};
  AisArpaPairExtractorConfig extractor_cfg{};
};
```

Add a new helper function `runBusScenarioWithBiasEstimator(...)` that mirrors the existing scenario runner but, after each tracker cycle, calls `extractPairs(manager.tracks(), cycle_time)` and feeds each observation to the estimator. The adapter constructions pass `&estimator` so corrections take effect on the *next* cycle. Clear `recent_contributions` on each track at the end of the cycle (or at the start of the next).

Returns the same metric struct the existing runners return.

- [ ] **Step 3: Compile only (no new test yet)**

```
cmake --build build --target navtracker_tests
```
Expected: success. No test exercises the new path yet.

- [ ] **Step 4: Commit**

```
git add tests/sim/BusComparisonHelpers.hpp
git commit -m "sim/helpers: scenario runner that wires bias estimator"
```

---

## Task 9: Sweep — §14.9 scenarios with AIS anchor, estimator on/off

**Files:**
- Create: `tests/sim/test_bus_bias_estimator_sweep.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §10 (Scenario sweep): extend `ClutterCrossing`, `BearingOnlyMoving`, `Maneuvering` with one AIS anchor present in the scene. Three rows per σ_h: (R-off no estimator), (R-on no estimator), (R-on + estimator). 20 seeds × 4 σ_h. SUCCEED-only; data capture for eval-log.

### Steps

- [ ] **Step 1: Augment scenario builders**

The existing `*WithHeading` factory functions in `BusComparisonHelpers.hpp` build the three scenarios. Add three new `*WithHeadingAndAisAnchor` variants that include one extra AIS-broadcasting target (e.g. a slow co-passing vessel ~1500 m off the beam) — adjust as needed so the AIS target is well-separated from the ARPA contacts to avoid spurious fusion. Use the existing AIS emitter pattern from other scenarios in the sim tests.

- [ ] **Step 2: Write the sweep test**

Create `tests/sim/test_bus_bias_estimator_sweep.cpp` modeled on `tests/sim/test_bus_heading_sweep.cpp` but with three rows per cell:

```cpp
// For each scenario in {ClutterCrossing, BearingOnlyMoving, Maneuvering}:
//   For each sigma_h in {0.0, 0.5, 1.0, 2.0} deg:
//     For seed in 201..220:
//       row_a = run with cfg.heading_std_deg = 0,            estimator off
//       row_b = run with cfg.heading_std_deg = sigma_h,      estimator off
//       row_c = run with cfg.heading_std_deg = sigma_h,      estimator ON
//       record (mean_ospa, mean_id_switches) per (scenario, sigma_h, row)
//     std::cout the three averaged rows so eval-log can quote them.
// SUCCEED(); — no gating
```

Follow the exact output format used in `test_bus_heading_sweep.cpp` so the eval-log diffing stays simple.

- [ ] **Step 3: Add to CMake**

```
  tests/sim/test_bus_bias_estimator_sweep.cpp
```

- [ ] **Step 4: Run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R BusBiasEstimatorSweep --output-on-failure
```
Expected: PASS (SUCCEED-only). Capture the printed rows for the next task.

- [ ] **Step 5: Commit**

```
git add tests/sim/test_bus_bias_estimator_sweep.cpp CMakeLists.txt \
        tests/sim/BusComparisonHelpers.hpp
git commit -m "sim: sweep §14.9 scenarios with AIS anchor + bias estimator"
```

---

## Task 10: Anchor-loss scenario test

**Files:**
- Create: `tests/sim/test_bus_anchor_loss.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §10 (Anchor-loss): verify gating closes when AIS disappears, behavior reverts to R-inflation-only, no accuracy cliff at the dropout moment.

### Steps

- [ ] **Step 1: Write the test**

Create `tests/sim/test_bus_anchor_loss.cpp`:

```cpp
// Scenario: BearingOnlyMoving + AIS anchor for t in [0, 60); no AIS for
// t in [60, 120). sigma_h_inject = 2 deg sim-side. R-on (sigma_h cfg).
// Estimator ON.
//
// During 0..60 s: estimator publishes; periodically assert
// estimator.current().is_published == true and biasRad close to
// injected bias.
//
// At t = 90 s (30 s after AIS dropout, stale window default 30 s):
// assert is_published == false.
//
// OSPA over [60, 120): assert no spike vs the steady-state OSPA
// during [40, 60). Tolerance: pass if mean_OSPA_post is within
// 1.3x of mean_OSPA_pre (loose — the real point is no cliff/divergence).
```

- [ ] **Step 2: Add to CMake**

```
  tests/sim/test_bus_anchor_loss.cpp
```

- [ ] **Step 3: Run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R BusAnchorLossTest --output-on-failure
```
Expected: PASS.

- [ ] **Step 4: Commit**

```
git add tests/sim/test_bus_anchor_loss.cpp CMakeLists.txt
git commit -m "sim: anchor-loss test — gating closes cleanly on AIS dropout"
```

---

## Task 11: Eval-log section

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

### Why

Per spec §10 (Eval-log): append a "Heading bias estimator (2026-06-03)" section with the three sweep tables (mirroring the §14.9 table format), the anchor-loss test outcome, and a verdict paragraph quantifying recovered accuracy beyond the R-inflation-only baseline.

### Steps

- [ ] **Step 1: Capture Task 9 + 10 numbers**

Re-run from a clean build to lock numbers:

```
ctest --test-dir build -R "BusBiasEstimatorSweep|BusAnchorLossTest" --output-on-failure 2>&1 | tee /tmp/bias_sweep.txt
```

- [ ] **Step 2: Append the section**

Append to `docs/algorithms/evaluation-log.md`:

```markdown
## Heading bias estimator (2026-06-03)

**Setup.** Extends the §14.9 heading sweep with an AIS-equipped target
added to each scenario. Three rows per σ_h cell: (R-off, no estimator),
(R-on, no estimator), (R-on + estimator). 20 seeds (201..220), EKF+GNN,
SUCCEED-only data capture.

### ClutterCrossing (σ_h sweep, 20 seeds)
| σ_h | R-off / no est | R-on / no est | R-on + est |
|---|---|---|---|
| 0.0° | <fill> | <fill> | <fill> |
| 0.5° | <fill> | <fill> | <fill> |
| 1.0° | <fill> | <fill> | <fill> |
| 2.0° | <fill> | <fill> | <fill> |

### BearingOnlyMoving (σ_h sweep, 20 seeds)
| σ_h | R-off / no est | R-on / no est | R-on + est |
|---|---|---|---|
| ... |

### Maneuvering (σ_h sweep, 20 seeds)
| σ_h | R-off / no est | R-on / no est | R-on + est |
|---|---|---|---|
| ... |

### Anchor-loss scenario
- Steady-state OSPA during AIS-present window: <fill>
- OSPA after AIS dropout (R-inflation-only fallback): <fill>
- Verified is_published flips false within stale window.

### Verdict
<2-4 sentence interpretation: confirm estimator clawed back accuracy
beyond the R-inflation-only baseline on long-range scenarios; note
where the effect is small (short range scenarios); note that anchor
loss degrades gracefully into the §14.9 path with no cliff.>
```

Fill `<fill>` from the captured numbers; write the verdict in the same tone as the §14.9 verdict (specific numbers, no superlatives).

- [ ] **Step 3: Commit**

```
git add docs/algorithms/evaluation-log.md
git commit -m "eval-log: heading bias estimator sweep + anchor-loss results"
```

---

## Done criteria

- All 11 tasks committed.
- Full suite green: `ctest --test-dir build --output-on-failure`.
- Eval-log contains a populated "Heading bias estimator (2026-06-03)" section with concrete numbers.
- `IHeadingBiasProvider` is the only seam between adapters and the bias subsystem — a future multi-track bearing-innovation observer (deferred Option 2) plugs in without adapter changes.
