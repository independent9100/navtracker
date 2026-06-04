# Multi-Track Bearing-Innovation Heading-Bias Observer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a second observation kind (bearing innovations from Tracker) to `HeadingBiasEstimator`, plumbed via a new `IBearingInnovationSink` port emitted by `Tracker` on every Bearing2D / RangeBearing2D hard-match update. Closes the eval-log gap where the bias estimator goes stale in AIS-free scenes.

**Architecture:** Hexagonal, single fused estimator (path A). Tracker computes innovation `r`, predicted-innovation variance `S`, predicted-state-variance projected to bearing, and range from the pre-update predicted track state. Estimator gates on (range, state-variance-dominance, outlier) and applies a scalar KF update. Composition-root wiring; nullable sink — no behavior change if not wired.

**Tech Stack:** C++17, Eigen, GoogleTest, existing `predictMeasurement` from `core/estimation/MeasurementModels`.

**Spec:** `docs/superpowers/specs/2026-06-04-multi-track-bearing-bias-observer-design.md`.

---

### Task 1 — IBearingInnovationSink port

**Files:**
- Create: `ports/IBearingInnovationSink.hpp`

- [ ] **Step 1: Write the port header**

```cpp
#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// One bearing-domain innovation emitted by the Tracker after a successful
// hard-match update on a Bearing2D or RangeBearing2D measurement. The
// values are computed from the PRE-update predicted track state so the
// innovation r is a measurement of the heading bias b (see spec §3).
//
// === Math ===
// r = wrap(β_observed - β_predicted)
// variance_rad2 = H · P · Hᵀ + R         (predicted innovation variance)
// predicted_state_var_rad2 = H · P · Hᵀ   (used for the state-dominance gate)
// where H is the bearing-component row of the measurement Jacobian and
// P is the predicted track covariance.
//
// === Assumptions ===
//   1. Track state and covariance reflect the predicted state at z.time
//      (Tracker calls manager_.predictAll before computing these fields).
//   2. innovation_rad is already wrapped to [-π, π].
//   3. The bearing component of the measurement noise R is populated by
//      the adapter; absent (zero) R makes the state-dominance gate
//      reject the observation harmlessly.
//
// === Rationale ===
//   - Sink (not callback / std::function): matches IDatumChangeSink and
//     keeps lifetime in the composition root.
//   - Pre-computed variance and range fields (not raw state/cov): the
//     estimator has no dependency on Tracker internals or on the Jacobian
//     code, keeping the bias module decoupled.
//
// === Ways to improve / what to test next ===
//   - JPDA soft-update emit (weighted innovation across betas).
//   - Per-sensor variant of the sink so adapters can register independently.
struct BearingInnovation {
  Timestamp time;
  TrackId track_id;             // diagnostic only
  double innovation_rad{0.0};
  double variance_rad2{0.0};
  double predicted_state_var_rad2{0.0};
  double range_m{0.0};
};

class IBearingInnovationSink {
 public:
  virtual ~IBearingInnovationSink() = default;
  virtual void onBearingInnovation(const BearingInnovation& obs) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Build (no compile target yet; will be pulled in by Task 2)**

Run: `cmake --build build --target navtracker_core 2>&1 | tail -5`
Expected: clean (header is unused so far).

- [ ] **Step 3: Commit**

```bash
git add ports/IBearingInnovationSink.hpp
git commit -m "feat(ports): add IBearingInnovationSink with four-part doc"
```

---

### Task 2 — Tracker emit point

**Files:**
- Modify: `core/pipeline/Tracker.hpp`
- Modify: `core/pipeline/Tracker.cpp`

- [ ] **Step 1: Extend Tracker.hpp**

In `core/pipeline/Tracker.hpp`, after the existing `#include` lines add:

```cpp
#include "ports/IBearingInnovationSink.hpp"
```

In the `Tracker` class public section, after `processBatch(...)`, add:

```cpp
  // Optional. When non-null, every successful hard-match update on a
  // Bearing2D or RangeBearing2D measurement triggers an onBearingInnovation
  // callback computed from the PRE-update predicted state. Soft (JPDA)
  // updates are not emitted in this revision.
  void setBearingInnovationSink(IBearingInnovationSink* sink) {
    bearing_innov_sink_ = sink;
  }
```

In the private section, add the member:

```cpp
  IBearingInnovationSink* bearing_innov_sink_{nullptr};
```

- [ ] **Step 2: Extend Tracker.cpp — helper for the emit**

In `core/pipeline/Tracker.cpp`, after the `#include` block add:

```cpp
#include <cmath>

#include "core/estimation/MeasurementModels.hpp"
```

After the existing `namespace navtracker {` opening brace, add the helper function (inside an anonymous namespace):

```cpp
namespace {

void emitBearingInnovationIfApplicable(IBearingInnovationSink* sink,
                                       const Track& tr_pred,
                                       const Measurement& z) {
  if (sink == nullptr) return;
  if (z.model != MeasurementModel::Bearing2D &&
      z.model != MeasurementModel::RangeBearing2D) {
    return;
  }
  const int bidx = (z.model == MeasurementModel::Bearing2D) ? 0 : 1;
  if (z.value.size() <= bidx) return;
  const auto pred = predictMeasurement(z.model, tr_pred.state,
                                       z.sensor_position_enu);
  if (pred.H.rows() <= bidx) return;
  const double beta_pred = pred.z_pred(bidx);
  const double beta_obs  = z.value(bidx);
  const double r = wrapAngle(beta_obs - beta_pred);
  const Eigen::RowVectorXd Hb = pred.H.row(bidx);
  const double state_var =
      (Hb * tr_pred.covariance * Hb.transpose())(0, 0);
  const double R_bb =
      (z.covariance.rows() > bidx && z.covariance.cols() > bidx)
          ? z.covariance(bidx, bidx)
          : 0.0;
  const double S = state_var + R_bb;
  const double dx = tr_pred.state(0) - z.sensor_position_enu.x();
  const double dy = tr_pred.state(1) - z.sensor_position_enu.y();
  BearingInnovation obs;
  obs.time = z.time;
  obs.track_id = tr_pred.id;
  obs.innovation_rad = r;
  obs.variance_rad2 = S;
  obs.predicted_state_var_rad2 = state_var;
  obs.range_m = std::hypot(dx, dy);
  sink->onBearingInnovation(obs);
}

}  // namespace
```

- [ ] **Step 3: Wire the call in Tracker::process**

In `Tracker::process`, the existing block:

```cpp
  if (!result.matches.empty()) {
    const std::size_t ti = result.matches.front().first;
    Track& tr = manager_.mutableTracks()[ti];
    estimator_.update(tr, z);
```

Insert the emit immediately before `estimator_.update(tr, z);`:

```cpp
    emitBearingInnovationIfApplicable(bearing_innov_sink_, tr, z);
    estimator_.update(tr, z);
```

- [ ] **Step 4: Wire the call in Tracker::processBatch (hard branch only)**

In `Tracker::processBatch`, inside `else { for (const auto& m : result.matches) { ... }`:

```cpp
      Track& tr = manager_.mutableTracks()[ti];
      estimator_.update(tr, scan[mi]);
```

Insert the emit immediately before `estimator_.update(tr, scan[mi]);`:

```cpp
      emitBearingInnovationIfApplicable(bearing_innov_sink_, tr, scan[mi]);
      estimator_.update(tr, scan[mi]);
```

Do **not** emit in the soft (`if (soft) { ... }`) branch — out of scope per spec §9.

- [ ] **Step 5: Build**

Run: `cmake --build build --target navtracker_core 2>&1 | tail -5`
Expected: clean.

- [ ] **Step 6: Sanity — existing tests still pass**

Run: `ctest --test-dir build -R Tracker --output-on-failure 2>&1 | tail -10`
Expected: all existing tracker tests still green (sink is nullptr default).

- [ ] **Step 7: Commit**

```bash
git add core/pipeline/Tracker.hpp core/pipeline/Tracker.cpp
git commit -m "feat(tracker): emit BearingInnovation on hard-match bearing updates"
```

---

### Task 3 — HeadingBiasEstimator extension

**Files:**
- Modify: `core/bias/HeadingBiasEstimator.hpp`
- Modify: `core/bias/HeadingBiasEstimator.cpp`

- [ ] **Step 1: Extend the config struct**

In `core/bias/HeadingBiasEstimator.hpp`, at the end of `HeadingBiasEstimatorConfig`, before the closing brace, add:

```cpp
  // Bearing-innovation observation gates (spec §4).
  double bi_min_range_m{50.0};       // G1
  double bi_state_var_ratio_max{1.0};// G2: predicted_state_var <= k * R
  double bi_outlier_sigma{5.0};      // G3: |r| <= N * sqrt(S)
```

- [ ] **Step 2: Update class declaration**

In the same file, change:

```cpp
class HeadingBiasEstimator : public IHeadingBiasProvider {
```

to:

```cpp
class HeadingBiasEstimator : public IHeadingBiasProvider,
                             public IBearingInnovationSink {
```

Add the include near the top:

```cpp
#include "ports/IBearingInnovationSink.hpp"
```

In the public section after the existing `void observe(const AisArpaPairObservation& obs);`, add:

```cpp
  // Apply one bearing-domain innovation produced by the Tracker.
  // Predicts to obs.time, then applies the three observability gates
  // (range / state-variance-dominance / outlier). If all gates pass,
  // performs a scalar KF update on b with measurement r ~ N(b, S).
  void observe(const BearingInnovation& obs);

  // IBearingInnovationSink — dispatches to observe(BearingInnovation).
  void onBearingInnovation(const BearingInnovation& obs) override {
    observe(obs);
  }

  // Diagnostics for the bearing-innovation path.
  std::size_t acceptedBearingObs()   const { return accepted_bi_; }
  std::size_t rejectedByRange()      const { return rej_range_; }
  std::size_t rejectedByStateVar()   const { return rej_state_var_; }
  std::size_t rejectedByOutlier()    const { return rej_outlier_; }
```

In the private section after `bool has_any_update_{false};`, add:

```cpp
  std::size_t accepted_bi_{0};
  std::size_t rej_range_{0};
  std::size_t rej_state_var_{0};
  std::size_t rej_outlier_{0};
```

- [ ] **Step 3: Implement the new observe in HeadingBiasEstimator.cpp**

At the end of `core/bias/HeadingBiasEstimator.cpp`, before the closing `}  // namespace navtracker`, add:

```cpp
void HeadingBiasEstimator::observe(const BearingInnovation& obs) {
  predictTo(obs.time);

  // G1 — minimum range.
  if (obs.range_m < cfg_.bi_min_range_m) {
    ++rej_range_;
    return;
  }

  // G2 — state-variance dominance gate.
  // S = state_var + R, so R = variance_rad2 - predicted_state_var_rad2.
  const double R = obs.variance_rad2 - obs.predicted_state_var_rad2;
  if (R <= 0.0 ||
      obs.predicted_state_var_rad2 > cfg_.bi_state_var_ratio_max * R) {
    ++rej_state_var_;
    return;
  }

  // G3 — outlier gate.
  const double s = obs.variance_rad2 + p_b_;  // total residual variance
  const double sigma = std::sqrt(s);
  if (std::abs(obs.innovation_rad) > cfg_.bi_outlier_sigma * sigma) {
    ++rej_outlier_;
    return;
  }

  // Scalar KF update.
  const double y = wrapToPi(obs.innovation_rad - b_hat_);
  const double k = p_b_ / s;
  b_hat_ += k * y;
  p_b_ = (1.0 - k) * p_b_;
  last_update_ = obs.time;
  has_any_update_ = true;
  ++accepted_bi_;
}
```

- [ ] **Step 4: Build**

Run: `cmake --build build --target navtracker_core 2>&1 | tail -5`
Expected: clean.

- [ ] **Step 5: Sanity — existing bias tests still pass**

Run: `ctest --test-dir build -R HeadingBias --output-on-failure 2>&1 | tail -10`
Expected: existing AIS-pair tests still pass.

- [ ] **Step 6: Commit**

```bash
git add core/bias/HeadingBiasEstimator.hpp core/bias/HeadingBiasEstimator.cpp
git commit -m "feat(bias): add bearing-innovation observation kind with three gates"
```

---

### Task 4 — Estimator unit tests (six cases)

**Files:**
- Create: `tests/bias/test_heading_bias_bearing_innovation.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IBearingInnovationSink.hpp"

using namespace navtracker;

namespace {

BearingInnovation makeBI(double t_s, double r_rad, double state_var_rad2,
                         double R_rad2, double range_m,
                         std::uint64_t tid = 1) {
  BearingInnovation obs;
  obs.time = Timestamp::fromSeconds(t_s);
  obs.track_id = TrackId{tid};
  obs.innovation_rad = r_rad;
  obs.predicted_state_var_rad2 = state_var_rad2;
  obs.variance_rad2 = state_var_rad2 + R_rad2;
  obs.range_m = range_m;
  return obs;
}

constexpr double kBiasTrue = 0.0349;  // ~2 deg

}  // namespace

TEST(BiasObsBearingInnovation, SingleObservationAppliesScalarKfUpdate) {
  HeadingBiasEstimatorConfig cfg;
  cfg.initial_bias_rad = 0.0;
  HeadingBiasEstimator est(cfg);
  const double r = 0.02;
  const double R = 1e-4;
  const double state_var = 1e-5;  // small relative to R
  const auto obs = makeBI(/*t_s=*/1.0, r, state_var, R, /*range=*/200.0);

  const double p0 = est.varianceRad2();
  est.observe(obs);

  const double S = state_var + R;
  const double s_full = S + p0;
  const double K_expected = p0 / s_full;
  EXPECT_NEAR(est.biasRad(), K_expected * r, 1e-9);
  EXPECT_NEAR(est.varianceRad2(), (1.0 - K_expected) * p0, 1e-9);
  EXPECT_EQ(est.acceptedBearingObs(), 1u);
}

TEST(BiasObsBearingInnovation, ManyDrawsConvergeToTruthWithin3Sigma) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  std::mt19937_64 rng(1234);
  const double R = 1e-4;
  const double state_var = 1e-5;
  std::normal_distribution<double> noise(0.0, std::sqrt(state_var + R));
  const int N = 200;
  for (int i = 0; i < N; ++i) {
    const double r = kBiasTrue + noise(rng);
    est.observe(makeBI(static_cast<double>(i + 1) * 0.1,
                       r, state_var, R, /*range=*/500.0));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue),
            3.0 * std::sqrt(est.varianceRad2()));
  EXPECT_LT(est.varianceRad2(), cfg.initial_variance_rad2);
  EXPECT_GT(est.acceptedBearingObs(), 0u);
}

TEST(BiasObsBearingInnovation, RangeGateRejectsShortRange) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double b_before = est.biasRad();
  const double p_before = est.varianceRad2();
  est.observe(makeBI(1.0, /*r=*/0.05, /*sv=*/1e-5, /*R=*/1e-4,
                     /*range=*/10.0));  // below default 50 m
  EXPECT_EQ(est.biasRad(), b_before);
  EXPECT_GE(est.varianceRad2(), p_before);  // possibly grew by predict
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByRange(), 1u);
}

TEST(BiasObsBearingInnovation, StateVarGateRejectsStateDominated) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double R = 1e-4;
  const double state_var = 10.0 * R;  // 10x R, dominates
  est.observe(makeBI(1.0, 0.02, state_var, R, /*range=*/500.0));
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByStateVar(), 1u);
}

TEST(BiasObsBearingInnovation, OutlierGateRejectsHugeInnovation) {
  HeadingBiasEstimatorConfig cfg;
  HeadingBiasEstimator est(cfg);
  const double R = 1e-4;
  const double state_var = 1e-5;
  const double S = state_var + R;
  const double sigma = std::sqrt(S + est.varianceRad2());
  const double r = 10.0 * sigma;  // way outside 5σ default
  est.observe(makeBI(1.0, r, state_var, R, /*range=*/500.0));
  EXPECT_EQ(est.acceptedBearingObs(), 0u);
  EXPECT_EQ(est.rejectedByOutlier(), 1u);
}

TEST(BiasObsBearingInnovation, NearPiInnovationApplies) {
  // Bias estimator should treat the wrapped value linearly (no further
  // wrap inside observe()) — pass r close to but inside (-π, π].
  HeadingBiasEstimatorConfig cfg;
  cfg.initial_bias_rad = 0.0;
  cfg.initial_variance_rad2 = 1.0;  // wide prior so K~1, easy to detect
  HeadingBiasEstimator est(cfg);
  const double r_in = 3.0;  // < π, well outside outlier gate by default
  cfg.bi_outlier_sigma = 1000.0;  // unused; only affects new estimator
  HeadingBiasEstimator est_loose({.initial_bias_rad = 0.0,
                                  .initial_variance_rad2 = 1.0,
                                  .bi_outlier_sigma = 1000.0});
  est_loose.observe(makeBI(1.0, r_in, 1e-5, 1e-4, /*range=*/500.0));
  EXPECT_GT(est_loose.biasRad(), 0.5);  // moved toward r_in
  EXPECT_EQ(est_loose.acceptedBearingObs(), 1u);
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, find the test list line for `tests/bias/test_heading_bias_estimator.cpp` and add `tests/bias/test_heading_bias_bearing_innovation.cpp` on the next line.

- [ ] **Step 3: Build and run**

Run:
```
cmake --build build --target navtracker_tests 2>&1 | tail -5
ctest --test-dir build -R BiasObsBearingInnovation --output-on-failure 2>&1 | tail -15
```
Expected: 6/6 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/bias/test_heading_bias_bearing_innovation.cpp CMakeLists.txt
git commit -m "test(bias): unit tests for bearing-innovation observation kind"
```

---

### Task 5 — Tracker emission tests

**Files:**
- Create: `tests/pipeline/test_tracker_bearing_innovation_emit.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IBearingInnovationSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public IBearingInnovationSink {
 public:
  std::vector<BearingInnovation> events;
  void onBearingInnovation(const BearingInnovation& obs) override {
    events.push_back(obs);
  }
};

struct Bench {
  std::shared_ptr<ConstantVelocity2D> motion =
      std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est{motion, 5.0};
  GnnAssociator assoc{300.0};  // generous gate for Bearing2D matches
  TrackManager mgr{1, 4};
  Tracker tracker{est, assoc, mgr, 60.0};
  RecordingSink sink;
  Bench() { tracker.setBearingInnovationSink(&sink); }
};

Measurement seedPosition(double x, double y, double t_s) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "seed";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 100.0;
  return m;
}

Measurement bearing(double beta_rad, double t_s, double sigma_b_rad,
                    const Eigen::Vector2d& sensor_enu = Eigen::Vector2d::Zero()) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::EoIr;
  m.source_id = "eoir";
  m.model = MeasurementModel::Bearing2D;
  Eigen::VectorXd v(1);
  v(0) = beta_rad;
  m.value = v;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma_b_rad * sigma_b_rad;
  m.covariance = R;
  m.sensor_position_enu = sensor_enu;
  return m;
}

}  // namespace

TEST(TrackerBearingInnovationEmit, Position2DDoesNotEmit) {
  Bench b;
  b.tracker.process(seedPosition(100.0, 0.0, 1.0));
  b.tracker.process(seedPosition(101.0, 0.0, 2.0));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, NullSinkIsSafe) {
  Bench b;
  b.tracker.setBearingInnovationSink(nullptr);
  b.tracker.process(seedPosition(100.0, 0.0, 1.0));
  b.tracker.process(bearing(0.0, 2.0, 0.01));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, InitiationDoesNotEmit) {
  Bench b;
  // First measurement with no existing tracks → initiate path, no match.
  b.tracker.process(bearing(0.0, 1.0, 0.01));
  EXPECT_TRUE(b.sink.events.empty());
}

TEST(TrackerBearingInnovationEmit, Bearing2DEmitsCorrectFields) {
  Bench b;
  // Seed a track far enough that the bearing is well-defined.
  b.tracker.process(seedPosition(500.0, 0.0, 1.0));
  // Match a Bearing2D measurement at the same predicted location.
  // Predicted bearing should be ~0; observed 0.01 rad.
  b.tracker.process(bearing(0.01, 2.0, 0.005));
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  EXPECT_NEAR(e.innovation_rad, 0.01, 5e-3);  // close to obs - 0
  EXPECT_GT(e.variance_rad2, 0.0);
  EXPECT_GE(e.predicted_state_var_rad2, 0.0);
  EXPECT_LE(e.predicted_state_var_rad2, e.variance_rad2);
  EXPECT_NEAR(e.range_m, 500.0, 50.0);  // post-predict drift small
}

TEST(TrackerBearingInnovationEmit, RangeBearing2DEmitsBearingComponentOnly) {
  Bench b;
  b.tracker.process(seedPosition(500.0, 0.0, 1.0));
  // Construct a RangeBearing2D measurement.
  Measurement m;
  m.time = Timestamp::fromSeconds(2.0);
  m.sensor = SensorKind::ArpaTtm;
  m.source_id = "rb";
  m.model = MeasurementModel::RangeBearing2D;
  m.value = Eigen::Vector2d(500.0, 0.01);  // (range, bearing)
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = 25.0;       // sigma_range = 5 m
  R(1, 1) = 1e-4;       // sigma_bearing = 0.01 rad
  m.covariance = R;
  b.tracker.process(m);
  ASSERT_EQ(b.sink.events.size(), 1u);
  const auto& e = b.sink.events.front();
  // Predicted bearing ~0 since track at (500,0); observed 0.01.
  EXPECT_NEAR(e.innovation_rad, 0.01, 5e-3);
  EXPECT_NEAR(e.range_m, 500.0, 50.0);
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, find `tests/pipeline/test_tracker.cpp` and add `tests/pipeline/test_tracker_bearing_innovation_emit.cpp` on the next line.

- [ ] **Step 3: Build and run**

Run:
```
cmake --build build --target navtracker_tests 2>&1 | tail -5
ctest --test-dir build -R TrackerBearingInnovationEmit --output-on-failure 2>&1 | tail -15
```
Expected: 5/5 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/pipeline/test_tracker_bearing_innovation_emit.cpp CMakeLists.txt
git commit -m "test(tracker): emission tests for BearingInnovation sink"
```

---

### Task 6 — Headline convergence scenario

**Files:**
- Create: `tests/scenario/test_bearing_bias_convergence.cpp`

- [ ] **Step 1: Write the test file**

```cpp
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"

using namespace navtracker;

namespace {

constexpr double kBiasTrue = 0.0349;  // 2 degrees

Scenario injectBias(Scenario s, double b_rad) {
  for (auto& m : s.measurements) {
    if (m.model == MeasurementModel::Bearing2D && m.value.size() >= 1) {
      m.value(0) += b_rad;
    } else if (m.model == MeasurementModel::RangeBearing2D &&
               m.value.size() >= 2) {
      m.value(1) += b_rad;
    }
  }
  return s;
}

}  // namespace

TEST(BearingBiasConvergence, BearingOnlyMovingSensorConvergesToWithinHalfDeg) {
  std::vector<double> times;
  for (int i = 0; i < 200; ++i) times.push_back(0.5 * (i + 1));
  const Scenario raw = buildBearingOnlyMovingSensorScenario(
      /*target_position=*/Eigen::Vector2d(800.0, 200.0),
      /*sensor_start=*/Eigen::Vector2d(0.0, 0.0),
      /*sensor_velocity=*/Eigen::Vector2d(5.0, 0.0),
      times,
      /*initial_position_std_m=*/200.0,
      /*bearing_std_rad=*/0.01,
      /*seed=*/7);
  const Scenario s = injectBias(raw, kBiasTrue);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(500.0);
  TrackManager mgr(1, 6);
  Tracker tracker(est, assoc, mgr, 200.0);

  HeadingBiasEstimator bias({});
  tracker.setBearingInnovationSink(&bias);

  for (const auto& m : s.measurements) tracker.process(m);

  const double err = std::abs(bias.biasRad() - kBiasTrue);
  EXPECT_LT(err, 0.5 * 3.14159265358979323846 / 180.0)
      << "bias=" << bias.biasRad() << " true=" << kBiasTrue;
  EXPECT_GT(bias.acceptedBearingObs(), 10u);
  EXPECT_TRUE(bias.current().is_published);
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, after the `test_reorder_buffer_e2e.cpp` line, add `tests/scenario/test_bearing_bias_convergence.cpp`.

- [ ] **Step 3: Build and run**

Run:
```
cmake --build build --target navtracker_tests 2>&1 | tail -5
ctest --test-dir build -R BearingBiasConvergence --output-on-failure 2>&1 | tail -15
```
Expected: 1/1 pass. If it fails, inspect the bias trajectory across the run (you may need to print `bias.biasRad()` every N iterations) before adjusting tolerances — defaults should converge well within 0.5° over 200 observations.

- [ ] **Step 4: Commit**

```bash
git add tests/scenario/test_bearing_bias_convergence.cpp CMakeLists.txt
git commit -m "test(scenario): bearing bias converges to truth in zero-AIS scene"
```

---

### Task 7 — v1 spec touchup + four-part doc

**Files:**
- Modify: `docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md`
- Modify: `core/bias/HeadingBiasEstimator.hpp` (header doc append)

- [ ] **Step 1: Update v1 spec**

Open `docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md` and:

- In `Out of scope` (line ~23), update the bullet to "Multi-track bearing-innovation observer **(v2, landed 2026-06-04 — see `2026-06-04-multi-track-bearing-bias-observer-design.md`)**".
- In §11 (Ways to improve), move the "Multi-track bearing-innovation observer" item from §11.1 into a new `## Landed` section above §11, with the same cross-reference.

- [ ] **Step 2: Append v2 doc block to HeadingBiasEstimator.hpp**

Find the existing four-part doc block above `class HeadingBiasEstimator` (if absent, add one summarizing v1 plus the new v2 paragraph). Append:

```cpp
// === v2 observation kind (bearing innovations from Tracker) ===
//
// The estimator also implements IBearingInnovationSink. When wired into a
// Tracker, every successful hard-match update on Bearing2D or
// RangeBearing2D produces a scalar measurement
//   r ~ N(b, S),   S = H·P·Hᵀ + R
// of the heading bias b, where (β̂, H, P) come from the PRE-update
// predicted track state and R is the measurement noise. Sequentially
// fusing innovations from independent tracks is mathematically correct
// because the projected state errors are conditionally independent given
// b (independent CV processes, independent associations).
//
// Three observability gates protect against state-error contamination
// (predicted_state_var <= k * R), short range (range >= min_range_m),
// and outliers (|r| <= N * sqrt(S + P_b)). See spec §4 for defaults.
```

- [ ] **Step 3: Run full suite**

Run: `ctest --test-dir build --output-on-failure 2>&1 | tail -5`
Expected: previous count + ~12 new tests across the three new files, all green.

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md core/bias/HeadingBiasEstimator.hpp
git commit -m "docs(bias): cross-reference v2 observer in v1 spec and header"
```

---

### Task 8 — Final sweep

- [ ] **Step 1: Acceptance checklist**

Re-read spec §14 and confirm:

- Full suite green (target: previous + ~12 new tests).
- §11.3 headline assertions pass.
- `IHeadingBiasProvider.current().is_published == true` in the bearing-only scene.
- No changes to `IEstimator`, adapters, or `Measurement`.
- Four-part doc on `IBearingInnovationSink.hpp` and v2 block appended to `HeadingBiasEstimator.hpp`.

- [ ] **Step 2: No-op commit if needed**

Only if a gap surfaced.
