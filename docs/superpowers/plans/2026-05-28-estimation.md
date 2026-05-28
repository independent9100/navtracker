# Estimation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the pluggable state-estimation layer — a constant-velocity motion model and an EKF that predicts tracks forward in time and updates them from linear (position) and nonlinear (range/bearing) measurements.

**Architecture:** Defines two domain ports, `IMotionModel` and `IEstimator` (in `ports/`), with concrete `ConstantVelocity2D` and `EkfEstimator` implementations in `core/estimation/`. Pure domain code, no I/O. Operates on the `Track`/`Measurement` types from the foundation plan. Strategies are swappable (spec D5/D6).

**Tech Stack:** C++17 · Eigen 3.4 · GoogleTest. Builds on plan 1 (already on `master`).

This is plan 2 of 6. Prereq: plan 1 (foundation) is merged — `navtracker_core`, `core/types/*`, `core/geo/*` exist and 17 tests pass. Design reference: `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md` §11.

**Documentation standard (CLAUDE.md):** Each algorithm carries Math / Assumptions / Rationale / Ways-to-improve. Concise notes appear in each task here; Task 5 writes the consolidated `docs/algorithms/estimation.md`.

---

## File Structure

```
ports/IMotionModel.hpp                  interface: F(dt), Q(dt), stateDim
ports/IEstimator.hpp                    interface: predict(track,to), update(track,z)
core/estimation/ConstantVelocity2D.hpp/.cpp   CV model, state [px,py,vx,vy]
core/estimation/MeasurementModels.hpp/.cpp     h(x), Jacobian H, angle-wrapped residual
core/estimation/EkfEstimator.hpp/.cpp          EKF predict/update + initiate
docs/algorithms/estimation.md           full math/assumptions/rationale/improve
tests/estimation/test_constant_velocity.cpp
tests/estimation/test_measurement_models.cpp
tests/estimation/test_ekf_estimator.cpp
```

Ports are pure-virtual headers (no `.cpp`). The three estimation `.cpp` files join `navtracker_core`.

## Current root CMakeLists.txt (post plan 1, for reference)

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
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
```

Build/test commands (from repo root):
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release   # after CMakeLists edits
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## Task 1: Constant-velocity motion model

**Math.** State `x = [px, py, vx, vy]ᵀ` (ENU m, m/s). `F(dt) = I₄` with `F(0,2)=F(1,3)=dt`. Process noise from continuous white-noise acceleration, scalar PSD `q`, per axis:
`Q(dt)` blocks per axis = `q·[[dt³/3, dt²/2],[dt²/2, dt]]`, placed at (pos,pos),(pos,vel),(vel,pos),(vel,vel).
**Assumptions.** ~Constant velocity between updates; maneuvers absorbed by `q`; axes independent; 2D surface motion.
**Rationale.** Standard robust baseline; nonlinearity confined to measurement models (spec §11.1).
**Ways to improve.** IMM (CV + coordinated-turn); per-class `q` tuning; 3D.

**Files:**
- Create: `ports/IMotionModel.hpp`
- Create: `core/estimation/ConstantVelocity2D.hpp`, `core/estimation/ConstantVelocity2D.cpp`
- Test: `tests/estimation/test_constant_velocity.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/estimation/test_constant_velocity.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"

using navtracker::ConstantVelocity2D;

TEST(ConstantVelocity2D, TransitionPropagatesPosition) {
  ConstantVelocity2D model(1.0);
  const Eigen::MatrixXd f = model.transitionMatrix(2.0);
  const Eigen::Vector4d x(0.0, 0.0, 3.0, -1.0);
  const Eigen::Vector4d xp = f * x;
  EXPECT_DOUBLE_EQ(xp(0), 6.0);
  EXPECT_DOUBLE_EQ(xp(1), -2.0);
  EXPECT_DOUBLE_EQ(xp(2), 3.0);
  EXPECT_DOUBLE_EQ(xp(3), -1.0);
}

TEST(ConstantVelocity2D, ProcessNoiseMatchesWhiteAccelModel) {
  ConstantVelocity2D model(1.0);
  const Eigen::MatrixXd q = model.processNoise(2.0);
  EXPECT_NEAR(q(0, 0), 8.0 / 3.0, 1e-12);  // dt^3/3
  EXPECT_NEAR(q(0, 2), 2.0, 1e-12);        // dt^2/2
  EXPECT_NEAR(q(2, 2), 2.0, 1e-12);        // dt
  EXPECT_TRUE(q.isApprox(q.transpose()));
}

TEST(ConstantVelocity2D, StateDimIsFour) {
  ConstantVelocity2D model(1.0);
  EXPECT_EQ(model.stateDim(), 4);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/estimation/ConstantVelocity2D.cpp` to the `navtracker_core` source list (after `core/geo/Datum.cpp`). Add `tests/estimation/test_constant_velocity.cpp` to the `navtracker_tests` source list (after `tests/types/test_track.cpp`).

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/estimation/ConstantVelocity2D.hpp` not found.

- [ ] **Step 4: Create `ports/IMotionModel.hpp`**

```cpp
#pragma once

#include <Eigen/Core>

namespace navtracker {

// Linear-Gaussian motion model for a fixed state layout: supplies the state
// transition matrix F(dt) and process-noise covariance Q(dt).
class IMotionModel {
 public:
  virtual ~IMotionModel() = default;
  virtual int stateDim() const = 0;
  virtual Eigen::MatrixXd transitionMatrix(double dt) const = 0;  // F(dt)
  virtual Eigen::MatrixXd processNoise(double dt) const = 0;      // Q(dt)
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/estimation/ConstantVelocity2D.hpp`**

```cpp
#pragma once

#include "ports/IMotionModel.hpp"

namespace navtracker {

// 2D constant-velocity model. State = [px, py, vx, vy] in ENU (m, m/s).
// Process noise from the continuous white-noise-acceleration model with
// scalar acceleration PSD q, applied independently per axis.
class ConstantVelocity2D : public IMotionModel {
 public:
  explicit ConstantVelocity2D(double accel_psd);

  int stateDim() const override { return 4; }
  Eigen::MatrixXd transitionMatrix(double dt) const override;
  Eigen::MatrixXd processNoise(double dt) const override;

 private:
  double q_;
};

}  // namespace navtracker
```

- [ ] **Step 6: Create `core/estimation/ConstantVelocity2D.cpp`**

```cpp
#include "core/estimation/ConstantVelocity2D.hpp"

namespace navtracker {

ConstantVelocity2D::ConstantVelocity2D(double accel_psd) : q_(accel_psd) {}

Eigen::MatrixXd ConstantVelocity2D::transitionMatrix(double dt) const {
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(4, 4);
  f(0, 2) = dt;
  f(1, 3) = dt;
  return f;
}

Eigen::MatrixXd ConstantVelocity2D::processNoise(double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double a = q_ * dt3 / 3.0;
  const double b = q_ * dt2 / 2.0;
  const double c = q_ * dt;
  Eigen::MatrixXd qm = Eigen::MatrixXd::Zero(4, 4);
  qm(0, 0) = a; qm(0, 2) = b;
  qm(1, 1) = a; qm(1, 3) = b;
  qm(2, 0) = b; qm(2, 2) = c;
  qm(3, 1) = b; qm(3, 3) = c;
  return qm;
}

}  // namespace navtracker
```

- [ ] **Step 7: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `ConstantVelocity2D.*` tests pass.

- [ ] **Step 8: Commit**

```bash
git add ports/IMotionModel.hpp core/estimation/ConstantVelocity2D.hpp core/estimation/ConstantVelocity2D.cpp tests/estimation/test_constant_velocity.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(estimation): add IMotionModel and ConstantVelocity2D"
```

---

## Task 2: Measurement models (h, Jacobian, angle-wrapped residual)

**Math.** Position2D: `h(x)=[px,py]`, `H=[[1,0,0,0],[0,1,0,0]]`. PositionVelocity2D: `h(x)=[px,py,vx,vy]`, `H=I₄`. RangeBearing2D (sensor at frame origin): `r=√(px²+py²)`, `β=atan2(py,px)`; `H=[[px/r, py/r, 0, 0], [−py/r², px/r², 0, 0]]`. Bearing residuals wrapped to (−π, π].
**Assumptions.** Range/bearing taken relative to the ENU origin (own-ship offset handled later in normalization, plan 5); `r>0` (guard at 1e-6); Gaussian noise.
**Rationale.** EKF linearization adequate for mild range/bearing nonlinearity (spec §11.2); cheaper than UKF/particle.
**Ways to improve.** UKF/particle for strong nonlinearity or bearing-only; per-sensor R calibration.

**Files:**
- Create: `core/estimation/MeasurementModels.hpp`, `core/estimation/MeasurementModels.cpp`
- Test: `tests/estimation/test_measurement_models.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/estimation/test_measurement_models.cpp`:

```cpp
#include <cmath>

#include <gtest/gtest.h>
#include "core/estimation/MeasurementModels.hpp"

using navtracker::MeasurementModel;
using navtracker::measurementResidual;
using navtracker::predictMeasurement;
using navtracker::wrapAngle;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(MeasurementModels, Position2DLinear) {
  const Eigen::Vector4d x(3.0, 4.0, 1.0, -2.0);
  const auto p = predictMeasurement(MeasurementModel::Position2D, x);
  EXPECT_EQ(p.z_pred.size(), 2);
  EXPECT_DOUBLE_EQ(p.z_pred(0), 3.0);
  EXPECT_DOUBLE_EQ(p.z_pred(1), 4.0);
  EXPECT_EQ(p.H.rows(), 2);
  EXPECT_EQ(p.H.cols(), 4);
  EXPECT_DOUBLE_EQ(p.H(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(p.H(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(p.H(0, 2), 0.0);
}

TEST(MeasurementModels, RangeBearingValuesAndJacobian) {
  const Eigen::Vector4d x(3.0, 4.0, 0.0, 0.0);
  const auto p = predictMeasurement(MeasurementModel::RangeBearing2D, x);
  EXPECT_NEAR(p.z_pred(0), 5.0, 1e-12);
  EXPECT_NEAR(p.z_pred(1), std::atan2(4.0, 3.0), 1e-12);
  EXPECT_NEAR(p.H(0, 0), 0.6, 1e-12);
  EXPECT_NEAR(p.H(0, 1), 0.8, 1e-12);
  EXPECT_NEAR(p.H(1, 0), -0.16, 1e-12);
  EXPECT_NEAR(p.H(1, 1), 0.12, 1e-12);
}

TEST(MeasurementModels, JacobianMatchesFiniteDifference) {
  const Eigen::Vector4d x(120.0, -50.0, 2.0, 1.0);
  const auto p = predictMeasurement(MeasurementModel::RangeBearing2D, x);
  const double eps = 1e-6;
  for (int j = 0; j < 4; ++j) {
    Eigen::Vector4d xp = x;
    xp(j) += eps;
    const auto pp = predictMeasurement(MeasurementModel::RangeBearing2D, xp);
    const Eigen::Vector2d num = (pp.z_pred - p.z_pred) / eps;
    EXPECT_NEAR(p.H(0, j), num(0), 1e-3);
    EXPECT_NEAR(p.H(1, j), num(1), 1e-3);
  }
}

TEST(MeasurementModels, BearingResidualWrapsAcrossPi) {
  const Eigen::Vector2d z(10.0, -3.0);
  const Eigen::Vector2d zpred(10.0, 3.0);
  const Eigen::VectorXd y =
      measurementResidual(MeasurementModel::RangeBearing2D, z, zpred);
  EXPECT_NEAR(y(0), 0.0, 1e-12);
  EXPECT_NEAR(y(1), -6.0 + 2.0 * kPi, 1e-9);
}

TEST(MeasurementModels, WrapAngleRange) {
  EXPECT_NEAR(wrapAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapAngle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(wrapAngle(-3.0 * kPi), kPi, 1e-9);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/estimation/MeasurementModels.cpp` to `navtracker_core`. Add `tests/estimation/test_measurement_models.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/estimation/MeasurementModels.hpp` not found.

- [ ] **Step 4: Create `core/estimation/MeasurementModels.hpp`**

```cpp
#pragma once

#include <Eigen/Core>

#include "core/types/Ids.hpp"  // MeasurementModel

namespace navtracker {

// Predicted measurement h(x) and its Jacobian H = dh/dx at a state.
struct MeasurementPrediction {
  Eigen::VectorXd z_pred;
  Eigen::MatrixXd H;
};

// Wrap an angle in radians to the interval (-pi, pi].
double wrapAngle(double radians);

// h(x) and H for `model` evaluated at state = [px, py, vx, vy].
MeasurementPrediction predictMeasurement(MeasurementModel model,
                                         const Eigen::VectorXd& state);

// Measurement residual z - h(x); bearing component is angle-wrapped for
// the RangeBearing2D model.
Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred);

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/estimation/MeasurementModels.cpp`**

```cpp
#include "core/estimation/MeasurementModels.hpp"

#include <cmath>

namespace navtracker {
namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double wrapAngle(double radians) {
  double a = std::fmod(radians + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

MeasurementPrediction predictMeasurement(MeasurementModel model,
                                         const Eigen::VectorXd& state) {
  const double px = state(0);
  const double py = state(1);
  MeasurementPrediction out;
  switch (model) {
    case MeasurementModel::Position2D: {
      out.z_pred = Eigen::Vector2d(px, py);
      out.H = Eigen::MatrixXd::Zero(2, 4);
      out.H(0, 0) = 1.0;
      out.H(1, 1) = 1.0;
      break;
    }
    case MeasurementModel::PositionVelocity2D: {
      out.z_pred = state.head<4>();
      out.H = Eigen::MatrixXd::Identity(4, 4);
      break;
    }
    case MeasurementModel::RangeBearing2D: {
      double r = std::hypot(px, py);
      if (r < 1e-6) r = 1e-6;
      out.z_pred = Eigen::Vector2d(r, std::atan2(py, px));
      out.H = Eigen::MatrixXd::Zero(2, 4);
      out.H(0, 0) = px / r;
      out.H(0, 1) = py / r;
      out.H(1, 0) = -py / (r * r);
      out.H(1, 1) = px / (r * r);
      break;
    }
  }
  return out;
}

Eigen::VectorXd measurementResidual(MeasurementModel model,
                                    const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& z_pred) {
  Eigen::VectorXd y = z - z_pred;
  if (model == MeasurementModel::RangeBearing2D) {
    y(1) = wrapAngle(y(1));
  }
  return y;
}

}  // namespace navtracker
```

- [ ] **Step 6: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `MeasurementModels.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add core/estimation/MeasurementModels.hpp core/estimation/MeasurementModels.cpp tests/estimation/test_measurement_models.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(estimation): add measurement models with Jacobians and angle wrapping"
```

---

## Task 3: EKF estimator (predict + update)

**Math.** Predict: `dt = to − last_update`; `x ← F(dt)x`; `P ← F P Fᵀ + Q(dt)`. Update: `(z_pred,H)=h(x)`; residual `y = z − z_pred` (bearing wrapped); `S = H P Hᵀ + R`; `K = P Hᵀ S⁻¹`; `x ← x + K y`; `P ← (I − K H) P`.
**Assumptions.** `update` assumes the state was already predicted to `z.time` (the pipeline predicts then updates); `dt ≤ 0` predict is a no-op; Gaussian noise; `R` supplied by the measurement.
**Rationale.** Standard EKF; small dense matrices so direct inverse is fine (spec §11).
**Ways to improve.** Joseph-form covariance update for numerical stability; UKF/IMM; outlier gating before update.

**Files:**
- Create: `ports/IEstimator.hpp`
- Create: `core/estimation/EkfEstimator.hpp`, `core/estimation/EkfEstimator.cpp`
- Test: `tests/estimation/test_ekf_estimator.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/estimation/test_ekf_estimator.cpp`:

```cpp
#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;
using navtracker::Track;

TEST(EkfEstimator, PredictAdvancesPositionAndGrowsCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(0.0, 0.0, 2.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  ekf.predict(t, Timestamp::fromSeconds(3.0));
  EXPECT_DOUBLE_EQ(t.state(0), 6.0);
  EXPECT_DOUBLE_EQ(t.state(1), 0.0);
  EXPECT_DOUBLE_EQ(t.state(2), 2.0);
  EXPECT_GT(t.covariance(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 3.0);
}

TEST(EkfEstimator, PositionUpdatePullsStateAndShrinksCovariance) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(10.0);
  t.state = Eigen::Vector4d::Zero();
  t.covariance = Eigen::Matrix4d::Identity() * 100.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(10.0);
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(10.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  ekf.update(t, z);
  EXPECT_GT(t.state(0), 9.0);
  EXPECT_LT(t.state(0), 10.0);
  EXPECT_NEAR(t.state(1), 0.0, 1e-9);
  EXPECT_LT(t.covariance(0, 0), 100.0);
}

TEST(EkfEstimator, RangeBearingUpdateOnConsistentMeasurementIsStable) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model);
  Track t;
  t.last_update = Timestamp::fromSeconds(0.0);
  t.state = Eigen::Vector4d(3.0, 4.0, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 10.0;
  Measurement z;
  z.time = Timestamp::fromSeconds(0.0);
  z.model = MeasurementModel::RangeBearing2D;
  z.value = Eigen::Vector2d(5.0, std::atan2(4.0, 3.0));
  z.covariance = Eigen::Matrix2d::Identity() * 0.01;
  ekf.update(t, z);
  EXPECT_NEAR(t.state(0), 3.0, 1e-6);
  EXPECT_NEAR(t.state(1), 4.0, 1e-6);
  EXPECT_LT(t.covariance(0, 0), 10.0);
}
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `core/estimation/EkfEstimator.cpp` to `navtracker_core`. Add `tests/estimation/test_ekf_estimator.cpp` to `navtracker_tests`.

- [ ] **Step 3: Verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/estimation/EkfEstimator.hpp` not found.

- [ ] **Step 4: Create `ports/IEstimator.hpp`**

```cpp
#pragma once

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Recursive state estimator strategy. Implementations advance and correct a
// track's kinematic state/covariance.
class IEstimator {
 public:
  virtual ~IEstimator() = default;

  // Advance the track's state and covariance to time `to`.
  virtual void predict(Track& track, Timestamp to) const = 0;

  // Fold a measurement into the track. Assumes the track was already
  // predicted to z.time.
  virtual void update(Track& track, const Measurement& z) const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/estimation/EkfEstimator.hpp`**

```cpp
#pragma once

#include <memory>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Extended Kalman Filter. Linear-Gaussian prediction via the supplied motion
// model; nonlinear measurement updates via Jacobian linearization.
class EkfEstimator : public IEstimator {
 public:
  explicit EkfEstimator(std::shared_ptr<const IMotionModel> motion);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

 private:
  std::shared_ptr<const IMotionModel> motion_;
};

}  // namespace navtracker
```

- [ ] **Step 6: Create `core/estimation/EkfEstimator.cpp`**

```cpp
#include "core/estimation/EkfEstimator.hpp"

#include <utility>

#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"

namespace navtracker {

EkfEstimator::EkfEstimator(std::shared_ptr<const IMotionModel> motion)
    : motion_(std::move(motion)) {}

void EkfEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const Eigen::MatrixXd f = motion_->transitionMatrix(dt);
  const Eigen::MatrixXd q = motion_->processNoise(dt);
  track.state = f * track.state;
  track.covariance = f * track.covariance * f.transpose() + q;
  track.last_update = to;
}

void EkfEstimator::update(Track& track, const Measurement& z) const {
  const MeasurementPrediction pred = predictMeasurement(z.model, track.state);
  const Eigen::VectorXd y = measurementResidual(z.model, z.value, pred.z_pred);
  const Eigen::MatrixXd& h = pred.H;
  const Eigen::MatrixXd s = h * track.covariance * h.transpose() + z.covariance;
  const Eigen::MatrixXd k = track.covariance * h.transpose() * s.inverse();
  track.state += k * y;
  const auto n = track.state.size();
  const Eigen::MatrixXd id = Eigen::MatrixXd::Identity(n, n);
  track.covariance = (id - k * h) * track.covariance;
  track.last_update = z.time;
}

}  // namespace navtracker
```

- [ ] **Step 7: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `EkfEstimator.*` tests pass.

- [ ] **Step 8: Commit**

```bash
git add ports/IEstimator.hpp core/estimation/EkfEstimator.hpp core/estimation/EkfEstimator.cpp tests/estimation/test_ekf_estimator.cpp CMakeLists.txt
git -c commit.gpgsign=false commit -m "feat(estimation): add EKF estimator predict and update"
```

---

## Task 4: EKF track initiation from a position measurement

**Math.** Seed `x = [zx, zy, 0, 0]`; `P = blockdiag(R_pos, σ_v²·I₂)` where `R_pos` is the measurement's 2×2 covariance and `σ_v` is an assumed initial speed std.
**Assumptions.** Initiation supports position-type measurements (uses the first two value components as position); velocity unknown → zero mean with large variance. Range/bearing initiation (needs conversion) is out of scope here.
**Rationale.** A one-point initiator is the simplest seed; the large velocity variance lets the first updates pull velocity in.
**Ways to improve.** Two-point differencing for velocity seed; per-sensor initial-speed priors.

**Files:**
- Modify: `core/estimation/EkfEstimator.hpp` (add ctor param + `initiate`)
- Modify: `core/estimation/EkfEstimator.cpp`
- Test: `tests/estimation/test_ekf_estimator.cpp` (append)

- [ ] **Step 1: Add the failing test**

Append to `tests/estimation/test_ekf_estimator.cpp`:

```cpp
TEST(EkfEstimator, InitiateSeedsStateFromPositionMeasurement) {
  auto model = std::make_shared<ConstantVelocity2D>(1.0);
  const EkfEstimator ekf(model, 8.0);
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.model = MeasurementModel::Position2D;
  z.source_id = "ais";
  z.value = Eigen::Vector2d(100.0, -50.0);
  z.covariance = Eigen::Matrix2d::Identity() * 9.0;
  z.hints.mmsi = 211000000u;
  const Track t = ekf.initiate(z);
  EXPECT_EQ(t.status, navtracker::TrackStatus::Tentative);
  EXPECT_DOUBLE_EQ(t.state(0), 100.0);
  EXPECT_DOUBLE_EQ(t.state(1), -50.0);
  EXPECT_DOUBLE_EQ(t.state(2), 0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 9.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2), 64.0);  // 8^2
  ASSERT_TRUE(t.attributes.mmsi.has_value());
  EXPECT_EQ(*t.attributes.mmsi, 211000000u);
  EXPECT_EQ(t.contributing_sources.size(), 1u);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 5.0);
}
```

- [ ] **Step 2: Verify it fails**

Run:
```bash
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `EkfEstimator` has no member `initiate`, and the two-argument constructor does not exist.

- [ ] **Step 3: Update `core/estimation/EkfEstimator.hpp`**

Replace the public section so the constructor takes an initial speed std and `initiate` is declared:

```cpp
  EkfEstimator(std::shared_ptr<const IMotionModel> motion,
               double init_speed_std = 10.0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;

  // Create a new Tentative track seeded from a position-type measurement.
  Track initiate(const Measurement& z) const;
```

And add a second private member below `motion_`:

```cpp
  double init_speed_std_;
```

- [ ] **Step 4: Update `core/estimation/EkfEstimator.cpp`**

Replace the constructor definition and add `initiate`. The constructor becomes:

```cpp
EkfEstimator::EkfEstimator(std::shared_ptr<const IMotionModel> motion,
                           double init_speed_std)
    : motion_(std::move(motion)), init_speed_std_(init_speed_std) {}
```

Add this method (after `update`):

```cpp
Track EkfEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = z.value(0);
  x(1) = z.value(1);
  t.state = x;

  Eigen::Matrix4d p = Eigen::Matrix4d::Zero();
  p(0, 0) = z.covariance(0, 0);
  p(0, 1) = z.covariance(0, 1);
  p(1, 0) = z.covariance(1, 0);
  p(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  p(2, 2) = vv;
  p(3, 3) = vv;
  t.covariance = p;

  if (z.hints.mmsi.has_value()) {
    t.attributes.mmsi = z.hints.mmsi;
  }
  t.contributing_sources.push_back(z.source_id);
  return t;
}
```

- [ ] **Step 5: Verify it passes**

Run `cmake --build build && ctest --test-dir build --output-on-failure`. Expected: all `EkfEstimator.*` tests pass (including initiation); full suite green.

- [ ] **Step 6: Commit**

```bash
git add core/estimation/EkfEstimator.hpp core/estimation/EkfEstimator.cpp tests/estimation/test_ekf_estimator.cpp
git -c commit.gpgsign=false commit -m "feat(estimation): add EKF one-point track initiation"
```

---

## Task 5: Estimation algorithm documentation

Write the consolidated algorithm reference following the CLAUDE.md four-part standard. No build/test step — documentation only.

**Files:**
- Create: `docs/algorithms/estimation.md`

- [ ] **Step 1: Create `docs/algorithms/estimation.md`**

```markdown
# Estimation Algorithms

Baseline state estimation for navtracker. Follows the project documentation
standard: Math / Assumptions / Rationale / Ways to improve. Cross-reference:
design spec section 11.

## 1. Constant-velocity motion model (`ConstantVelocity2D`)

**Math.** State `x = [px, py, vx, vy]ᵀ` in ENU (m, m/s).
`F(dt) = I₄` with `F(0,2)=F(1,3)=dt`, so `[px+vx·dt, py+vy·dt, vx, vy]`.
Process noise (continuous white-noise acceleration, scalar PSD `q`, per axis):
per-axis block `q·[[dt³/3, dt²/2],[dt²/2, dt]]`, placed at the (pos,pos),
(pos,vel),(vel,pos),(vel,vel) entries for each of x and y.

**Assumptions.** Near-constant velocity between updates; maneuvers absorbed by
`q`; x/y independent; 2D surface motion.

**Rationale.** Standard, robust baseline; keeps all nonlinearity in the
measurement models. Chosen over constant-acceleration (overfits) and
coordinated-turn (premature) for the first cut.

**Ways to improve / test next.** IMM mixing CV + coordinated-turn for
maneuvering vessels; tune/learn `q` per vessel class; extend to 3D if needed.

## 2. Measurement models (`MeasurementModels`)

**Math.**
- Position2D: `h(x)=[px,py]`, `H=[[1,0,0,0],[0,1,0,0]]`.
- PositionVelocity2D: `h(x)=[px,py,vx,vy]`, `H=I₄`.
- RangeBearing2D: `r=√(px²+py²)`, `β=atan2(py,px)`;
  `H=[[px/r, py/r, 0, 0], [−py/r², px/r², 0, 0]]`.
- Residual: `y=z−h(x)`; for bearing, `y` wrapped to (−π, π].

**Assumptions.** Range/bearing relative to the ENU frame origin (own-ship
offset is applied later in normalization); `r>0` (guarded at 1e-6); Gaussian,
zero-mean measurement noise with covariance `R` provided per measurement.

**Rationale.** EKF Jacobian linearization is adequate for the mild range/bearing
nonlinearity at operational geometry and is far cheaper than UKF/particle.

**Ways to improve / test next.** UKF for stronger nonlinearity or bearing-only
geometry; particle filter for multimodal cases (bearing-only before range
converges); per-sensor `R` calibration from data.

## 3. Extended Kalman Filter (`EkfEstimator`)

**Math.**
- Predict: `dt = to − last_update`; `x ← F(dt)x`; `P ← F P Fᵀ + Q(dt)`.
- Update: `(ẑ,H) = h(x)`; `y = z − ẑ` (bearing wrapped); `S = H P Hᵀ + R`;
  `K = P Hᵀ S⁻¹`; `x ← x + K y`; `P ← (I − K H) P`.
- One-point initiation: `x=[zx,zy,0,0]`, `P=blockdiag(R_pos, σ_v²I₂)`.

**Assumptions.** `update` is called after `predict` advanced the state to
`z.time`; non-positive `dt` predict is a no-op; small dense matrices so a direct
`S⁻¹` is acceptable; Gaussian noise.

**Rationale.** Standard EKF; the simplest estimator that handles the nonlinear
range/bearing models while remaining cheap and pluggable behind `IEstimator`.

**Ways to improve / test next.** Joseph-form covariance update for numerical
stability; outlier/innovation gating before update; UKF/IMM swap-in via the
`IEstimator` port; two-point velocity initiation.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/estimation.md
git -c commit.gpgsign=false commit -m "docs: add estimation algorithm reference (math/assumptions/rationale/improve)"
```

---

## Done criteria

- Full suite green: `cmake --build build && ctest --test-dir build --output-on-failure`.
- `IMotionModel`, `IEstimator` ports exist; `ConstantVelocity2D` and `EkfEstimator` implemented in `core/estimation/`, all in `navtracker_core` (no I/O).
- EKF predicts forward, updates from position and range/bearing measurements (with bearing wraparound), and initiates tracks from a position measurement.
- `docs/algorithms/estimation.md` documents the math, assumptions, rationale, and improvement paths.

## Roadmap (remaining plans)

3. **Association + track management** — `IDataAssociator` (Mahalanobis gating + GNN), lifecycle (initiate/confirm/coast/delete), stable-ID allocation. Consumes `EkfEstimator::initiate` and predict/update.
4. **Pipeline + time** — time-ordered reorder buffer, tracker orchestration, `ISensorAdapter`/`ITrackSink` ports, deterministic replay.
5. **Sensor adapters** — AIS, ARPA (TTM/TLL), EO/IR, own-ship; normalization/geo-projection into ENU with per-sensor R (resolves the range/bearing own-ship-offset assumption from Task 2).
6. **Scenario harness + metrics** — synthetic ground-truth scenarios, OSPA/track-accuracy.
```
