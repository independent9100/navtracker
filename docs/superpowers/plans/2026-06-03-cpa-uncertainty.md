# CPA Uncertainty Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Jacobian-based uncertainty propagation through the closed-form CPA function, outputting `(cpa_distance, σ_cpa, tcpa, σ_tcpa, P(<d_threshold))`. Synthesise own-ship as a Track so the existing pair-CPA primitive serves both own-ship-vs-target and target-vs-target uniformly.

**Architecture:** New `CpaPrediction` struct alongside the existing `CpaResult`. New `computeCpaWithUncertainty` function in `core/collision/Cpa.{hpp,cpp}`. New `synthesizeOwnShipTrack` helper in `core/collision/CpaOwnShip.{hpp,cpp}`. Existing `computeCpa` / `CpaResult` API unchanged.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-03-cpa-uncertainty-design.md`. Section references in tasks below refer to that spec.

---

## Task 1: `CpaPrediction` and `computeCpaWithUncertainty`

**Files:**
- Modify: `core/collision/Cpa.hpp`
- Modify: `core/collision/Cpa.cpp`
- Create: `tests/collision/test_cpa_uncertainty.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §4 (math), §8 (unit tests 1-8): the core math and singularity branches. Eight unit tests cover both the well-behaved cases and the three singularity branches (parallel velocities, past CPA, head-on near-zero CPA).

### Steps

- [ ] **Step 1: Header — append the struct and function**

In `core/collision/Cpa.hpp`, after the existing `CpaResult` and `computeCpa` declarations, append:

```cpp
// Linear (Jacobian-based) uncertainty propagation through the CPA
// closed-form. Inputs:
//   a, b              — tracks; each must have 4D state [px, py, vx, vy]
//                       and 4x4 covariance.
//   t_ref             — reference time (e.g. now); each track is
//                       extrapolated from its own last_update.
//   d_threshold_m     — collision-alarm distance; output probability is
//                       P(CPA < d_threshold_m) under 1D Gaussian on CPA.
//
// Outputs (mean values match computeCpa byte-for-byte):
//   cpa_distance_m / sigma_cpa_m / tcpa_seconds / sigma_tcpa_seconds
//   probability_below_threshold ∈ [0, 1]
//   is_diverging — same semantics as computeCpa
//
// Singularity branches mirror computeCpa: parallel velocities and past
// CPA fall back to current-distance with σ from dp covariance and
// σ_tcpa = +infinity (sentinel). Head-on near-zero CPA uses an isotropic
// fallback for σ_cpa.
struct CpaPrediction {
  double cpa_distance_m;
  double sigma_cpa_m;
  double tcpa_seconds;
  double sigma_tcpa_seconds;
  double probability_below_threshold;
  double d_threshold_m;
  bool is_diverging;
};

CpaPrediction computeCpaWithUncertainty(const Track& a, const Track& b,
                                        Timestamp t_ref,
                                        double d_threshold_m);
```

- [ ] **Step 2: Implementation — math + branches**

In `core/collision/Cpa.cpp`, implement `computeCpaWithUncertainty`. Follow spec §4 verbatim. Sketch with the critical structural pieces:

```cpp
#include <cmath>
#include <limits>

namespace {

double standardNormalCdf(double x) {
  return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

constexpr double kEpsDv2 = 1e-12;    // |dv|^2 below this -> parallel
constexpr double kEpsCpa = 1.0;      // cpa below this -> head-on fallback

}  // namespace

CpaPrediction computeCpaWithUncertainty(const Track& a, const Track& b,
                                        Timestamp t_ref,
                                        double d_threshold_m) {
  // 1. Extrapolate mean states to t_ref (mirrors computeCpa).
  const double dt_a = t_ref.secondsSince(a.last_update);
  const double dt_b = t_ref.secondsSince(b.last_update);
  const Eigen::Vector2d pa(a.state(0) + a.state(2) * dt_a,
                           a.state(1) + a.state(3) * dt_a);
  const Eigen::Vector2d pb(b.state(0) + b.state(2) * dt_b,
                           b.state(1) + b.state(3) * dt_b);
  const Eigen::Vector2d va(a.state(2), a.state(3));
  const Eigen::Vector2d vb(b.state(2), b.state(3));
  const Eigen::Vector2d dp = pa - pb;
  const Eigen::Vector2d dv = va - vb;

  // 2. Build joint covariance Σ (8x8 block-diag).
  Eigen::Matrix<double, 8, 8> Sigma = Eigen::Matrix<double, 8, 8>::Zero();
  Sigma.block<4, 4>(0, 0) = a.covariance.topLeftCorner<4, 4>();
  Sigma.block<4, 4>(4, 4) = b.covariance.topLeftCorner<4, 4>();

  // 3. Build ∂dp/∂x and ∂dv/∂x as 2x8 matrices (per spec §4.3).
  Eigen::Matrix<double, 2, 8> dDp_dx = Eigen::Matrix<double, 2, 8>::Zero();
  dDp_dx.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 2) = dt_a * Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 4) = -Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 6) = -dt_b * Eigen::Matrix2d::Identity();

  Eigen::Matrix<double, 2, 8> dDv_dx = Eigen::Matrix<double, 2, 8>::Zero();
  dDv_dx.block<2, 2>(0, 2) =  Eigen::Matrix2d::Identity();
  dDv_dx.block<2, 2>(0, 6) = -Eigen::Matrix2d::Identity();

  const double dv2 = dv.dot(dv);
  const double t_cpa_raw = (dv2 > 0.0) ? -dp.dot(dv) / dv2
                                       : 0.0;

  CpaPrediction r;
  r.d_threshold_m = d_threshold_m;

  // 4. Branch A — parallel velocities or past CPA.
  if (dv2 < kEpsDv2 || t_cpa_raw <= 0.0) {
    const double cur_dist = dp.norm();
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = cur_dist;
    r.sigma_tcpa_seconds = std::numeric_limits<double>::infinity();
    r.is_diverging = (t_cpa_raw < 0.0);

    // σ_cpa from current dp covariance via direction-projection.
    const Eigen::Matrix<double, 2, 2> cov_dp =
        dDp_dx * Sigma * dDp_dx.transpose();
    double sigma_cpa = 0.0;
    if (cur_dist > 0.0) {
      const Eigen::Vector2d dp_hat = dp / cur_dist;
      sigma_cpa = std::sqrt(std::max(dp_hat.transpose() * cov_dp * dp_hat,
                                     0.0));
    } else {
      sigma_cpa = std::sqrt(std::max(0.5 * cov_dp.trace(), 0.0));
    }
    r.sigma_cpa_m = sigma_cpa;
    r.probability_below_threshold = (sigma_cpa > 0.0)
        ? standardNormalCdf((d_threshold_m - cur_dist) / sigma_cpa)
        : (cur_dist < d_threshold_m ? 1.0 :
           (cur_dist > d_threshold_m ? 0.0 : 0.5));
    return r;
  }

  // 5. General case — compute Jacobians per spec §4.3.
  const double t_cpa = t_cpa_raw;
  const Eigen::Vector2d p_cpa = dp + dv * t_cpa;
  const double cpa = p_cpa.norm();

  // ∂t_cpa/∂x = (1x8)
  Eigen::Matrix<double, 1, 8> J_tcpa;
  {
    // numerator: -[dvᵀ · ∂dp/∂x + dpᵀ · ∂dv/∂x]
    Eigen::Matrix<double, 1, 8> num = -(dv.transpose() * dDp_dx
                                         + dp.transpose() * dDv_dx);
    Eigen::Matrix<double, 1, 8> chain = 2.0 * t_cpa
                                          * (dv.transpose() * dDv_dx);
    J_tcpa = (num + chain) / dv2;
  }

  // ∂p_cpa/∂x = ∂dp/∂x + t_cpa · ∂dv/∂x + dv · J_tcpa   (2x8)
  Eigen::Matrix<double, 2, 8> J_p_cpa = dDp_dx + t_cpa * dDv_dx
                                       + dv * J_tcpa;

  // σ²_tcpa = J_tcpa · Σ · J_tcpaᵀ
  const double var_tcpa = (J_tcpa * Sigma * J_tcpa.transpose())(0, 0);

  // σ²_cpa via direction projection (or isotropic fallback if cpa ≈ 0).
  const Eigen::Matrix<double, 2, 2> cov_p_cpa =
      J_p_cpa * Sigma * J_p_cpa.transpose();
  double sigma_cpa;
  if (cpa > kEpsCpa) {
    const Eigen::Vector2d p_cpa_hat = p_cpa / cpa;
    const double var_cpa = (p_cpa_hat.transpose() * cov_p_cpa * p_cpa_hat);
    sigma_cpa = std::sqrt(std::max(var_cpa, 0.0));
  } else {
    sigma_cpa = std::sqrt(std::max(0.5 * cov_p_cpa.trace(), 0.0));
  }

  r.cpa_distance_m = cpa;
  r.sigma_cpa_m = sigma_cpa;
  r.tcpa_seconds = t_cpa;
  r.sigma_tcpa_seconds = std::sqrt(std::max(var_tcpa, 0.0));
  r.is_diverging = false;
  r.probability_below_threshold = (sigma_cpa > 0.0)
      ? standardNormalCdf((d_threshold_m - cpa) / sigma_cpa)
      : (cpa < d_threshold_m ? 1.0 :
         (cpa > d_threshold_m ? 0.0 : 0.5));
  return r;
}
```

Notes for the implementer:
- `Track::state` and `Track::covariance` are `Eigen::VectorXd` / `Eigen::MatrixXd`. Use `.head<4>()` / `.topLeftCorner<4, 4>()` style access; check current usage in `Cpa.cpp` and match its idiom.
- The standard normal CDF via `std::erfc` (or `std::erf`) is numerically stable across the tail.
- If covariance is malformed (e.g., `a.covariance.rows() < 4`), the existing `computeCpa` doesn't validate either — treat that as a precondition.

- [ ] **Step 3: Wire into CMake**

The existing `Cpa.cpp` is already in `navtracker_core`. No change needed.

- [ ] **Step 4: Write the unit tests**

Create `tests/collision/test_cpa_uncertainty.cpp` covering the eight tests from spec §8.1.

Provide a small helper to build a Track with given (px, py, vx, vy) and (σ_pos, σ_vel):

```cpp
namespace {

Track makeTrack(double px, double py, double vx, double vy,
                double sigma_pos = 0.0, double sigma_vel = 0.0,
                Timestamp last_update = Timestamp::fromSeconds(0.0)) {
  Track t;
  t.id = TrackId{1};
  t.last_update = last_update;
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << px, py, vx, vy;
  t.covariance = Eigen::Matrix4d::Zero();
  const double pp = sigma_pos * sigma_pos;
  const double vv = sigma_vel * sigma_vel;
  t.covariance.diagonal() << pp, pp, vv, vv;
  return t;
}

}  // namespace
```

Each TEST follows spec §8.1 — see the spec for the full set. Sample scaffold:

```cpp
TEST(CpaUncertainty, ZeroCovGivesZeroSigma) {
  // Head-on collision, no input uncertainty -> output sigma all zero.
  const Track a = makeTrack( 0.0, 0.0,  10.0, 0.0);
  const Track b = makeTrack(20.0, 0.0, -10.0, 0.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           5.0);
  EXPECT_NEAR(r.cpa_distance_m, 0.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,   1.0, 1e-9);
  EXPECT_NEAR(r.sigma_cpa_m,    0.0, 1e-9);
  EXPECT_NEAR(r.sigma_tcpa_seconds, 0.0, 1e-9);
  EXPECT_NEAR(r.probability_below_threshold, 1.0, 1e-9);
}

TEST(CpaUncertainty, ParallelVelocitiesSigmaTcpaInfinite) {
  const Track a = makeTrack( 0.0, 0.0, 10.0, 0.0, /*sigma_pos=*/3.0);
  const Track b = makeTrack( 0.0, 20.0, 10.0, 0.0, /*sigma_pos=*/3.0);
  const auto r = computeCpaWithUncertainty(a, b,
                                           Timestamp::fromSeconds(0.0),
                                           5.0);
  EXPECT_NEAR(r.cpa_distance_m, 20.0, 1e-9);
  EXPECT_NEAR(r.tcpa_seconds,   0.0, 1e-9);
  EXPECT_TRUE(std::isinf(r.sigma_tcpa_seconds));
  EXPECT_FALSE(r.is_diverging);
  // σ_dp at distance 20m direction-projected: σ_dp_y = sqrt(σ²_pos_a + σ²_pos_b) = sqrt(18) ≈ 4.24
  EXPECT_NEAR(r.sigma_cpa_m, std::sqrt(18.0), 1e-6);
}
```

Cover the remaining six tests per spec §8.1.

- [ ] **Step 5: Add test to CMake**

Append `tests/collision/test_cpa_uncertainty.cpp` to the `navtracker_tests` source list in `CMakeLists.txt`.

- [ ] **Step 6: Build + run targeted tests**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R CpaUncertainty --output-on-failure
```
Expected: all 8 PASS.

- [ ] **Step 7: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 281/281 green (was 273; +8 new).

- [ ] **Step 8: Commit**

```
git add core/collision/Cpa.hpp core/collision/Cpa.cpp \
        tests/collision/test_cpa_uncertainty.cpp CMakeLists.txt
git commit -m "cpa: add computeCpaWithUncertainty (Jacobian-based sigma + P)"
```

---

## Task 2: `synthesizeOwnShipTrack` helper

**Files:**
- Create: `core/collision/CpaOwnShip.hpp`
- Create: `core/collision/CpaOwnShip.cpp`
- Create: `tests/collision/test_cpa_synthesize_own_ship.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §5: synthesise own-ship as a Track so the uniform pair-CPA primitive serves both own-ship-vs-target and target-vs-target. Lives in `core/collision/` for natural grouping with the CPA primitive.

### Steps

- [ ] **Step 1: Header**

Create `core/collision/CpaOwnShip.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Build a Track from own-ship state for use with computeCpaWithUncertainty.
// state = [ex, ey, vx, vy] in ENU; covariance = diag(σ_pos², σ_pos², 0, 0).
// id is the reserved sentinel TrackId{0}; not entered into TrackManager.
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum);

}  // namespace navtracker
```

- [ ] **Step 2: Implementation**

Create `core/collision/CpaOwnShip.cpp`:

```cpp
#include "core/collision/CpaOwnShip.hpp"

namespace navtracker {

Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum) {
  Track tr;
  tr.id = TrackId{0};
  tr.last_update = t;
  tr.status = TrackStatus::Confirmed;

  const Eigen::Vector3d enu = datum.toEnu({pose.lat_deg, pose.lon_deg, 0.0});
  tr.state.resize(4);
  tr.state << enu.x(), enu.y(), velocity_enu.x(), velocity_enu.y();

  tr.covariance = Eigen::Matrix4d::Zero();
  const double pp = sigma_pos_m * sigma_pos_m;
  tr.covariance(0, 0) = pp;
  tr.covariance(1, 1) = pp;
  // velocity covariance zero per v1 decision (caller knows velocity).

  return tr;
}

}  // namespace navtracker
```

- [ ] **Step 3: Wire into CMake**

Add `core/collision/CpaOwnShip.cpp` to the `navtracker_core` source list (alphabetically near `core/collision/Cpa.cpp`).

- [ ] **Step 4: Unit tests**

Create `tests/collision/test_cpa_synthesize_own_ship.cpp`:

```cpp
#include "core/collision/CpaOwnShip.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

namespace navtracker {

TEST(SynthesizeOwnShipTrack, PlacesPoseAtCorrectEnu) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  const Eigen::Vector2d v(5.0, 3.0);

  const Track t = synthesizeOwnShipTrack(pose, v, 1.0,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_NEAR(t.state(0), 0.0, 1e-3);
  EXPECT_NEAR(t.state(1), 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(t.state(2), 5.0);
  EXPECT_DOUBLE_EQ(t.state(3), 3.0);
}

TEST(SynthesizeOwnShipTrack, CovarianceMatchesSigmaPos) {
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  const Track t = synthesizeOwnShipTrack(pose, Eigen::Vector2d::Zero(),
                                         5.0,
                                         Timestamp::fromSeconds(0.0),
                                         datum);
  EXPECT_DOUBLE_EQ(t.covariance(0, 0), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 1), 25.0);
  EXPECT_DOUBLE_EQ(t.covariance(2, 2),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(3, 3),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(0, 1),  0.0);
  EXPECT_DOUBLE_EQ(t.covariance(1, 0),  0.0);
}

}  // namespace navtracker
```

- [ ] **Step 5: Add test to CMake**

Append `tests/collision/test_cpa_synthesize_own_ship.cpp` to `navtracker_tests`.

- [ ] **Step 6: Build, run, full suite**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R SynthesizeOwnShipTrack --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 2 new PASS; full suite 283/283 green.

- [ ] **Step 7: Commit**

```
git add core/collision/CpaOwnShip.hpp core/collision/CpaOwnShip.cpp \
        tests/collision/test_cpa_synthesize_own_ship.cpp CMakeLists.txt
git commit -m "collision: add synthesizeOwnShipTrack helper for pair-CPA"
```

---

## Task 3: Scenario integration test + eval-log

**Files:**
- Create: `tests/scenario/test_cpa_scenario.cpp` (or add to an existing scenario test file)
- Modify: `CMakeLists.txt`
- Modify: `docs/algorithms/evaluation-log.md`

### Why

Per spec §8 (integration test 11) and "Eval-log": demonstrate that the predicted CPA confidence band contains truth on a known scenario, and connect the past three sessions' covariance work to a single operational number per scenario.

### Steps

- [ ] **Step 1: Write the integration test**

Create `tests/scenario/test_cpa_scenario.cpp`:

```cpp
// Perpendicular-pass scenario: own-ship at origin, stationary; target
// passes north of own-ship at 1000 m N, moving east at 10 m/s. Truth
// CPA = 1000 m (target's closest approach is at t=0). 20 measurements
// at 1 Hz drive tracker; at t=20 s call computeCpaWithUncertainty.
// Assert |cpa_predicted - cpa_truth| < 2 * sigma_cpa.
//
// Builders patterns from tests/scenario/test_filter_comparison.cpp.
```

Build the scenario, drive the tracker, synthesize own-ship via the Task-2 helper (pose at origin, velocity zero, σ_pos = 1.0 m), and call `computeCpaWithUncertainty(own_ship_track, target_track, t_ref=10s, d_threshold=500m)`.

Assertions:
- `cpa_distance_m` close to 1000 m (truth).
- `sigma_cpa_m > 0` (uncertainty propagates).
- `|cpa_predicted - 1000| < 2 * sigma_cpa_m` (within 2σ).
- `probability_below_threshold < 0.01` (CPA truth = 1000 m >> threshold = 500 m).

Note on scenarios: the existing `tests/scenario/test_*` files use harness-style scenarios that feed Position2D measurements directly. For this test, the simplest path is to build the scenario inline (truth + a few measurements) rather than going through the bus.

- [ ] **Step 2: Add test to CMake**

Append `tests/scenario/test_cpa_scenario.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run and capture**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R CpaScenario --output-on-failure 2>&1 | tee /tmp/cpa_scenario.txt
```

Also run the existing 3 §14.9 sweep scenarios with `computeCpaWithUncertainty` applied to a representative pair (target 1 at mid-scenario) and capture mean/σ_cpa/P. This can either be a one-off `std::cerr` instrumentation inside the existing sweep tests OR a dedicated small test that just reports numbers. Pick whichever fits cleanly.

Concretely: extend the `tests/sim/test_bus_gps_sweep.cpp` (or write a parallel `tests/sim/test_bus_cpa_uncertainty.cpp`) so that at the end of each cell's runs you compute mean CPA + σ_cpa + P over all confirmed tracks-vs-own-ship using `synthesizeOwnShipTrack` + `computeCpaWithUncertainty`. Print one row per cell with these new columns. Reuse the existing sweep harness; don't replicate the full bus.

- [ ] **Step 4: Append eval-log section**

In `docs/algorithms/evaluation-log.md`, append:

```markdown
## CPA uncertainty (2026-06-03)

**Setup.** Jacobian-based linear propagation of joint track covariance
through the closed-form CPA function. Output: mean, σ on cpa and tcpa,
and P(CPA < 200 m) under 1D-Gaussian. Own-ship synthesised as a Track
via `synthesizeOwnShipTrack` with σ_pos from the GPS work; σ_v_own = 0
per v1 decision.

### Predicted CPA on a known perpendicular-pass

Truth CPA = 1000 m, tracker at t = 20 s.

| measurement noise | predicted CPA | σ_cpa | P(<500m) | in 2σ band? |
|---|---|---|---|---|
| σ_pos = 1 m, σ_h = 0 | <fill> | <fill> | <fill> | <yes/no> |
| σ_pos = 1 m, σ_h = 1° | <fill> | <fill> | <fill> | <> |
| σ_pos = 5 m, σ_h = 0 | <fill> | <fill> | <fill> | <> |
| σ_pos = 5 m, σ_h = 1° | <fill> | <fill> | <fill> | <> |

### CPA bands across §14.9 sweep scenarios (20 seeds, R-on)

| scenario | σ_h | σ_GPS | mean cpa | σ_cpa | P(<200m) |
|---|---|---|---|---|---|
| ClutterCrossing | 0° | 0 m | <fill> | <fill> | <fill> |
| ClutterCrossing | 2° | 0 m | <fill> | <fill> | <fill> |
| ClutterCrossing | 2° | 5 m | <fill> | <fill> | <fill> |
| Maneuvering | 0° | 0 m | <fill> | <fill> | <fill> |
| Maneuvering | 2° | 0 m | <fill> | <fill> | <fill> |
| Maneuvering | 2° | 5 m | <fill> | <fill> | <fill> |
| BearingOnlyMoving | 0° | 0 m | <fill> | <fill> | <fill> |
| BearingOnlyMoving | 2° | 0 m | <fill> | <fill> | <fill> |
| BearingOnlyMoving | 2° | 5 m | <fill> | <fill> | <fill> |

### Verdict

<3–5 sentences. Truth-CPA falls inside the 2σ band on the known
scenario. σ_cpa grows materially as σ_h or σ_GPS grows, confirming
covariance from prior work propagates correctly. P(<200m) is the
operational output operators consume; for the worst-case sweep cell
(BearingOnlyMoving at σ_h=2°, σ_GPS=5m) it's <fill>, and consumers can
threshold on it directly for alarm logic. The 1D-Gaussian
approximation is documented for near-collision cases (see §11).>
```

Fill `<fill>` from the captured numbers.

- [ ] **Step 5: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 284/284 green (was 283; +1 scenario test). If the §14.9-sweep extension also adds a test, count grows accordingly — confirm.

- [ ] **Step 6: Commit**

```
git add tests/scenario/test_cpa_scenario.cpp CMakeLists.txt \
        docs/algorithms/evaluation-log.md
# Plus the sweep-instrumentation file(s) modified or created.
git commit -m "scenario+eval: CPA uncertainty bands on known scenario and 14.9 sweep"
```

---

## Done criteria

- All 3 tasks committed.
- Full suite green (≥ 284/284).
- Eval-log section populated with concrete numbers showing truth-CPA inside 2σ band on the known scenario and σ_cpa growth with σ_h and σ_GPS on the §14.9 sweep.
- `computeCpa` / `CpaResult` API unchanged.
- `Track` API unchanged (synthesizeOwnShipTrack creates an in-memory Track without entering it into a TrackManager).
