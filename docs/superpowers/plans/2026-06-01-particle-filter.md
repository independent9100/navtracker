# Particle Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a bootstrap (sequential importance resampling) particle filter behind `IEstimator`, swappable with the EKF / UKF, and record its behavior on the existing nonlinear range/bearing scenarios.

**Architecture:** A new `ParticleFilterEstimator` carries a weighted particle ensemble per track. `predict` propagates each particle through the linear `F(dt)` plus a sampled process-noise draw `~ N(0, Q(dt))`. `update` re-weights particles by the measurement likelihood `p(z|x_i)`, normalizes via log-sum-exp, and runs systematic resampling when effective sample size drops below `N/2`. After every step the ensemble is projected to a weighted Gaussian mean / covariance written into `Track.state` / `Track.covariance` so the rest of the pipeline (gating, association, sinks) does not care which estimator produced it. The ensemble lives **on the Track itself** as `Eigen::MatrixXd particles` and `Eigen::VectorXd particle_weights`; EKF / UKF tracks leave both empty.

**Tech Stack:** C++17, Eigen 3.4, gtest. Reuses `IMotionModel`, `MeasurementModels` (`predictMeasurement`, `wrapAngle`), and the `IEstimator` port. Adds `<random>` for sampling.

---

## File Structure

**Create:**
- `core/estimation/Resampling.hpp` / `.cpp` — `systematicResample`, `effectiveSampleSize` (testable independently of the estimator).
- `core/estimation/ParticleFilterEstimator.hpp` / `.cpp` — bootstrap PF behind `IEstimator`.
- `tests/estimation/test_resampling.cpp` — resampling and ESS unit tests.
- `tests/estimation/test_particle_filter_estimator.cpp` — predict / update / initiate / determinism tests.

**Modify:**
- `core/types/Track.hpp` — add ensemble carrier fields.
- `CMakeLists.txt` — register new sources + tests.
- `tests/scenario/test_filter_comparison.cpp` — run PF alongside EKF + UKF on the two range/bearing scenarios.
- `docs/algorithms/estimation.md` — add the PF section (Math / Assumptions / Rationale / Ways-to-improve).
- `docs/algorithms/evaluation-log.md` — add the EKF / UKF / PF comparison entry.

---

## Task 1: Add particle ensemble carrier to `Track`

**Files:**
- Modify: `core/types/Track.hpp:27-35`
- Test: `tests/types/test_track.cpp` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/types/test_track.cpp`:

```cpp
TEST(Track, DefaultHasEmptyParticleEnsemble) {
  navtracker::Track t;
  EXPECT_EQ(t.particles.rows(), 0);
  EXPECT_EQ(t.particles.cols(), 0);
  EXPECT_EQ(t.particle_weights.size(), 0);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=Track.DefaultHasEmptyParticleEnsemble`
Expected: FAIL (`particles` / `particle_weights` not members of `Track`).

- [ ] **Step 3: Add ensemble fields**

Edit `core/types/Track.hpp` — add two members to the `Track` struct (after `contributing_sources`):

```cpp
  // Optional ensemble carrier used by ensemble-based estimators (particle
  // filter today; IMM later). The Gaussian (state, covariance) above remains
  // the canonical kinematic snapshot consumed by gating / association / sinks.
  Eigen::MatrixXd particles;        // n_state x N_particles, empty if unused
  Eigen::VectorXd particle_weights; // N_particles, sums to 1, empty if unused
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=Track.*`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/types/Track.hpp tests/types/test_track.cpp
git commit -m "track: add optional particle ensemble carrier"
```

---

## Task 2: Systematic resampling utility

**Files:**
- Create: `core/estimation/Resampling.hpp`
- Create: `core/estimation/Resampling.cpp`
- Create: `tests/estimation/test_resampling.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the header**

Create `core/estimation/Resampling.hpp`:

```cpp
#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtracker {

// Effective sample size of a normalized weight vector: 1 / Σ w_i².
// Equals N for uniform weights, 1 for a fully concentrated weight vector.
double effectiveSampleSize(const Eigen::VectorXd& weights);

// Systematic resampling: returns N indices drawn from the CDF of `weights`
// at the equally-spaced positions {u, u + 1/N, u + 2/N, ...}. Caller supplies
// the starting offset `u ∈ [0, 1/N)` so the RNG is owned externally.
// Weights are assumed normalized (Σ w_i = 1); behavior is undefined otherwise.
std::vector<int> systematicResample(const Eigen::VectorXd& weights, double u);

}  // namespace navtracker
```

- [ ] **Step 2: Write the failing tests**

Create `tests/estimation/test_resampling.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/estimation/Resampling.hpp"

using navtracker::effectiveSampleSize;
using navtracker::systematicResample;

TEST(Resampling, EssIsNForUniformWeights) {
  Eigen::VectorXd w = Eigen::VectorXd::Constant(8, 1.0 / 8.0);
  EXPECT_NEAR(effectiveSampleSize(w), 8.0, 1e-12);
}

TEST(Resampling, EssIsOneForConcentratedWeights) {
  Eigen::VectorXd w(4);
  w << 1.0, 0.0, 0.0, 0.0;
  EXPECT_NEAR(effectiveSampleSize(w), 1.0, 1e-12);
}

TEST(Resampling, UniformWeightsGiveIdentityMapping) {
  const int N = 4;
  Eigen::VectorXd w = Eigen::VectorXd::Constant(N, 1.0 / N);
  const std::vector<int> idx = systematicResample(w, 0.5 / N);
  ASSERT_EQ(idx.size(), 4u);
  for (int i = 0; i < N; ++i) EXPECT_EQ(idx[i], i);
}

TEST(Resampling, ConcentratedWeightReplicatesTopParticle) {
  Eigen::VectorXd w(4);
  w << 1.0, 0.0, 0.0, 0.0;
  const std::vector<int> idx = systematicResample(w, 0.1);
  for (int i : idx) EXPECT_EQ(i, 0);
}

TEST(Resampling, DeterministicForSameOffset) {
  Eigen::VectorXd w(5);
  w << 0.05, 0.05, 0.30, 0.50, 0.10;
  const auto a = systematicResample(w, 0.07);
  const auto b = systematicResample(w, 0.07);
  EXPECT_EQ(a, b);
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests` (link error: `Resampling.cpp` missing).

- [ ] **Step 4: Implement Resampling.cpp**

Create `core/estimation/Resampling.cpp`:

```cpp
#include "core/estimation/Resampling.hpp"

namespace navtracker {

double effectiveSampleSize(const Eigen::VectorXd& weights) {
  return 1.0 / weights.squaredNorm();
}

std::vector<int> systematicResample(const Eigen::VectorXd& weights, double u) {
  const int N = static_cast<int>(weights.size());
  std::vector<int> idx(N);
  double c = weights(0);
  int i = 0;
  for (int j = 0; j < N; ++j) {
    const double Uj = u + static_cast<double>(j) / static_cast<double>(N);
    while (Uj > c && i < N - 1) {
      ++i;
      c += weights(i);
    }
    idx[j] = i;
  }
  return idx;
}

}  // namespace navtracker
```

- [ ] **Step 5: Register sources in CMakeLists.txt**

Edit `CMakeLists.txt`: add `core/estimation/Resampling.cpp` to `navtracker_core` (after `UkfEstimator.cpp`), and `tests/estimation/test_resampling.cpp` to `navtracker_tests` (after `test_ukf_estimator.cpp`).

- [ ] **Step 6: Build and run**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=Resampling.*`
Expected: 5/5 PASS.

- [ ] **Step 7: Commit**

```bash
git add core/estimation/Resampling.hpp core/estimation/Resampling.cpp tests/estimation/test_resampling.cpp CMakeLists.txt
git commit -m "estimation: add systematic resampling + ESS utility"
```

---

## Task 3: `ParticleFilterEstimator` skeleton + `initiate`

**Files:**
- Create: `core/estimation/ParticleFilterEstimator.hpp`
- Create: `core/estimation/ParticleFilterEstimator.cpp`
- Create: `tests/estimation/test_particle_filter_estimator.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the header**

Create `core/estimation/ParticleFilterEstimator.hpp`:

```cpp
#pragma once

#include <memory>
#include <random>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Bootstrap (SIR) particle filter behind IEstimator. Carries a weighted
// ensemble on the Track itself; projects to a Gaussian (mean, covariance) so
// downstream consumers (gating, sinks) are estimator-agnostic.
//
// Determinism: a single internal RNG is advanced by every predict / update /
// initiate call. Replaying the same message stream against a freshly-seeded
// instance reproduces identical particles, weights, and projected state.
class ParticleFilterEstimator : public IEstimator {
 public:
  ParticleFilterEstimator(std::shared_ptr<const IMotionModel> motion,
                          int particle_count = 500,
                          double init_speed_std = 10.0,
                          double ess_fraction_threshold = 0.5,
                          std::uint64_t seed = 0);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;
  Track initiate(const Measurement& z) const override;

 private:
  void projectToGaussian(Track& track) const;

  std::shared_ptr<const IMotionModel> motion_;
  int particle_count_;
  double init_speed_std_;
  double ess_threshold_;     // absolute (= fraction · N)
  mutable std::mt19937_64 rng_;
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the failing tests**

Create `tests/estimation/test_particle_filter_estimator.cpp`:

```cpp
#include <gtest/gtest.h>

#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::ParticleFilterEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Timestamp;

namespace {

Measurement positionMeas(double x, double y, double std_m, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m);
  m.source_id = "test";
  return m;
}

}  // namespace

TEST(ParticleFilterEstimator, InitiateSeedsEnsembleAndCarrier) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 500, 5.0, 0.5, 42);
  const navtracker::Track t = pf.initiate(positionMeas(100.0, -50.0, 3.0, 1.0));

  EXPECT_EQ(t.particles.rows(), 4);
  EXPECT_EQ(t.particles.cols(), 500);
  EXPECT_EQ(t.particle_weights.size(), 500);
  EXPECT_NEAR(t.particle_weights.sum(), 1.0, 1e-9);
  for (int i = 0; i < 500; ++i)
    EXPECT_NEAR(t.particle_weights(i), 1.0 / 500.0, 1e-12);

  // Carrier mean within Monte-Carlo tolerance of the measurement.
  EXPECT_NEAR(t.state(0), 100.0, 1.5);
  EXPECT_NEAR(t.state(1), -50.0, 1.5);
  // Velocity initialized at zero (no info), spread = init_speed_std_.
  EXPECT_NEAR(t.state(2), 0.0, 1.5);
  EXPECT_NEAR(t.state(3), 0.0, 1.5);
  EXPECT_GT(t.covariance(2, 2), 1.0);
  EXPECT_GT(t.covariance(3, 3), 1.0);
}
```

- [ ] **Step 3: Implement the cpp**

Create `core/estimation/ParticleFilterEstimator.cpp`:

```cpp
#include "core/estimation/ParticleFilterEstimator.hpp"

#include <cmath>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include "core/estimation/MeasurementModels.hpp"
#include "core/estimation/Resampling.hpp"

namespace navtracker {

ParticleFilterEstimator::ParticleFilterEstimator(
    std::shared_ptr<const IMotionModel> motion,
    int particle_count,
    double init_speed_std,
    double ess_fraction_threshold,
    std::uint64_t seed)
    : motion_(std::move(motion)),
      particle_count_(particle_count),
      init_speed_std_(init_speed_std),
      ess_threshold_(ess_fraction_threshold * particle_count),
      rng_(seed) {}

void ParticleFilterEstimator::projectToGaussian(Track& track) const {
  const int n = static_cast<int>(track.particles.rows());
  const int N = static_cast<int>(track.particles.cols());
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < N; ++i)
    mean += track.particle_weights(i) * track.particles.col(i);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < N; ++i) {
    const Eigen::VectorXd d = track.particles.col(i) - mean;
    cov += track.particle_weights(i) * d * d.transpose();
  }
  track.state = mean;
  track.covariance = cov;
}

void ParticleFilterEstimator::predict(Track& /*track*/, Timestamp /*to*/) const {
  // Implemented in Task 4.
}

void ParticleFilterEstimator::update(Track& /*track*/,
                                     const Measurement& /*z*/) const {
  // Implemented in Task 5.
}

Track ParticleFilterEstimator::initiate(const Measurement& z) const {
  Track t;
  t.last_update = z.time;
  t.status = TrackStatus::Tentative;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x(0) = z.value(0);
  x(1) = z.value(1);

  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = z.covariance(0, 0);
  P(0, 1) = z.covariance(0, 1);
  P(1, 0) = z.covariance(1, 0);
  P(1, 1) = z.covariance(1, 1);
  const double vv = init_speed_std_ * init_speed_std_;
  P(2, 2) = vv;
  P(3, 3) = vv;

  const Eigen::LLT<Eigen::Matrix4d> llt(P);
  const Eigen::Matrix4d L = llt.matrixL();

  std::normal_distribution<double> n01(0.0, 1.0);
  Eigen::MatrixXd particles(4, particle_count_);
  for (int i = 0; i < particle_count_; ++i) {
    Eigen::Vector4d eta;
    for (int j = 0; j < 4; ++j) eta(j) = n01(rng_);
    particles.col(i) = x + L * eta;
  }
  t.particles = particles;
  t.particle_weights =
      Eigen::VectorXd::Constant(particle_count_, 1.0 / particle_count_);

  projectToGaussian(t);

  if (z.hints.mmsi.has_value()) t.attributes.mmsi = z.hints.mmsi;
  t.contributing_sources.push_back(z.source_id);
  return t;
}

}  // namespace navtracker
```

- [ ] **Step 4: Register in CMakeLists.txt**

Add `core/estimation/ParticleFilterEstimator.cpp` to `navtracker_core` (after `Resampling.cpp`), and `tests/estimation/test_particle_filter_estimator.cpp` to `navtracker_tests` (after `test_resampling.cpp`).

- [ ] **Step 5: Build and run**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.*`
Expected: 1/1 PASS.

- [ ] **Step 6: Commit**

```bash
git add core/estimation/ParticleFilterEstimator.hpp core/estimation/ParticleFilterEstimator.cpp tests/estimation/test_particle_filter_estimator.cpp CMakeLists.txt
git commit -m "estimation: ParticleFilterEstimator skeleton with Gaussian-seeded initiate"
```

---

## Task 4: Implement `predict`

**Files:**
- Modify: `core/estimation/ParticleFilterEstimator.cpp` (replace the `predict` stub)
- Modify: `tests/estimation/test_particle_filter_estimator.cpp` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/estimation/test_particle_filter_estimator.cpp`:

```cpp
TEST(ParticleFilterEstimator, PredictAdvancesMeanByMotionModel) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.0);  // zero process noise
  ParticleFilterEstimator pf(motion, 2000, 5.0, 0.5, 7);
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  // Inject a deterministic velocity into every particle so the predict step
  // has something to advance.
  for (int i = 0; i < t.particles.cols(); ++i) {
    t.particles(2, i) = 5.0;
    t.particles(3, i) = -3.0;
  }
  pf.predict(t, Timestamp::fromSeconds(2.0));
  EXPECT_NEAR(t.state(0), 10.0, 0.3);
  EXPECT_NEAR(t.state(1), -6.0, 0.3);
  EXPECT_DOUBLE_EQ(t.last_update.seconds(), 2.0);
}

TEST(ParticleFilterEstimator, PredictGrowsCovarianceWithProcessNoise) {
  auto motion_q0 = std::make_shared<ConstantVelocity2D>(0.0);
  auto motion_q1 = std::make_shared<ConstantVelocity2D>(1.0);
  ParticleFilterEstimator pf0(motion_q0, 2000, 5.0, 0.5, 11);
  ParticleFilterEstimator pf1(motion_q1, 2000, 5.0, 0.5, 11);

  navtracker::Track t0 = pf0.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  navtracker::Track t1 = pf1.initiate(positionMeas(0.0, 0.0, 1.0, 0.0));
  pf0.predict(t0, Timestamp::fromSeconds(5.0));
  pf1.predict(t1, Timestamp::fromSeconds(5.0));

  // The q=1 predict must add appreciably more position spread than q=0.
  EXPECT_GT(t1.covariance(0, 0), t0.covariance(0, 0) + 1.0);
  EXPECT_GT(t1.covariance(1, 1), t0.covariance(1, 1) + 1.0);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.Predict*`
Expected: FAIL (mean / covariance untouched by stub).

- [ ] **Step 3: Replace the `predict` stub**

In `core/estimation/ParticleFilterEstimator.cpp`, replace the stubbed `predict` with:

```cpp
void ParticleFilterEstimator::predict(Track& track, Timestamp to) const {
  const double dt = to.secondsSince(track.last_update);
  if (dt <= 0.0) return;
  const int n = static_cast<int>(track.particles.rows());
  const int N = static_cast<int>(track.particles.cols());
  const Eigen::MatrixXd F = motion_->transitionMatrix(dt);
  const Eigen::MatrixXd Q = motion_->processNoise(dt);

  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(n, N);
  const Eigen::LLT<Eigen::MatrixXd> llt(Q);
  if (llt.info() == Eigen::Success) {
    const Eigen::MatrixXd L = llt.matrixL();
    std::normal_distribution<double> n01(0.0, 1.0);
    Eigen::MatrixXd eta(n, N);
    for (int j = 0; j < N; ++j)
      for (int i = 0; i < n; ++i) eta(i, j) = n01(rng_);
    noise = L * eta;
  }
  // If Q is not PD (e.g. q == 0 → singular), skip the noise term entirely;
  // the predict becomes deterministic F·x for every particle.

  track.particles = (F * track.particles) + noise;
  projectToGaussian(track);
  track.last_update = to;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.Predict*`
Expected: 2/2 PASS.

- [ ] **Step 5: Commit**

```bash
git add core/estimation/ParticleFilterEstimator.cpp tests/estimation/test_particle_filter_estimator.cpp
git commit -m "estimation: PF predict with sampled process noise"
```

---

## Task 5: Implement `update` (importance weighting + resampling)

**Files:**
- Modify: `core/estimation/ParticleFilterEstimator.cpp` (replace the `update` stub)
- Modify: `tests/estimation/test_particle_filter_estimator.cpp` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/estimation/test_particle_filter_estimator.cpp`:

```cpp
TEST(ParticleFilterEstimator, UpdateShrinksPositionCovariance) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 2000, 10.0, 0.5, 23);
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 20.0, 0.0));
  const double var_before = t.covariance(0, 0);
  pf.update(t, positionMeas(0.0, 0.0, 1.0, 0.0));
  EXPECT_LT(t.covariance(0, 0), var_before * 0.5);
  EXPECT_LT(t.covariance(1, 1), var_before * 0.5);
  EXPECT_NEAR(t.state(0), 0.0, 1.0);
  EXPECT_NEAR(t.state(1), 0.0, 1.0);
}

TEST(ParticleFilterEstimator, UpdateResamplesWhenEssCollapses) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator pf(motion, 1000, 50.0, 0.5, 31);
  // Wide prior so a sharp measurement gives a heavily concentrated weight
  // distribution → ESS collapses → triggers systematic resampling.
  navtracker::Track t = pf.initiate(positionMeas(0.0, 0.0, 100.0, 0.0));
  pf.update(t, positionMeas(0.0, 0.0, 0.5, 0.0));
  // Post-resample weights are uniform.
  for (int i = 0; i < t.particle_weights.size(); ++i)
    EXPECT_NEAR(t.particle_weights(i), 1.0 / 1000.0, 1e-12);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.Update*`
Expected: FAIL (`update` is still a stub).

- [ ] **Step 3: Replace the `update` stub**

In `core/estimation/ParticleFilterEstimator.cpp`, replace the stubbed `update` with:

```cpp
void ParticleFilterEstimator::update(Track& track, const Measurement& z) const {
  const int N = static_cast<int>(track.particles.cols());
  const Eigen::MatrixXd Rinv = z.covariance.inverse();

  Eigen::VectorXd log_w(N);
  for (int i = 0; i < N; ++i) log_w(i) = std::log(track.particle_weights(i));

  for (int i = 0; i < N; ++i) {
    const MeasurementPrediction pred =
        predictMeasurement(z.model, track.particles.col(i));
    Eigen::VectorXd y = z.value - pred.z_pred;
    if (z.model == MeasurementModel::RangeBearing2D) y(1) = wrapAngle(y(1));
    log_w(i) += -0.5 * y.transpose() * Rinv * y;
  }

  const double max_lw = log_w.maxCoeff();
  Eigen::VectorXd w = (log_w.array() - max_lw).exp();
  const double sum = w.sum();
  if (!std::isfinite(sum) || sum <= 0.0) {
    // All particles deemed impossible: reset to uniform — degenerate case.
    w = Eigen::VectorXd::Constant(N, 1.0 / N);
  } else {
    w /= sum;
  }
  track.particle_weights = w;

  if (effectiveSampleSize(w) < ess_threshold_) {
    std::uniform_real_distribution<double> u01(
        0.0, 1.0 / static_cast<double>(N));
    const double u = u01(rng_);
    const std::vector<int> idx = systematicResample(w, u);
    Eigen::MatrixXd resampled(track.particles.rows(), N);
    for (int i = 0; i < N; ++i) resampled.col(i) = track.particles.col(idx[i]);
    track.particles = resampled;
    track.particle_weights = Eigen::VectorXd::Constant(N, 1.0 / N);
  }

  projectToGaussian(track);
  track.last_update = z.time;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.*`
Expected: 5/5 PASS.

- [ ] **Step 5: Commit**

```bash
git add core/estimation/ParticleFilterEstimator.cpp tests/estimation/test_particle_filter_estimator.cpp
git commit -m "estimation: PF update with importance weighting + systematic resampling"
```

---

## Task 6: Determinism test

**Files:**
- Modify: `tests/estimation/test_particle_filter_estimator.cpp` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/estimation/test_particle_filter_estimator.cpp`:

```cpp
TEST(ParticleFilterEstimator, DeterministicForSameSeed) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  ParticleFilterEstimator a(motion, 256, 5.0, 0.5, 99);
  ParticleFilterEstimator b(motion, 256, 5.0, 0.5, 99);

  navtracker::Track ta = a.initiate(positionMeas(10.0, 20.0, 2.0, 0.0));
  navtracker::Track tb = b.initiate(positionMeas(10.0, 20.0, 2.0, 0.0));
  a.predict(ta, Timestamp::fromSeconds(1.0));
  b.predict(tb, Timestamp::fromSeconds(1.0));
  a.update(ta, positionMeas(11.0, 19.5, 1.0, 1.0));
  b.update(tb, positionMeas(11.0, 19.5, 1.0, 1.0));

  ASSERT_EQ(ta.particles.rows(), tb.particles.rows());
  ASSERT_EQ(ta.particles.cols(), tb.particles.cols());
  for (int j = 0; j < ta.particles.cols(); ++j)
    for (int i = 0; i < ta.particles.rows(); ++i)
      EXPECT_DOUBLE_EQ(ta.particles(i, j), tb.particles(i, j));
  for (int i = 0; i < ta.particle_weights.size(); ++i)
    EXPECT_DOUBLE_EQ(ta.particle_weights(i), tb.particle_weights(i));
}
```

- [ ] **Step 2: Run test to verify it passes**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=ParticleFilterEstimator.DeterministicForSameSeed`
Expected: PASS — proves that two PFs with identical seeds and identical call sequences produce bit-identical ensembles.

- [ ] **Step 3: Commit**

```bash
git add tests/estimation/test_particle_filter_estimator.cpp
git commit -m "estimation: PF determinism test"
```

---

## Task 7: Add PF to the filter comparison harness

**Files:**
- Modify: `tests/scenario/test_filter_comparison.cpp:1-22` (includes), `:123-171` (range/bearing tests)

- [ ] **Step 1: Add the include**

At the top of `tests/scenario/test_filter_comparison.cpp`, add after the `UkfEstimator.hpp` include:

```cpp
#include "core/estimation/ParticleFilterEstimator.hpp"
```

- [ ] **Step 2: Extend `FilterComparison.ShortRangePass`**

Replace the body of `TEST(FilterComparison, ShortRangePass)` with:

```cpp
  std::vector<double> times;
  for (int i = 0; i <= 40; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 5.0 * kPi / 180.0;
  const Scenario s = buildRangeBearingPassScenario(
      Eigen::Vector2d(500.0, 50.0), Eigen::Vector2d(-25.0, 0.0),
      times, 10.0, 10.0, bearing_std, 41);
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator ekf(motion, 10.0);
  const UkfEstimator ukf(motion, 10.0);
  const ParticleFilterEstimator pf(motion, 1000, 10.0, 0.5, 41);

  const RunOutput e = run(ekf, s, 1000.0, 200.0, 1, 5, 60.0);
  const RunOutput u = run(ukf, s, 1000.0, 200.0, 1, 5, 60.0);
  const RunOutput p = run(pf,  s, 1000.0, 200.0, 1, 5, 60.0);

  std::fprintf(stderr,
               "\n[ShortRangePass] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ShortRangePass] UKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[ShortRangePass] PF  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count,
               p.mean_ospa, p.id_switches, p.final_track_count);
```

- [ ] **Step 3: Extend `FilterComparison.VeryShortRangePass`**

Replace the body of `TEST(FilterComparison, VeryShortRangePass)` with:

```cpp
  std::vector<double> times;
  for (int i = 0; i <= 40; ++i) times.push_back(static_cast<double>(i));
  constexpr double kPi = 3.14159265358979323846;
  const double bearing_std = 10.0 * kPi / 180.0;
  const Scenario s = buildRangeBearingPassScenario(
      Eigen::Vector2d(400.0, 20.0), Eigen::Vector2d(-20.0, 0.0),
      times, 15.0, 20.0, bearing_std, 53);
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  const EkfEstimator ekf(motion, 10.0);
  const UkfEstimator ukf(motion, 10.0);
  const ParticleFilterEstimator pf(motion, 1000, 10.0, 0.5, 53);

  const RunOutput e = run(ekf, s, 1500.0, 300.0, 1, 5, 60.0);
  const RunOutput u = run(ukf, s, 1500.0, 300.0, 1, 5, 60.0);
  const RunOutput p = run(pf,  s, 1500.0, 300.0, 1, 5, 60.0);

  std::fprintf(stderr,
               "\n[VeryShortRangePass] EKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[VeryShortRangePass] UKF mean_ospa=%.4f id_switches=%d tracks=%zu"
               "\n[VeryShortRangePass] PF  mean_ospa=%.4f id_switches=%d tracks=%zu\n",
               e.mean_ospa, e.id_switches, e.final_track_count,
               u.mean_ospa, u.id_switches, u.final_track_count,
               p.mean_ospa, p.id_switches, p.final_track_count);
```

- [ ] **Step 4: Build and run**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests --gtest_filter=FilterComparison.*RangePass`
Expected: PASS, with stderr lines reporting EKF / UKF / PF OSPA for each scenario. Record the numbers — they go into Task 9.

- [ ] **Step 5: Commit**

```bash
git add tests/scenario/test_filter_comparison.cpp
git commit -m "scenario: include PF in range/bearing filter comparison"
```

---

## Task 8: Algorithm documentation (estimation.md)

**Files:**
- Modify: `docs/algorithms/estimation.md` (append a section 5)

- [ ] **Step 1: Append the PF section**

Append to `docs/algorithms/estimation.md`:

```markdown
## 5. Particle Filter (`ParticleFilterEstimator`)

**Math.** Bootstrap (sequential importance resampling) with `N` weighted
particles `{xⁱ, wⁱ}`. State and motion model as for the EKF.

- **Initiate:** sample `xⁱ ~ N(μ₀, P₀)` from the same one-point Gaussian
  initiator as EKF / UKF; `wⁱ = 1/N`. Project ensemble → `(track.state,
  track.covariance)`.
- **Predict:** `xⁱ ← F(dt)·xⁱ + ηⁱ`, `ηⁱ ~ N(0, Q(dt))` via Cholesky
  `Q = L Lᵀ`. Weights unchanged. Project ensemble → carrier.
- **Update:** `log wⁱ ← log wⁱ − ½ yⁱᵀ R⁻¹ yⁱ`, `yⁱ = z − h(xⁱ)`
  (bearing wrapped to (−π, π]). Normalize via log-sum-exp:
  `wⁱ ∝ exp(log wⁱ − max log w)`, then `w /= Σ w`. Compute
  `ESS = 1 / Σ (wⁱ)²`. If `ESS < N/2`, **systematic resampling** with a
  single uniform draw `u ∈ [0, 1/N)`. Project ensemble → carrier.
- **Carrier projection:** `x̂ = Σ wⁱ xⁱ`, `P̂ = Σ wⁱ (xⁱ − x̂)(xⁱ − x̂)ᵀ`.

**Assumptions.** Process noise `Q(dt)` is positive-semidefinite (Cholesky
falls back to a deterministic predict when `q = 0` makes `Q` singular);
measurement noise covariance `R` is positive-definite (used inverted in the
log-likelihood); state and measurement spaces are Euclidean apart from the
bearing channel (handled by the same `wrapAngle` residual the EKF / UKF
use). Determinism requires that the call order against a freshly-seeded
estimator be deterministic — the scenario harness guarantees this.

**Rationale.** First estimator that can represent non-Gaussian posteriors
(multimodal range/bearing fusion, bearing-only flow before range converges).
Chosen as the **second** comparison after the UKF because (a) it requires
exactly the same nonlinear measurement model wiring as the UKF, (b) it
trivially handles non-Gaussian priors that an IMM cannot represent at all,
(c) projecting to a Gaussian carrier keeps the pipeline (gating,
association, sinks) estimator-agnostic. Stored as
`(Eigen::MatrixXd, Eigen::VectorXd)` on `Track` itself — colocation gives
clean ownership semantics (no side map, no leak on track deletion) and
parallels the optional ensemble carrier a future IMM would need.

**Ways to improve / test next.** (1) Auxiliary or marginalized particle
filters that use the measurement to bias the predict step — reduces particle
count needed for sharp likelihoods. (2) Stratified or residual resampling
instead of systematic — lower variance in some regimes. (3) Particle
diversity injection (regularized PF) to recover from over-confident
posteriors. (4) Adaptive `N` based on observed ESS — most updates do not
need 1000 particles. (5) Bearing-only scenario (no range channel) where
the PF's multimodal representation will definitively dominate
EKF / UKF. (6) Multi-seed Monte-Carlo sweep over `N ∈ {200, 500, 1000,
2000}` to plot the cost / accuracy frontier.

**Measured behaviour on the current scenario suite.** See
`docs/algorithms/evaluation-log.md`.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/estimation.md
git commit -m "docs: particle filter math, assumptions, rationale, improvements"
```

---

## Task 9: Evaluation log entry

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

- [ ] **Step 1: Append the entry with measured numbers**

After Task 7 the test stderr printed three numbers per scenario. Append to `docs/algorithms/evaluation-log.md` using those numbers in place of `<…>`:

```markdown
## 2026-06-01 — PF vs EKF vs UKF on range/bearing pass scenarios

`ParticleFilterEstimator` with `N=1000`, `ess_fraction=0.5`,
`init_speed_std=10`, seed = scenario seed. Same scenarios, gates, and
thresholds as the previous entry.

| Scenario | Filter | mean OSPA (m) | Δ vs EKF |
|----------|--------|---------------|----------|
| ShortRangePass | EKF | <ekf_short> | — |
| ShortRangePass | UKF | <ukf_short> | <ukf_short_delta> |
| ShortRangePass | PF  | <pf_short>  | <pf_short_delta> |
| VeryShortRangePass | EKF | <ekf_very> | — |
| VeryShortRangePass | UKF | <ukf_very> | <ukf_very_delta> |
| VeryShortRangePass | PF  | <pf_very>  | <pf_very_delta> |

**Takeaway.** <One paragraph: where the PF lands relative to EKF/UKF on each
scenario, whether the gap matches theoretical expectation (PF should match
UKF on mild nonlinearity, pull ahead under strong nonlinearity and
non-Gaussian posteriors), and what the runtime cost looks like by
inspection of `N=1000` vs `2n+1=9` sigma points.>

**Methodology notes.** Single seed per filter, N=1000, ESS threshold 0.5·N.
No bearing-only or true multimodal scenario yet — both range/bearing passes
are still unimodal posteriors once range converges, so the PF's main
theoretical advantage (multimodality) is not exercised. Documented as the
next thing to build.
```

- [ ] **Step 2: Commit**

```bash
git add docs/algorithms/evaluation-log.md
git commit -m "docs: log PF vs EKF/UKF measurements on range/bearing pass scenarios"
```

---

## Task 10: Full test sweep + merge

**Files:** none — verification only.

- [ ] **Step 1: Run the full suite**

Run: `cmake --build build --target navtracker_tests && ./build/navtracker_tests`
Expected: all tests pass (existing 91 + Resampling 5 + ParticleFilterEstimator 6 = 102, give or take).

- [ ] **Step 2: Merge to master**

```bash
git checkout master
git merge --no-ff <feature-branch>
```

- [ ] **Step 3: Stop here**

Plan complete. Next filter (IMM) is a separate plan.
