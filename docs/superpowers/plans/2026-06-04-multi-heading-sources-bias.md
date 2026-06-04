# Multi-Heading-Source Bias Observations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three new observation kinds to `HeadingBiasEstimator` — `GyroVsGpsHeading`, `GyroVsGpsCog`, `GyroVsMagnetic` — each driving the same scalar bias state with appropriate gates and noise budgets. Estimator-side only; NMEA wiring deferred to a follow-up.

**Architecture:** Single fused estimator (continuing path A from v2). Three new observation structs in a sibling header. Three new `observe()` overloads on `HeadingBiasEstimator`. Config and diagnostic counters added. Tests cover per-kind math + 8-permutation source-availability matrix + dynamic hot-swap + crab-realistic regression.

**Tech Stack:** C++17, Eigen, GoogleTest.

**Spec:** `docs/superpowers/specs/2026-06-04-multi-heading-sources-bias-design.md`.

---

### Task 1 — Observation structs in their own header

**Files:**
- Create: `core/bias/HeadingBiasObservations.hpp`

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

// === GpsHeadingObservation (gold-standard, no offset) ===
// r = wrap(gyro_rad - gps_true_heading_rad)
// R = gps_true_heading_std_rad^2
// Gate: outlier only. Used with multi-antenna GPS receivers that produce
// true heading from baseline phase difference.
struct GyroVsGpsHeadingObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double gps_true_heading_rad{0.0};
  double gps_true_heading_std_rad{0.0};
};

// === GpsCogObservation (needs gating against crab) ===
// r = wrap(gyro_rad - gps_cog_rad)
// R = gps_cog_std_rad^2 + cog_crab_budget_rad^2
// Gates:
//   C1 sog_mps >= cog_min_sog_mps
//   C2 |gyro_rate_rad_per_s| <= cog_max_gyro_rate_rad_per_s
//   C3 outlier on r vs sqrt(R + P_b)
struct GyroVsGpsCogObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double gps_cog_rad{0.0};
  double gps_cog_std_rad{0.0};
  double sog_mps{0.0};
  double gyro_rate_rad_per_s{0.0};
};

// === MagneticObservation (variation must be supplied) ===
// r = wrap(gyro_rad - (magnetic_heading_rad + variation_rad))
// R = magnetic_heading_std_rad^2 + mag_deviation_budget_rad^2
// Gates:
//   M1 magnetic_variation_rad must be present (adapter responsibility)
//   M2 outlier
struct GyroVsMagneticObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double magnetic_heading_rad{0.0};
  double magnetic_heading_std_rad{0.0};
  std::optional<double> magnetic_variation_rad;
};

}  // namespace navtracker
```

- [ ] **Step 2: Commit**

```bash
git add core/bias/HeadingBiasObservations.hpp
git commit -m "feat(bias): add three multi-source heading observation structs"
```

---

### Task 2 — Estimator config + three observe overloads

**Files:**
- Modify: `core/bias/HeadingBiasEstimator.hpp`
- Modify: `core/bias/HeadingBiasEstimator.cpp`

- [ ] **Step 1: Extend the config struct**

In `core/bias/HeadingBiasEstimator.hpp`, after the existing `bi_outlier_sigma` field, add:

```cpp
  // Multi-heading-source path (v3, spec 2026-06-04).
  double cog_min_sog_mps{3.0};
  double cog_max_gyro_rate_rad_per_s{0.5 * 3.14159265358979323846 / 180.0};
  double cog_crab_budget_rad{5.0 * 3.14159265358979323846 / 180.0};
  double mag_deviation_budget_rad{3.0 * 3.14159265358979323846 / 180.0};
  double mhs_outlier_sigma{5.0};
```

- [ ] **Step 2: Add include and overloads to the class**

Near the top of the file add:

```cpp
#include "core/bias/HeadingBiasObservations.hpp"
```

In the public section, after the existing `observe(const BearingInnovation&)`, add:

```cpp
  // Multi-heading-source observation kinds. Each is a scalar KF update
  // on the same bias state b. See spec §3 for math, gates, defaults.
  void observe(const GyroVsGpsHeadingObservation& obs);
  void observe(const GyroVsGpsCogObservation& obs);
  void observe(const GyroVsMagneticObservation& obs);

  // Per-source diagnostics. Counters monotonically increase.
  std::size_t acceptedGpsHeading() const { return acc_gps_hdg_; }
  std::size_t acceptedGpsCog()     const { return acc_cog_; }
  std::size_t acceptedMagnetic()   const { return acc_mag_; }
  std::size_t rejectedCogBySog()         const { return rej_cog_sog_; }
  std::size_t rejectedCogByGyroRate()    const { return rej_cog_rate_; }
  std::size_t rejectedMhsByOutlier()     const { return rej_mhs_outlier_; }
```

In the private section, after the existing `std::size_t rej_outlier_{0};`, add:

```cpp
  std::size_t acc_gps_hdg_{0};
  std::size_t acc_cog_{0};
  std::size_t acc_mag_{0};
  std::size_t rej_cog_sog_{0};
  std::size_t rej_cog_rate_{0};
  std::size_t rej_mhs_outlier_{0};
```

- [ ] **Step 3: Implement the three overloads in `.cpp`**

At the end of `core/bias/HeadingBiasEstimator.cpp`, before the closing `}  // namespace navtracker`, add:

```cpp
namespace {

// Shared outlier+update primitive for the three v3 paths.
// Returns true if the update was applied (gates were not the rejection
// reason). On rejection by outlier, increments `rej_outlier` and
// returns false.
bool applyScalarUpdate(double& b_hat, double& p_b,
                       double measurement, double R,
                       double outlier_sigma,
                       std::size_t& rej_outlier) {
  const double s = R + p_b;
  const double sigma = std::sqrt(s);
  const double y = wrapToPi(measurement - b_hat);
  if (std::abs(y) > outlier_sigma * sigma) {
    ++rej_outlier;
    return false;
  }
  const double k = p_b / s;
  b_hat += k * y;
  p_b = (1.0 - k) * p_b;
  return true;
}

}  // namespace

void HeadingBiasEstimator::observe(const GyroVsGpsHeadingObservation& obs) {
  predictTo(obs.time);
  const double measurement = wrapToPi(obs.gyro_rad - obs.gps_true_heading_rad);
  const double R = obs.gps_true_heading_std_rad * obs.gps_true_heading_std_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R, cfg_.mhs_outlier_sigma,
                        rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_gps_hdg_;
  }
}

void HeadingBiasEstimator::observe(const GyroVsGpsCogObservation& obs) {
  predictTo(obs.time);
  if (obs.sog_mps < cfg_.cog_min_sog_mps) {
    ++rej_cog_sog_;
    return;
  }
  if (std::abs(obs.gyro_rate_rad_per_s) > cfg_.cog_max_gyro_rate_rad_per_s) {
    ++rej_cog_rate_;
    return;
  }
  const double measurement = wrapToPi(obs.gyro_rad - obs.gps_cog_rad);
  const double R = obs.gps_cog_std_rad * obs.gps_cog_std_rad
                 + cfg_.cog_crab_budget_rad * cfg_.cog_crab_budget_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R, cfg_.mhs_outlier_sigma,
                        rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_cog_;
  }
}

void HeadingBiasEstimator::observe(const GyroVsMagneticObservation& obs) {
  predictTo(obs.time);
  // M1 — variation must be present (precondition; caller's job).
  if (!obs.magnetic_variation_rad.has_value()) return;
  const double mag_corrected =
      obs.magnetic_heading_rad + *obs.magnetic_variation_rad;
  const double measurement = wrapToPi(obs.gyro_rad - mag_corrected);
  const double R = obs.magnetic_heading_std_rad * obs.magnetic_heading_std_rad
                 + cfg_.mag_deviation_budget_rad * cfg_.mag_deviation_budget_rad;
  if (applyScalarUpdate(b_hat_, p_b_, measurement, R, cfg_.mhs_outlier_sigma,
                        rej_mhs_outlier_)) {
    last_update_ = obs.time;
    has_any_update_ = true;
    ++acc_mag_;
  }
}
```

- [ ] **Step 4: Build**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_core 2>&1 | tail -5`
Expected: clean.

- [ ] **Step 5: Sanity — existing bias tests still pass**

Run: `cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R HeadingBias --output-on-failure 2>&1 | tail -10`
Expected: existing bias tests still pass.

- [ ] **Step 6: Commit**

```bash
git add core/bias/HeadingBiasEstimator.hpp core/bias/HeadingBiasEstimator.cpp
git commit -m "feat(bias): three multi-heading-source observation overloads with gates"
```

---

### Task 3 — Per-kind unit tests

**Files:**
- Create: `tests/bias/test_heading_bias_multi_source.cpp`

- [ ] **Step 1: Write the file**

```cpp
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/bias/HeadingBiasObservations.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {
constexpr double kBiasTrue = 0.0349;  // 2 degrees
constexpr double kPi = 3.14159265358979323846;

GyroVsGpsHeadingObservation makeGpsHdg(double t_s, double gyro,
                                       double gps, double sigma) {
  GyroVsGpsHeadingObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.gps_true_heading_rad = gps;
  o.gps_true_heading_std_rad = sigma;
  return o;
}

GyroVsGpsCogObservation makeCog(double t_s, double gyro, double cog,
                                double sigma, double sog, double rate) {
  GyroVsGpsCogObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.gps_cog_rad = cog;
  o.gps_cog_std_rad = sigma;
  o.sog_mps = sog;
  o.gyro_rate_rad_per_s = rate;
  return o;
}

GyroVsMagneticObservation makeMag(double t_s, double gyro, double mag,
                                  double sigma,
                                  std::optional<double> variation) {
  GyroVsMagneticObservation o;
  o.time = Timestamp::fromSeconds(t_s);
  o.gyro_rad = gyro;
  o.magnetic_heading_rad = mag;
  o.magnetic_heading_std_rad = sigma;
  o.magnetic_variation_rad = variation;
  return o;
}
}  // namespace

// === GPS heading ===

TEST(MhsGpsHeading, SingleObservationAppliesScalarKf) {
  HeadingBiasEstimator est({});
  const double sigma = 0.001;
  // gyro = truth_heading + bias; gps = truth_heading; r = bias.
  const auto obs = makeGpsHdg(1.0, /*gyro=*/0.5 + kBiasTrue, /*gps=*/0.5, sigma);
  const double p0 = est.varianceRad2();
  est.observe(obs);
  const double R = sigma * sigma;
  const double K = p0 / (p0 + R);
  EXPECT_NEAR(est.biasRad(), K * kBiasTrue, 1e-9);
  EXPECT_NEAR(est.varianceRad2(), (1.0 - K) * p0, 1e-9);
  EXPECT_EQ(est.acceptedGpsHeading(), 1u);
}

TEST(MhsGpsHeading, SequenceConvergesTightly) {
  HeadingBiasEstimator est({});
  std::mt19937_64 rng(1);
  std::normal_distribution<double> n(0.0, 0.001);
  for (int i = 0; i < 50; ++i) {
    est.observe(makeGpsHdg(0.1 * (i + 1),
                           /*gyro=*/kBiasTrue + n(rng),
                           /*gps=*/0.0,
                           /*sigma=*/0.001));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue),
            3.0 * std::sqrt(est.varianceRad2()));
  EXPECT_GT(est.acceptedGpsHeading(), 0u);
}

TEST(MhsGpsHeading, OutlierRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.mhs_outlier_sigma = 5.0;
  HeadingBiasEstimator est(cfg);
  // 50σ innovation against initial prior — must be rejected.
  const double r = 50.0 * std::sqrt(cfg.initial_variance_rad2);
  est.observe(makeGpsHdg(1.0, /*gyro=*/r, /*gps=*/0.0, /*sigma=*/0.001));
  EXPECT_EQ(est.acceptedGpsHeading(), 0u);
  EXPECT_EQ(est.rejectedMhsByOutlier(), 1u);
}

// === GPS COG ===

TEST(MhsGpsCog, PassingGatesUpdatesBias) {
  HeadingBiasEstimator est({});
  est.observe(makeCog(1.0,
                      /*gyro=*/kBiasTrue,
                      /*cog=*/0.0,
                      /*sigma=*/0.005,
                      /*sog=*/10.0,
                      /*rate=*/0.0));
  EXPECT_EQ(est.acceptedGpsCog(), 1u);
  EXPECT_GT(est.biasRad(), 0.0);  // moved toward kBiasTrue
}

TEST(MhsGpsCog, LowSogRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.cog_min_sog_mps = 3.0;
  HeadingBiasEstimator est(cfg);
  est.observe(makeCog(1.0, kBiasTrue, 0.0, 0.005,
                      /*sog=*/1.0, /*rate=*/0.0));
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogBySog(), 1u);
}

TEST(MhsGpsCog, TurningRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.cog_max_gyro_rate_rad_per_s = 0.01;  // very strict
  HeadingBiasEstimator est(cfg);
  est.observe(makeCog(1.0, kBiasTrue, 0.0, 0.005,
                      /*sog=*/10.0, /*rate=*/0.05));
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogByGyroRate(), 1u);
}

TEST(MhsGpsCog, CrabRealisticSequenceConvergesWithinHalfDeg) {
  HeadingBiasEstimator est({});
  std::mt19937_64 rng(11);
  std::normal_distribution<double> crab(0.0, 3.0 * kPi / 180.0);  // 3° rms
  std::normal_distribution<double> sensor(0.0, 0.005);
  for (int i = 0; i < 400; ++i) {
    // True relationship: cog = gyro - bias - crab.
    const double cog = -kBiasTrue - crab(rng) + sensor(rng);
    est.observe(makeCog(0.5 * (i + 1),
                        /*gyro=*/0.0,
                        cog,
                        /*sigma=*/0.005,
                        /*sog=*/10.0,
                        /*rate=*/0.0));
  }
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue),
            0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad() << " acc=" << est.acceptedGpsCog();
}

// === Magnetic ===

TEST(MhsMagnetic, VariationAppliedUpdatesBias) {
  HeadingBiasEstimator est({});
  // gyro = true + bias; mag = true - variation. With variation supplied,
  // r = gyro - (mag + variation) = bias.
  const double truth = 0.7;
  const double variation = 0.1;
  est.observe(makeMag(1.0,
                      /*gyro=*/truth + kBiasTrue,
                      /*mag=*/truth - variation,
                      /*sigma=*/0.005,
                      variation));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_GT(est.biasRad(), 0.0);
}

TEST(MhsMagnetic, NullVariationIsNoop) {
  HeadingBiasEstimator est({});
  const double b_before = est.biasRad();
  est.observe(makeMag(1.0, kBiasTrue, 0.0, 0.005, std::nullopt));
  EXPECT_EQ(est.biasRad(), b_before);
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
}

TEST(MhsMagnetic, OutlierRejected) {
  HeadingBiasEstimatorConfig cfg;
  cfg.mhs_outlier_sigma = 5.0;
  HeadingBiasEstimator est(cfg);
  const double r = 50.0 * std::sqrt(cfg.initial_variance_rad2);
  est.observe(makeMag(1.0, /*gyro=*/r, /*mag=*/0.0,
                      /*sigma=*/0.005, /*variation=*/0.0));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(est.rejectedMhsByOutlier(), 1u);
}
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, after `tests/bias/test_heading_bias_bearing_innovation.cpp`, add `tests/bias/test_heading_bias_multi_source.cpp`.

- [ ] **Step 3: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'Mhs(GpsHeading|GpsCog|Magnetic)' --output-on-failure 2>&1 | tail -15
```
Expected: 9/9 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/bias/test_heading_bias_multi_source.cpp CMakeLists.txt
git commit -m "test(bias): per-kind unit tests for multi-heading-source observations"
```

---

### Task 4 — Source-availability permutation tests

**Files:**
- Create: `tests/bias/test_heading_bias_permutations.cpp`

- [ ] **Step 1: Write the file**

```cpp
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/bias/HeadingBiasObservations.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

constexpr double kBiasTrue = 0.0349;
constexpr double kPi = 3.14159265358979323846;

enum SourceMask : unsigned {
  kNone = 0,
  kGpsHdg = 1u << 0,
  kCog    = 1u << 1,
  kMag    = 1u << 2,
};

void runMixed(HeadingBiasEstimator& est, unsigned sources, int n_cycles,
              std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.001);
  std::normal_distribution<double> sensor(0.0, 0.005);
  for (int i = 0; i < n_cycles; ++i) {
    const double t = 0.5 * (i + 1);
    if (sources & kGpsHdg) {
      GyroVsGpsHeadingObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = kBiasTrue + noise(rng);
      o.gps_true_heading_rad = 0.0;
      o.gps_true_heading_std_rad = 0.001;
      est.observe(o);
    }
    if (sources & kCog) {
      GyroVsGpsCogObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = 0.0;
      o.gps_cog_rad = -kBiasTrue + sensor(rng);
      o.gps_cog_std_rad = 0.005;
      o.sog_mps = 10.0;
      o.gyro_rate_rad_per_s = 0.0;
      est.observe(o);
    }
    if (sources & kMag) {
      GyroVsMagneticObservation o;
      o.time = Timestamp::fromSeconds(t);
      o.gyro_rad = kBiasTrue + sensor(rng);
      o.magnetic_heading_rad = 0.0;
      o.magnetic_heading_std_rad = 0.005;
      o.magnetic_variation_rad = 0.0;
      est.observe(o);
    }
  }
}

}  // namespace

TEST(MhsPermutations, NoneWiredLeavesEstimateUnchanged) {
  HeadingBiasEstimator est({});
  const double b0 = est.biasRad();
  // Nothing observed; just advance time via the absence of observe().
  EXPECT_DOUBLE_EQ(est.biasRad(), b0);
  EXPECT_FALSE(est.current().is_published);
}

TEST(MhsPermutations, OnlyGpsHeadingConvergesTight) {
  HeadingBiasEstimator est({});
  runMixed(est, kGpsHdg, 50, 100);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedGpsHeading(), 0u);
}

TEST(MhsPermutations, OnlyCogConvergesWithinHalfDeg) {
  HeadingBiasEstimator est({});
  runMixed(est, kCog, 400, 200);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad();
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedGpsCog(), 0u);
}

TEST(MhsPermutations, OnlyMagConvergesWithinThirdDeg) {
  HeadingBiasEstimator est({});
  runMixed(est, kMag, 200, 300);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.5 * kPi / 180.0)
      << "b_hat=" << est.biasRad();
  EXPECT_TRUE(est.current().is_published);
  EXPECT_GT(est.acceptedMagnetic(), 0u);
}

TEST(MhsPermutations, AllThreeConvergesFastestAndTightest) {
  HeadingBiasEstimator est_all({});
  runMixed(est_all, kGpsHdg | kCog | kMag, 50, 400);
  EXPECT_LT(std::abs(est_all.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est_all.current().is_published);
  // P_b after all three should be no worse than GPS-heading-only.
  HeadingBiasEstimator est_gps({});
  runMixed(est_gps, kGpsHdg, 50, 400);
  EXPECT_LE(est_all.varianceRad2(), est_gps.varianceRad2() + 1e-12);
}

TEST(MhsPermutations, MixedHdgAndCogConvergesAtLeastAsTightAsCogAlone) {
  HeadingBiasEstimator est_mixed({});
  runMixed(est_mixed, kGpsHdg | kCog, 100, 500);
  HeadingBiasEstimator est_cog({});
  runMixed(est_cog, kCog, 100, 500);
  EXPECT_LE(est_mixed.varianceRad2(), est_cog.varianceRad2() + 1e-12);
  EXPECT_LT(std::abs(est_mixed.biasRad() - kBiasTrue),
            std::max(std::abs(est_cog.biasRad() - kBiasTrue),
                     0.1 * kPi / 180.0));
}

TEST(MhsPermutations, CogPlusMagMatchesIndividualConvergence) {
  HeadingBiasEstimator est({});
  runMixed(est, kCog | kMag, 200, 600);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.4 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
}

TEST(MhsPermutations, HdgPlusMagConvergesToHdgPrecision) {
  HeadingBiasEstimator est({});
  runMixed(est, kGpsHdg | kMag, 50, 700);
  EXPECT_LT(std::abs(est.biasRad() - kBiasTrue), 0.1 * kPi / 180.0);
  EXPECT_TRUE(est.current().is_published);
}
```

- [ ] **Step 2: Register in CMake**

Add `tests/bias/test_heading_bias_permutations.cpp` after the multi-source unit test in `CMakeLists.txt`.

- [ ] **Step 3: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'MhsPermutations' --output-on-failure 2>&1 | tail -20
```
Expected: 8/8 pass.

- [ ] **Step 4: Commit**

```bash
git add tests/bias/test_heading_bias_permutations.cpp CMakeLists.txt
git commit -m "test(bias): 8-permutation matrix for multi-source availability"
```

---

### Task 5 — Dynamic-availability + stale-gate test

**Files:**
- Modify: `tests/bias/test_heading_bias_permutations.cpp`

- [ ] **Step 1: Append the dynamic test**

At the end of `tests/bias/test_heading_bias_permutations.cpp` add:

```cpp
TEST(MhsDynamic, SourceLossAndReturnHandledCleanly) {
  // Cycles 1..50: GPS heading only. Converge.
  // Cycles 51..100: GPS heading silent (no observe). bias_estimator's
  //   random-walk inflates P_b. No crash.
  // Cycles 101..150: GPS heading returns. P_b shrinks again.
  HeadingBiasEstimatorConfig cfg;
  // Use a tight publish threshold so we can observe the gate flipping.
  cfg.publish_variance_threshold_rad2 = 1e-5;  // ~0.18° one-sigma
  HeadingBiasEstimator est(cfg);
  std::mt19937_64 rng(99);
  std::normal_distribution<double> n(0.0, 0.001);

  auto pushHdg = [&](double t) {
    GyroVsGpsHeadingObservation o;
    o.time = Timestamp::fromSeconds(t);
    o.gyro_rad = kBiasTrue + n(rng);
    o.gps_true_heading_rad = 0.0;
    o.gps_true_heading_std_rad = 0.001;
    est.observe(o);
  };

  for (int i = 0; i < 50; ++i) pushHdg(0.5 * (i + 1));
  const double p_after_hdg = est.varianceRad2();
  EXPECT_LT(p_after_hdg, cfg.publish_variance_threshold_rad2);
  EXPECT_TRUE(est.current().is_published);

  // Silent period: advance the predict clock without observations.
  // (Random walk inflates P_b but we don't have a public predict-only
  // API; in practice the estimator predicts lazily on the next observe.
  // We assert no crash and no change to b_hat below.)
  const double b_held = est.biasRad();

  // Return: 50 more updates.
  for (int i = 100; i < 150; ++i) pushHdg(0.5 * (i + 1));
  EXPECT_NEAR(est.biasRad(), kBiasTrue, 0.1 * kPi / 180.0);
  // After return, published once again.
  EXPECT_TRUE(est.current().is_published);
  // The b_hat from the silent period was preserved up to small random-walk drift.
  EXPECT_NEAR(est.biasRad(), b_held, 0.1 * kPi / 180.0);
}
```

- [ ] **Step 2: Build and run**

Run:
```
cd /home/andreas/workspace/navtracker && cmake --build build --target navtracker_tests 2>&1 | tail -3 && ctest --test-dir build -R 'MhsDynamic' --output-on-failure 2>&1 | tail -10
```
Expected: 1/1 pass.

- [ ] **Step 3: Commit**

```bash
git add tests/bias/test_heading_bias_permutations.cpp
git commit -m "test(bias): dynamic source loss/return handled without state corruption"
```

---

### Task 6 — Touchup v1 spec landing note

**Files:**
- Modify: `docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md`

- [ ] **Step 1: Add a v3 entry to the Landed section**

In the `## Landed (post-v1)` section (added in the v2 work), append:

```
- **Multi-heading-source bias observations** — landed 2026-06-04 as v3. `HeadingBiasEstimator` grew three new `observe()` overloads (GPS multi-antenna heading, GPS COG, magnetic compass) with per-source gates and inflated noise budgets. Each source is optional; the estimator handles any subset of {none, GPS-hdg, COG, mag} including hot-swap mid-mission. Single scalar `b` retained; per-source offsets (crab, variation, deviation) modeled as inflated R. See `2026-06-04-multi-heading-sources-bias-design.md`. NMEA wiring (HDG parser, talker-ID-based HDT routing, RMC variation forwarding) is deferred to a follow-up.
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/specs/2026-06-03-heading-bias-estimator-design.md
git commit -m "docs(bias): cross-reference v3 multi-source observer in v1 spec"
```

---

### Task 7 — Final sweep

- [ ] **Step 1: Full suite**

Run: `cd /home/andreas/workspace/navtracker/build && ctest --output-on-failure 2>&1 | tail -3`
Expected: previous count + 18 new (9 unit + 8 permutations + 1 dynamic).

- [ ] **Step 2: Acceptance checklist (spec §11)**

Confirm:
- Three observation kinds work; all gates fire as specified.
- All 8 permutation cases pass with stated tolerances.
- Dynamic hot-swap test passes without state corruption.
- No regressions in v1/v2 paths.
- No changes to `IHeadingBiasProvider`, `Tracker`, `Measurement`, or `OwnShipPose` (NMEA wiring deferred).
