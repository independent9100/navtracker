# RMC Own-Ship Velocity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Parse NMEA RMC at the own-ship adapter for SOG/COG-derived velocity; provide a GGA-finite-difference fallback when RMC is absent; expose velocity + σ_v on `OwnShipPose`; drop `synthesizeOwnShipTrack`'s explicit velocity args; let CPA σ correctly include own-ship velocity uncertainty.

**Architecture:** New `core/own_ship/OwnShipVelocityEstimator` (sliding-window LS, sibling of `UereEstimator`). `OwnShipNmeaAdapter` gains RMC parsing, holds the velocity estimator, applies the precedence rule. `OwnShipPose` carries the new velocity fields. `synthesizeOwnShipTrack` reads them. Sim's `OwnShipEmitter` learns to emit RMC and to populate the velocity fields when `report_velocity = true` (default false for backward compat).

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-04-rmc-own-ship-velocity-design.md`. Section references in tasks below refer to that spec.

---

## Task 1: `OwnShipPose` velocity fields

**Files:**
- Modify: `adapters/own_ship/OwnShipProvider.hpp`
- Test: `tests/adapters/own_ship/test_own_ship_provider.cpp` (extend)

### Why

Per spec §5.1: add the three fields with backward-compatible defaults. No behavior change downstream until other tasks consume them.

### Steps

- [ ] **Step 1: Add fields**

In `OwnShipPose` (after `position_std_m`):

```cpp
Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
double velocity_std_m_per_s{0.0};
bool velocity_is_valid{false};
```

You'll also need `#include <Eigen/Core>` if not already present.

- [ ] **Step 2: Smoke test**

Append a small test confirming the defaults:

```cpp
TEST(OwnShipPoseTest, VelocityFieldsDefaultUnset) {
  OwnShipPose p;
  EXPECT_DOUBLE_EQ(p.velocity_enu.x(), 0.0);
  EXPECT_DOUBLE_EQ(p.velocity_enu.y(), 0.0);
  EXPECT_DOUBLE_EQ(p.velocity_std_m_per_s, 0.0);
  EXPECT_FALSE(p.velocity_is_valid);
}
```

- [ ] **Step 3: Build + run full suite**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```
Expected: 303/303 green (was 302; +1 new test). All existing tests unaffected because new fields default to safe values.

- [ ] **Step 4: Commit**

```
git add adapters/own_ship/OwnShipProvider.hpp \
        tests/adapters/own_ship/test_own_ship_provider.cpp
git commit -m "own-ship: OwnShipPose carries velocity_enu + std + valid flag"
```

---

## Task 2: `OwnShipVelocityEstimator` core + unit tests

**Files:**
- Create: `core/own_ship/OwnShipVelocityEstimator.hpp`
- Create: `core/own_ship/OwnShipVelocityEstimator.cpp`
- Create: `tests/own_ship/test_own_ship_velocity_estimator.cpp`
- Modify: `CMakeLists.txt` (add to `navtracker_core` + `navtracker_tests`)

### Why

Per spec §4.3 and §5.3: GGA finite-difference velocity estimate with σ_v from LS slope standard error. Mirrors `UereEstimator`'s sliding-window pattern (commit `b87925b`). Lives in `core/` so library users can also wire it directly to a non-NMEA GGA source.

### Steps

- [ ] **Step 1: Write the header**

Create `core/own_ship/OwnShipVelocityEstimator.hpp`:

```cpp
#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipVelocityEstimatorConfig {
  std::size_t window_size{8};
  double maneuver_dv_threshold_mps{0.5};
  double min_sigma_v_m_per_s{0.05};
};

struct OwnShipVelocityEstimate {
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
  double sigma_v_m_per_s{0.0};
  bool is_published{false};
};

class OwnShipVelocityEstimator {
 public:
  explicit OwnShipVelocityEstimator(OwnShipVelocityEstimatorConfig cfg = {});

  void observe(Timestamp t, double x_enu, double y_enu);
  OwnShipVelocityEstimate current() const;
  std::size_t windowSize() const { return samples_.size(); }

 private:
  struct Sample { double t; double x; double y; };
  OwnShipVelocityEstimatorConfig cfg_;
  std::deque<Sample> samples_;
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

Create `core/own_ship/OwnShipVelocityEstimator.cpp`. Mirror `UereEstimator.cpp`'s structure (commit `b87925b`):

- `observe(t, x, y)` pushes a sample and pops front if size > window_size.
- `current()` returns unpublished if size < window_size or < 4.
- LS fit per axis using the same `fitAxis` template the UereEstimator uses (you can either copy the helper or extract it to a shared header — implementer's call; if duplicated, document with a one-line comment).
- Velocity is `(fit_x.b, fit_y.b)` (the slopes).
- σ_v_per_axis = σ_pos_residual / √Σ(dt_i − dt̄)² where σ_pos_residual = √(ss_res_total / (2(N-2))).
- Isotropic σ_v = average per-axis σ.
- Maneuver gating: two-halves velocity comparison with the same noise-aware threshold (Δv > threshold + 3·√2·σ_v_half). When triggered, return `is_published = false`.
- Apply min_sigma_v_m_per_s floor.

- [ ] **Step 3: Wire into CMake**

In `CMakeLists.txt`, add `core/own_ship/OwnShipVelocityEstimator.cpp` to `navtracker_core` (near the existing `core/own_ship/UereEstimator.cpp`).

- [ ] **Step 4: Write unit tests**

Create `tests/own_ship/test_own_ship_velocity_estimator.cpp` with 5 tests per spec §10 (#2):

```cpp
#include "core/own_ship/OwnShipVelocityEstimator.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}
}  // namespace

TEST(OwnShipVelocityEstimatorTest, UnpublishedWhenWindowEmpty) {
  OwnShipVelocityEstimator est{};
  EXPECT_FALSE(est.current().is_published);
}

TEST(OwnShipVelocityEstimatorTest, ConvergesOnConstantVelocityInput) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
  std::mt19937 rng{42};
  const double sigma_pos = 1.0;
  std::normal_distribution<double> n(0.0, sigma_pos);
  const Eigen::Vector2d v_truth(5.0, 3.0);
  for (std::size_t i = 0; i < cfg.window_size; ++i) {
    est.observe(tAt(i * 1.0),
                v_truth.x() * i + n(rng),
                v_truth.y() * i + n(rng));
  }
  const auto e = est.current();
  ASSERT_TRUE(e.is_published);
  EXPECT_NEAR(e.velocity_enu.x(), v_truth.x(), 1.0);
  EXPECT_NEAR(e.velocity_enu.y(), v_truth.y(), 1.0);
  EXPECT_GT(e.sigma_v_m_per_s, 0.0);
}

TEST(OwnShipVelocityEstimatorTest, SuppressesDuringManeuver) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
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

TEST(OwnShipVelocityEstimatorTest, SigmaVTracksNoise) {
  for (double sigma_pos : {0.5, 2.0, 5.0}) {
    OwnShipVelocityEstimatorConfig cfg{};
    OwnShipVelocityEstimator est{cfg};
    std::mt19937 rng{42};
    std::normal_distribution<double> n(0.0, sigma_pos);
    for (int i = 0; i < 20; ++i) {
      est.observe(tAt(i * 1.0), 5.0 * i + n(rng), n(rng));
    }
    const auto e = est.current();
    if (!e.is_published) continue;  // tolerate noisy-window suppression
    EXPECT_GT(e.sigma_v_m_per_s, 0.0);
    EXPECT_LT(e.sigma_v_m_per_s, 3.0 * sigma_pos);  // loose envelope
  }
}

TEST(OwnShipVelocityEstimatorTest, ResumesAfterManeuver) {
  OwnShipVelocityEstimatorConfig cfg{};
  OwnShipVelocityEstimator est{cfg};
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
  std::mt19937 rng{42};
  std::normal_distribution<double> n(0.0, 1.0);
  double t = cfg.window_size * 1.0;
  for (std::size_t i = 0; i < cfg.window_size; ++i, t += 1.0) {
    y += 5.0;
    est.observe(tAt(t), x + n(rng), y + n(rng));
  }
  EXPECT_TRUE(est.current().is_published);
}

}  // namespace navtracker
```

- [ ] **Step 5: Wire test into CMake**

Add `tests/own_ship/test_own_ship_velocity_estimator.cpp` to `navtracker_tests`.

- [ ] **Step 6: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R OwnShipVelocityEstimatorTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 5 new PASS; full suite 308/308 green.

- [ ] **Step 7: Commit**

```
git add core/own_ship/OwnShipVelocityEstimator.hpp core/own_ship/OwnShipVelocityEstimator.cpp \
        tests/own_ship/test_own_ship_velocity_estimator.cpp CMakeLists.txt
git commit -m "own-ship: add OwnShipVelocityEstimator (sliding-window LS)"
```

---

## Task 3: `OwnShipNmeaAdapter` RMC parsing + estimator integration

**Files:**
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.hpp`
- Modify: `adapters/own_ship/OwnShipNmeaAdapter.cpp`
- Modify: `tests/adapters/own_ship/test_own_ship_nmea.cpp` (extend)

### Why

Per spec §3, §4, §5.2: RMC parse path + GGA-derived fallback + precedence logic. The adapter is the single integration point; consumers downstream see a populated `pose.velocity_enu / std / valid` automatically.

### Steps

- [ ] **Step 1: Extend the config**

In `OwnShipNmeaAdapter.hpp`, append to `OwnShipNmeaAdapterConfig`:

```cpp
double sigma_sog_m_per_s{0.5};
double sigma_cog_deg{1.0};
bool prefer_rmc_velocity{true};
double rmc_stale_seconds{5.0};
bool enable_velocity_estimator{true};
OwnShipVelocityEstimatorConfig velocity_estimator_cfg{};
```

Include `core/own_ship/OwnShipVelocityEstimator.hpp`.

- [ ] **Step 2: Add estimator + RMC buffer to the adapter**

Add private members:

```cpp
OwnShipVelocityEstimator velocity_estimator_;
struct RmcBuffer {
  Timestamp time;
  Eigen::Vector2d velocity_enu;
  double sigma_v_m_per_s;
  bool has_value{false};
};
RmcBuffer rmc_buffer_;
```

Construct `velocity_estimator_` from `cfg.velocity_estimator_cfg`.

- [ ] **Step 3: Parse RMC in `ingest`**

Add a branch for `parsed->formatter == "RMC"`:

```cpp
if (parsed->formatter == "RMC") {
  if (parsed->fields.size() < 8) return false;
  // Field 1 = status (A/V). Skip when invalid.
  if (parsed->fields[1] != "A") return false;
  // Fields 2-5 = lat / N|S / lon / E|W. Already-known from GGA path.
  // Field 6 = SOG (knots).
  const double sog_knots = std::strtod(parsed->fields[6].c_str(), nullptr);
  const double sog_m_per_s = sog_knots * 0.514444;
  // Field 7 = COG (degrees true).
  const double cog_deg = std::strtod(parsed->fields[7].c_str(), nullptr);
  const double cog_rad = cog_deg * kDeg2Rad;

  RmcBuffer b;
  b.time = t;
  b.velocity_enu = Eigen::Vector2d(sog_m_per_s * std::sin(cog_rad),
                                   sog_m_per_s * std::cos(cog_rad));
  const double sigma_cog_rad = cfg_.sigma_cog_deg * kDeg2Rad;
  b.sigma_v_m_per_s = std::sqrt(cfg_.sigma_sog_m_per_s * cfg_.sigma_sog_m_per_s
                                + (sog_m_per_s * sigma_cog_rad) * (sog_m_per_s * sigma_cog_rad));
  b.has_value = true;
  rmc_buffer_ = b;
  return true;
}
```

- [ ] **Step 4: Drive estimator from GGA**

In the existing GGA branch, after deriving `enu`, also push to the velocity estimator:

```cpp
if (cfg_.enable_velocity_estimator) {
  velocity_estimator_.observe(t, enu_x, enu_y);
}
```

- [ ] **Step 5: Compose the pose at the end of GGA handling**

After setting position-related pose fields, compute velocity per the precedence rule:

```cpp
const double dt_rmc_s = (rmc_buffer_.has_value)
    ? t.secondsSince(rmc_buffer_.time)
    : std::numeric_limits<double>::infinity();
const bool rmc_fresh = cfg_.prefer_rmc_velocity
                    && rmc_buffer_.has_value
                    && dt_rmc_s >= 0.0
                    && dt_rmc_s <= cfg_.rmc_stale_seconds;

if (rmc_fresh) {
  pose.velocity_enu = rmc_buffer_.velocity_enu;
  pose.velocity_std_m_per_s = rmc_buffer_.sigma_v_m_per_s;
  pose.velocity_is_valid = true;
} else {
  const auto v_est = velocity_estimator_.current();
  if (v_est.is_published) {
    pose.velocity_enu = v_est.velocity_enu;
    pose.velocity_std_m_per_s = v_est.sigma_v_m_per_s;
    pose.velocity_is_valid = true;
  } else {
    pose.velocity_enu = Eigen::Vector2d::Zero();
    pose.velocity_std_m_per_s = 0.0;
    pose.velocity_is_valid = false;
  }
}
```

This block runs each GGA. The RMC branch only updates `rmc_buffer_`; pose composition happens in GGA so the velocity is freshly committed per fix.

- [ ] **Step 6: Write integration tests**

Extend `tests/adapters/own_ship/test_own_ship_nmea.cpp`:

```cpp
TEST(OwnShipNmeaAdapterTest, ParsesRmcSogCogIntoVelocityEnu) {
  // RMC with SOG=10 knots = 5.14 m/s, COG=045° → velocity_enu ≈ (3.64, 3.64).
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});
  const std::string rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,10.0,045.0,230394,003.1,W*<cksum>";
  adapter.ingest(rmc, Timestamp::fromSeconds(0.0));
  // RMC only updates the buffer; pose isn't pushed until a GGA arrives.
  // Feed a minimal GGA to drive composition.
  const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,0.0,M,0.0,M,,*<cksum>";
  adapter.ingest(gga, Timestamp::fromSeconds(0.0));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_TRUE(provider.latest()->velocity_is_valid);
  EXPECT_NEAR(provider.latest()->velocity_enu.x(), 3.6378, 0.01);
  EXPECT_NEAR(provider.latest()->velocity_enu.y(), 3.6378, 0.01);
}

TEST(OwnShipNmeaAdapterTest, RmcZeroSogProducesSigmaSogAsSigmaV) {
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.sigma_sog_m_per_s = 0.5;
  cfg.sigma_cog_deg = 1.0;
  OwnShipNmeaAdapter adapter(provider, cfg);
  const std::string rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,0.0,000.0,230394,,*<cksum>";
  adapter.ingest(rmc, Timestamp::fromSeconds(0.0));
  const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,0.0,M,0.0,M,,*<cksum>";
  adapter.ingest(gga, Timestamp::fromSeconds(0.0));
  EXPECT_NEAR(provider.latest()->velocity_std_m_per_s, 0.5, 1e-6);
}

TEST(OwnShipNmeaAdapterTest, RmcAbsentTriggersEstimatorFallback) {
  // Feed 10 GGAs at 5 m/s east — estimator should publish and pose.velocity
  // becomes valid.
  // ... implementation details ...
}

TEST(OwnShipNmeaAdapterTest, RmcStaleTriggersEstimatorFallback) {
  // Feed one RMC at t=0, then GGAs only for 10 s. Past 5 s the RMC is stale;
  // velocity comes from estimator. Verify velocity_is_valid stays true
  // throughout and the source switches at the right moment.
  // ... implementation details ...
}
```

Use the existing GGA/RMC test helpers if present; otherwise compose sentences inline. For the checksum: existing test_own_ship_nmea.cpp tests pass GGA with placeholder checksums — if the adapter doesn't validate, just keep the placeholder format.

- [ ] **Step 7: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R OwnShipNmeaAdapterTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 4 new PASS; full suite 312/312 green.

- [ ] **Step 8: Commit**

```
git add adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp \
        tests/adapters/own_ship/test_own_ship_nmea.cpp
git commit -m "own-ship-nmea: parse RMC SOG/COG; GGA-derived velocity fallback"
```

---

## Task 4: Sim `OwnShipEmitter` RMC + velocity reporting

**Files:**
- Modify: `sim/OwnShipEmitter.hpp`
- Modify: `sim/OwnShipEmitter.cpp`
- Modify: `tests/sim/test_own_ship_emitter.cpp` (extend)

### Why

Per spec §7: sim emits RMC alongside GGA/HDT when `emit_rmc = true` (default true so RMC path is exercised). When `report_velocity = true` (default false), the emitter also pushes a pose with populated velocity fields (bypasses the adapter-side estimator's warmup). Backward compat is preserved by both defaults.

### Steps

- [ ] **Step 1: Extend the config**

```cpp
bool report_velocity{false};
bool emit_rmc{true};
double sigma_sog_emit_m_per_s{0.1};   // sim's truth-noise floor on SOG
double sigma_cog_emit_deg{0.5};
```

- [ ] **Step 2: Compute truth velocity per cycle**

Per `emit()` cycle, the emitter has `ctx.now` and the trajectory. Two ways to get velocity:
- Differentiate the trajectory analytically if the `ITruthTrajectory` interface exposes velocity (likely yes — `TruthState s = trajectory.eval(now)` probably has a `velocity` field; check).
- Otherwise finite-difference the position between successive cycles.

Use whichever is cleaner.

- [ ] **Step 3: Compose RMC**

When `cfg_.emit_rmc` is true, after the existing GGA emission, compose an `$GPRMC` sentence:

```cpp
const double sog_truth = velocity_enu.norm();
const double cog_truth = std::atan2(velocity_enu.x(), velocity_enu.y()) * 180.0 / kPi;
// Convert SOG to knots; clamp COG to [0, 360).
// Compose RMC body: "GPRMC,000000.00,A,lat,N/S,lon,E/W,SOG_knots,COG_deg,date,,"
// wrap with checksum, push to adapter.
```

- [ ] **Step 4: Optionally push pose velocity directly**

When `cfg_.report_velocity` is true, set the velocity on the pose directly via a sticky setter:

```cpp
if (cfg_.report_velocity) {
  adapter_.setNextVelocity(velocity_enu, cfg_.sigma_sog_emit_m_per_s);
}
```

This means adding `setNextVelocity` (a small "sticky setter" mirroring `setPositionStd`) to `OwnShipNmeaAdapter`. When set, the adapter overrides the RMC/estimator-derived velocity with this value on the next pose. Documented as a sim-only hook.

Alternative: just rely on the emitted RMC. Skip the sticky setter unless it simplifies sim tests. Implementer's call.

- [ ] **Step 5: Test**

```cpp
TEST(OwnShipEmitter, EmittedRmcParsesBackToVelocity) {
  // Set up emitter with emit_rmc=true, sigma_gps_pos_m=0, gps_pos_std=...,
  // truth trajectory with v=(5,3). After one emit cycle, provider.latest()
  // velocity_enu should be close to (5,3) — within parsing + noise.
}

TEST(OwnShipEmitter, DefaultDoesNotReportVelocity) {
  // emit_rmc defaults to true so RMC is emitted, BUT the adapter's
  // velocity_is_valid won't necessarily be true on the first GGA. Verify
  // backward compat: tests that previously passed still pass.
  //
  // Easier: assert that with report_velocity = false (default) and a
  // single-tick emit, behavior matches the pre-RMC code path on
  // existing tests. This may require keeping emit_rmc default = false
  // for stricter compat. Implementer choice.
}
```

There's a real backward-compat decision here. If `emit_rmc` defaults true, every existing sim test will start receiving RMCs and the adapter will populate velocity on every pose. This likely changes downstream behavior in subtle ways — for example, `synthesizeOwnShipTrack` in CPA tests will now get non-zero velocity covariance.

**Recommendation:** default `emit_rmc = false` to match the conservative pattern set by `report_gps_std = false`. Tests opting in get RMC. Document the symmetry in the spec note.

- [ ] **Step 6: Build + run**

```
ctest --test-dir build --output-on-failure
```
Expected: full suite green. Watch for any IMM3-vs-CV style margin flips — if anything fails, isolate the cause (almost certainly the new velocity propagation changing downstream covariance) and revert to the most-conservative default that keeps existing tests byte-identical.

- [ ] **Step 7: Commit**

```
git add sim/OwnShipEmitter.hpp sim/OwnShipEmitter.cpp \
        adapters/own_ship/OwnShipNmeaAdapter.hpp adapters/own_ship/OwnShipNmeaAdapter.cpp \
        tests/sim/test_own_ship_emitter.cpp
git commit -m "sim: own-ship emitter emits RMC; report_velocity flag (default off)"
```

---

## Task 5: `synthesizeOwnShipTrack` reads velocity from pose

**Files:**
- Modify: `core/collision/CpaOwnShip.hpp`
- Modify: `core/collision/CpaOwnShip.cpp`
- Modify: `tests/collision/test_cpa_synthesize_own_ship.cpp` (update existing + add 2)
- Modify: `tests/scenario/test_cpa_scenario.cpp` (update existing + add 1)
- Modify: `tests/sim/test_bus_cpa_uncertainty.cpp` (update if affected)

### Why

Per spec §6: drop the explicit `velocity_enu` and `sigma_pos_m` arguments — both come from the pose now. CPA σ_cpa now reflects σ_v through the existing Jacobian (no changes to `computeCpaWithUncertainty` itself).

### Steps

- [ ] **Step 1: Update the helper**

In `core/collision/CpaOwnShip.hpp`:

```cpp
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const geo::Datum& datum);
```

Implementation in `core/collision/CpaOwnShip.cpp`:
- ENU position from datum conversion (unchanged).
- Velocity from `pose.velocity_enu`.
- Covariance: position diagonal from `pose.position_std_m²`; velocity diagonal from `pose.velocity_std_m_per_s² · pose.velocity_is_valid` (i.e., zero when invalid).

- [ ] **Step 2: Update existing CpaOwnShip tests**

`tests/collision/test_cpa_synthesize_own_ship.cpp`:
- Update the two existing tests' calls to the new signature.
- Add `SynthesizedTrackCarriesVelocityCovariance`: pose with `velocity_enu=(5,3)`, `velocity_std=1.0`, `velocity_is_valid=true` → state(2..3) = (5,3), covariance(2,2) = covariance(3,3) = 1.0.
- Add `InvalidVelocityProducesZeroVelocityCovariance`: same setup but `velocity_is_valid=false` → covariance(2..3,2..3) all zero.

- [ ] **Step 3: Update existing CPA scenario test**

`tests/scenario/test_cpa_scenario.cpp`: the existing test passes `velocity=zero` and `sigma_pos=1.0` as separate args. Update to construct `OwnShipPose` with those fields. Add one new test that sets `velocity_is_valid = true` and `velocity_std = 1.0` and asserts σ_cpa grows compared to the σ_v = 0 baseline.

- [ ] **Step 4: Update sim CPA test**

`tests/sim/test_bus_cpa_uncertainty.cpp` (added in commit 2d1d741) also calls `synthesizeOwnShipTrack`. Update the call signature; since the sim test runs without RMC (default off), `pose.velocity_is_valid = false` → covariance stays compatible with the pre-change behavior. Verify.

- [ ] **Step 5: Build + run**

```
ctest --test-dir build --output-on-failure
```

Expected: green. Existing CPA numbers byte-identical when velocity_is_valid is false; new tests pass.

- [ ] **Step 6: Commit**

```
git add core/collision/CpaOwnShip.hpp core/collision/CpaOwnShip.cpp \
        tests/collision/test_cpa_synthesize_own_ship.cpp \
        tests/scenario/test_cpa_scenario.cpp \
        tests/sim/test_bus_cpa_uncertainty.cpp
git commit -m "cpa: synthesizeOwnShipTrack reads velocity + std from pose"
```

---

## Task 6: Eval-log section

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

### Why

Per spec §10: record CPA σ growth when σ_v is non-zero on a representative scenario.

### Steps

- [ ] **Step 1: Capture numbers from the new CPA scenario test**

Re-run `test_cpa_scenario` and capture both variants' σ_cpa:

```
ctest --test-dir build -R CpaScenario --output-on-failure 2>&1 | tee /tmp/cpa_rmc.txt
```

- [ ] **Step 2: Append eval-log section**

```markdown
## RMC velocity + CPA σ (2026-06-04)

**Setup.** Closes the v1 simplification σ_v_own = 0 from the CPA spec.
RMC SOG/COG parsing in OwnShipNmeaAdapter, with a GGA-finite-difference
fallback when RMC is absent. synthesizeOwnShipTrack now reads velocity
and σ_v from the pose; CPA's existing Jacobian propagates σ_v into
σ_cpa with no further changes.

### Perpendicular-pass scenario (truth CPA = 1000 m, t_ref = 10 s)

| pose σ_pos | pose σ_v | predicted CPA | σ_cpa | P(<500m) |
|---|---|---|---|---|
| 1 m | 0 m/s | <fill> | <fill> | <fill> |
| 1 m | 1 m/s | <fill> | <fill> | <fill> |
| 5 m | 0 m/s | <fill> | <fill> | <fill> |
| 5 m | 1 m/s | <fill> | <fill> | <fill> |

### Verdict

<3-5 sentences. σ_cpa grows by `O(σ_v · TCPA)` as expected — at
TCPA = 100 s and σ_v = 1 m/s the contribution is ~100 m to σ_cpa,
visible in the table. CPA mean is unchanged (velocity uncertainty
doesn't bias the prediction, only widens its band). RMC parsing
closes the input-contract gap from the CPA spec; the GGA fallback
ensures degraded operation when only GGA is being received.>
```

Fill `<fill>` from the captured numbers; write the verdict in the same tone as prior eval-log sections.

- [ ] **Step 3: Commit**

```
git add docs/algorithms/evaluation-log.md
git commit -m "eval-log: RMC velocity flows into CPA σ; perpendicular-pass table"
```

---

## Done criteria

- All 6 tasks committed.
- Full suite green (≥ 312/312).
- `OwnShipPose` carries velocity + σ_v + valid flag.
- `OwnShipNmeaAdapter` parses RMC; GGA-derived fallback works; precedence rule honored.
- `synthesizeOwnShipTrack` consumes the new pose fields cleanly.
- Eval-log records the σ_v contribution to σ_cpa.
- Existing tests byte-identical when `report_velocity = false` and `emit_rmc = false` (sim) — backward compat preserved.
