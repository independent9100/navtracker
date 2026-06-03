# Library-Friendliness Pass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make navtracker first-class usable as a library — pre-parsed `Measurement` and `OwnShipPose` as the canonical contract; pessimistic defaults for missing covariance; small builders for the common construction patterns; timestamp-aware pose lookup; CMake split so consumers can link the core without NMEA parsing; an end-to-end example; and a brief documentation pass.

**Architecture:** Three internal CMake libraries — `navtracker_core` (domain + ports + helpers, no I/O), `navtracker_nmea` (NMEA-format adapters, optional), `navtracker_sim` (synthetic generators, optional). New `SensorDefaults` struct and helpers live in `core/types/`. `MeasurementBuilders` provides the relative-bearing / true-bearing / ENU-position construction surface. `OwnShipProvider` gains a small ring buffer of recent poses with `poseAtOrBefore(t)` lookup. `Projection` moves to `core/projection/` so core code can use it without violating the architecture invariant.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-03-library-friendliness-design.md`. The numbered sections in tasks below refer to that spec.

---

## Task 1: `SensorDefaults` + `Measurement.covariance_is_default` flag

**Files:**
- Create: `core/types/SensorDefaults.hpp`, `core/types/SensorDefaults.cpp`
- Modify: `core/types/Measurement.hpp` (add field)
- Create: `tests/types/test_sensor_defaults.cpp`
- Modify: `CMakeLists.txt` (add new source + test file)

### Why

Per spec §4: pessimistic per-`(SensorKind, MeasurementModel)` defaults + `applyDefaultsIfEmpty` helper. The diagnostic flag lets downstream sinks mark low-confidence tracks. Backward compat preserved: existing constructors that explicitly fill covariance are unaffected; the flag defaults `false` so existing test expectations don't change.

### Steps

- [ ] **Step 1: Add the diagnostic flag**

In `core/types/Measurement.hpp`, append to the `Measurement` struct:

```cpp
// True when the covariance was populated from SensorDefaults rather
// than from a real sensor uncertainty. Diagnostic only — the tracker
// behaves identically regardless of this flag.
bool covariance_is_default{false};
```

Run the full suite to confirm no test breaks. Expect: 286/286 still green.

- [ ] **Step 2: Write the SensorDefaults header**

Create `core/types/SensorDefaults.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

struct PerSensorCov {
  double sigma_pos_m{0.0};
  double sigma_range_m{0.0};
  double sigma_bearing_rad{0.0};
};

struct SensorDefaults {
  PerSensorCov ais_position;
  PerSensorCov arpa_tll_position;
  PerSensorCov arpa_ttm_range_bearing;
  PerSensorCov eoir_range_bearing;
  PerSensorCov eoir_bearing_only;

  Eigen::MatrixXd covarianceFor(SensorKind sensor,
                                MeasurementModel model) const;
};

// Pessimistic, literature-based defaults. Operators with real specs
// override the relevant fields after constructing this.
SensorDefaults pessimisticSensorDefaults();

// If m.covariance is empty (size==0), fill it from defaults and set
// m.covariance_is_default = true. No-op when covariance already set
// or when defaults don't have a value for the (sensor, model) pair.
void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d);

}  // namespace navtracker
```

- [ ] **Step 3: Write the implementation**

Create `core/types/SensorDefaults.cpp`:

```cpp
#include "core/types/SensorDefaults.hpp"

#include <cmath>

namespace navtracker {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

Eigen::MatrixXd SensorDefaults::covarianceFor(SensorKind sensor,
                                              MeasurementModel model) const {
  auto pos2D = [](double sigma) {
    Eigen::Matrix2d c = Eigen::Matrix2d::Zero();
    c(0, 0) = sigma * sigma;
    c(1, 1) = sigma * sigma;
    return c;
  };
  auto rb2D = [](double sigma_r, double sigma_b) {
    Eigen::Matrix2d c = Eigen::Matrix2d::Zero();
    c(0, 0) = sigma_r * sigma_r;
    c(1, 1) = sigma_b * sigma_b;
    return c;
  };
  auto b1D = [](double sigma_b) {
    Eigen::Matrix<double, 1, 1> c;
    c(0, 0) = sigma_b * sigma_b;
    return c;
  };

  if (sensor == SensorKind::Ais && model == MeasurementModel::Position2D)
    return pos2D(ais_position.sigma_pos_m);
  if (sensor == SensorKind::ArpaTll && model == MeasurementModel::Position2D)
    return pos2D(arpa_tll_position.sigma_pos_m);
  if (sensor == SensorKind::ArpaTtm && model == MeasurementModel::RangeBearing2D)
    return rb2D(arpa_ttm_range_bearing.sigma_range_m,
                arpa_ttm_range_bearing.sigma_bearing_rad);
  if (sensor == SensorKind::EoIr && model == MeasurementModel::RangeBearing2D)
    return rb2D(eoir_range_bearing.sigma_range_m,
                eoir_range_bearing.sigma_bearing_rad);
  if (sensor == SensorKind::EoIr && model == MeasurementModel::BearingOnly2D)
    return b1D(eoir_bearing_only.sigma_bearing_rad);

  return Eigen::MatrixXd{};  // empty: unknown combination
}

SensorDefaults pessimisticSensorDefaults() {
  SensorDefaults d;
  d.ais_position.sigma_pos_m              = 30.0;
  d.arpa_tll_position.sigma_pos_m         = 50.0;
  d.arpa_ttm_range_bearing.sigma_range_m   = 75.0;
  d.arpa_ttm_range_bearing.sigma_bearing_rad = 1.5 * kDeg2Rad;
  d.eoir_range_bearing.sigma_range_m       = 50.0;
  d.eoir_range_bearing.sigma_bearing_rad   = 1.0 * kDeg2Rad;
  d.eoir_bearing_only.sigma_bearing_rad    = 1.5 * kDeg2Rad;
  return d;
}

void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d) {
  if (m.covariance.size() != 0) return;
  Eigen::MatrixXd cov = d.covarianceFor(m.sensor, m.model);
  if (cov.size() == 0) return;  // unknown combination, leave empty
  m.covariance = std::move(cov);
  m.covariance_is_default = true;
}

}  // namespace navtracker
```

- [ ] **Step 4: Wire into CMake**

Add `core/types/SensorDefaults.cpp` to the `navtracker_core` source list in `CMakeLists.txt`.

- [ ] **Step 5: Write unit tests**

Create `tests/types/test_sensor_defaults.cpp` with five tests per spec §11.1:

```cpp
#include "core/types/SensorDefaults.hpp"

#include <Eigen/Core>
#include <cmath>

#include <gtest/gtest.h>

namespace navtracker {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

TEST(SensorDefaultsTest, PessimisticFactoryMatchesSpecValues) {
  const SensorDefaults d = pessimisticSensorDefaults();
  EXPECT_DOUBLE_EQ(d.ais_position.sigma_pos_m, 30.0);
  EXPECT_DOUBLE_EQ(d.arpa_tll_position.sigma_pos_m, 50.0);
  EXPECT_DOUBLE_EQ(d.arpa_ttm_range_bearing.sigma_range_m, 75.0);
  EXPECT_DOUBLE_EQ(d.arpa_ttm_range_bearing.sigma_bearing_rad,
                   1.5 * kDeg2Rad);
  EXPECT_DOUBLE_EQ(d.eoir_range_bearing.sigma_range_m, 50.0);
  EXPECT_DOUBLE_EQ(d.eoir_range_bearing.sigma_bearing_rad,
                   1.0 * kDeg2Rad);
  EXPECT_DOUBLE_EQ(d.eoir_bearing_only.sigma_bearing_rad,
                   1.5 * kDeg2Rad);
}

TEST(SensorDefaultsTest, CovarianceForReturnsCorrectShape) {
  const SensorDefaults d = pessimisticSensorDefaults();
  const auto ais_cov = d.covarianceFor(SensorKind::Ais,
                                       MeasurementModel::Position2D);
  ASSERT_EQ(ais_cov.rows(), 2);
  ASSERT_EQ(ais_cov.cols(), 2);
  EXPECT_DOUBLE_EQ(ais_cov(0, 0), 900.0);  // 30^2
  EXPECT_DOUBLE_EQ(ais_cov(1, 1), 900.0);

  const auto ttm_cov = d.covarianceFor(SensorKind::ArpaTtm,
                                       MeasurementModel::RangeBearing2D);
  ASSERT_EQ(ttm_cov.rows(), 2);
  EXPECT_DOUBLE_EQ(ttm_cov(0, 0), 75.0 * 75.0);
  EXPECT_NEAR(ttm_cov(1, 1),
              (1.5 * kDeg2Rad) * (1.5 * kDeg2Rad), 1e-12);
}

TEST(SensorDefaultsTest, ApplyDefaultsFillsEmptyAndFlags) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::Position2D;
  // covariance left empty.
  applyDefaultsIfEmpty(m, d);
  EXPECT_TRUE(m.covariance_is_default);
  EXPECT_EQ(m.covariance.rows(), 2);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 900.0);
}

TEST(SensorDefaultsTest, ApplyDefaultsNoOpWhenCovSet) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::Position2D;
  m.covariance = Eigen::Matrix2d::Identity() * 4.0;
  applyDefaultsIfEmpty(m, d);
  EXPECT_FALSE(m.covariance_is_default);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 4.0);
}

TEST(SensorDefaultsTest, ApplyDefaultsUnknownComboLeavesEmpty) {
  const SensorDefaults d = pessimisticSensorDefaults();
  Measurement m;
  m.sensor = SensorKind::Ais;
  m.model = MeasurementModel::RangeBearing2D;  // unknown combo
  applyDefaultsIfEmpty(m, d);
  EXPECT_FALSE(m.covariance_is_default);
  EXPECT_EQ(m.covariance.size(), 0);
}

}  // namespace navtracker
```

- [ ] **Step 6: Wire test into CMake**

Add `tests/types/test_sensor_defaults.cpp` to the `navtracker_tests` source list.

- [ ] **Step 7: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R SensorDefaultsTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 5 new PASS; full suite 291/291 green (was 286).

- [ ] **Step 8: Commit**

```
git add core/types/SensorDefaults.hpp core/types/SensorDefaults.cpp \
        core/types/Measurement.hpp \
        tests/types/test_sensor_defaults.cpp CMakeLists.txt
git commit -m "types: SensorDefaults + applyDefaultsIfEmpty + covariance_is_default flag"
```

---

## Task 2: `OwnShipProvider` ring history + adapter migration

**Files:**
- Modify: `adapters/own_ship/OwnShipProvider.hpp`
- Modify: `adapters/own_ship/OwnShipProvider.cpp`
- Modify: `adapters/arpa/ArpaAdapter.cpp` (migrate to `poseAtOrBefore`)
- Modify: `adapters/eoir/EoIrAdapter.cpp` (migrate to `poseAtOrBefore`)
- Modify: `tests/adapters/own_ship/test_own_ship_provider.cpp` (extend)

### Why

Per spec §5.3 and §5.4: keep a small ring buffer (default 16), expose `poseAtOrBefore(Timestamp)`, migrate existing NMEA adapters to use it for timestamp correctness. Closes the "use of `latest()` during a turn" footgun. `latest()` semantics preserved for tests that don't care.

### Steps

- [ ] **Step 1: Extend the header**

In `adapters/own_ship/OwnShipProvider.hpp`:

```cpp
#pragma once

#include <cstddef>
#include <deque>
#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
};

class OwnShipProvider {
 public:
  explicit OwnShipProvider(std::size_t history_size = 16);

  void update(const OwnShipPose& pose);

  // Most recently pushed pose.
  std::optional<OwnShipPose> latest() const;

  // Most recent pose with pose.time <= t. Returns nullopt when the
  // history is empty or every stored pose is strictly newer than t.
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;

  // Diagnostic: how many poses are currently stored.
  std::size_t historySize() const { return history_.size(); }

 private:
  std::deque<OwnShipPose> history_;
  std::size_t history_size_limit_;
};

}  // namespace navtracker
```

- [ ] **Step 2: Update the implementation**

Modify `adapters/own_ship/OwnShipProvider.cpp` to:
- Construct with a configurable `history_size_limit_`.
- `update(pose)` pushes to back; if size exceeds limit, pop front.
- `latest()` returns `history_.back()` (preserving existing behavior — verify tests still pass).
- `poseAtOrBefore(t)` walks the ring from newest to oldest, returns the first with `pose.time <= t`.

Reference implementation:

```cpp
#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

OwnShipProvider::OwnShipProvider(std::size_t history_size)
    : history_size_limit_(history_size > 0 ? history_size : 1) {}

void OwnShipProvider::update(const OwnShipPose& pose) {
  history_.push_back(pose);
  while (history_.size() > history_size_limit_) history_.pop_front();
}

std::optional<OwnShipPose> OwnShipProvider::latest() const {
  if (history_.empty()) return std::nullopt;
  return history_.back();
}

std::optional<OwnShipPose> OwnShipProvider::poseAtOrBefore(Timestamp t) const {
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (!(t < it->time)) return *it;  // it->time <= t
  }
  return std::nullopt;
}

}  // namespace navtracker
```

- [ ] **Step 3: Migrate ARPA adapter**

In `adapters/arpa/ArpaAdapter.cpp` TTM branch, replace `own_ship_.latest()` with `own_ship_.poseAtOrBefore(t)`. The local variable name (`own_opt`) stays. This is a 1-line change.

- [ ] **Step 4: Migrate EOIR adapter**

Same change in `adapters/eoir/EoIrAdapter.cpp`. 1-line.

- [ ] **Step 5: Extend tests**

In `tests/adapters/own_ship/test_own_ship_provider.cpp`, add:

```cpp
TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsExactMatch) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(b); p.update(c);

  const auto pose_at_2 = p.poseAtOrBefore(Timestamp::fromSeconds(2.0));
  ASSERT_TRUE(pose_at_2.has_value());
  EXPECT_DOUBLE_EQ(pose_at_2->lat_deg, 2.0);
}

TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsMostRecentEarlier) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(c);

  // No pose at exactly t=2 -> falls back to the pose at t=1.
  const auto pose_at_2 = p.poseAtOrBefore(Timestamp::fromSeconds(2.0));
  ASSERT_TRUE(pose_at_2.has_value());
  EXPECT_DOUBLE_EQ(pose_at_2->lat_deg, 1.0);
}

TEST(OwnShipProviderTest, PoseAtOrBeforeReturnsNulloptWhenAllPosesNewer) {
  OwnShipProvider p(8);
  OwnShipPose a; a.time = Timestamp::fromSeconds(5.0); a.lat_deg = 5.0;
  p.update(a);

  const auto pose_at_3 = p.poseAtOrBefore(Timestamp::fromSeconds(3.0));
  EXPECT_FALSE(pose_at_3.has_value());
}

TEST(OwnShipProviderTest, HistoryDropsOldestWhenLimitReached) {
  OwnShipProvider p(2);  // tiny limit for the test
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  OwnShipPose c; c.time = Timestamp::fromSeconds(3.0); c.lat_deg = 3.0;
  p.update(a); p.update(b); p.update(c);

  // Pose at t=1 has been dropped.
  EXPECT_FALSE(p.poseAtOrBefore(Timestamp::fromSeconds(1.0)).has_value());
  EXPECT_TRUE(p.poseAtOrBefore(Timestamp::fromSeconds(2.0)).has_value());
}

TEST(OwnShipProviderTest, LatestSemanticsPreserved) {
  OwnShipProvider p;  // default size
  OwnShipPose a; a.time = Timestamp::fromSeconds(1.0); a.lat_deg = 1.0;
  OwnShipPose b; b.time = Timestamp::fromSeconds(2.0); b.lat_deg = 2.0;
  p.update(a); p.update(b);
  const auto latest = p.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_DOUBLE_EQ(latest->lat_deg, 2.0);
}
```

- [ ] **Step 6: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R OwnShipProviderTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```

Expected: 5 new PASS. Full suite: 296/296 green. The adapter migrations don't break anything because the sim feeds GGA at the same timestamps as the sensor messages — `poseAtOrBefore(t)` returns the right pose.

- [ ] **Step 7: Commit**

```
git add adapters/own_ship/OwnShipProvider.hpp adapters/own_ship/OwnShipProvider.cpp \
        adapters/arpa/ArpaAdapter.cpp adapters/eoir/EoIrAdapter.cpp \
        tests/adapters/own_ship/test_own_ship_provider.cpp
git commit -m "own-ship: ring-buffer history; ARPA/EOIR use poseAtOrBefore(t)"
```

---

## Task 3: Move `Projection.{hpp,cpp}` to `core/projection/`

**Files:**
- Move: `adapters/util/Projection.hpp` → `core/projection/Projection.hpp`
- Move: `adapters/util/Projection.cpp` → `core/projection/Projection.cpp`
- Modify: every file that includes the old path (search and replace).
- Modify: `CMakeLists.txt` (update source path).

### Why

Per spec §6.2: `Projection` is pure math and is about to be used by `MeasurementBuilders` (in core). Keeping it under `adapters/` would force core code to depend on adapters — the inverse of the architectural rule.

### Steps

- [ ] **Step 1: Identify all includes of the old path**

```
grep -rn '"adapters/util/Projection.hpp"' . --include='*.hpp' --include='*.cpp' | head -20
```

Expect a handful: the ARPA/EOIR adapter sources, possibly some tests, possibly the bias estimator if it touches the projection.

- [ ] **Step 2: Move the files**

```
mkdir -p core/projection
git mv adapters/util/Projection.hpp core/projection/Projection.hpp
git mv adapters/util/Projection.cpp core/projection/Projection.cpp
```

- [ ] **Step 3: Update all includes**

```
grep -rl '"adapters/util/Projection.hpp"' . --include='*.hpp' --include='*.cpp' | \
  xargs sed -i 's#"adapters/util/Projection\.hpp"#"core/projection/Projection.hpp"#g'
```

Verify with another grep that no stale references remain.

- [ ] **Step 4: Update CMake**

In `CMakeLists.txt`, find `adapters/util/Projection.cpp` in the `navtracker_core` sources and replace with `core/projection/Projection.cpp`.

- [ ] **Step 5: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```

Expected: 296/296 still green. Pure mechanical move.

- [ ] **Step 6: Commit**

```
git add -A
git commit -m "projection: move from adapters/util/ to core/projection/

Pure-math helper used by both NMEA adapters and forthcoming
MeasurementBuilders. Move to core so library users can construct
measurements without depending on the adapters target."
```

---

## Task 4: `MeasurementBuilders`

**Files:**
- Create: `core/types/MeasurementBuilders.hpp`, `core/types/MeasurementBuilders.cpp`
- Create: `tests/types/test_measurement_builders.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §5: three constructors covering the common patterns (relative-bearing, true-bearing, ENU position). All take the measurement timestamp and `OwnShipProvider&`; the bearing variants look up the right pose via Task 2's new API. Returns Position2D Measurements with full covariance composition through the projection helper from Task 3.

### Steps

- [ ] **Step 1: Write the header**

Create `core/types/MeasurementBuilders.hpp` per spec §5.1.

- [ ] **Step 2: Write the implementation**

Create `core/types/MeasurementBuilders.cpp`. The relative-bearing builder:
- Calls `provider.poseAtOrBefore(t)`. If `nullopt`, returns empty `Measurement`.
- Computes `bearing_true_rad = relative_bearing_rad + pose.heading_true_deg * kDeg2Rad`.
- Computes `own_xy` from `datum.toEnu({pose.lat_deg, pose.lon_deg, 0.0})`.
- Decides `sigma_heading_rad`: for v1, since there's no explicit heading uncertainty channel from `OwnShipPose` yet, use `0.0` and document. (Later work — when heading-bias estimator / RMC parsing exposes heading uncertainty — feeds into here.) The user can pass it through their `range_std_m`/`bearing_std_rad` arguments if they want; the projection already composes the rest.
- Calls `projectRangeBearingToEnu(range_m, bearing_true_rad, range_std_m, bearing_std_rad, 0.0, pose.position_std_m, own_xy)`.
- Builds the resulting Measurement.

The true-bearing builder is identical minus the heading addition.

The ENU-position builder is trivial:

```cpp
Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor, std::string source_id,
    Timestamp t, Eigen::Vector2d enu_xy, Eigen::Matrix2d covariance,
    AssociationHints hints) {
  Measurement m;
  m.time = t;
  m.sensor = sensor;
  m.source_id = std::move(source_id);
  m.model = MeasurementModel::Position2D;
  m.value = enu_xy;
  if (covariance.size() > 0 && covariance.rows() == 2 && covariance.cols() == 2)
    m.covariance = covariance;
  m.hints = std::move(hints);
  return m;
}
```

- [ ] **Step 3: Wire into CMake**

Add `core/types/MeasurementBuilders.cpp` to `navtracker_core`.

- [ ] **Step 4: Write unit tests**

Create `tests/types/test_measurement_builders.cpp` covering the five tests from spec §11.2. Key tests:

- `RelativeBearingProducesEnuConsistentWithDirectProjection`: build a Measurement via the helper; build the same answer by calling `projectRangeBearingToEnu` directly with the same heading combination. Compare value and covariance to 1e-9.
- `RelativeBearingUsesPoseFromHistoryAtTimestamp`: push two poses (different headings) at t=1 and t=3; call the builder with t=2; verify the resulting bearing uses the heading from the t=1 pose.
- `TrueBearingSkipsHeadingCombo`: same setup, true-bearing version, output bearing direction matches input.
- `EnuPositionPassesThroughCovariance`: explicit covariance arg matches output.
- `EnuPositionEmptyCovariancePlaysWithDefaults`: empty cov + `applyDefaultsIfEmpty` → populated and flag.

- [ ] **Step 5: Add test to CMake**

Append `tests/types/test_measurement_builders.cpp` to `navtracker_tests`.

- [ ] **Step 6: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R MeasurementBuildersTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 5 new PASS. Full suite 301/301 green.

- [ ] **Step 7: Commit**

```
git add core/types/MeasurementBuilders.hpp core/types/MeasurementBuilders.cpp \
        tests/types/test_measurement_builders.cpp CMakeLists.txt
git commit -m "types: MeasurementBuilders (relative-bearing, true-bearing, ENU)"
```

---

## Task 5: CMake three-way split (core / nmea / sim)

**Files:**
- Modify: `CMakeLists.txt`

### Why

Per spec §6: split `navtracker_core` into three targets so library consumers can link only what they need. Existing `navtracker_tests` links all three to keep behavior byte-identical.

### Steps

- [ ] **Step 1: Inventory current sources**

The current `navtracker_core` lists ~60 .cpp files. They group cleanly:
- `core/...`, `core/projection/Projection.cpp`, `adapters/own_ship/OwnShipProvider.cpp` → `navtracker_core` (post-split).
- `adapters/ais/AisAdapter.cpp`, `adapters/arpa/ArpaAdapter.cpp`, `adapters/eoir/EoIrAdapter.cpp`, `adapters/own_ship/OwnShipNmeaAdapter.cpp`, `adapters/util/Nmea.cpp` → `navtracker_nmea`.
- `sim/...` → `navtracker_sim`.

- [ ] **Step 2: Rewrite the CMakeLists structure**

Replace the single `add_library(navtracker_core ...)` with three `add_library`s per spec §6.1. Order matters because `navtracker_nmea` depends on `navtracker_core`, and `navtracker_sim` depends on `navtracker_nmea` (because emitters call into NMEA adapter sinks).

Important detail: `navtracker_sim` likely doesn't strictly need `navtracker_nmea` if the emitters only call adapter sinks via reference. Verify by trying to link `navtracker_sim` against just `navtracker_core` first — if it links, drop the nmea dependency from sim.

Update `navtracker_tests` to:

```cmake
target_link_libraries(navtracker_tests PRIVATE
    navtracker_core navtracker_nmea navtracker_sim
    GTest::gtest_main Eigen3::Eigen)
```

- [ ] **Step 3: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```

Expected: 301/301 still green. If a link error surfaces because some core file actually pulls a symbol from an adapter, identify it — that's a hidden architecture violation worth recording and fixing.

- [ ] **Step 4: Add a tiny core-only build check**

To verify the contract, add a small `tests/types/test_link_core_only.cpp` that includes `core/types/SensorDefaults.hpp` and `core/types/MeasurementBuilders.hpp` and calls one function from each. This file is built into a separate small executable that links *only* `navtracker_core`:

```cmake
add_executable(navtracker_core_only_smoke
    tests/types/test_link_core_only.cpp)
target_link_libraries(navtracker_core_only_smoke PRIVATE
    navtracker_core Eigen3::Eigen)
```

The build of this target is the test — if `navtracker_core` accidentally references a symbol from NMEA or sim, this link fails. Add a CTest entry:

```cmake
add_test(NAME CoreOnlyLinks COMMAND $<TARGET_FILE:navtracker_core_only_smoke>)
```

The executable's `main()` just calls the SensorDefaults factory and exits.

- [ ] **Step 5: Build the smoke target + full suite**

```
cmake --build build --target navtracker_core_only_smoke && \
  cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```

Expected: smoke builds and runs; full suite stays green.

- [ ] **Step 6: Commit**

```
git add CMakeLists.txt tests/types/test_link_core_only.cpp
git commit -m "cmake: split into core / nmea / sim; add core-only link smoke"
```

---

## Task 6: `app/example.cpp`

**Files:**
- Create: `app/example.cpp`
- Modify: `CMakeLists.txt` (add stand-alone target)

### Why

Per spec §7: a canonical end-to-end library use example. Builds against `navtracker_core` only (or `+ navtracker_nmea` if it parses AIS bytes — pick whichever is cleaner for the example to read). Compiles as part of the suite to stay maintainable.

### Steps

- [ ] **Step 1: Create the example**

Create `app/example.cpp` with the content from spec §7. The example uses `makeMeasurementFromEnuPosition` (no NMEA dependency) and `makeMeasurementFromRelativeBearing`, plus the SensorDefaults helpers. It depends only on `navtracker_core`.

- [ ] **Step 2: Add CMake target**

In `CMakeLists.txt`:

```cmake
add_executable(navtracker_example app/example.cpp)
target_link_libraries(navtracker_example PRIVATE
    navtracker_core Eigen3::Eigen)
```

- [ ] **Step 3: Build**

```
cmake --build build --target navtracker_example
```

Expected: success. Optionally run it:

```
./build/navtracker_example
```

Should print a couple of track snapshots (one each for the AIS and radar measurements).

- [ ] **Step 4: Full suite**

```
ctest --test-dir build --output-on-failure
```
Expected: 301+ green (example target doesn't add to the test count unless wired into CTest).

- [ ] **Step 5: Commit**

```
git add app/example.cpp CMakeLists.txt
git commit -m "app: canonical library-use example"
```

---

## Task 7: Documentation pass

**Files:**
- Modify: `CLAUDE.md` (add "Library use" section)
- Create: `README.md` (~20 lines)

### Why

Per spec §8: brief documentation pass — sets the contract, points to the example, lists CMake targets.

### Steps

- [ ] **Step 1: Update `CLAUDE.md`**

Append the "Library use" section per spec §8.1 after the existing "Module layout" section.

- [ ] **Step 2: Create `README.md`**

```markdown
# navtracker

Maritime sensor-fusion tracker. Fuses AIS, navigation radar (ARPA),
EO/IR camera, and own-ship navigation into a single authoritative set
of vessel tracks.

See `CLAUDE.md` for architecture overview, `docs/superpowers/specs/`
for design decisions, and `docs/algorithms/evaluation-log.md` for
measured behavior.

## Library use

Pre-parsed `Measurement` and `OwnShipPose` are the canonical contract.
The NMEA adapters in `adapters/` are one optional implementation — if
your pipeline produces parsed sensor data, skip them.

See `app/example.cpp` for a complete end-to-end example. CMake targets:

- `navtracker_core` — domain + ports + helpers. No I/O. Link this alone.
- `navtracker_nmea` — NMEA-format adapters. Link when you consume NMEA.
- `navtracker_sim` — synthetic generators. Tests only.

## Build

```bash
conan install . --build=missing --output-folder=build/ -s build_type=Release
cmake --preset conan-release
cmake --build build/Release --target navtracker_tests
ctest --test-dir build/Release --output-on-failure
```

## License

(Set per project policy.)
```

- [ ] **Step 3: Verify**

`README.md` renders correctly in plain text; no broken markdown. CLAUDE.md still consistent.

- [ ] **Step 4: Commit**

```
git add CLAUDE.md README.md
git commit -m "docs: CLAUDE.md library-use section; add README"
```

---

## Done criteria

- All 7 tasks committed.
- Full suite green (≥ 301 tests after T4, including the new `CoreOnlyLinks` ctest entry).
- `navtracker_core_only_smoke` executable builds without linking the NMEA or sim targets.
- `navtracker_example` builds and runs.
- `CLAUDE.md` has the "Library use" section; `README.md` exists.
- Spec §13 (ways to improve) carries the Layer-3 interpolation entry as the explicit next step.
