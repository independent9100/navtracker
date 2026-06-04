# Auto-Datum + Re-Center Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the user-facing `geo::Datum` concept. `OwnShipProvider` owns it, lazy-initialises from the first pose, auto-re-centers when own-ship moves > 30 km, and fires `IDatumChangeSink` so consumers reading ENU can react. Pattern A: composition-root adapter struct bridges the event to a free utility that shifts every track's state and covariance.

**Architecture:** `OwnShipProvider` gains an `std::optional<geo::Datum>`, a `DatumRecenterPolicy`, and a vector of `IDatumChangeSink*`. On `update(pose)`: lazy-init datum if absent; if present and threshold exceeded, replace datum and fire sinks. New port `IDatumChangeSink`. New free utility `shiftTracksOnDatumChange(TrackManager&, old, new)` in `core/tracking/`. `MeasurementBuilders` and `synthesizeOwnShipTrack` drop their `Datum` args; pull from `provider.datum()`. Existing tests use either the new library-friendly ctor (auto-datum path) or the backward-compat explicit-datum ctor.

**Tech Stack:** C++17, Eigen 3.4, CMake/Conan, GoogleTest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-04-auto-datum-design.md`. Section references refer to that spec.

---

## Task 1: `OwnShipProvider` lazy datum + recenter logic + sink port

**Files:**
- Modify: `adapters/own_ship/OwnShipProvider.hpp`
- Modify: `adapters/own_ship/OwnShipProvider.cpp`
- Modify: `tests/adapters/own_ship/test_own_ship_provider.cpp` (extend)

### Why

Per spec §3, §5, §6: the provider owns the datum, lazy-initialises it, auto-re-centers, exposes sink registration. Both constructors (library-friendly + backward-compat) ship in this task.

### Steps

- [ ] **Step 1: Add `IDatumChangeSink` port + `DatumRecenterPolicy`**

In `adapters/own_ship/OwnShipProvider.hpp`, add (before the `OwnShipProvider` class):

```cpp
#include <optional>
#include <vector>
#include <stdexcept>

#include "core/geo/Datum.hpp"

class IDatumChangeSink {
 public:
  virtual ~IDatumChangeSink() = default;
  virtual void onDatumRecentered(const geo::Datum& old_datum,
                                 const geo::Datum& new_datum) = 0;
};

struct DatumRecenterPolicy {
  bool enable_auto_recenter{true};
  double recenter_threshold_km{30.0};
};
```

- [ ] **Step 2: Extend `OwnShipProvider` class**

Add two new constructors, the datum-related members and methods:

```cpp
class OwnShipProvider {
 public:
  // Library-friendly: no datum. Lazy-init from first update().
  explicit OwnShipProvider(std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  // Backward-compat: pin the datum.
  OwnShipProvider(geo::Datum initial_datum,
                  std::size_t history_size = 16,
                  DatumRecenterPolicy policy = {});

  // Existing.
  void update(const OwnShipPose& pose);
  std::optional<OwnShipPose> latest() const;
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;
  std::size_t historySize() const;

  // New.
  const geo::Datum& datum() const;
  bool hasDatum() const noexcept;
  void registerDatumSink(IDatumChangeSink* sink);
  void unregisterDatumSink(IDatumChangeSink* sink);

 private:
  std::deque<OwnShipPose> history_;
  std::size_t history_size_limit_;
  std::optional<geo::Datum> current_datum_;
  DatumRecenterPolicy policy_;
  std::vector<IDatumChangeSink*> sinks_;
};
```

- [ ] **Step 3: Implement constructors**

```cpp
OwnShipProvider::OwnShipProvider(std::size_t history_size,
                                 DatumRecenterPolicy policy)
    : history_size_limit_(history_size > 0 ? history_size : 1),
      policy_(policy) {}

OwnShipProvider::OwnShipProvider(geo::Datum initial_datum,
                                 std::size_t history_size,
                                 DatumRecenterPolicy policy)
    : history_size_limit_(history_size > 0 ? history_size : 1),
      current_datum_(std::move(initial_datum)),
      policy_(policy) {}
```

- [ ] **Step 4: Implement `update(pose)` with lazy-init + recenter**

In `OwnShipProvider.cpp`:

```cpp
void OwnShipProvider::update(const OwnShipPose& pose) {
  if (!current_datum_) {
    current_datum_ = geo::Datum({pose.lat_deg, pose.lon_deg, pose.alt_m});
  } else if (policy_.enable_auto_recenter) {
    const Eigen::Vector3d enu = current_datum_->toEnu(
        {pose.lat_deg, pose.lon_deg, pose.alt_m});
    const double d_m = std::sqrt(enu.x() * enu.x() + enu.y() * enu.y());
    if (d_m > policy_.recenter_threshold_km * 1000.0) {
      const geo::Datum old_datum = *current_datum_;
      current_datum_ = geo::Datum({pose.lat_deg, pose.lon_deg, pose.alt_m});
      for (IDatumChangeSink* sink : sinks_) {
        if (sink) sink->onDatumRecentered(old_datum, *current_datum_);
      }
    }
  }
  history_.push_back(pose);
  while (history_.size() > history_size_limit_) history_.pop_front();
}
```

- [ ] **Step 5: Implement accessors + sink registration**

```cpp
const geo::Datum& OwnShipProvider::datum() const {
  if (!current_datum_) {
    throw std::runtime_error(
        "OwnShipProvider::datum(): no datum yet; call update(pose) first "
        "or construct with an explicit datum.");
  }
  return *current_datum_;
}

bool OwnShipProvider::hasDatum() const noexcept {
  return current_datum_.has_value();
}

void OwnShipProvider::registerDatumSink(IDatumChangeSink* sink) {
  if (sink) sinks_.push_back(sink);
}

void OwnShipProvider::unregisterDatumSink(IDatumChangeSink* sink) {
  sinks_.erase(std::remove(sinks_.begin(), sinks_.end(), sink), sinks_.end());
}
```

- [ ] **Step 6: Write the unit tests**

Append to `tests/adapters/own_ship/test_own_ship_provider.cpp` the 8 tests from spec §10 (#1). Sketch:

```cpp
namespace {
class CountingSink : public IDatumChangeSink {
 public:
  int call_count{0};
  geo::Datum last_old{{0,0,0}};
  geo::Datum last_new{{0,0,0}};
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    ++call_count; last_old = o; last_new = n;
  }
};
}  // namespace

TEST(OwnShipProviderTest, NoDatumBeforeFirstUpdate) {
  OwnShipProvider p;
  EXPECT_FALSE(p.hasDatum());
  EXPECT_THROW((void)p.datum(), std::runtime_error);
}

TEST(OwnShipProviderTest, DatumInitialisesFromFirstUpdate) {
  OwnShipProvider p;
  OwnShipPose pose; pose.lat_deg = 53.5; pose.lon_deg = 8.0;
  p.update(pose);
  ASSERT_TRUE(p.hasDatum());
  // Datum at the pose -> ENU origin is (0,0).
  const auto enu = p.datum().toEnu({53.5, 8.0, 0.0});
  EXPECT_NEAR(enu.x(), 0.0, 1e-3);
  EXPECT_NEAR(enu.y(), 0.0, 1e-3);
}

TEST(OwnShipProviderTest, DatumStaysFixedBelowThreshold) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a; a.lat_deg = 53.5; a.lon_deg = 8.0;
  p.update(a);
  // Move ~10 km east at 53.5°N: 1° lon ~= 66 km, so +0.15° ~= 10 km.
  OwnShipPose b = a; b.lon_deg += 0.15;
  p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, RecenterFiresAtThreshold) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a; a.lat_deg = 53.5; a.lon_deg = 8.0;
  p.update(a);
  // Move ~50 km east: well past 30 km threshold.
  OwnShipPose b = a; b.lon_deg += 0.75;
  p.update(b);
  EXPECT_EQ(sink.call_count, 1);
  const auto enu_b_in_new = p.datum().toEnu({b.lat_deg, b.lon_deg, 0.0});
  // After recenter, b is at the new origin.
  EXPECT_NEAR(enu_b_in_new.x(), 0.0, 1e-3);
}

TEST(OwnShipProviderTest, RecenterDisabledByPolicy) {
  DatumRecenterPolicy policy;
  policy.enable_auto_recenter = false;
  OwnShipProvider p(16, policy);
  CountingSink sink;
  p.registerDatumSink(&sink);
  OwnShipPose a; a.lat_deg = 53.5; a.lon_deg = 8.0;
  OwnShipPose b = a; b.lon_deg += 1.5;  // ~100 km
  p.update(a); p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, MultipleSinksAllFire) {
  OwnShipProvider p;
  CountingSink s1, s2;
  p.registerDatumSink(&s1);
  p.registerDatumSink(&s2);
  OwnShipPose a; a.lat_deg = 53.5; a.lon_deg = 8.0;
  OwnShipPose b = a; b.lon_deg += 0.75;
  p.update(a); p.update(b);
  EXPECT_EQ(s1.call_count, 1);
  EXPECT_EQ(s2.call_count, 1);
}

TEST(OwnShipProviderTest, UnregisteredSinkDoesNotFire) {
  OwnShipProvider p;
  CountingSink sink;
  p.registerDatumSink(&sink);
  p.unregisterDatumSink(&sink);
  OwnShipPose a; a.lat_deg = 53.5; a.lon_deg = 8.0;
  OwnShipPose b = a; b.lon_deg += 0.75;
  p.update(a); p.update(b);
  EXPECT_EQ(sink.call_count, 0);
}

TEST(OwnShipProviderTest, ExplicitDatumConstructorHasDatumImmediately) {
  geo::Datum d({53.5, 8.0, 0.0});
  OwnShipProvider p(d);
  EXPECT_TRUE(p.hasDatum());
}
```

- [ ] **Step 7: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R OwnShipProviderTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 8 new PASS. Existing tests stay green if they use `OwnShipProvider provider;` + update (auto-init now triggers) or `OwnShipProvider(datum)` (backward compat). Full suite ~326/326.

If any existing test breaks because it uses a no-arg `OwnShipProvider` and then constructs measurements without pushing a pose first, that's an existing latent bug exposed by `datum()` throwing — investigate.

- [ ] **Step 8: Commit**

```
git add adapters/own_ship/OwnShipProvider.hpp adapters/own_ship/OwnShipProvider.cpp \
        tests/adapters/own_ship/test_own_ship_provider.cpp
git commit -m "own-ship: lazy datum + recenter policy + IDatumChangeSink port"
```

---

## Task 2: `shiftTracksOnDatumChange` utility + tests

**Files:**
- Create: `core/tracking/DatumShift.hpp`, `core/tracking/DatumShift.cpp`
- Create: `tests/tracking/test_datum_shift.cpp`
- Modify: `CMakeLists.txt` (add to `navtracker_core` + `navtracker_tests`)

### Why

Per spec §4.2–§4.4 and §10 (#2): the free utility that shifts every track's state and covariance from old to new ENU frame. Pattern A — TrackManager doesn't need to know; the composition root wires this through `TrackShifterSink`.

### Steps

- [ ] **Step 1: Write the header**

Create `core/tracking/DatumShift.hpp`:

```cpp
#pragma once

#include "core/geo/Datum.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

// Mutate every track in `mgr` so that its state and covariance are
// expressed in `new_datum`'s ENU frame instead of `old_datum`'s.
// Position is re-projected via geodetic; velocity is rotated by the
// ENU axis-convergence angle; covariance is block-rotated.
// Multi-mode carriers (IMM means/covariances, particles) are also
// shifted.
void shiftTracksOnDatumChange(TrackManager& mgr,
                              const geo::Datum& old_datum,
                              const geo::Datum& new_datum);

// Internal helper, exposed for testing. Computes the 2x2 ENU-axis
// rotation matrix between two datums per spec §4.3.
Eigen::Matrix2d datumAxisRotation(const geo::Datum& old_datum,
                                  const geo::Datum& new_datum);

}  // namespace navtracker
```

- [ ] **Step 2: Write the implementation**

Create `core/tracking/DatumShift.cpp`:

```cpp
#include "core/tracking/DatumShift.hpp"

#include <cmath>

namespace navtracker {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
}

Eigen::Matrix2d datumAxisRotation(const geo::Datum& old_datum,
                                  const geo::Datum& new_datum) {
  const auto& o = old_datum.origin();
  const auto& n = new_datum.origin();
  const double delta_lon_rad = (n.lon_deg - o.lon_deg) * kDeg2Rad;
  const double mean_lat_rad = 0.5 * (o.lat_deg + n.lat_deg) * kDeg2Rad;
  const double gamma = delta_lon_rad * std::sin(mean_lat_rad);
  const double c = std::cos(gamma), s = std::sin(gamma);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

namespace {
void shiftPosition(double& px, double& py,
                   const geo::Datum& old_d, const geo::Datum& new_d) {
  const auto geo = old_d.toGeodetic(Eigen::Vector3d(px, py, 0.0));
  const auto enu_new = new_d.toEnu(geo);
  px = enu_new.x();
  py = enu_new.y();
}

void rotateVelocityInPlace(double& vx, double& vy,
                           const Eigen::Matrix2d& R) {
  const Eigen::Vector2d v(vx, vy);
  const Eigen::Vector2d v_rot = R * v;
  vx = v_rot.x();
  vy = v_rot.y();
}

void rotateCovarianceInPlace(Eigen::MatrixXd& cov,
                             const Eigen::Matrix2d& R) {
  if (cov.rows() < 4 || cov.cols() < 4) return;  // smaller states skip
  Eigen::Matrix4d Rbar = Eigen::Matrix4d::Zero();
  Rbar.topLeftCorner<2, 2>()     = R;
  Rbar.bottomRightCorner<2, 2>() = R;
  cov.topLeftCorner<4, 4>() =
      Rbar * cov.topLeftCorner<4, 4>().eval() * Rbar.transpose();
}
}  // namespace

void shiftTracksOnDatumChange(TrackManager& mgr,
                              const geo::Datum& old_datum,
                              const geo::Datum& new_datum) {
  const Eigen::Matrix2d R = datumAxisRotation(old_datum, new_datum);
  for (auto& t : mgr.mutableTracks()) {
    if (t.state.size() >= 2) {
      shiftPosition(t.state(0), t.state(1), old_datum, new_datum);
    }
    if (t.state.size() >= 4) {
      rotateVelocityInPlace(t.state(2), t.state(3), R);
    }
    rotateCovarianceInPlace(t.covariance, R);

    // IMM modes: shift each column.
    if (t.imm_means.size() > 0) {
      for (int k = 0; k < t.imm_means.cols(); ++k) {
        if (t.imm_means.rows() >= 2) {
          double px = t.imm_means(0, k), py = t.imm_means(1, k);
          shiftPosition(px, py, old_datum, new_datum);
          t.imm_means(0, k) = px;
          t.imm_means(1, k) = py;
        }
        if (t.imm_means.rows() >= 4) {
          double vx = t.imm_means(2, k), vy = t.imm_means(3, k);
          rotateVelocityInPlace(vx, vy, R);
          t.imm_means(2, k) = vx;
          t.imm_means(3, k) = vy;
        }
      }
    }
    for (auto& cov_k : t.imm_covariances) {
      Eigen::MatrixXd cov_dyn = cov_k;
      rotateCovarianceInPlace(cov_dyn, R);
      cov_k = cov_dyn;
    }

    // Particles: shift each column's position.
    if (t.particles.size() > 0 && t.particles.rows() >= 2) {
      for (int k = 0; k < t.particles.cols(); ++k) {
        double px = t.particles(0, k), py = t.particles(1, k);
        shiftPosition(px, py, old_datum, new_datum);
        t.particles(0, k) = px;
        t.particles(1, k) = py;
        if (t.particles.rows() >= 4) {
          double vx = t.particles(2, k), vy = t.particles(3, k);
          rotateVelocityInPlace(vx, vy, R);
          t.particles(2, k) = vx;
          t.particles(3, k) = vy;
        }
      }
    }
  }
}

}  // namespace navtracker
```

Note: the implementation walks every carrier (state, imm_means, imm_covariances, particles). For tracks that don't use ensemble carriers (the common case), the loops are zero-cost.

If `geo::Datum::origin()` doesn't exist as a public method, expose the lat/lon in some way — check `core/geo/Datum.hpp` first and adjust accordingly.

- [ ] **Step 3: Wire into CMake**

Add `core/tracking/DatumShift.cpp` to `navtracker_core` source list in `CMakeLists.txt`.

- [ ] **Step 4: Write unit tests**

Create `tests/tracking/test_datum_shift.cpp`:

```cpp
#include "core/tracking/DatumShift.hpp"

#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {

namespace {
Track makeTrack(double px, double py, double vx, double vy) {
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << px, py, vx, vy;
  t.covariance = Eigen::Matrix4d::Zero();
  t.covariance.diagonal() << 25.0, 25.0, 1.0, 1.0;
  return t;
}
}  // namespace

TEST(DatumShiftTest, PreservesGeodeticPositionUnderShift) {
  geo::Datum old_d({53.5, 8.0, 0.0});
  geo::Datum new_d({53.5, 8.5, 0.0});  // ~33 km east

  // Place a track at a known lat/lon by converting it through old_d.
  const auto enu = old_d.toEnu({53.6, 8.2, 0.0});

  TrackManager mgr(2, 3);
  Track t = makeTrack(enu.x(), enu.y(), 1.0, 0.0);
  mgr.add(t);

  shiftTracksOnDatumChange(mgr, old_d, new_d);

  // After the shift, converting back through new_d should yield the
  // original lat/lon (within LTP tolerance).
  const auto& shifted = mgr.tracks()[0];
  const auto geo_after = new_d.toGeodetic(
      Eigen::Vector3d(shifted.state(0), shifted.state(1), 0.0));
  EXPECT_NEAR(geo_after.lat_deg, 53.6, 1e-4);
  EXPECT_NEAR(geo_after.lon_deg, 8.2, 1e-4);
}

TEST(DatumShiftTest, RotatesVelocityByConvergenceAngle) {
  geo::Datum old_d({60.0, 0.0, 0.0});
  geo::Datum new_d({60.0, 1.0, 0.0});   // ~55 km east at 60N
  TrackManager mgr(2, 3);
  Track t = makeTrack(0.0, 0.0, 1.0, 0.0);  // 1 m/s east
  mgr.add(t);

  const Eigen::Matrix2d R = datumAxisRotation(old_d, new_d);
  const double gamma_expected = 1.0 * 3.14159265358979 / 180.0
                                * std::sin(60.0 * 3.14159265358979 / 180.0);
  EXPECT_NEAR(std::atan2(R(1,0), R(0,0)), gamma_expected, 1e-9);

  shiftTracksOnDatumChange(mgr, old_d, new_d);

  // Velocity rotated by gamma: (1, 0) becomes (cos(gamma), sin(gamma)).
  const auto& shifted = mgr.tracks()[0];
  EXPECT_NEAR(shifted.state(2), std::cos(gamma_expected), 1e-6);
  EXPECT_NEAR(shifted.state(3), std::sin(gamma_expected), 1e-6);
}

TEST(DatumShiftTest, RotatesCovarianceBlocks) {
  geo::Datum old_d({60.0, 0.0, 0.0});
  geo::Datum new_d({60.0, 1.0, 0.0});

  TrackManager mgr(2, 3);
  Track t = makeTrack(0.0, 0.0, 0.0, 0.0);
  // Anisotropic covariance to confirm rotation does something.
  t.covariance = Eigen::Matrix4d::Zero();
  t.covariance(0, 0) = 100.0;
  t.covariance(1, 1) = 1.0;
  t.covariance(2, 2) = 4.0;
  t.covariance(3, 3) = 0.25;
  mgr.add(t);

  const Eigen::Matrix4d cov_before = mgr.tracks()[0].covariance;
  shiftTracksOnDatumChange(mgr, old_d, new_d);
  const Eigen::Matrix4d cov_after = mgr.tracks()[0].covariance;
  // Rotation by ~0.96 deg in the position block.
  EXPECT_NE(cov_after(0, 1), cov_before(0, 1));
}

TEST(DatumShiftTest, EmptyTrackListNoOp) {
  geo::Datum old_d({53.5, 8.0, 0.0});
  geo::Datum new_d({53.6, 8.0, 0.0});
  TrackManager mgr(2, 3);
  EXPECT_NO_FATAL_FAILURE(shiftTracksOnDatumChange(mgr, old_d, new_d));
}

}  // namespace navtracker
```

- [ ] **Step 5: Wire test into CMake**

Add `tests/tracking/test_datum_shift.cpp` to `navtracker_tests` source list.

- [ ] **Step 6: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R DatumShiftTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: 4 new PASS. Full suite stays green.

- [ ] **Step 7: Commit**

```
git add core/tracking/DatumShift.hpp core/tracking/DatumShift.cpp \
        tests/tracking/test_datum_shift.cpp CMakeLists.txt
git commit -m "tracking: shiftTracksOnDatumChange utility + axis rotation"
```

---

## Task 3: `MeasurementBuilders` drop the `Datum` arg

**Files:**
- Modify: `core/types/MeasurementBuilders.hpp`
- Modify: `core/types/MeasurementBuilders.cpp`
- Modify: `tests/types/test_measurement_builders.cpp` (update existing + add 1)

### Why

Per spec §6.1: the builders pull the datum from the provider. Library users construct fewer concepts.

### Steps

- [ ] **Step 1: Update the headers**

In `core/types/MeasurementBuilders.hpp`, remove `const geo::Datum& datum` parameter from `makeMeasurementFromRelativeBearing` and `makeMeasurementFromTrueBearing`. `makeMeasurementFromEnuPosition` is unchanged.

- [ ] **Step 2: Update implementation**

In `core/types/MeasurementBuilders.cpp`:
- Replace each `datum.toEnu(...)` call with `provider.datum().toEnu(...)`.
- Before the call, check `provider.hasDatum()` — if false, return empty `Measurement` (the existing "no pose available" semantics from §5.2).

- [ ] **Step 3: Update existing tests**

In `tests/types/test_measurement_builders.cpp`, every call to `makeMeasurementFromRelativeBearing` / `makeMeasurementFromTrueBearing` drops the `datum` arg. The tests already construct `OwnShipProvider` and push poses — the provider's auto-datum will fire on the first pose update.

Add one new test:

```cpp
TEST(MeasurementBuildersTest, EmptyWhenProviderHasNoDatum) {
  OwnShipProvider provider;  // no pose pushed -> no datum
  // No registerDatumSink / update calls.
  Measurement m = makeMeasurementFromRelativeBearing(
      SensorKind::ArpaTtm, "test", Timestamp::fromSeconds(0.0),
      1500.0, 0.5, 50.0, 1.0 * 3.14159265358979 / 180.0,
      provider, {});
  EXPECT_EQ(m.value.size(), 0);
  EXPECT_EQ(m.covariance.size(), 0);
}
```

- [ ] **Step 4: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build -R MeasurementBuildersTest --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: green. Existing tests adapt to the signature change; the new test passes.

- [ ] **Step 5: Commit**

```
git add core/types/MeasurementBuilders.hpp core/types/MeasurementBuilders.cpp \
        tests/types/test_measurement_builders.cpp
git commit -m "builders: drop Datum arg; pull from provider"
```

---

## Task 4: `synthesizeOwnShipTrack` takes the provider

**Files:**
- Modify: `core/collision/CpaOwnShip.hpp`
- Modify: `core/collision/CpaOwnShip.cpp`
- Modify: `tests/collision/test_cpa_synthesize_own_ship.cpp` (update existing)
- Modify: `tests/scenario/test_cpa_scenario.cpp` (update existing)
- Modify: `tests/sim/test_bus_cpa_uncertainty.cpp` (update existing)

### Why

Per spec §6.2: drop the `Datum` arg; take the provider. Mirrors the builder change.

### Steps

- [ ] **Step 1: Update signature**

```cpp
// Before:
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const geo::Datum& datum);

// After:
Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const OwnShipProvider& provider);
```

Implementation uses `provider.datum()`; throws naturally if datum isn't set (matches the rest of the contract).

- [ ] **Step 2: Update existing tests**

Three test files (per "Files:" above). Each call to `synthesizeOwnShipTrack(pose, t, datum)` becomes `synthesizeOwnShipTrack(pose, t, provider)`. The tests already construct a `Datum` and call `provider.update(pose)` — the auto-datum path now handles datum population.

- [ ] **Step 3: Build + run**

```
cmake --build build --target navtracker_tests && \
  ctest --test-dir build --output-on-failure
```
Expected: green.

- [ ] **Step 4: Commit**

```
git add core/collision/CpaOwnShip.hpp core/collision/CpaOwnShip.cpp \
        tests/collision/test_cpa_synthesize_own_ship.cpp \
        tests/scenario/test_cpa_scenario.cpp \
        tests/sim/test_bus_cpa_uncertainty.cpp
git commit -m "cpa: synthesizeOwnShipTrack takes provider instead of datum"
```

---

## Task 5: Scenario test — full integration

**Files:**
- Create: `tests/scenario/test_datum_recenter_scenario.cpp`
- Modify: `CMakeLists.txt`

### Why

Per spec §10 (#5): integration test verifying the end-to-end auto-recenter path. Build a tracker + provider + a track at known (lat, lon). Push own-ship poses crossing the 30 km threshold so a recenter fires. Verify the track's geodetic position is preserved.

### Steps

- [ ] **Step 1: Write the test**

Create `tests/scenario/test_datum_recenter_scenario.cpp`:

```cpp
#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/tracking/DatumShift.hpp"
#include "core/tracking/TrackManager.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

namespace {
class TrackShifterSink : public IDatumChangeSink {
 public:
  TrackManager* mgr;
  explicit TrackShifterSink(TrackManager* m) : mgr(m) {}
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    shiftTracksOnDatumChange(*mgr, o, n);
  }
};
}  // namespace

TEST(DatumRecenterScenario, GeodeticPositionPreservedAcrossRecenter) {
  OwnShipProvider provider;
  TrackManager mgr(2, 3);
  TrackShifterSink sink(&mgr);
  provider.registerDatumSink(&sink);

  // Initial pose -> datum at (53.5, 8.0).
  OwnShipPose start;
  start.time = Timestamp::fromSeconds(0.0);
  start.lat_deg = 53.5;
  start.lon_deg = 8.0;
  provider.update(start);

  // Place a track at a known target lat/lon (a bit north-east).
  const double target_lat = 53.6, target_lon = 8.2;
  const auto enu_target = provider.datum().toEnu({target_lat, target_lon, 0.0});
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.state.resize(4);
  t.state << enu_target.x(), enu_target.y(), 0.0, 0.0;
  t.covariance = Eigen::Matrix4d::Identity() * 25.0;
  mgr.add(t);

  // Push poses crossing the 30 km threshold so recenter fires.
  OwnShipPose far;
  far.time = Timestamp::fromSeconds(60.0);
  far.lat_deg = 53.5;
  far.lon_deg = 9.0;  // ~66 km east — well past 30 km
  provider.update(far);

  // Confirm datum moved to the far pose.
  const auto& new_datum = provider.datum();
  const auto enu_far_in_new = new_datum.toEnu({far.lat_deg, far.lon_deg, 0.0});
  EXPECT_NEAR(enu_far_in_new.x(), 0.0, 1e-3);

  // Verify the track's geodetic position is preserved.
  const Track& t_after = mgr.tracks()[0];
  const auto geo_after = new_datum.toGeodetic(
      Eigen::Vector3d(t_after.state(0), t_after.state(1), 0.0));
  EXPECT_NEAR(geo_after.lat_deg, target_lat, 1e-4);
  EXPECT_NEAR(geo_after.lon_deg, target_lon, 1e-4);
}

}  // namespace navtracker
```

- [ ] **Step 2: Wire into CMake**

Add `tests/scenario/test_datum_recenter_scenario.cpp` to `navtracker_tests`.

- [ ] **Step 3: Build + run**

```
ctest --test-dir build -R DatumRecenterScenario --output-on-failure && \
  ctest --test-dir build --output-on-failure
```
Expected: PASS; full suite green.

- [ ] **Step 4: Commit**

```
git add tests/scenario/test_datum_recenter_scenario.cpp CMakeLists.txt
git commit -m "scenario: end-to-end auto-recenter test (geodetic position preserved)"
```

---

## Task 6: `app/example.cpp` + docs update

**Files:**
- Modify: `app/example.cpp`
- Modify: `CLAUDE.md` (Library use section)
- Modify: `README.md` (if necessary)

### Why

The example currently constructs `Datum({53.5, 8.0, 0.0})` and passes it into builders. With the auto-datum work, that line disappears. Document the new pattern.

### Steps

- [ ] **Step 1: Update `app/example.cpp`**

Remove the explicit `Datum` construction. The example becomes:

```cpp
OwnShipProvider provider;       // library handles datum automatically
TrackManager mgr{2, 3};

// Wire datum-recenter event so tracks stay in the current ENU frame.
struct TrackShifterSink : IDatumChangeSink {
  TrackManager* mgr;
  void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
    shiftTracksOnDatumChange(*mgr, o, n);
  }
};
TrackShifterSink mgr_sink{&mgr};
provider.registerDatumSink(&mgr_sink);

// Build measurements without an explicit datum.
Measurement m = makeMeasurementFromRelativeBearing(
    SensorKind::ArpaTtm, "my_radar",
    Timestamp::fromSeconds(123.0),
    range_m, rel_bearing_rad,
    range_std_m, bearing_std_rad,
    provider);
```

Make sure the example also pushes an `OwnShipPose` before any measurement construction so the datum is initialised.

- [ ] **Step 2: Update CLAUDE.md "Library use" section**

Replace any "construct a `Datum`" prose with the auto-datum pattern. Add a short paragraph on the recenter event and the TrackShifterSink pattern.

- [ ] **Step 3: Build + run**

```
cmake --build build --target navtracker_example && \
  ctest --test-dir build --output-on-failure
```
Expected: example builds and runs; full suite green.

- [ ] **Step 4: Commit**

```
git add app/example.cpp CLAUDE.md README.md
git commit -m "docs: auto-datum + TrackShifterSink in example and library-use docs"
```

---

## Done criteria

- All 6 tasks committed.
- Full suite green at the end.
- `app/example.cpp` no longer constructs an explicit `Datum`.
- `OwnShipProvider provider;` (no-arg) followed by `provider.update(pose)` is the documented library-friendly pattern.
- Sim adapters still take an explicit `Datum`; sim tests unaffected.
- Existing tests that constructed `Datum` and passed it through builders / `synthesizeOwnShipTrack` adapt to the new signatures; backward-compat constructor handles the explicit-datum path.
