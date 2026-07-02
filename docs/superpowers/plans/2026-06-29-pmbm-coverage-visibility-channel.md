# PMBM Coverage / Visibility Channel Implementation Plan (Task 4)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give PMBM an honest per-(channel, track) coverage model so it can tell "target gone" from "nobody looked", retiring three crutches (wrong per-blip miss-math, `idle_halflife_sec`, the `source_id="ais"` patch).

**Architecture:** A new nullable port `ISensorActivity` (in `ports/`, zero I/O) answers "did this channel have a real chance to observe this track now?". Surveillance sensors (radar/EO/lidar) charge **one** misdetection per completed *duty cycle* over covered ground; cooperative-announce sources (AIS, the encrypted Cooperative channel) never touch existence — an overdue self-report raises a **comms-loss/stale signal**. Cadence/coverage comes from an *exchangeable provider interface* with a declared-profile implementation now (adaptive later). All new behaviour is behind config/flags defaulting to today's behaviour; the determinism invariant is preserved (pure function of declared profiles + timestamps, no wall-clock, no RNG).

**Tech Stack:** C++17, Eigen 3.4, CMake + Conan, GoogleTest/gmock.

**Source spec:** `docs/superpowers/specs/2026-06-26-pmbm-coverage-visibility-channel.md` (all decisions settled 2026-06-29; `platform_id` is numeric `std::uint64_t`).

## Global Constraints

- **C++17 only** (no later standard without discussion).
- **Hexagonal:** `core/` and `ports/` have **zero I/O** and zero sensor-format knowledge. Any philos activity wiring lives in `adapters/` or the bench harness.
- **Determinism test stays green:** no wall-clock, no RNG; replay of a log twice → identical output.
- **Back-compat:** the port is nullable; every new config field defaults to today's behaviour; existing bare predict/update tests must be **bit-identical**.
- **Autoferry guard:** any change promoted to a canonical config must not regress the existing PMBM autoferry win.
- **Stable track identity (invariant 5):** `platform_id`/`mmsi` are *hints*, never the fusion key; internal `track_id` stays primary.
- **Build commands** (from repo root; `conan install` only when deps change — it needs the sandbox **disabled** due to the `~/.conan2` readonly-cache gotcha):
  - `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build -j`
  - `ctest --test-dir build --output-on-failure`
- **Documentation standard:** any non-trivial algorithm change updates the four-section algorithm doc (`docs/algorithms/pmbm-design.md`) and the plain-English `docs/learning/` chapter + a diagram.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `ports/ISensorActivity.hpp` | The new port: channel kind, coverage, cadence; `MissOpportunity` query; exchangeable cadence-provider interface + declared-profile impl declaration | Create |
| `core/sensor_activity/DeclaredSensorActivity.{hpp,cpp}` | Declared-profile implementation of the provider (static config → answers). Pure, deterministic, no I/O. | Create |
| `core/types/Measurement.hpp` | Add `std::optional<std::uint64_t> platform_id` to `AssociationHints` | Modify (`:15-18`) |
| `core/pmbm/PmbmTracker.hpp` | `setSensorActivity(const ISensorActivity*)`, member, new Config flags (`use_sensor_activity`, `cooperative_stale_timeout_sec`) | Modify |
| `core/pmbm/PmbmTracker.cpp` | Per-duty-cycle surveillance miss; cooperative stale signal (no existence change); generalised identity gate (`mmsi` else `platform_id`); retire `idle_halflife`/per-blip miss when activity wired | Modify (`:495-779`, `:1236`) |
| `ports/ITrackSink.hpp` (or new `ports/IStaleSignalSink.hpp`) | Mechanism to surface the comms-loss/stale signal | Modify or Create |
| `adapters/benchmark/` (philos replay) | Supply philos activity profile (radar=surveillance+cadence+coverage; AIS=cooperative) | Modify |
| `core/benchmark/Config.cpp` | New bench config `imm_cv_ct_pmbm_coverage` derived from the bundle, with activity wired | Modify (`:649-677`) |
| `tests/pmbm/test_pmbm_sensor_activity.cpp` | All Task-4 unit tests | Create |
| `tests/sensor_activity/test_declared_sensor_activity.cpp` | Provider unit tests | Create |
| `docs/algorithms/pmbm-design.md` | Four-section update | Modify |
| `docs/learning/` | Plain-English coverage/cadence chapter + diagram | Modify |

---

## Task 1: The `ISensorActivity` port + declared-profile provider

**Files:**
- Create: `ports/ISensorActivity.hpp`
- Create: `core/sensor_activity/DeclaredSensorActivity.hpp`, `core/sensor_activity/DeclaredSensorActivity.cpp`
- Test: `tests/sensor_activity/test_declared_sensor_activity.cpp`

**Interfaces:**
- Consumes: `navtracker::SensorKind` (`core/types/Ids.hpp:14-16`), `navtracker::Timestamp`, `Eigen::Vector2d`.
- Produces: the `ISensorActivity` interface and `MissOpportunity` value type used by Task 4; `DeclaredSensorActivity` ctor `DeclaredSensorActivity(std::vector<ChannelProfile>)`.

- [ ] **Step 1: Write the port header.** Create `ports/ISensorActivity.hpp`. Follow the minimal-pure-virtual style of `ports/ITrackSink.hpp:31-38` and `ports/ISensorDetectionModel.hpp`. Document the surveillance-vs-cooperative split (spec §2–§4) in the header comment.

```cpp
#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Two sensor archetypes (see docs spec §2):
//  - Surveillance (radar/EO/lidar): searches a place on a duty cycle;
//    silence over covered ground is STRONG, symmetric evidence.
//  - Cooperative-announce (AIS, Cooperative channel): the target announces
//    itself; silence is WEAK, asymmetric evidence keyed on identity.
enum class ChannelKind { Surveillance, Cooperative };

// What the misdetection step needs to know for one (channel, track) pair
// at one query time. A pure value; no I/O.
struct MissOpportunity {
  // A surveillance sensor completed a sweep that COVERED this track's
  // predicted position and returned nothing for it -> charge exactly one
  // miss with `p_D`. False for cooperative channels and for surveillance
  // channels that were off / out of coverage / mid-sweep.
  bool surveillance_miss{false};
  double p_D{0.0};  // only meaningful when surveillance_miss == true

  // A cooperative source's own-identity report was overdue at the query
  // time. Raises a comms-loss/stale signal; NEVER changes existence
  // (decision spec §9c).
  bool cooperative_overdue{false};
};

// Nullable port. If unwired, PMBM behaves exactly as before.
class ISensorActivity {
 public:
  virtual ~ISensorActivity() = default;

  // Aggregate over every channel this provider knows. `track_pos_enu` is
  // the track's predicted position; `mmsi`/`platform_id` are its identity
  // hints (either may be empty); `now` and `last_checked` bound the
  // interval being evaluated. Implementations MUST be a pure function of
  // declared profiles + the arguments (no wall-clock, no RNG).
  virtual MissOpportunity evaluate(const Eigen::Vector2d& track_pos_enu,
                                   std::optional<std::uint32_t> mmsi,
                                   std::optional<std::uint64_t> platform_id,
                                   Timestamp last_checked,
                                   Timestamp now) const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the failing provider test.** Create `tests/sensor_activity/test_declared_sensor_activity.cpp`.

```cpp
#include <gtest/gtest.h>

#include "core/sensor_activity/DeclaredSensorActivity.hpp"

using navtracker::ChannelKind;
using navtracker::DeclaredSensorActivity;
using navtracker::Timestamp;

namespace {
DeclaredSensorActivity::ChannelProfile radar() {
  DeclaredSensorActivity::ChannelProfile p;
  p.kind = ChannelKind::Surveillance;
  p.sensor = navtracker::SensorKind::ArpaTtm;
  p.duty_cycle_sec = 60.0;
  p.max_range_m = 10000.0;
  p.p_D = 0.9;
  return p;
}
}  // namespace

TEST(DeclaredSensorActivity, SurveillanceChargesOneMissPerCompletedSweepInCoverage) {
  DeclaredSensorActivity act({radar()});
  // In coverage (5 km), a full 60 s duty cycle elapsed -> one miss at p_D.
  auto r = act.evaluate({5000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_TRUE(r.surveillance_miss);
  EXPECT_DOUBLE_EQ(r.p_D, 0.9);
  EXPECT_FALSE(r.cooperative_overdue);
}

TEST(DeclaredSensorActivity, NoMissOutsideCoverage) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({20000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(61.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, NoMissMidSweep) {
  DeclaredSensorActivity act({radar()});
  auto r = act.evaluate({5000.0, 0.0}, std::nullopt, std::nullopt,
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(30.0));
  EXPECT_FALSE(r.surveillance_miss);
}

TEST(DeclaredSensorActivity, CooperativeOverdueRaisesSignalNotMiss) {
  DeclaredSensorActivity::ChannelProfile coop;
  coop.kind = ChannelKind::Cooperative;
  coop.sensor = navtracker::SensorKind::Cooperative;
  coop.expected_report_interval_sec = 10.0;
  DeclaredSensorActivity act({coop});
  auto r = act.evaluate({0.0, 0.0}, std::nullopt, std::optional<std::uint64_t>{42},
                        Timestamp::fromSeconds(0.0), Timestamp::fromSeconds(25.0));
  EXPECT_FALSE(r.surveillance_miss);
  EXPECT_TRUE(r.cooperative_overdue);
}
```

- [ ] **Step 3: Run it to confirm it fails to compile/link.**

Run: `cmake --build build -j --target navtracker_sensor_activity_tests 2>&1 | tail -20` (or the unified test target). Expected: FAIL — `DeclaredSensorActivity.hpp` not found.

- [ ] **Step 4: Implement the declared provider.** Create `core/sensor_activity/DeclaredSensorActivity.hpp`:

```cpp
#pragma once

#include <vector>

#include "ports/ISensorActivity.hpp"

namespace navtracker {

// Static, deterministic implementation of ISensorActivity (decision §9a:
// declared profile behind the exchangeable port). An adaptive learned
// provider is a future implementation of the same ISensorActivity
// interface (spec roadmap §13.1).
class DeclaredSensorActivity : public ISensorActivity {
 public:
  struct ChannelProfile {
    ChannelKind kind{ChannelKind::Surveillance};
    SensorKind sensor{SensorKind::Unknown};
    // Surveillance:
    double duty_cycle_sec{0.0};
    double max_range_m{0.0};
    double sector_center_rad{0.0};
    double sector_width_rad{6.283185307179586};  // 2*pi = full circle
    double p_D{0.0};
    // Cooperative:
    double expected_report_interval_sec{0.0};
  };

  explicit DeclaredSensorActivity(std::vector<ChannelProfile> profiles)
      : profiles_(std::move(profiles)) {}

  MissOpportunity evaluate(const Eigen::Vector2d& track_pos_enu,
                           std::optional<std::uint32_t> mmsi,
                           std::optional<std::uint64_t> platform_id,
                           Timestamp last_checked,
                           Timestamp now) const override;

 private:
  std::vector<ChannelProfile> profiles_;
};

}  // namespace navtracker
```

- [ ] **Step 5: Implement `.cpp`.** Create `core/sensor_activity/DeclaredSensorActivity.cpp`:

```cpp
#include "core/sensor_activity/DeclaredSensorActivity.hpp"

#include <cmath>

namespace navtracker {

namespace {
bool inCoverage(const DeclaredSensorActivity::ChannelProfile& p,
                const Eigen::Vector2d& track_pos_enu) {
  // Sensor assumed at the ENU origin for the declared profile (own-ship
  // datum). Range gate then optional azimuth-sector gate.
  const double range = track_pos_enu.norm();
  if (range > p.max_range_m) return false;
  if (p.sector_width_rad < 6.283185307179586) {
    const double az = std::atan2(track_pos_enu.y(), track_pos_enu.x());
    const double off = std::remainder(az - p.sector_center_rad,
                                      6.283185307179586);
    if (std::abs(off) > 0.5 * p.sector_width_rad) return false;
  }
  return true;
}
}  // namespace

MissOpportunity DeclaredSensorActivity::evaluate(
    const Eigen::Vector2d& track_pos_enu,
    std::optional<std::uint32_t> /*mmsi*/,
    std::optional<std::uint64_t> /*platform_id*/,
    Timestamp last_checked, Timestamp now) const {
  MissOpportunity out;
  const double dt = now.seconds() - last_checked.seconds();
  if (dt <= 0.0) return out;
  for (const auto& p : profiles_) {
    if (p.kind == ChannelKind::Surveillance) {
      if (p.duty_cycle_sec <= 0.0) continue;
      if (dt < p.duty_cycle_sec) continue;          // mid-sweep: no chance
      if (!inCoverage(p, track_pos_enu)) continue;  // not covered: no chance
      // One completed sweep that covered the track and returned nothing.
      // Aggregate: keep the strongest p_D among surveillance channels.
      out.surveillance_miss = true;
      if (p.p_D > out.p_D) out.p_D = p.p_D;
    } else {  // Cooperative
      if (p.expected_report_interval_sec <= 0.0) continue;
      if (dt > p.expected_report_interval_sec) out.cooperative_overdue = true;
    }
  }
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 6: Register both source files in CMake.** Add `core/sensor_activity/DeclaredSensorActivity.cpp` to the `navtracker_core` target source list and the new test file to the test target. Find the existing pattern (e.g. `core/pmbm/PmbmTracker.cpp` in the core target's `target_sources`/glob, and `tests/pmbm/*.cpp` in the test target) in `CMakeLists.txt` and mirror it. If the project globs sources, no edit may be needed — verify by building.

- [ ] **Step 7: Build & run the provider tests; confirm PASS.**

Run: `cmake --build build -j && ctest --test-dir build -R DeclaredSensorActivity --output-on-failure`
Expected: 4 tests PASS.

- [ ] **Step 8: Commit.**

```bash
git add ports/ISensorActivity.hpp core/sensor_activity/ tests/sensor_activity/ CMakeLists.txt
git commit -m "Task 4 Step 1: ISensorActivity port + declared-profile provider"
```

---

## Task 2: Add numeric `platform_id` to `AssociationHints`

**Files:**
- Modify: `core/types/Measurement.hpp:15-18`
- Test: `tests/pmbm/test_pmbm_sensor_activity.cpp` (new file; first test here)

**Interfaces:**
- Produces: `AssociationHints::platform_id` (`std::optional<std::uint64_t>`) read by Task 3's identity gate.

- [ ] **Step 1: Write the failing test.** Create `tests/pmbm/test_pmbm_sensor_activity.cpp` with just:

```cpp
#include <gtest/gtest.h>

#include "core/types/Measurement.hpp"

TEST(AssociationHints, CarriesNumericPlatformId) {
  navtracker::AssociationHints h;
  EXPECT_FALSE(h.platform_id.has_value());
  h.platform_id = std::uint64_t{1234567890123ULL};
  ASSERT_TRUE(h.platform_id.has_value());
  EXPECT_EQ(*h.platform_id, 1234567890123ULL);
}
```

- [ ] **Step 2: Run it; confirm it fails to compile** (`'platform_id' is not a member`).

Run: `cmake --build build -j 2>&1 | grep -i platform_id | head`

- [ ] **Step 3: Add the field.** Edit `core/types/Measurement.hpp`:

```cpp
// Opportunistic identity cues from a sensor; never the fusion key.
struct AssociationHints {
  std::optional<std::uint32_t> mmsi;
  std::optional<std::int32_t> sensor_track_id;
  // Cooperative-channel native id (numeric, settled 2026-06-29). Always
  // set by the Cooperative adapter; assumed unique per fleet member. A
  // strong association prior but still a hint, never the fusion key.
  std::optional<std::uint64_t> platform_id;
};
```

- [ ] **Step 4: Build & run; confirm PASS.**

Run: `cmake --build build -j && ctest --test-dir build -R AssociationHints --output-on-failure`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add core/types/Measurement.hpp tests/pmbm/test_pmbm_sensor_activity.cpp
git commit -m "Task 4 Step 3a: numeric platform_id on AssociationHints"
```

---

## Task 3: Generalise the identity gate to `mmsi` else `platform_id`

**Files:**
- Modify: `core/pmbm/PmbmTracker.cpp:495-523` (scan-identity collection + `should_misdetect`), `:1236` (touch.vessel_id population)
- Modify: `core/pmbm/PmbmTracker.hpp` (the `SourceTouch`/contribution_history identity field, if `vessel_id` is `uint32`-typed — widen to a unified key)
- Test: `tests/pmbm/test_pmbm_sensor_activity.cpp`

**Interfaces:**
- Consumes: `AssociationHints::platform_id` (Task 2), `AssociationHints::mmsi`.
- Produces: a unified identity key used by `should_misdetect` so two observations are "the same vessel" if they share **either** `mmsi` **or** `platform_id`.

**Background (current code, verbatim `core/pmbm/PmbmTracker.cpp:503-523`):**
```cpp
  for (const auto& z : scan) {
    scan_sources.insert(z.source_id);
    if (cfg_.source_aware_identity && z.hints.mmsi.has_value())
      scan_vessels.insert(*z.hints.mmsi);
  }
  // ...
  auto should_misdetect = [&](BernoulliId id) {
    if (!cfg_.source_aware_misdetection) return true;
    auto it = contribution_history_.find(id);
    if (it == contribution_history_.end() || it->second.empty()) return true;
    for (const auto& touch : it->second) {
      if (cfg_.source_aware_identity && touch.vessel_id.has_value()) {
        if (scan_vessels.count(*touch.vessel_id)) return true;
        continue;
      }
      if (scan_sources.count(touch.source_id)) return true;
    }
    return false;
  };
```
`touch.vessel_id` is set at `:1236` from `best->hints.mmsi`.

- [ ] **Step 1: Inspect the identity types.** Read `core/pmbm/PmbmTracker.hpp` around the `SourceTouch`/`contribution_history_` declaration and confirm the type of `touch.vessel_id` (expected `std::optional<std::uint32_t>`). The unified key must hold either an MMSI or a `platform_id` (`uint64`). Choose: widen `vessel_id` to `std::optional<std::uint64_t>` and add a small disambiguator, OR add a parallel `std::optional<std::uint64_t> platform_id` to `SourceTouch`. Prefer the **parallel field** (no collision risk between an MMSI value and a platform_id value).

- [ ] **Step 2: Write the failing tests** (append to `tests/pmbm/test_pmbm_sensor_activity.cpp`). These assert the gate semantics via the public tracker API. Use the existing `tests/pmbm/test_pmbm_tracker_update.cpp` fixture style (motion + `EkfEstimator`, `mutableDensityForTesting()`, `processBatch`).

```cpp
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"

using navtracker::EkfEstimator;
using navtracker::PmbmTracker;

// (a) A cooperative-only track is keyed by platform_id: an in-scan
// observation sharing the platform_id keeps the track "observable"
// (gate returns true -> miss math applies); an unrelated platform_id
// does not.
TEST(PmbmIdentityGate, CooperativeOnlyKeyedByPlatformId) {
  // Build a tracker with source_aware_misdetection + source_aware_identity,
  // seed one Bernoulli whose contribution history carries platform_id=7,
  // feed a scan whose only measurement carries platform_id=7, assert the
  // Bernoulli is treated as observable (existence drops on a real miss);
  // then repeat with platform_id=8 and assert existence is unchanged.
  // ... full body using mutableDensityForTesting() + processBatch ...
}

// (b) A member carrying BOTH platform_id and mmsi fuses with its AIS track:
// two observations sharing EITHER key are the same vessel.
TEST(PmbmIdentityGate, SharedEitherKeyIsSameVessel) {
  // Seed a Bernoulli whose history has mmsi=111; feed a cooperative
  // measurement with platform_id=7 AND mmsi=111; assert the gate treats it
  // as the same vessel (shared mmsi) -> observable.
}
```

> Implementer note: fill the test bodies completely following `test_pmbm_tracker_update.cpp`'s `mkBernoulli` helper and `mutableDensityForTesting().mbm.push_back(...)` pattern; seed `contribution_history_` via a public test seam if one exists, otherwise by running one `processBatch` that detects the seed measurement so the touch is recorded. No placeholders in the committed test.

- [ ] **Step 3: Run; confirm FAIL** (platform_id not yet considered by the gate).

Run: `ctest --test-dir build -R PmbmIdentityGate --output-on-failure`

- [ ] **Step 4: Implement the unified key.** In `core/pmbm/PmbmTracker.cpp`:
  - Add a `std::set<std::uint64_t> scan_platforms;` alongside `scan_vessels`, populated from `z.hints.platform_id`.
  - In `should_misdetect`, before returning false, also: `if (touch.platform_id.has_value() && scan_platforms.count(*touch.platform_id)) return true;`
  - At `:1236`, also set `touch.platform_id = best->hints.platform_id;`.
  - Add the `platform_id` field to `SourceTouch` in `PmbmTracker.hpp` (per Step 1 decision).

- [ ] **Step 5: Build & run; confirm PASS** (both new gate tests + the existing `tests/pmbm` suite still green).

Run: `cmake --build build -j && ctest --test-dir build -R "PmbmIdentityGate|Pmbm" --output-on-failure`

- [ ] **Step 6: Commit.**

```bash
git add core/pmbm/PmbmTracker.hpp core/pmbm/PmbmTracker.cpp tests/pmbm/test_pmbm_sensor_activity.cpp
git commit -m "Task 4 Step 3b: unified identity gate (mmsi else platform_id)"
```

---

## Task 4: Wire `setSensorActivity` into PmbmTracker (nullable, behaviour-neutral)

**Files:**
- Modify: `core/pmbm/PmbmTracker.hpp` (setter + member + Config flags)
- Test: `tests/pmbm/test_pmbm_sensor_activity.cpp`

**Interfaces:**
- Consumes: `ISensorActivity*` (Task 1).
- Produces: `PmbmTracker::setSensorActivity(const ISensorActivity*)`; Config fields `bool use_sensor_activity = false;` and `double cooperative_stale_timeout_sec = 0.0;` for Tasks 5–6.

- [ ] **Step 1: Write the failing test** (append). A tracker with no activity wired must be bit-identical to today on the canonical misdetection test.

```cpp
TEST(PmbmSensorActivity, NullActivityIsBehaviourNeutral) {
  // Same setup as PmbmTrackerUpdate.MisdetectionDecaysExistenceCorrectly
  // (pD=0.9, r0=0.8, empty scan), WITHOUT calling setSensorActivity.
  // Expect existence == 0.08/0.28 exactly (today's behaviour).
}
```

- [ ] **Step 2: Run; confirm FAIL to compile** (no `setSensorActivity`). Add only the test reference that needs the symbol so it fails for the right reason. (If you prefer, this test compiles once the symbol exists; it documents the contract.)

- [ ] **Step 3: Add setter, member, and Config flags.** In `core/pmbm/PmbmTracker.hpp`, next to `setSensorDetectionModel` (`:537-539`):

```cpp
  void setSensorActivity(const ISensorActivity* a) { sensor_activity_ = a; }
```
Add member next to `detection_model_`:
```cpp
  const ISensorActivity* sensor_activity_{nullptr};
```
Add `#include "ports/ISensorActivity.hpp"` to the header includes. In `Config`, near `idle_halflife_sec` (`:356`) and `dedup_miss_pd` (`:278`):
```cpp
    // Task 4: when true AND sensor_activity is wired, existence moves only
    // on a genuine per-duty-cycle surveillance miss; idle_halflife and the
    // per-blip miss path are bypassed (spec §4, §12). Default false ->
    // today's behaviour, bit-identical.
    bool use_sensor_activity = false;
    // Task 4 (spec §4 case 2): a cooperative-only track is retired only
    // after this many seconds with no own-identity report (0 = never by
    // this rule). Never a per-sweep existence penalty.
    double cooperative_stale_timeout_sec = 0.0;
```

- [ ] **Step 4: Build & run; confirm PASS** and the whole pmbm suite is unchanged.

Run: `cmake --build build -j && ctest --test-dir build -R Pmbm --output-on-failure`

- [ ] **Step 5: Commit.**

```bash
git add core/pmbm/PmbmTracker.hpp tests/pmbm/test_pmbm_sensor_activity.cpp
git commit -m "Task 4 Step 2: setSensorActivity (nullable) + coverage config flags"
```

---

## Task 5: Per-duty-cycle surveillance miss

**Files:**
- Modify: `core/pmbm/PmbmTracker.cpp` (empty-scan miss branch `:599-634` and per-assignment misdetection branch `:747-779`)
- Test: `tests/pmbm/test_pmbm_sensor_activity.cpp`

**Interfaces:**
- Consumes: `sensor_activity_->evaluate(...)` (Task 1, Task 4), `cfg_.use_sensor_activity`.
- Produces: misdetection that, when activity is wired + `use_sensor_activity`, charges existence **only** from `MissOpportunity::surveillance_miss` (with its `p_D`), bypassing `compute_miss_pD`/`idle_decay_for`.

- [ ] **Step 1: Write the failing tests** (append). Use a `DeclaredSensorActivity` with a radar profile.

```cpp
// Active radar that covers the track and completes a sweep with no return
// applies EXACTLY ONE miss at the profile p_D.
TEST(PmbmSensorActivity, SurveillanceCoveredSweepAppliesOneMiss) {
  // cfg.use_sensor_activity = true; wire DeclaredSensorActivity{radar pD=0.9,
  // range=10km, duty=60s}. Seed Bernoulli r0=0.8 at (5km,0). predict to t=0,
  // processBatch({}) at a time >= 60s later. Expect r == 0.08/0.28.
}

// Out of coverage -> no penalty (existence unchanged).
TEST(PmbmSensorActivity, OutOfCoverageNoPenalty) {
  // Same but track at (20km,0). Expect r == 0.8 (unchanged).
}

// Mid-sweep (dt < duty_cycle) -> no penalty.
TEST(PmbmSensorActivity, MidSweepNoPenalty) {
  // dt = 30s < 60s. Expect r == 0.8 (unchanged).
}
```

> Implementer note: the time bookkeeping needs a per-Bernoulli "last activity check" timestamp so `evaluate(last_checked, now)` has a window. Store it in `contribution_history_`/Bernoulli or thread `current_time_` and the previous scan time. Use `scan_time_sec`/`current_time_` already present in `enumerateChildren` (`:534`). Keep it deterministic.

- [ ] **Step 2: Run; confirm FAIL.**

Run: `ctest --test-dir build -R PmbmSensorActivity --output-on-failure`

- [ ] **Step 3: Implement.** In both miss branches, when `cfg_.use_sensor_activity && sensor_activity_`, replace the `compute_miss_pD(b)` + `idle_decay_for(b)` path with:

```cpp
// Task 4: honest per-duty-cycle coverage model (spec §4).
const MissOpportunity opp = sensor_activity_->evaluate(
    b.mean.head<2>(), /*mmsi*/std::nullopt, /*platform_id*/std::nullopt,
    Timestamp::fromSeconds(last_check_s), current_time_);
if (!opp.surveillance_miss) {
  // No surveillance chance -> existence UNCHANGED (replaces idle_halflife).
  // (cooperative_overdue is handled by the stale-signal path, Task 6.)
  child.bernoullis.push_back(std::move(updated /*or miss*/));
  continue;
}
const double r = b.existence_probability;
const double pD = opp.p_D;
const double miss_norm = 1.0 - r * pD;
updated.existence_probability = (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
child.log_weight += std::log(std::max(miss_norm, 1e-300));
```
Identity hints for `evaluate` can come from the Bernoulli's last touch (mmsi/platform_id) — pull from `contribution_history_` if available; surveillance coverage does not need identity, so `std::nullopt` is acceptable for Task 5 and refined in Task 6. Leave the legacy path (`compute_miss_pD`/`idle_decay_for`) intact for `use_sensor_activity == false`.

- [ ] **Step 4: Build & run; confirm PASS** (new tests green; legacy pmbm suite unchanged).

Run: `cmake --build build -j && ctest --test-dir build -R Pmbm --output-on-failure`

- [ ] **Step 5: Commit.**

```bash
git add core/pmbm/PmbmTracker.cpp tests/pmbm/test_pmbm_sensor_activity.cpp
git commit -m "Task 4 Step 4: per-duty-cycle surveillance miss behind use_sensor_activity"
```

---

## Task 6: Cooperative stale / comms-loss signal (never touches existence)

**Files:**
- Create: `ports/IStaleSignalSink.hpp` (mirror `ports/ITrackSink.hpp:31-38`)
- Modify: `core/pmbm/PmbmTracker.{hpp,cpp}` (emit signal; cooperative-only retirement via `cooperative_stale_timeout_sec`)
- Test: `tests/pmbm/test_pmbm_sensor_activity.cpp`

**Interfaces:**
- Consumes: `MissOpportunity::cooperative_overdue` (Task 1), `cfg_.cooperative_stale_timeout_sec` (Task 4).
- Produces: `IStaleSignalSink` with `onTrackStale(TrackId, Timestamp)`; `PmbmTracker::setStaleSignalSink(IStaleSignalSink*)`.

- [ ] **Step 1: Write the port.** Create `ports/IStaleSignalSink.hpp`:

```cpp
#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Raised when a cooperative source's own-identity report is overdue
// (spec §9c): "we lost comms", NOT "the vessel sank". Pure notification;
// it MUST NOT be wired to anything that lowers existence.
class IStaleSignalSink {
 public:
  virtual ~IStaleSignalSink() = default;
  virtual void onTrackStale(TrackId id, Timestamp now) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Write the failing tests** (append). A recording fake sink.

```cpp
struct RecordingStaleSink : navtracker::IStaleSignalSink {
  std::vector<navtracker::TrackId> stale;
  void onTrackStale(navtracker::TrackId id, navtracker::Timestamp) override {
    stale.push_back(id);
  }
};

// (a) Overdue own-identity report leaves existence unchanged AND flags stale.
TEST(PmbmStaleSignal, OverdueLeavesExistenceUnchangedAndFlagsStale) {
  // Cooperative profile interval=10s, NO surveillance. Seed cooperative-only
  // Bernoulli (platform_id=42, r0=0.8). Advance 25s with empty scan.
  // Expect r == 0.8 (unchanged) AND sink recorded the track stale.
}

// (b) A cooperative source never affects a DIFFERENT identity's track.
TEST(PmbmStaleSignal, DoesNotAffectDifferentIdentity) {
  // Two cooperative-only tracks (platform_id 42 and 99). A scan carrying a
  // report for 42 must not flag 99 as stale due to 42's activity, and must
  // not change 99's existence via the miss math.
}

// (c) A cooperative-only track is retired ONLY by the long max-stale
// timeout, never by the miss math.
TEST(PmbmStaleSignal, CooperativeOnlyRetiredOnlyByTimeout) {
  // cooperative_stale_timeout_sec = 600. After 25s overdue: still alive
  // (r unchanged, status stale/coasting). After 601s overdue: retired.
}
```

- [ ] **Step 3: Run; confirm FAIL.**

Run: `ctest --test-dir build -R PmbmStaleSignal --output-on-failure`

- [ ] **Step 4: Implement.**
  - Add `IStaleSignalSink* stale_sink_{nullptr};` + `setStaleSignalSink(...)` to `PmbmTracker.hpp` (include the new port).
  - In the miss branches, when `opp.cooperative_overdue` (and not surveillance_miss): **do not** alter existence; if `stale_sink_` fire `onTrackStale(track_id, current_time_)`; mark the aggregated track `TrackStatus::Coasting` in `refreshAggregatedTracks`.
  - Track per-Bernoulli "seconds since last own-identity report"; when it exceeds `cfg_.cooperative_stale_timeout_sec` (and the track has no surveillance coverage), drop the Bernoulli (existence → 0 / prune) — this is the **only** cooperative retirement path. Guard `cooperative_stale_timeout_sec <= 0` → never retire by this rule.

- [ ] **Step 5: Build & run; confirm PASS** (3 new tests + full pmbm suite green).

Run: `cmake --build build -j && ctest --test-dir build -R Pmbm --output-on-failure`

- [ ] **Step 6: Commit.**

```bash
git add ports/IStaleSignalSink.hpp core/pmbm/PmbmTracker.hpp core/pmbm/PmbmTracker.cpp tests/pmbm/test_pmbm_sensor_activity.cpp
git commit -m "Task 4 Step 5: cooperative stale/comms-loss signal (existence-neutral)"
```

---

## Task 7: Philos activity model + bench config + A/B

**Files:**
- Modify: philos replay wiring under `adapters/benchmark/` (the scenario runner that builds the PMBM tracker for philos)
- Modify: `core/benchmark/Config.cpp:649-677` (new `imm_cv_ct_pmbm_coverage` config)
- Output: a CSV under `docs/baselines/`

**Interfaces:**
- Consumes: `DeclaredSensorActivity` (Task 1), `setSensorActivity` (Task 4), `setStaleSignalSink` (Task 6).
- Produces: a runnable bench config whose PMBM uses the coverage model with `idle_halflife_sec=0` and the per-blip miss path off (`use_sensor_activity=true`).

- [ ] **Step 1: Add the bench config.** In `core/benchmark/Config.cpp`, clone `imm_cv_ct_pmbm_bundle` (`:649-677`) as `imm_cv_ct_pmbm_coverage`, set `cfg.use_sensor_activity = true; cfg.idle_halflife_sec = 0.0;` and leave `dedup_miss_pd`/wrong-math off; register the name in the same switch/map that lists the bundle.

- [ ] **Step 2: Wire the activity provider in the philos runner.** Where the philos PMBM tracker is constructed, build a `DeclaredSensorActivity` from the philos sensor set: radar = `ChannelKind::Surveillance` with its rotation period + range/sector; AIS = `ChannelKind::Cooperative` with a per-vessel `expected_report_interval_sec`. Call `tracker.setSensorActivity(&activity)` only for the coverage config. Keep it in `adapters/`/bench (no core I/O).

- [ ] **Step 3: Build the bench binary.**

Run: `cmake --build build -j --target navtracker_bench_baseline` (confirm exact target name in `CMakeLists.txt`).

- [ ] **Step 4: Run the philos A/B.** Bench philos under both `imm_cv_ct_pmbm_bundle` and `imm_cv_ct_pmbm_coverage`; write to `docs/baselines/cl3_coverage_ab_<UTC>.csv`. Use the canonical bench invocation from `docs/algorithms/comparison-baselines.md` (the per-env / philos runner). Background long runs per the build-env note (output file appears at launch; tool prints nothing until "Wrote N rows").

- [ ] **Step 5: Record results.** Capture GOSPA mean + cardinality (over/under-count) + id-switches for both configs into the eval log; do not assert a winner yet — Task 8 decides.

- [ ] **Step 6: Commit** the config + bench artifacts.

```bash
git add core/benchmark/Config.cpp adapters/benchmark/ docs/baselines/cl3_coverage_ab_*.csv
git commit -m "Task 4 Step 7: philos activity model + coverage bench config + A/B run"
```

---

## Task 8: Decision + autoferry guard + retire crutches in the canonical config

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`, `docs/algorithms/comparison-baselines.md`
- Modify (only if promoted): `core/benchmark/Config.cpp` canonical config

**Interfaces:**
- Consumes: Task 7 philos A/B numbers + an autoferry bench run.

- [ ] **Step 1: Run the autoferry guard.** Bench autoferry under `imm_cv_ct_pmbm_coverage` vs the current canonical PMBM config. Confirm **no regression** of the existing PMBM autoferry win.

- [ ] **Step 2: Apply the decision rule (spec §10 Step 8).** If coverage **matches or beats** the Task-2 bundle on philos **with fewer knobs** (no `idle_halflife`, no wrong-math) **and** does not regress autoferry → promote `imm_cv_ct_pmbm_coverage` toward canonical. Otherwise keep it opt-in and record why.

- [ ] **Step 3: Write up** both A/B tables (philos + autoferry) in `docs/algorithms/evaluation-log.md` with the decision and rationale; update `comparison-baselines.md` Cl-3 status.

- [ ] **Step 4: Commit.**

```bash
git add docs/algorithms/evaluation-log.md docs/algorithms/comparison-baselines.md core/benchmark/Config.cpp
git commit -m "Task 4 Step 8: coverage-model decision + autoferry guard"
```

---

## Task 9: Documentation (algorithm doc + learning chapter)

**Files:**
- Modify: `docs/algorithms/pmbm-design.md` (four sections)
- Modify/Create: `docs/learning/` chapter + figure; `docs/learning/00-index.md` + glossary if new chapter

- [ ] **Step 1: Update `pmbm-design.md`** four sections (Math/Assumptions/Rationale/Ways-to-improve) to describe the per-duty-cycle surveillance miss, the cooperative existence-neutral stale signal, the unified identity gate, and the three retired crutches. Pull the prose from spec §4, §8, §12.

- [ ] **Step 2: Add a plain-English `docs/learning/` chapter** on "coverage & cadence: did a sensor really have a chance to see this?" with a mermaid sequence/flow diagram (surveillance sweep vs cooperative announce) and, if a quantitative figure helps, add a `fig_*()` to `docs/learning/figures/generate.py` and re-run it (never hand-edit PNGs). Cross-reference from `pmbm-design.md`.

- [ ] **Step 3: Update `docs/learning/00-index.md` and the glossary** (`19-glossary.md`) if a new chapter/term was added.

- [ ] **Step 4: Commit.**

```bash
git add docs/algorithms/pmbm-design.md docs/learning/
git commit -m "Task 4 Step 9: coverage/visibility docs + learning chapter"
```

---

## Prerequisite (do first if bench numbers must be trusted)

The spec (§11) names a known **PMBM adaptive-birth non-determinism** as a prereq for trustworthy single-seed philos A/B. The Explore pass found **no RNG/wall-clock** in `core/pmbm/`; the suspected source is **scan-order dependence** in the birth-id cache (`scan_birth_id_cache_`, ~`:662, :801-826`). Before Task 7's A/B, run the determinism test twice on philos and confirm bit-identical output; if it diverges, fix the ordering (stable sort / order-independent keying) under TDD and only then trust Task 7/8 numbers. Tasks 1–6 are unaffected (unit-level, deterministic by construction).

---

## Self-Review notes

- **Spec coverage:** §3 port → Task 1; §4 math (surveillance miss) → Task 5; §4 case 2 + §9c (stale signal) → Task 6; §5 identity (numeric platform_id + unified gate) → Tasks 2–3; §6 input contract (activity not stale re-feeds) → Task 7 wiring; §7/§9b/§13 (deferred birth-confidence/adaptive/RCS) → intentionally NOT tasks (roadmap); §10 steps map 1:1; §11 guardrails → Global Constraints + prereq; §12 retire crutches → Tasks 5–6 + Task 8 promotion.
- **Determinism / back-compat:** every behavioural change gated by `use_sensor_activity` (default false) and nullable ports; Task 4 Step 1 test pins bit-identical default behaviour.
- **Types are consistent across tasks:** `platform_id` is `std::optional<std::uint64_t>` everywhere (hints field, `evaluate` arg, `SourceTouch`); `MissOpportunity{bool surveillance_miss; double p_D; bool cooperative_overdue;}`; `ISensorActivity::evaluate(Vector2d, optional<uint32>, optional<uint64>, Timestamp, Timestamp)`.
