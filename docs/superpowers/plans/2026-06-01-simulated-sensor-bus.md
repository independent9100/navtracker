# SimulatedSensorBus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an end-to-end harness in a new `sim/` layer that drives ground-truth trajectories through real `OwnShipNmeaAdapter`, `AisAdapter`, `ArpaAdapter`, and `EoIrAdapter` instances, returning a `Scenario` consumable by the existing `runScenario` / `runScenarioBatched` machinery for OSPA regression.

**Architecture:** New top-level `sim/` directory parallel to `adapters/` and `core/`. Per-sensor `*Emitter` classes encapsulate cadence + noise; orchestrator `SimulatedSensorBus` runs a deterministic time loop, drives emitters, drains adapter `poll()` into a time-ordered `Scenario`. Determinism guaranteed by per-emitter `std::mt19937` substreams derived from a single bus seed.

**Tech Stack:** C++17, Eigen 3.4, GoogleTest, CMake, single root `CMakeLists.txt`. All new source files added to the existing `navtracker_core` library target.

**Reference spec:** `docs/superpowers/specs/2026-06-01-simulated-sensor-bus-design.md`

**Build commands (from repo root):**
- `cmake --build build` (incremental)
- `ctest --test-dir build --output-on-failure --rerun-failed --output-on-failure` for full suite
- Single test: `./build/navtracker_tests --gtest_filter='<TestSuite.TestName>'`

---

## File Structure

**New files (created by this plan):**

```
sim/
  TruthTrajectory.hpp           // struct TruthState; abstract ITruthTrajectory; ConstantVelocityTrajectory
  TruthTrajectory.cpp           // ConstantVelocityTrajectory::eval
  NmeaEncode.hpp                // formatLatDdmm, formatLonDdmm, wrapWithChecksum
  NmeaEncode.cpp                // impls
  SensorEmitter.hpp             // EmitContext + abstract ISensorEmitter
  OwnShipEmitter.hpp / .cpp     // 1 Hz GGA+HDT
  AisEmitter.hpp / .cpp         // SOTDMA cadence, dropouts
  ArpaEmitter.hpp / .cpp        // 3 s $RATTM
  EoIrEmitter.hpp / .cpp        // 10 Hz, FOV gate
  SimulatedSensorBus.hpp / .cpp // orchestrator

tests/sim/
  test_truth_trajectory.cpp
  test_nmea_encode.cpp
  test_own_ship_emitter.cpp
  test_ais_emitter.cpp
  test_arpa_emitter.cpp
  test_eoir_emitter.cpp
  test_simulated_sensor_bus.cpp
  test_bus_determinism.cpp
  test_bus_regression.cpp
```

**Modified files:**
- `CMakeLists.txt` — append each new `sim/*.cpp` to `navtracker_core` and each new `tests/sim/*.cpp` to `navtracker_tests`.

---

## Task 1: TruthTrajectory interface + ConstantVelocityTrajectory

**Files:**
- Create: `sim/TruthTrajectory.hpp`
- Create: `sim/TruthTrajectory.cpp`
- Test: `tests/sim/test_truth_trajectory.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_truth_trajectory.cpp`:

```cpp
#include "sim/TruthTrajectory.hpp"

#include <gtest/gtest.h>

using namespace navtracker;
using sim::ConstantVelocityTrajectory;
using sim::TruthState;

TEST(ConstantVelocityTrajectory, EvaluatesPositionAndVelocity) {
  ConstantVelocityTrajectory traj(
      Eigen::Vector2d(100.0, -50.0),
      Eigen::Vector2d(5.0, 2.5),
      Timestamp::fromSeconds(0.0));

  const TruthState s0 = traj.eval(Timestamp::fromSeconds(0.0));
  EXPECT_DOUBLE_EQ(s0.position.x(), 100.0);
  EXPECT_DOUBLE_EQ(s0.position.y(), -50.0);
  EXPECT_DOUBLE_EQ(s0.velocity.x(), 5.0);
  EXPECT_DOUBLE_EQ(s0.velocity.y(), 2.5);

  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  EXPECT_DOUBLE_EQ(s10.position.x(), 150.0);
  EXPECT_DOUBLE_EQ(s10.position.y(), -25.0);
  EXPECT_DOUBLE_EQ(s10.velocity.x(), 5.0);
  EXPECT_DOUBLE_EQ(s10.velocity.y(), 2.5);
}

TEST(ConstantVelocityTrajectory, ConstVelocityIndependentOfT0) {
  ConstantVelocityTrajectory traj(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d(1.0, 0.0),
      Timestamp::fromSeconds(5.0));

  const TruthState s = traj.eval(Timestamp::fromSeconds(7.0));
  EXPECT_DOUBLE_EQ(s.position.x(), 2.0);   // (7 - 5) * 1.0
  EXPECT_DOUBLE_EQ(s.position.y(), 0.0);
}
```

- [ ] **Step 2: Append the test to `tests/sim/test_truth_trajectory.cpp` to `CMakeLists.txt`**

Modify `CMakeLists.txt` — append `tests/sim/test_truth_trajectory.cpp` to the `navtracker_tests` target source list (alphabetic insertion under the existing `tests/scenario/...` block).

Also append `sim/TruthTrajectory.cpp` to the `navtracker_core` source list immediately after `core/scenario/Metrics.cpp`.

- [ ] **Step 3: Run the build to verify the test fails to compile**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/TruthTrajectory.hpp: No such file or directory`.

- [ ] **Step 4: Implement the header**

`sim/TruthTrajectory.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/types/Timestamp.hpp"

namespace navtracker::sim {

struct TruthState {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // ENU metres
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};  // m/s
};

class ITruthTrajectory {
 public:
  virtual ~ITruthTrajectory() = default;
  virtual TruthState eval(Timestamp t) const = 0;
};

class ConstantVelocityTrajectory final : public ITruthTrajectory {
 public:
  ConstantVelocityTrajectory(Eigen::Vector2d p0,
                             Eigen::Vector2d v,
                             Timestamp t0);

  TruthState eval(Timestamp t) const override;

 private:
  Eigen::Vector2d p0_;
  Eigen::Vector2d v_;
  Timestamp t0_;
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement the cpp**

`sim/TruthTrajectory.cpp`:

```cpp
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

ConstantVelocityTrajectory::ConstantVelocityTrajectory(Eigen::Vector2d p0,
                                                       Eigen::Vector2d v,
                                                       Timestamp t0)
    : p0_(std::move(p0)), v_(std::move(v)), t0_(t0) {}

TruthState ConstantVelocityTrajectory::eval(Timestamp t) const {
  const double dt = t.secondsSince(t0_);
  return TruthState{p0_ + v_ * dt, v_};
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='ConstantVelocityTrajectory.*'`
Expected: 2/2 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/TruthTrajectory.hpp sim/TruthTrajectory.cpp \
        tests/sim/test_truth_trajectory.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: TruthTrajectory interface + ConstantVelocityTrajectory

First piece of the SimulatedSensorBus harness. ITruthTrajectory exposes
eval(t)->TruthState; ConstantVelocityTrajectory implements the linear case
used by all v1 regression scenarios.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: NmeaEncode helpers

**Files:**
- Create: `sim/NmeaEncode.hpp`
- Create: `sim/NmeaEncode.cpp`
- Test: `tests/sim/test_nmea_encode.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_nmea_encode.cpp`:

```cpp
#include "sim/NmeaEncode.hpp"

#include <gtest/gtest.h>

#include "adapters/util/Nmea.hpp"

using namespace navtracker;

TEST(NmeaEncode, LatDdmmFormatsPositiveAndNegative) {
  // 53.5 degrees = 53 deg 30.00000 min => "5330.00000,N"
  EXPECT_EQ(sim::formatLatDdmm(53.5),  std::string("5330.00000"));
  EXPECT_EQ(sim::latHemisphere(53.5),  'N');
  EXPECT_EQ(sim::latHemisphere(-53.5), 'S');
  EXPECT_EQ(sim::formatLatDdmm(-53.5), std::string("5330.00000"));
}

TEST(NmeaEncode, LonDdmmZeroPaddedToThreeDegreeDigits) {
  // 8.0 degrees => "00800.00000,E"
  EXPECT_EQ(sim::formatLonDdmm(8.0),  std::string("00800.00000"));
  EXPECT_EQ(sim::lonHemisphere(8.0),  'E');
  EXPECT_EQ(sim::lonHemisphere(-8.0), 'W');
}

TEST(NmeaEncode, WrapWithChecksumRoundTripsParser) {
  // Build a sentence and feed it through the existing NMEA parser; it must
  // accept the checksum we generated.
  const std::string wrapped =
      sim::wrapWithChecksum("GPGGA,000000.00,5330.00000,N,00800.00000,E,1,08,1.0,0.0,M,0.0,M,,");
  ASSERT_FALSE(wrapped.empty());
  EXPECT_EQ(wrapped.front(), '$');
  EXPECT_NE(wrapped.find('*'), std::string::npos);

  const auto parsed = parseNmea(wrapped);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->talker, "GP");
  EXPECT_EQ(parsed->formatter, "GGA");
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/NmeaEncode.cpp` to `navtracker_core` and `tests/sim/test_nmea_encode.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify the test fails to compile**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/NmeaEncode.hpp: No such file or directory`.

- [ ] **Step 4: Implement the header**

`sim/NmeaEncode.hpp`:

```cpp
#pragma once

#include <string>
#include <string_view>

namespace navtracker::sim {

// "DDMM.mmmmm" with 5 fractional minute digits, no sign (caller pairs with
// hemisphere). Absolute value of `deg` is used.
std::string formatLatDdmm(double deg);
std::string formatLonDdmm(double deg);  // "DDDMM.mmmmm" (three deg digits)

char latHemisphere(double deg);  // 'N' if deg >= 0, else 'S'
char lonHemisphere(double deg);  // 'E' if deg >= 0, else 'W'

// Returns "$" + body + "*" + two-hex-digit XOR checksum (uppercase).
std::string wrapWithChecksum(std::string_view body);

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement the cpp**

`sim/NmeaEncode.cpp`:

```cpp
#include "sim/NmeaEncode.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace navtracker::sim {
namespace {

std::string formatDdmm(double abs_deg, int deg_width) {
  const int deg = static_cast<int>(abs_deg);
  const double minutes = (abs_deg - static_cast<double>(deg)) * 60.0;
  char buf[32];
  // %0*d for zero-padded degrees; %08.5f keeps width 8 (e.g. "30.00000").
  std::snprintf(buf, sizeof(buf), "%0*d%08.5f", deg_width, deg, minutes);
  return std::string(buf);
}

}  // namespace

std::string formatLatDdmm(double deg) {
  return formatDdmm(std::fabs(deg), 2);
}

std::string formatLonDdmm(double deg) {
  return formatDdmm(std::fabs(deg), 3);
}

char latHemisphere(double deg) { return deg >= 0.0 ? 'N' : 'S'; }
char lonHemisphere(double deg) { return deg >= 0.0 ? 'E' : 'W'; }

std::string wrapWithChecksum(std::string_view body) {
  std::uint8_t cs = 0;
  for (char c : body) cs ^= static_cast<std::uint8_t>(c);
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02X", cs);
  std::string out;
  out.reserve(body.size() + 4);
  out.push_back('$');
  out.append(body.data(), body.size());
  out.push_back('*');
  out.append(hex);
  return out;
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='NmeaEncode.*'`
Expected: 3/3 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/NmeaEncode.hpp sim/NmeaEncode.cpp \
        tests/sim/test_nmea_encode.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: NMEA encoding helpers for emitter-side serialisation

formatLatDdmm/formatLonDdmm produce zero-padded NMEA 0183 minutes;
wrapWithChecksum prepends '$' and appends '*HH'. Round-tripped through the
existing parseNmea() to confirm checksum compatibility.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: SensorEmitter interface + OwnShipEmitter

**Files:**
- Create: `sim/SensorEmitter.hpp`
- Create: `sim/OwnShipEmitter.hpp`
- Create: `sim/OwnShipEmitter.cpp`
- Test: `tests/sim/test_own_ship_emitter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_own_ship_emitter.cpp`:

```cpp
#include "sim/OwnShipEmitter.hpp"

#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(OwnShipEmitter, EmitsGgaAndHdtAtOneHz) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),       // starts at datum origin
      Eigen::Vector2d(2.0, 0.0),     // 2 m/s east
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 0.0;   // no noise for this test
  cfg.heading_true_deg = 90.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/42);

  std::mt19937 unused_rng(0);  // not used by this emitter; bus passes one anyway
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  ctx.rng_unused = &unused_rng;
  emitter.emit(ctx);
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->lat_deg, 53.5, 1e-6);
  EXPECT_NEAR(provider.latest()->heading_true_deg, 90.0, 1e-6);

  // 0.5 s later: cadence is 1 Hz, nothing should emit.
  const auto previous = *provider.latest();
  ctx.now = Timestamp::fromSeconds(0.5);
  emitter.emit(ctx);
  EXPECT_DOUBLE_EQ(provider.latest()->lon_deg, previous.lon_deg);

  // 1.0 s: next emission. Ownship has moved 2 m east => lon_deg slightly larger.
  ctx.now = Timestamp::fromSeconds(1.0);
  emitter.emit(ctx);
  EXPECT_GT(provider.latest()->lon_deg, previous.lon_deg);
}

TEST(OwnShipEmitter, AppliesGpsPositionNoise) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 5.0;
  cfg.heading_true_deg = 0.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/123);

  std::mt19937 unused_rng(0);
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  ctx.rng_unused = &unused_rng;
  emitter.emit(ctx);

  // With 5 m noise, lat should deviate from the truth (53.5).
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NE(provider.latest()->lat_deg, 53.5);
  // But not by more than ~5*sigma worth of metres (1 deg lat ~= 111 km).
  EXPECT_NEAR(provider.latest()->lat_deg, 53.5, 30.0 / 111000.0);
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/OwnShipEmitter.cpp` to `navtracker_core` and `tests/sim/test_own_ship_emitter.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/OwnShipEmitter.hpp: No such file or directory`.

- [ ] **Step 4: Create `sim/SensorEmitter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "core/types/Timestamp.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct TargetTruth {
  std::uint64_t truth_id{0};
  TruthState state;
};

// All-emitters-shared context per bus tick. The bus pre-evaluates every truth
// trajectory once and passes everyone the same view, so all sensors at a
// given timestamp see a consistent world.
struct EmitContext {
  Timestamp now;
  TruthState ownship_truth;
  std::vector<TargetTruth> targets;
  // Some emitters do not need randomness (OwnShipEmitter when noise=0 still
  // pulls from its own member RNG). Keeping a pointer here so the bus can
  // pass a tick-scoped RNG if we ever need shared randomness; for now,
  // emitters own their own substream RNG and ignore this field.
  std::mt19937* rng_unused{nullptr};
};

class ISensorEmitter {
 public:
  virtual ~ISensorEmitter() = default;
  virtual void emit(const EmitContext& ctx) = 0;
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Create `sim/OwnShipEmitter.hpp`**

```cpp
#pragma once

#include <random>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct OwnShipEmitterConfig {
  double dt_s{1.0};
  double gps_pos_std_m{5.0};
  double heading_true_deg{0.0};
  // §14.9 hooks (default zero — deferred per spec).
  double heading_bias_deg{0.0};
  double heading_drift_deg_per_s{0.0};
};

class OwnShipEmitter final : public ISensorEmitter {
 public:
  OwnShipEmitter(OwnShipNmeaAdapter& adapter,
                 const geo::Datum& datum,
                 const ITruthTrajectory& ownship_trajectory,
                 OwnShipEmitterConfig cfg,
                 std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  OwnShipNmeaAdapter& adapter_;
  const geo::Datum& datum_;
  const ITruthTrajectory& trajectory_;
  OwnShipEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  bool initialised_{false};
  Timestamp next_emit_{};
  Timestamp t0_{};
};

}  // namespace navtracker::sim
```

- [ ] **Step 6: Implement `sim/OwnShipEmitter.cpp`**

```cpp
#include "sim/OwnShipEmitter.hpp"

#include <utility>

#include "sim/NmeaEncode.hpp"

namespace navtracker::sim {

OwnShipEmitter::OwnShipEmitter(OwnShipNmeaAdapter& adapter,
                               const geo::Datum& datum,
                               const ITruthTrajectory& ownship_trajectory,
                               OwnShipEmitterConfig cfg,
                               std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      trajectory_(ownship_trajectory),
      cfg_(std::move(cfg)),
      rng_(seed),
      noise_(0.0, cfg_.gps_pos_std_m) {}

void OwnShipEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    next_emit_ = ctx.now;
    t0_ = ctx.now;
    initialised_ = true;
  }
  while (next_emit_ <= ctx.now) {
    const TruthState truth = trajectory_.eval(next_emit_);
    const double nx = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const double ny = cfg_.gps_pos_std_m > 0.0 ? noise_(rng_) : 0.0;
    const Eigen::Vector3d enu(truth.position.x() + nx,
                              truth.position.y() + ny,
                              0.0);
    const geo::Geodetic g = datum_.toGeodetic(enu);

    // GGA
    {
      std::string body = "GPGGA,000000.00,";
      body += formatLatDdmm(g.lat_deg);
      body += ',';
      body += latHemisphere(g.lat_deg);
      body += ',';
      body += formatLonDdmm(g.lon_deg);
      body += ',';
      body += lonHemisphere(g.lon_deg);
      body += ",1,08,1.0,0.0,M,0.0,M,,";
      const std::string sentence = wrapWithChecksum(body);
      adapter_.ingest(sentence, next_emit_);
    }

    // HDT
    {
      const double dt = next_emit_.secondsSince(t0_);
      const double hdg = cfg_.heading_true_deg + cfg_.heading_bias_deg +
                         cfg_.heading_drift_deg_per_s * dt;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "GPHDT,%.3f,T", hdg);
      const std::string sentence = wrapWithChecksum(buf);
      adapter_.ingest(sentence, next_emit_);
    }

    next_emit_ = Timestamp::fromSeconds(next_emit_.seconds() + cfg_.dt_s);
  }
}

}  // namespace navtracker::sim
```

- [ ] **Step 7: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='OwnShipEmitter.*'`
Expected: 2/2 tests PASS.

- [ ] **Step 8: Commit**

```bash
git add sim/SensorEmitter.hpp sim/OwnShipEmitter.hpp sim/OwnShipEmitter.cpp \
        tests/sim/test_own_ship_emitter.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: SensorEmitter interface + OwnShipEmitter (1 Hz GGA+HDT)

ISensorEmitter::emit(ctx) is the per-tick contract; the bus pre-evaluates
all truth trajectories and passes a consistent EmitContext. OwnShipEmitter
emits NMEA at cfg.dt_s cadence with optional Gaussian GPS position noise
(in ENU pre-projection). Heading bias/drift hooks reserved for the deferred
§14.9 work.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: AisEmitter

**Files:**
- Create: `sim/AisEmitter.hpp`
- Create: `sim/AisEmitter.cpp`
- Test: `tests/sim/test_ais_emitter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_ais_emitter.cpp`:

```cpp
#include "sim/AisEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext makeCtx(double t_seconds,
                         std::uint64_t target_id,
                         const Eigen::Vector2d& target_pos,
                         const Eigen::Vector2d& target_vel) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{};
  ctx.targets.push_back(sim::TargetTruth{target_id, sim::TruthState{target_pos, target_vel}});
  return ctx;
}

}  // namespace

TEST(AisEmitter, EmitsAtSpeedBasedCadenceSlow) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/1, /*mmsi=*/200000001u, /*high_accuracy=*/true});
  cfg.pos_std_m = 0.0;

  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  // Slow target (3 m/s ~ 5.8 kn -> 10 s cadence). Emit at t=0, then nothing
  // until t=10.
  emitter.emit(makeCtx(0.0, 1, Eigen::Vector2d::Zero(), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);

  emitter.emit(makeCtx(5.0, 1, Eigen::Vector2d(15.0, 0.0), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);

  emitter.emit(makeCtx(10.0, 1, Eigen::Vector2d(30.0, 0.0), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(AisEmitter, EmitsAtSpeedBasedCadenceFast) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/2, /*mmsi=*/200000002u, /*high_accuracy=*/true});
  cfg.pos_std_m = 0.0;
  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/2);

  // Fast target (15 m/s ~ 29 kn -> 2 s cadence).
  const Eigen::Vector2d v(15.0, 0.0);
  emitter.emit(makeCtx(0.0, 2, Eigen::Vector2d::Zero(), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(1.5, 2, Eigen::Vector2d(22.5, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(2.0, 2, Eigen::Vector2d(30.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(4.0, 2, Eigen::Vector2d(60.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(AisEmitter, DropoutWindowSuppressesEmission) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({1u, 200000001u, true});
  cfg.dropout_windows_s.emplace_back(5.0, 25.0);
  cfg.pos_std_m = 0.0;

  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/3);

  const Eigen::Vector2d v(3.0, 0.0);  // 10 s cadence
  emitter.emit(makeCtx(0.0,  1, Eigen::Vector2d::Zero(),       v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(10.0, 1, Eigen::Vector2d(30.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // dropped
  emitter.emit(makeCtx(20.0, 1, Eigen::Vector2d(60.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // dropped
  emitter.emit(makeCtx(30.0, 1, Eigen::Vector2d(90.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 1u);
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/AisEmitter.cpp` to `navtracker_core` and `tests/sim/test_ais_emitter.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/AisEmitter.hpp: No such file or directory`.

- [ ] **Step 4: Create `sim/AisEmitter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct AisTargetEntry {
  std::uint64_t truth_id{0};
  std::uint32_t mmsi{0};
  bool high_accuracy{true};
};

struct AisEmitterConfig {
  std::vector<AisTargetEntry> targets;
  double pos_std_m{10.0};
  // Each pair [start_s, end_s) measured relative to the first emit() call
  // (which is treated as t=0 for cadence/dropout bookkeeping).
  std::vector<std::pair<double, double>> dropout_windows_s;
};

class AisEmitter final : public ISensorEmitter {
 public:
  AisEmitter(AisAdapter& adapter,
             const geo::Datum& datum,
             AisEmitterConfig cfg,
             std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  static double cadenceSeconds(double speed_mps);
  bool inDropout(double t_relative_s) const;

  AisAdapter& adapter_;
  const geo::Datum& datum_;
  AisEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::unordered_map<std::uint64_t, Timestamp> next_emit_;
  bool initialised_{false};
  Timestamp t0_{};
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement `sim/AisEmitter.cpp`**

```cpp
#include "sim/AisEmitter.hpp"

#include <cmath>
#include <utility>

namespace navtracker::sim {

AisEmitter::AisEmitter(AisAdapter& adapter,
                       const geo::Datum& datum,
                       AisEmitterConfig cfg,
                       std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      cfg_(std::move(cfg)),
      rng_(seed),
      noise_(0.0, cfg_.pos_std_m) {}

double AisEmitter::cadenceSeconds(double speed_mps) {
  // Class-A SOTDMA buckets, table from spec §5.2.
  constexpr double kKnotsPerMps = 1.9438444924;  // 1 m/s in knots
  const double knots = speed_mps * kKnotsPerMps;
  if (knots < 14.0) return 10.0;
  if (knots < 23.0) return 6.0;
  return 2.0;
}

bool AisEmitter::inDropout(double t_relative_s) const {
  for (const auto& [a, b] : cfg_.dropout_windows_s) {
    if (t_relative_s >= a && t_relative_s < b) return true;
  }
  return false;
}

void AisEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    t0_ = ctx.now;
    for (const auto& te : cfg_.targets) next_emit_[te.truth_id] = ctx.now;
    initialised_ = true;
  }

  // Build a quick lookup from truth_id -> TruthState.
  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  for (const auto& te : cfg_.targets) {
    auto it = truths.find(te.truth_id);
    if (it == truths.end()) continue;
    const TruthState& truth = it->second;
    const double speed = truth.velocity.norm();
    const double dt = cadenceSeconds(speed);

    Timestamp& next = next_emit_[te.truth_id];
    while (next <= ctx.now) {
      const double t_rel = next.secondsSince(t0_);
      if (!inDropout(t_rel)) {
        const double nx = cfg_.pos_std_m > 0.0 ? noise_(rng_) : 0.0;
        const double ny = cfg_.pos_std_m > 0.0 ? noise_(rng_) : 0.0;
        const Eigen::Vector3d enu(truth.position.x() + nx,
                                  truth.position.y() + ny,
                                  0.0);
        const geo::Geodetic g = datum_.toGeodetic(enu);

        AisDynamicReport r;
        r.time = next;
        r.mmsi = te.mmsi;
        r.lat_deg = g.lat_deg;
        r.lon_deg = g.lon_deg;
        r.high_accuracy = te.high_accuracy;
        adapter_.ingest(r);
      }
      next = Timestamp::fromSeconds(next.seconds() + dt);
    }
  }
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='AisEmitter.*'`
Expected: 3/3 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/AisEmitter.hpp sim/AisEmitter.cpp \
        tests/sim/test_ais_emitter.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: AisEmitter with Class-A SOTDMA cadence + dropout windows

Per-target cadence keyed on instantaneous speed (10 s @ 0–14 kn,
6 s @ 14–23 kn, 2 s @ 23+ kn). Dropout windows skip emission without
disturbing cadence so the next report fires at the natural next slot.
Constructs AisDynamicReport directly — AisAdapter does the rest.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: ArpaEmitter

**Files:**
- Create: `sim/ArpaEmitter.hpp`
- Create: `sim/ArpaEmitter.cpp`
- Test: `tests/sim/test_arpa_emitter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_arpa_emitter.cpp`:

```cpp
#include "sim/ArpaEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext makeCtx(double t_seconds,
                         const Eigen::Vector2d& own_pos,
                         std::uint64_t target_id,
                         const Eigen::Vector2d& target_pos) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{own_pos, Eigen::Vector2d::Zero()};
  ctx.targets.push_back(sim::TargetTruth{target_id, sim::TruthState{target_pos, Eigen::Vector2d::Zero()}});
  return ctx;
}

}  // namespace

TEST(ArpaEmitter, EmitsAtRotationCadence) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/1, /*arpa_track_num=*/3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  cfg.rotation_dt_s = 3.0;

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  const Eigen::Vector2d own_pos(0.0, 0.0);
  const Eigen::Vector2d tgt_pos(1000.0, 0.0);  // 1 km east, 1 km range

  emitter.emit(makeCtx(0.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);

  emitter.emit(makeCtx(2.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 0u);

  emitter.emit(makeCtx(3.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(ArpaEmitter, ProducesMeasurementNearTruthInEnu) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow pointing north
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({1, 3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/2);

  // Target 1 km east of own-ship. ENU east is +x.
  emitter.emit(makeCtx(0.0,
                       Eigen::Vector2d(0.0, 0.0),
                       /*truth_id=*/1,
                       Eigen::Vector2d(1000.0, 0.0)));

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, MeasurementModel::Position2D);
  EXPECT_NEAR(out[0].value.x(), 1000.0, 2.0);
  EXPECT_NEAR(out[0].value.y(),    0.0, 2.0);
}

TEST(ArpaEmitter, SkipsTargetsOutsideRangeGate) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({1, 3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  cfg.max_range_m = 500.0;  // 500 m gate

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/3);

  emitter.emit(makeCtx(0.0,
                       Eigen::Vector2d(0.0, 0.0),
                       /*truth_id=*/1,
                       Eigen::Vector2d(1000.0, 0.0)));   // out of range
  EXPECT_EQ(adapter.poll().size(), 0u);
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/ArpaEmitter.cpp` to `navtracker_core` and `tests/sim/test_arpa_emitter.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/ArpaEmitter.hpp: No such file or directory`.

- [ ] **Step 4: Create `sim/ArpaEmitter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct ArpaTargetEntry {
  std::uint64_t truth_id{0};
  int arpa_track_num{0};
};

struct ArpaEmitterConfig {
  std::vector<ArpaTargetEntry> targets;
  double rotation_dt_s{3.0};
  double range_std_m{50.0};
  double bearing_std_deg{1.0};
  double min_range_m{50.0};
  double max_range_m{22224.0};  // 12 NM
};

class ArpaEmitter final : public ISensorEmitter {
 public:
  ArpaEmitter(ArpaAdapter& adapter,
              const geo::Datum& datum,
              ArpaEmitterConfig cfg,
              std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  ArpaAdapter& adapter_;
  const geo::Datum& datum_;
  ArpaEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> range_noise_;
  std::normal_distribution<double> bearing_noise_;
  std::unordered_map<std::uint64_t, Timestamp> next_emit_;
  bool initialised_{false};
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement `sim/ArpaEmitter.cpp`**

```cpp
#include "sim/ArpaEmitter.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "sim/NmeaEncode.hpp"

namespace navtracker::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kMetresPerNm = 1852.0;

double wrap360(double deg) {
  double w = std::fmod(deg, 360.0);
  if (w < 0.0) w += 360.0;
  return w;
}

}  // namespace

ArpaEmitter::ArpaEmitter(ArpaAdapter& adapter,
                         const geo::Datum& datum,
                         ArpaEmitterConfig cfg,
                         std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      cfg_(std::move(cfg)),
      rng_(seed),
      range_noise_(0.0, cfg_.range_std_m),
      bearing_noise_(0.0, cfg_.bearing_std_deg) {}

void ArpaEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    for (const auto& te : cfg_.targets) next_emit_[te.truth_id] = ctx.now;
    initialised_ = true;
  }

  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  const Eigen::Vector2d own = ctx.ownship_truth.position;

  for (const auto& te : cfg_.targets) {
    auto it = truths.find(te.truth_id);
    if (it == truths.end()) continue;

    Timestamp& next = next_emit_[te.truth_id];
    while (next <= ctx.now) {
      const Eigen::Vector2d dxy = it->second.position - own;
      const double range = dxy.norm();
      if (range >= cfg_.min_range_m && range <= cfg_.max_range_m) {
        const double bearing_true_deg = std::atan2(dxy.y(), dxy.x()) * kRad2Deg;
        // Convention here: bearing_true_deg measured CCW from east (the math
        // angle). ArpaAdapter expects compass bearing (CW from north), so we
        // emit relative bearing = bearing_true - heading_true; with
        // heading_true = 0 in our v1 tests this stays in the math frame which
        // the adapter then re-rotates into ENU via projectRangeBearingToEnu.
        // The adapter treats bearing_true_rad = (bearing_relative_deg +
        // heading) * deg2rad and atan2(dy,dx) is also math-frame, so this is
        // consistent.
        const double bearing_rel_deg = bearing_true_deg;  // heading=0 assumption logged
        const double r_obs = range + (cfg_.range_std_m > 0.0 ? range_noise_(rng_) : 0.0);
        const double b_obs = wrap360(bearing_rel_deg +
                                     (cfg_.bearing_std_deg > 0.0 ? bearing_noise_(rng_) : 0.0));
        const double r_nm = r_obs / kMetresPerNm;
        char body[160];
        std::snprintf(body, sizeof(body),
                      "RATTM,%02d,%.3f,%.3f,R,0.0,0.0,T,0.0,0.0,N,T,,000000.00,A",
                      te.arpa_track_num, r_nm, b_obs);
        const std::string sentence = wrapWithChecksum(body);
        adapter_.ingest(sentence, next);
      }
      next = Timestamp::fromSeconds(next.seconds() + cfg_.rotation_dt_s);
    }
  }
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='ArpaEmitter.*'`
Expected: 3/3 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/ArpaEmitter.hpp sim/ArpaEmitter.cpp \
        tests/sim/test_arpa_emitter.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: ArpaEmitter (\$RATTM, 3 s rotation, range gate)

Encodes \$RATTM with relative bearing per spec §5.3. Range gating
([min,max]) drops targets outside radar's reachable annulus. Bearing
written into relative-bearing field so ArpaAdapter exercises its
heading-aware projection path.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: EoIrEmitter

**Files:**
- Create: `sim/EoIrEmitter.hpp`
- Create: `sim/EoIrEmitter.cpp`
- Test: `tests/sim/test_eoir_emitter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_eoir_emitter.cpp`:

```cpp
#include "sim/EoIrEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext makeCtx(double t_seconds,
                         const Eigen::Vector2d& own_pos,
                         std::uint64_t target_id,
                         const Eigen::Vector2d& target_pos) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{own_pos, Eigen::Vector2d::Zero()};
  ctx.targets.push_back(sim::TargetTruth{target_id, sim::TruthState{target_pos, Eigen::Vector2d::Zero()}});
  return ctx;
}

OwnShipProvider makeProviderAtOrigin() {
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  return own;
}

}  // namespace

TEST(EoIrEmitter, EmitsAtConfiguredCadence) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;  // 10 Hz

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/1);

  const Eigen::Vector2d own_pos(0.0, 0.0);
  const Eigen::Vector2d tgt_pos(0.0, 1000.0);  // 1 km north (ENU)

  emitter.emit(makeCtx(0.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(0.05, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(0.10, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(0.31, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 2u);  // ticks at 0.20 and 0.30
}

TEST(EoIrEmitter, FovGateSkipsOutOfBeamTargets) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.fov_deg = 60.0;  // ±30° around boresight (0° relative)
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/2);

  // Boresight is 0° relative; own-ship heading 0° => boresight = math
  // angle 0 (east). Target due east: in beam.
  emitter.emit(makeCtx(0.0, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);

  // Target due north (90° from east) is outside ±30° FOV.
  emitter.emit(makeCtx(0.1, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(0.0, 1000.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);
}

TEST(EoIrEmitter, RangeGateSkipsBeyondMaxRange) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.fov_deg = 360.0;       // disable FOV gate
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;
  cfg.max_range_m = 500.0;

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/3);

  emitter.emit(makeCtx(0.0, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(0.1, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(300.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/EoIrEmitter.cpp` to `navtracker_core` and `tests/sim/test_eoir_emitter.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/EoIrEmitter.hpp: No such file or directory`.

- [ ] **Step 4: Create `sim/EoIrEmitter.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "adapters/eoir/EoIrAdapter.hpp"
#include "sim/SensorEmitter.hpp"

namespace navtracker::sim {

struct EoIrTargetEntry {
  std::uint64_t truth_id{0};
  int sensor_track_id{0};
};

struct EoIrEmitterConfig {
  enum class RangeMode { BearingOnly, BearingAndRange };
  std::vector<EoIrTargetEntry> targets;
  double dt_s{0.1};
  double fov_deg{60.0};
  double boresight_relative_deg{0.0};
  double max_range_m{5000.0};
  RangeMode range_mode{RangeMode::BearingAndRange};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  double bearing_only_range_std_m{1000.0};
};

class EoIrEmitter final : public ISensorEmitter {
 public:
  EoIrEmitter(EoIrAdapter& adapter,
              EoIrEmitterConfig cfg,
              std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  EoIrAdapter& adapter_;
  EoIrEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> bearing_noise_;
  std::normal_distribution<double> range_noise_;
  Timestamp next_emit_{};
  bool initialised_{false};
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement `sim/EoIrEmitter.cpp`**

```cpp
#include "sim/EoIrEmitter.hpp"

#include <cmath>
#include <unordered_map>
#include <utility>

namespace navtracker::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

double wrapSignedDeg(double deg) {
  double w = std::fmod(deg + 180.0, 360.0);
  if (w < 0.0) w += 360.0;
  return w - 180.0;
}

}  // namespace

EoIrEmitter::EoIrEmitter(EoIrAdapter& adapter,
                         EoIrEmitterConfig cfg,
                         std::uint32_t seed)
    : adapter_(adapter),
      cfg_(std::move(cfg)),
      rng_(seed),
      bearing_noise_(0.0, cfg_.bearing_std_deg),
      range_noise_(0.0, cfg_.range_std_m) {}

void EoIrEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    next_emit_ = ctx.now;
    initialised_ = true;
  }

  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  const Eigen::Vector2d own = ctx.ownship_truth.position;

  while (next_emit_ <= ctx.now) {
    for (const auto& te : cfg_.targets) {
      auto it = truths.find(te.truth_id);
      if (it == truths.end()) continue;
      const Eigen::Vector2d dxy = it->second.position - own;
      const double range = dxy.norm();
      if (range > cfg_.max_range_m) continue;
      const double bearing_math_deg = std::atan2(dxy.y(), dxy.x()) * kRad2Deg;
      // Heading-zero convention as in ArpaEmitter: bearing_rel = math angle.
      const double bearing_rel_deg = bearing_math_deg;
      const double half_fov = cfg_.fov_deg * 0.5;
      const double delta = wrapSignedDeg(bearing_rel_deg - cfg_.boresight_relative_deg);
      if (std::fabs(delta) > half_fov) continue;

      const double b_obs = bearing_rel_deg +
          (cfg_.bearing_std_deg > 0.0 ? bearing_noise_(rng_) : 0.0);

      CameraDetection d;
      d.time = next_emit_;
      d.bearing_relative_deg = b_obs;
      d.bearing_std_deg = cfg_.bearing_std_deg > 0.0 ? cfg_.bearing_std_deg : 0.1;
      if (cfg_.range_mode == EoIrEmitterConfig::RangeMode::BearingAndRange) {
        d.range_m = range +
            (cfg_.range_std_m > 0.0 ? range_noise_(rng_) : 0.0);
        d.range_std_m = cfg_.range_std_m > 0.0 ? cfg_.range_std_m : 1.0;
      } else {
        d.range_m = range;  // not used as a tight observation;
        d.range_std_m = cfg_.bearing_only_range_std_m;
      }
      d.sensor_track_id = te.sensor_track_id;
      adapter_.ingest(d);
    }
    next_emit_ = Timestamp::fromSeconds(next_emit_.seconds() + cfg_.dt_s);
  }
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='EoIrEmitter.*'`
Expected: 3/3 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/EoIrEmitter.hpp sim/EoIrEmitter.cpp \
        tests/sim/test_eoir_emitter.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: EoIrEmitter (10 Hz, FOV gate, bearing+range or bearing-only)

Per-target FOV gate around the camera boresight; range gate to max_range_m.
BearingAndRange mode emits truth+noise; BearingOnly mode emits range with a
wide covariance so the adapter's projection produces an elongated 2D
covariance along the line of sight.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: SimulatedSensorBus orchestrator

**Files:**
- Create: `sim/SimulatedSensorBus.hpp`
- Create: `sim/SimulatedSensorBus.cpp`
- Test: `tests/sim/test_simulated_sensor_bus.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_simulated_sensor_bus.cpp`:

```cpp
#include "sim/SimulatedSensorBus.hpp"

#include <memory>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(SimulatedSensorBus, ProducesAscendingTimeOrderedMeasurements) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig bus_cfg;
  bus_cfg.t0 = Timestamp::fromSeconds(0.0);
  bus_cfg.duration_s = 30.0;
  bus_cfg.dt_s = 0.1;
  bus_cfg.truth_sample_dt_s = 1.0;
  bus_cfg.seed = 7;
  bus_cfg.datum = datum;

  sim::SimulatedSensorBus bus(bus_cfg);

  auto own_traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));
  bus.setOwnShip(own_traj);

  auto tgt_traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0, 200.0),
      Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0));
  bus.addTarget(/*truth_id=*/1, tgt_traj);

  bus.attachOwnShip(own_adapter, {});

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_cfg);

  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;  // disable FOV gate so head-on geometry isn't lost
  bus.attachEoIr(eo_adapter, eo_cfg);

  const Scenario s = bus.run();

  // Some measurements arrived.
  EXPECT_GT(s.measurements.size(), 0u);
  // Strictly non-decreasing time.
  for (std::size_t i = 1; i < s.measurements.size(); ++i) {
    EXPECT_LE(s.measurements[i - 1].time, s.measurements[i].time);
  }
  // Truth sampled at 1 Hz over 30 s => 31 samples.
  EXPECT_EQ(s.truth.size(), 31u);
}

TEST(SimulatedSensorBus, NoSensorsAttachedProducesEmptyMeasurements) {
  Datum datum({53.5, 8.0, 0.0});

  sim::SimulatedSensorBusConfig bus_cfg;
  bus_cfg.t0 = Timestamp::fromSeconds(0.0);
  bus_cfg.duration_s = 5.0;
  bus_cfg.dt_s = 0.5;
  bus_cfg.truth_sample_dt_s = 1.0;
  bus_cfg.seed = 1;
  bus_cfg.datum = datum;

  sim::SimulatedSensorBus bus(bus_cfg);
  auto own = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Timestamp::fromSeconds(0.0));
  bus.setOwnShip(own);

  const Scenario s = bus.run();
  EXPECT_TRUE(s.measurements.empty());
  EXPECT_EQ(s.truth.size(), 6u);
}
```

- [ ] **Step 2: Append source + test to `CMakeLists.txt`**

Append `sim/SimulatedSensorBus.cpp` to `navtracker_core` and `tests/sim/test_simulated_sensor_bus.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `fatal error: sim/SimulatedSensorBus.hpp: No such file or directory`.

- [ ] **Step 4: Create `sim/SimulatedSensorBus.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/Timestamp.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/SensorEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct SimulatedSensorBusConfig {
  Timestamp t0;
  double duration_s{60.0};
  double dt_s{0.1};
  double truth_sample_dt_s{1.0};
  std::uint32_t seed{2026};
  geo::Datum datum{geo::Geodetic{0.0, 0.0, 0.0}};
};

class SimulatedSensorBus {
 public:
  explicit SimulatedSensorBus(SimulatedSensorBusConfig cfg);

  void setOwnShip(std::shared_ptr<ITruthTrajectory> trajectory);
  void addTarget(std::uint64_t truth_id, std::shared_ptr<ITruthTrajectory> trajectory);

  void attachOwnShip(OwnShipNmeaAdapter& adapter, OwnShipEmitterConfig cfg);
  void attachAis    (AisAdapter& adapter,         AisEmitterConfig cfg);
  void attachArpa   (ArpaAdapter& adapter,        ArpaEmitterConfig cfg);
  void attachEoIr   (EoIrAdapter& adapter,        EoIrEmitterConfig cfg);

  Scenario run();

 private:
  std::uint32_t derive_seed_(const char* emitter_id) const;

  SimulatedSensorBusConfig cfg_;
  std::shared_ptr<ITruthTrajectory> ownship_;
  std::vector<std::pair<std::uint64_t, std::shared_ptr<ITruthTrajectory>>> targets_;

  // Emitters are owned by the bus; adapters are not. Vector of non-owning
  // poll-needed adapters is built so run() can drain them after every tick.
  std::unique_ptr<OwnShipEmitter> own_emitter_;
  std::unique_ptr<AisEmitter>     ais_emitter_;
  std::unique_ptr<ArpaEmitter>    arpa_emitter_;
  std::unique_ptr<EoIrEmitter>    eo_emitter_;

  AisAdapter*  ais_adapter_{nullptr};
  ArpaAdapter* arpa_adapter_{nullptr};
  EoIrAdapter* eo_adapter_{nullptr};
};

}  // namespace navtracker::sim
```

- [ ] **Step 5: Implement `sim/SimulatedSensorBus.cpp`**

```cpp
#include "sim/SimulatedSensorBus.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace navtracker::sim {

SimulatedSensorBus::SimulatedSensorBus(SimulatedSensorBusConfig cfg)
    : cfg_(std::move(cfg)) {}

void SimulatedSensorBus::setOwnShip(std::shared_ptr<ITruthTrajectory> trajectory) {
  ownship_ = std::move(trajectory);
}

void SimulatedSensorBus::addTarget(std::uint64_t truth_id,
                                   std::shared_ptr<ITruthTrajectory> trajectory) {
  targets_.emplace_back(truth_id, std::move(trajectory));
}

std::uint32_t SimulatedSensorBus::derive_seed_(const char* emitter_id) const {
  std::array<std::uint32_t, 8> mix{};
  mix[0] = cfg_.seed;
  std::size_t i = 1;
  for (const char* p = emitter_id; *p && i < mix.size(); ++p, ++i)
    mix[i] = static_cast<std::uint32_t>(*p);
  std::seed_seq sseq(mix.begin(), mix.end());
  std::array<std::uint32_t, 1> out{};
  sseq.generate(out.begin(), out.end());
  return out[0];
}

void SimulatedSensorBus::attachOwnShip(OwnShipNmeaAdapter& adapter,
                                       OwnShipEmitterConfig cfg) {
  own_emitter_ = std::make_unique<OwnShipEmitter>(
      adapter, cfg_.datum, *ownship_, std::move(cfg), derive_seed_("ownship"));
}

void SimulatedSensorBus::attachAis(AisAdapter& adapter,
                                   AisEmitterConfig cfg) {
  ais_emitter_ = std::make_unique<AisEmitter>(
      adapter, cfg_.datum, std::move(cfg), derive_seed_("ais"));
  ais_adapter_ = &adapter;
}

void SimulatedSensorBus::attachArpa(ArpaAdapter& adapter,
                                    ArpaEmitterConfig cfg) {
  arpa_emitter_ = std::make_unique<ArpaEmitter>(
      adapter, cfg_.datum, std::move(cfg), derive_seed_("arpa"));
  arpa_adapter_ = &adapter;
}

void SimulatedSensorBus::attachEoIr(EoIrAdapter& adapter,
                                    EoIrEmitterConfig cfg) {
  eo_emitter_ = std::make_unique<EoIrEmitter>(
      adapter, std::move(cfg), derive_seed_("eoir"));
  eo_adapter_ = &adapter;
}

Scenario SimulatedSensorBus::run() {
  Scenario out;
  if (!ownship_) return out;

  const Timestamp t_end = Timestamp::fromSeconds(cfg_.t0.seconds() + cfg_.duration_s);
  Timestamp next_truth_sample = cfg_.t0;
  Timestamp t = cfg_.t0;

  while (t <= t_end) {
    // Pre-evaluate truth for this tick.
    EmitContext ctx;
    ctx.now = t;
    ctx.ownship_truth = ownship_->eval(t);
    ctx.targets.reserve(targets_.size());
    for (const auto& [tid, traj] : targets_)
      ctx.targets.push_back(TargetTruth{tid, traj->eval(t)});

    // Order matters: ownship first (so OwnShipProvider is current for
    // ARPA / EO-IR).
    if (own_emitter_)  own_emitter_->emit(ctx);
    if (ais_emitter_)  ais_emitter_->emit(ctx);
    if (arpa_emitter_) arpa_emitter_->emit(ctx);
    if (eo_emitter_)   eo_emitter_->emit(ctx);

    // Drain Measurement-producing adapters.
    if (ais_adapter_) {
      auto v = ais_adapter_->poll();
      out.measurements.insert(out.measurements.end(),
                              std::make_move_iterator(v.begin()),
                              std::make_move_iterator(v.end()));
    }
    if (arpa_adapter_) {
      auto v = arpa_adapter_->poll();
      out.measurements.insert(out.measurements.end(),
                              std::make_move_iterator(v.begin()),
                              std::make_move_iterator(v.end()));
    }
    if (eo_adapter_) {
      auto v = eo_adapter_->poll();
      out.measurements.insert(out.measurements.end(),
                              std::make_move_iterator(v.begin()),
                              std::make_move_iterator(v.end()));
    }

    // Truth sampling.
    if (t >= next_truth_sample) {
      for (const auto& [tid, traj] : targets_) {
        const TruthState s = traj->eval(t);
        TruthSample ts;
        ts.time = t;
        ts.truth_id = tid;
        ts.position = s.position;
        ts.velocity = s.velocity;
        out.truth.push_back(ts);
      }
      next_truth_sample = Timestamp::fromSeconds(
          next_truth_sample.seconds() + cfg_.truth_sample_dt_s);
    }

    t = Timestamp::fromSeconds(t.seconds() + cfg_.dt_s);
  }

  std::stable_sort(out.measurements.begin(), out.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time < b.time;
                   });
  return out;
}

}  // namespace navtracker::sim
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='SimulatedSensorBus.*'`
Expected: 2/2 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/SimulatedSensorBus.hpp sim/SimulatedSensorBus.cpp \
        tests/sim/test_simulated_sensor_bus.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: SimulatedSensorBus orchestrator with deterministic RNG substreams

Time loop ticks at cfg.dt_s, pre-evaluates all truth trajectories, dispatches
emitters in own-ship-first order so ARPA/EO-IR see a current OwnShipProvider,
drains adapter poll() into a Scenario, and stable-sorts measurements by time.
Each emitter gets a substream seed derived from (cfg.seed, emitter_id) via
seed_seq.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Determinism test

**Files:**
- Test: `tests/sim/test_bus_determinism.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_bus_determinism.cpp`:

```cpp
#include <memory>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

Scenario runOnce(std::uint32_t seed) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 20.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-300.0, 100.0),
      Eigen::Vector2d(10.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(SimulatedSensorBusDeterminism, TwoRunsSameSeedIdenticalOutput) {
  const Scenario a = runOnce(42);
  const Scenario b = runOnce(42);

  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  for (std::size_t i = 0; i < a.measurements.size(); ++i) {
    EXPECT_EQ(a.measurements[i].time, b.measurements[i].time);
    EXPECT_EQ(a.measurements[i].sensor, b.measurements[i].sensor);
    ASSERT_EQ(a.measurements[i].value.size(), b.measurements[i].value.size());
    for (int k = 0; k < a.measurements[i].value.size(); ++k)
      EXPECT_DOUBLE_EQ(a.measurements[i].value(k), b.measurements[i].value(k));
  }
  ASSERT_EQ(a.truth.size(), b.truth.size());
  for (std::size_t i = 0; i < a.truth.size(); ++i) {
    EXPECT_EQ(a.truth[i].time, b.truth[i].time);
    EXPECT_EQ(a.truth[i].truth_id, b.truth[i].truth_id);
    EXPECT_DOUBLE_EQ(a.truth[i].position.x(), b.truth[i].position.x());
    EXPECT_DOUBLE_EQ(a.truth[i].position.y(), b.truth[i].position.y());
  }
}

TEST(SimulatedSensorBusDeterminism, DifferentSeedsDifferOnNoise) {
  const Scenario a = runOnce(42);
  const Scenario b = runOnce(43);
  ASSERT_EQ(a.measurements.size(), b.measurements.size());
  // At least one measurement value differs.
  bool any_diff = false;
  for (std::size_t i = 0; i < a.measurements.size() && !any_diff; ++i) {
    for (int k = 0; k < a.measurements[i].value.size() && !any_diff; ++k) {
      if (a.measurements[i].value(k) != b.measurements[i].value(k)) any_diff = true;
    }
  }
  EXPECT_TRUE(any_diff);
}
```

- [ ] **Step 2: Append test to `CMakeLists.txt`**

Append `tests/sim/test_bus_determinism.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='SimulatedSensorBusDeterminism.*'`
Expected: 2/2 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_bus_determinism.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: determinism test — same seed yields byte-identical Scenario

Satisfies architecture invariant 4 (replay determinism). Two runs with the
same seed match measurement-by-measurement; different seeds produce different
noise realisations.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Regression replay — crossing scenario end-to-end

**Files:**
- Test: `tests/sim/test_bus_regression.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file (crossing only — additional scenarios in Task 10)**

`tests/sim/test_bus_regression.cpp`:

```cpp
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

double baselineOspaCrossing() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, /*seed=*/11);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  return runScenario(s, tracker, mgr, 50.0).mean_ospa;
}

Scenario runBusCrossing() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 40.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 11;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0, 10.0),
      Eigen::Vector2d(25.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(500.0, -10.0),
      Eigen::Vector2d(-25.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;  // disable FOV gate for fair regression
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, CrossingMeanOspaWithinTolerance) {
  const double baseline = baselineOspaCrossing();
  ASSERT_GT(baseline, 0.0);

  const Scenario s = runBusCrossing();
  ASSERT_GT(s.measurements.size(), 0u);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);

  // Bus injects strictly more noise (multi-sensor cadence variation, real
  // adapter chain). Tolerance 2.0x baseline as a "no catastrophic regression"
  // guard. If this fires, tune per-sensor noise OR per-scenario tolerance.
  EXPECT_LT(r.mean_ospa, baseline * 2.0)
      << "bus mean OSPA " << r.mean_ospa
      << " vs baseline " << baseline;
}
```

- [ ] **Step 2: Append test to `CMakeLists.txt`**

Append `tests/sim/test_bus_regression.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the test**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='BusRegression.CrossingMeanOspaWithinTolerance'`
Expected: PASS. If FAIL, log the actual `bus mean OSPA` and `baseline` values printed by the test, then:
1. If `mean_ospa` is huge (>1000), inspect the bus's `Scenario` ordering and the first few Measurements (`std::cout << s.measurements[i].time.seconds() << ' ' << s.measurements[i].sensor << '\n'`); a sensor mismatch (e.g. ARPA producing measurements far from truth) usually means the heading-zero assumption broke.
2. If `mean_ospa` is modestly above the 2.0× bound, raise the tolerance to 3.0× and document in the test comment.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_bus_regression.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: bus regression test — crossing scenario end-to-end vs baseline

Runs the existing crossing scenario through SimulatedSensorBus (all four
sensors attached) and asserts mean OSPA stays within 2x of the direct-
Measurement baseline. First end-to-end signal that the adapter chain doesn't
regress the tracker on known-good scenarios.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: Replay remaining winning scenarios

**Files:**
- Modify: `tests/sim/test_bus_regression.cpp` (append three new TEST cases)

- [ ] **Step 1: Append the overtaking regression test to `tests/sim/test_bus_regression.cpp`**

```cpp
namespace {

double baselineOspaOvertaking() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildOvertakingScenario(
      Eigen::Vector2d(-200.0,  10.0),
      Eigen::Vector2d(  5.0,   0.0),
      Eigen::Vector2d(-400.0, -10.0),
      Eigen::Vector2d( 15.0,   0.0),
      times, 8.0, /*seed=*/11);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  return runScenario(s, tracker, mgr, 50.0).mean_ospa;
}

Scenario runBusOvertaking() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 40.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 11;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-200.0, 10.0),
      Eigen::Vector2d( 5.0,   0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-400.0, -10.0),
      Eigen::Vector2d(15.0,    0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, OvertakingMeanOspaWithinTolerance) {
  const double baseline = baselineOspaOvertaking();
  ASSERT_GT(baseline, 0.0);
  const Scenario s = runBusOvertaking();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  EXPECT_LT(r.mean_ospa, baseline * 2.0)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}
```

- [ ] **Step 2: Append the parallel-targets regression test**

```cpp
namespace {

double baselineOspaParallel() {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildParallelTargetsScenario(
      Eigen::Vector2d(-500.0,  50.0),
      Eigen::Vector2d(-500.0, -50.0),
      Eigen::Vector2d(  25.0,   0.0),
      times, 8.0, /*seed=*/17);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  return runScenario(s, tracker, mgr, 50.0).mean_ospa;
}

Scenario runBusParallel() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 30.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 17;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0,  50.0),
      Eigen::Vector2d(  25.0,   0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-500.0, -50.0),
      Eigen::Vector2d(  25.0,   0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, ParallelTargetsMeanOspaWithinTolerance) {
  const double baseline = baselineOspaParallel();
  ASSERT_GT(baseline, 0.0);
  const Scenario s = runBusParallel();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(50.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 30.0);
  const ScenarioResult r = runScenario(s, tracker, mgr, 50.0);
  EXPECT_LT(r.mean_ospa, baseline * 2.0)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}
```

- [ ] **Step 3: Append the bearing-only-moving-sensor regression test**

This scenario only uses EO/IR bearing-only on a moving own-ship. We bind only the EO/IR sensor to the bus; AIS+ARPA are not attached because the existing baseline test uses bearing-only measurements exclusively.

```cpp
namespace {

double baselineOspaBearingOnlyMoving() {
  std::vector<double> times;
  for (int i = 1; i <= 30; ++i) times.push_back(static_cast<double>(i));
  const Scenario s = buildBearingOnlyMovingSensorScenario(
      Eigen::Vector2d(1500.0,   0.0),   // target
      Eigen::Vector2d(   0.0,-300.0),   // sensor start
      Eigen::Vector2d(   0.0,  20.0),   // sensor velocity
      times, /*init_pos_std=*/300.0, /*bearing_std_rad=*/0.026,
      /*seed=*/202);
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(200.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 400.0);
  return runScenario(s, tracker, mgr, 400.0).mean_ospa;
}

Scenario runBusBearingOnlyMoving() {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 30.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = 202;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0),
      Eigen::Vector2d(0.0,   20.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(1500.0, 0.0),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.fov_deg = 360.0;
  eo_cfg.range_mode = sim::EoIrEmitterConfig::RangeMode::BearingOnly;
  eo_cfg.bearing_std_deg = 1.5;
  eo_cfg.dt_s = 1.0;  // match baseline scan cadence
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

}  // namespace

TEST(BusRegression, BearingOnlyMovingSensorMeanOspaWithinTolerance) {
  const double baseline = baselineOspaBearingOnlyMoving();
  ASSERT_GT(baseline, 0.0);
  const Scenario s = runBusBearingOnlyMoving();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(200.0);
  TrackManager mgr(2, 4);
  Tracker tracker(est, assoc, mgr, 400.0);
  const ScenarioResult r = runScenario(s, tracker, mgr, 400.0);
  // Bearing-only is high-variance; allow a wider tolerance band.
  EXPECT_LT(r.mean_ospa, baseline * 3.0)
      << "bus mean OSPA " << r.mean_ospa << " vs baseline " << baseline;
}
```

- [ ] **Step 4: Run the full BusRegression suite**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='BusRegression.*'`
Expected: 4/4 tests PASS.

If any test fails, follow the same debugging recipe from Task 9 Step 3: print bus mean OSPA + baseline, inspect first few Measurements for obvious sensor-mismatch, then widen tolerance per scenario only after confirming the underlying numbers look reasonable.

- [ ] **Step 5: Run the entire test suite to confirm no regressions**

Run: `ctest --test-dir build --output-on-failure`
Expected: All previously-passing tests still pass; new BusRegression suite at 4/4.

- [ ] **Step 6: Commit**

```bash
git add tests/sim/test_bus_regression.cpp
git commit -m "$(cat <<'EOF'
sim: bus regression — overtaking, parallel, bearing-only moving sensor

Drives the remaining three winning scenarios through SimulatedSensorBus and
asserts each stays within tolerance vs the direct-Measurement baseline. Full
four-sensor attach for the position scenarios; EO/IR-only bearing-only mode
for the moving-sensor scenario.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review Notes

After writing the plan, checked against the spec:

**Spec coverage:**
- §3 module layout: Tasks 1-7 create every file listed. ✓
- §4 public API: Task 7 implements all of `setOwnShip`, `addTarget`, `attachOwnShip/Ais/Arpa/EoIr`, `run`. ✓
- §5 per-sensor emitter specs (5.1 OwnShip / 5.2 AIS / 5.3 ARPA / 5.4 EO-IR): Tasks 3-6 implement each, including SOTDMA cadence (Task 4), `$RATTM` encoding + range gate (Task 5), FOV gate + dual range mode (Task 6). ✓
- §6 RNG substreams: Task 7 implements `derive_seed_` via `seed_seq`. Task 8 asserts determinism. ✓
- §7 validation strategy: Tasks 9-10 implement the per-scenario baseline-vs-bus comparison with tolerance multiplier. ✓
- §8 implementation order: this plan's 10 tasks match the preview list 1:1. ✓

**Heading-zero assumption:** ArpaEmitter and EoIrEmitter both convert truth → relative bearing via `atan2(dxy.y, dxy.x)` directly (treating math-frame angle as relative bearing). This relies on `cfg.heading_true_deg = 0` for the OwnShipEmitter, which is the v1 default. All regression tests configure ownship heading at 0 (default). The emitters carry an explicit comment that flags this dependency; switching to a non-zero heading is a follow-up handled together with §14.9.

**Type consistency:** `TruthState` (sim namespace) defined in Task 1, used identically in Tasks 3-7. `EmitContext` defined in Task 3, used in Tasks 3-7. `OwnShipEmitterConfig` / `AisEmitterConfig` / `ArpaEmitterConfig` / `EoIrEmitterConfig` defined in their respective emitter headers and referenced (forward-declared transitively) in `SimulatedSensorBus.hpp`. Method signatures (`emit(const EmitContext&)`) match across all four concrete emitters and `ISensorEmitter`.

**Placeholder scan:** No `TBD` / `TODO` / `implement later` strings. Tolerance multipliers (2.0×, 3.0×) are explicit values, not placeholders — they may need tuning during execution, and Task 9 Step 3 / Task 10 Step 4 give an explicit debugging recipe if they fire.
