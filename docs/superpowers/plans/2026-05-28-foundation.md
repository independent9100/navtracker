# Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the navtracker build system and the I/O-free core data model — WGS-84 geodetic↔ENU conversion, a `Timestamp` type, and the `Measurement`/`Track` domain types — all unit-tested.

**Architecture:** This is the center of the hexagon (`docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md`). Pure C++17 with no I/O and no sensor-format knowledge. Linear algebra via Eigen; coordinates handled with self-implemented WGS-84 transforms behind a `Datum` class. Everything here is consumed by later plans (estimation, association, pipeline, adapters).

**Tech Stack:** C++17 · CMake (≥3.20) · Conan 2 · Eigen 3.4 · GoogleTest.

This is plan 1 of 6. Subsequent plans: 2) estimation, 3) association + track management, 4) pipeline + time buffer, 5) sensor adapters, 6) scenario harness + metrics. See roadmap at the end.

**Documentation standard (CLAUDE.md):** Plans 2+ introduce algorithms and MUST document each with Math / Assumptions / Rationale / Ways-to-improve. This foundation plan is data structures + well-defined geodesy, so the math is captured inline in the relevant tasks.

---

## File Structure

```
conanfile.txt                 Conan 2 dependencies (eigen, gtest)
CMakeLists.txt                root build: navtracker_core lib + navtracker_tests exe
core/types/Timestamp.hpp      strong int64-nanosecond timestamp (header-only)
core/types/Ids.hpp            SensorKind, TrackStatus, TrackId, MeasurementModel enums (header-only)
core/geo/Wgs84.hpp/.cpp       WGS-84 constants + geodetic<->ECEF free functions
core/geo/Datum.hpp/.cpp       ENU local tangent plane about an origin
core/types/Measurement.hpp    normalized sensor measurement (header-only)
core/types/Track.hpp          fused track entity (header-only)
tests/smoke_test.cpp          build-system smoke test
tests/types/test_timestamp.cpp
tests/geo/test_wgs84.cpp
tests/geo/test_datum.cpp
tests/types/test_measurement.cpp
tests/types/test_track.cpp
```

Header-only types (`Timestamp`, `Ids`, `Measurement`, `Track`) compile into no `.cpp`; only `Wgs84.cpp` and `Datum.cpp` are compiled into `navtracker_core`. Include root is the repo root, so includes read `#include "core/geo/Datum.hpp"`.

---

## Task 1: Project scaffolding (Conan + CMake + gtest smoke test)

**Files:**
- Create: `conanfile.txt`
- Create: `CMakeLists.txt`
- Create: `tests/smoke_test.cpp`

- [ ] **Step 1: Write the smoke test**

Create `tests/smoke_test.cpp`:

```cpp
#include <gtest/gtest.h>

TEST(Smoke, BuildSystemWorks) {
  EXPECT_EQ(2 + 2, 4);
}
```

- [ ] **Step 2: Create the Conan dependencies file**

Create `conanfile.txt`:

```ini
[requires]
eigen/3.4.0
gtest/1.14.0

[generators]
CMakeDeps
CMakeToolchain
```

- [ ] **Step 3: Create the root CMakeLists.txt**

Create `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(navtracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

enable_testing()

add_executable(navtracker_tests
  tests/smoke_test.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
```

- [ ] **Step 4: Install dependencies and configure**

Run:
```bash
conan install . -of=build -s build_type=Release -s compiler.cppstd=17 --build=missing
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
```
Expected: Conan resolves eigen + gtest; CMake configures with "Configuring done / Generating done".

(If this is the first Conan use on the machine, run `conan profile detect` once first.)

- [ ] **Step 5: Build and run the smoke test**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — `Smoke.BuildSystemWorks` passes; ctest reports `100% tests passed`.

- [ ] **Step 6: Add .gitignore and commit**

Create `.gitignore`:
```
/build/
CMakeUserPresets.json
```

Run:
```bash
git add .gitignore conanfile.txt CMakeLists.txt tests/smoke_test.cpp
git commit -m "build: scaffold CMake/Conan project with gtest smoke test"
```

---

## Task 2: Timestamp type

A strong type wrapping int64 nanoseconds. The engine is timestamp-driven (spec D2), so a precise, comparable, arithmetic-friendly time type is foundational.

**Files:**
- Create: `core/types/Timestamp.hpp`
- Test: `tests/types/test_timestamp.cpp`
- Modify: `CMakeLists.txt` (add the test source)

- [ ] **Step 1: Write the failing test**

Create `tests/types/test_timestamp.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/types/Timestamp.hpp"

using navtracker::Timestamp;

TEST(Timestamp, RoundTripsSeconds) {
  const Timestamp t = Timestamp::fromSeconds(12.5);
  EXPECT_DOUBLE_EQ(t.seconds(), 12.5);
  EXPECT_EQ(t.nanos(), 12'500'000'000);
}

TEST(Timestamp, SecondsSinceIsSignedDelta) {
  const Timestamp a = Timestamp::fromSeconds(10.0);
  const Timestamp b = Timestamp::fromSeconds(13.0);
  EXPECT_DOUBLE_EQ(b.secondsSince(a), 3.0);
  EXPECT_DOUBLE_EQ(a.secondsSince(b), -3.0);
}

TEST(Timestamp, OrdersChronologically) {
  const Timestamp a = Timestamp::fromSeconds(1.0);
  const Timestamp b = Timestamp::fromSeconds(2.0);
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(b > a);
  EXPECT_TRUE(a <= a);
  EXPECT_EQ(a, Timestamp::fromSeconds(1.0));
}
```

- [ ] **Step 2: Add the test to the build**

In `CMakeLists.txt`, change the `add_executable(navtracker_tests ...)` block to:

```cmake
add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
)
```

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake --build build 2>&1 | head -20
```
Expected: FAIL — compile error, `core/types/Timestamp.hpp` not found.

- [ ] **Step 4: Write the implementation**

Create `core/types/Timestamp.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace navtracker {

// Monotonic, source-provided time as signed nanoseconds since an epoch.
// The engine advances on these, never on wall-clock (spec D2).
class Timestamp {
 public:
  constexpr Timestamp() = default;
  constexpr explicit Timestamp(std::int64_t nanos) : nanos_(nanos) {}

  static constexpr Timestamp fromSeconds(double seconds) {
    return Timestamp(static_cast<std::int64_t>(seconds * 1e9));
  }

  constexpr std::int64_t nanos() const { return nanos_; }
  constexpr double seconds() const { return static_cast<double>(nanos_) * 1e-9; }

  // Signed difference in seconds: (*this - other).
  constexpr double secondsSince(Timestamp other) const {
    return static_cast<double>(nanos_ - other.nanos_) * 1e-9;
  }

  friend constexpr bool operator<(Timestamp a, Timestamp b) { return a.nanos_ < b.nanos_; }
  friend constexpr bool operator>(Timestamp a, Timestamp b) { return a.nanos_ > b.nanos_; }
  friend constexpr bool operator<=(Timestamp a, Timestamp b) { return a.nanos_ <= b.nanos_; }
  friend constexpr bool operator>=(Timestamp a, Timestamp b) { return a.nanos_ >= b.nanos_; }
  friend constexpr bool operator==(Timestamp a, Timestamp b) { return a.nanos_ == b.nanos_; }
  friend constexpr bool operator!=(Timestamp a, Timestamp b) { return a.nanos_ != b.nanos_; }

 private:
  std::int64_t nanos_{0};
};

}  // namespace navtracker
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Timestamp.*` tests pass.

- [ ] **Step 6: Commit**

```bash
git add core/types/Timestamp.hpp tests/types/test_timestamp.cpp CMakeLists.txt
git commit -m "feat(core): add Timestamp type"
```

---

## Task 3: WGS-84 geodetic <-> ECEF conversion

**Math.** WGS-84 ellipsoid: semi-major `a = 6378137.0 m`, flattening `f = 1/298.257223563`, `e² = f(2−f)`.
Geodetic (φ lat, λ lon, h) → ECEF: `N = a/√(1−e²sin²φ)`; `X=(N+h)cosφcosλ`, `Y=(N+h)cosφsinλ`, `Z=(N(1−e²)+h)sinφ`.
ECEF → geodetic via Bowring's closed-form (longitude exact; latitude one-pass with auxiliary angle θ).

**Files:**
- Create: `core/geo/Wgs84.hpp`
- Create: `core/geo/Wgs84.cpp`
- Test: `tests/geo/test_wgs84.cpp`
- Modify: `CMakeLists.txt` (introduce `navtracker_core` library + add test)

- [ ] **Step 1: Write the failing test**

Create `tests/geo/test_wgs84.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/geo/Wgs84.hpp"

using navtracker::geo::Geodetic;
using navtracker::geo::geodeticToEcef;
using navtracker::geo::ecefToGeodetic;

TEST(Wgs84, OriginOnEquatorPrimeMeridian) {
  const Eigen::Vector3d ecef = geodeticToEcef({0.0, 0.0, 0.0});
  EXPECT_NEAR(ecef.x(), 6378137.0, 1e-3);
  EXPECT_NEAR(ecef.y(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.z(), 0.0, 1e-3);
}

TEST(Wgs84, NorthPoleHitsSemiMinorAxis) {
  const Eigen::Vector3d ecef = geodeticToEcef({90.0, 0.0, 0.0});
  EXPECT_NEAR(ecef.x(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.y(), 0.0, 1e-3);
  EXPECT_NEAR(ecef.z(), 6356752.314245, 1e-3);  // b = a(1-f)
}

TEST(Wgs84, RoundTripGeodetic) {
  const Geodetic g{53.5, 8.2, 25.0};
  const Geodetic back = ecefToGeodetic(geodeticToEcef(g));
  EXPECT_NEAR(back.lat_deg, g.lat_deg, 1e-9);
  EXPECT_NEAR(back.lon_deg, g.lon_deg, 1e-9);
  EXPECT_NEAR(back.alt_m, g.alt_m, 1e-6);
}
```

- [ ] **Step 2: Introduce the core library and add the test in CMake**

In `CMakeLists.txt`, after `enable_testing()`, add the library:

```cmake
add_library(navtracker_core
  core/geo/Wgs84.cpp
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)
```

Then update the test executable to include the new test and link the core library:

```cmake
add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/geo/test_wgs84.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)
```

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/geo/Wgs84.hpp` not found.

- [ ] **Step 4: Write the header**

Create `core/geo/Wgs84.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

namespace navtracker::geo {

// Geodetic coordinate on the WGS-84 ellipsoid. Degrees, degrees, meters.
struct Geodetic {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

// Earth-Centered Earth-Fixed position in meters, as an Eigen vector (X, Y, Z).
Eigen::Vector3d geodeticToEcef(const Geodetic& g);

// Inverse of geodeticToEcef using Bowring's method.
Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef);

}  // namespace navtracker::geo
```

- [ ] **Step 5: Write the implementation**

Create `core/geo/Wgs84.cpp`:

```cpp
#include "core/geo/Wgs84.hpp"

#include <cmath>

namespace navtracker::geo {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0;                 // semi-major axis (m)
constexpr double kF = 1.0 / 298.257223563;       // flattening
constexpr double kE2 = kF * (2.0 - kF);          // first eccentricity squared
constexpr double kB = kA * (1.0 - kF);           // semi-minor axis (m)
constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);  // second eccentricity squared

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }

}  // namespace

Eigen::Vector3d geodeticToEcef(const Geodetic& g) {
  const double lat = deg2rad(g.lat_deg);
  const double lon = deg2rad(g.lon_deg);
  const double s = std::sin(lat);
  const double c = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * s * s);
  const double x = (n + g.alt_m) * c * std::cos(lon);
  const double y = (n + g.alt_m) * c * std::sin(lon);
  const double z = (n * (1.0 - kE2) + g.alt_m) * s;
  return {x, y, z};
}

Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef) {
  const double x = ecef.x();
  const double y = ecef.y();
  const double z = ecef.z();

  const double lon = std::atan2(y, x);
  const double p = std::hypot(x, y);
  const double theta = std::atan2(z * kA, p * kB);
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double lat = std::atan2(z + kEp2 * kB * st * st * st,
                                p - kE2 * kA * ct * ct * ct);
  const double sl = std::sin(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
  const double alt = p / std::cos(lat) - n;

  return {rad2deg(lat), rad2deg(lon), alt};
}

}  // namespace navtracker::geo
```

- [ ] **Step 6: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Wgs84.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add core/geo/Wgs84.hpp core/geo/Wgs84.cpp tests/geo/test_wgs84.cpp CMakeLists.txt
git commit -m "feat(geo): add WGS-84 geodetic<->ECEF conversion"
```

---

## Task 4: Datum (ENU local tangent plane)

**Math.** About an origin (φ0, λ0, h0) with ECEF origin `O`, the rotation from ECEF deltas to ENU is
`R = [[−sinλ0, cosλ0, 0], [−sinφ0cosλ0, −sinφ0sinλ0, cosφ0], [cosφ0cosλ0, cosφ0sinλ0, sinφ0]]`.
`enu = R (ecef(g) − O)`; inverse `ecef = Rᵀ enu + O`. This is spec decision D3 (common frame = ENU about a configurable datum).

**Files:**
- Create: `core/geo/Datum.hpp`
- Create: `core/geo/Datum.cpp`
- Test: `tests/geo/test_datum.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/geo/test_datum.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/geo/Datum.hpp"

using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

TEST(Datum, OriginMapsToZero) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.5, 8.0, 0.0});
  EXPECT_NEAR(enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(enu.y(), 0.0, 1e-6);
  EXPECT_NEAR(enu.z(), 0.0, 1e-6);
}

TEST(Datum, NorthwardPointHasPositiveNorthZeroEast) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.51, 8.0, 0.0});  // ~0.01 deg north
  EXPECT_NEAR(enu.x(), 0.0, 1e-3);          // east ~ 0
  EXPECT_GT(enu.y(), 1000.0);               // ~1.1 km north
  EXPECT_LT(enu.y(), 1200.0);
}

TEST(Datum, EastwardPointHasPositiveEast) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu = datum.toEnu({53.5, 8.01, 0.0});  // ~0.01 deg east
  EXPECT_GT(enu.x(), 500.0);
  EXPECT_NEAR(enu.y(), 0.0, 1.0);
}

TEST(Datum, RoundTripEnuGeodetic) {
  const Datum datum({53.5, 8.0, 0.0});
  const Eigen::Vector3d enu(1234.0, -5678.0, 12.0);
  const Geodetic g = datum.toGeodetic(enu);
  const Eigen::Vector3d back = datum.toEnu(g);
  EXPECT_NEAR(back.x(), enu.x(), 1e-4);
  EXPECT_NEAR(back.y(), enu.y(), 1e-4);
  EXPECT_NEAR(back.z(), enu.z(), 1e-4);
}
```

- [ ] **Step 2: Add Datum.cpp to the library and the test to the build**

In `CMakeLists.txt`, add `core/geo/Datum.cpp` to `navtracker_core` sources:

```cmake
add_library(navtracker_core
  core/geo/Wgs84.cpp
  core/geo/Datum.cpp
)
```

And add the test source to `navtracker_tests`:

```cmake
add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
)
```

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/geo/Datum.hpp` not found.

- [ ] **Step 4: Write the header**

Create `core/geo/Datum.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/geo/Wgs84.hpp"

namespace navtracker::geo {

// Local East-North-Up tangent plane about a fixed geodetic origin.
// The common working frame for the fusion engine (spec D3).
class Datum {
 public:
  explicit Datum(const Geodetic& origin);

  Eigen::Vector3d toEnu(const Geodetic& g) const;
  Geodetic toGeodetic(const Eigen::Vector3d& enu) const;

  const Geodetic& origin() const { return origin_; }

 private:
  Geodetic origin_;
  Eigen::Vector3d origin_ecef_;
  Eigen::Matrix3d ecef_to_enu_;  // rotation R
};

}  // namespace navtracker::geo
```

- [ ] **Step 5: Write the implementation**

Create `core/geo/Datum.cpp`:

```cpp
#include "core/geo/Datum.hpp"

#include <cmath>

namespace navtracker::geo {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double d) { return d * kPi / 180.0; }
}  // namespace

Datum::Datum(const Geodetic& origin) : origin_(origin) {
  origin_ecef_ = geodeticToEcef(origin);
  const double lat = deg2rad(origin.lat_deg);
  const double lon = deg2rad(origin.lon_deg);
  const double s_lat = std::sin(lat);
  const double c_lat = std::cos(lat);
  const double s_lon = std::sin(lon);
  const double c_lon = std::cos(lon);
  ecef_to_enu_ << -s_lon,          c_lon,         0.0,
                  -s_lat * c_lon,  -s_lat * s_lon, c_lat,
                   c_lat * c_lon,   c_lat * s_lon, s_lat;
}

Eigen::Vector3d Datum::toEnu(const Geodetic& g) const {
  return ecef_to_enu_ * (geodeticToEcef(g) - origin_ecef_);
}

Geodetic Datum::toGeodetic(const Eigen::Vector3d& enu) const {
  const Eigen::Vector3d ecef = ecef_to_enu_.transpose() * enu + origin_ecef_;
  return ecefToGeodetic(ecef);
}

}  // namespace navtracker::geo
```

- [ ] **Step 6: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Datum.*` tests pass.

- [ ] **Step 7: Commit**

```bash
git add core/geo/Datum.hpp core/geo/Datum.cpp tests/geo/test_datum.cpp CMakeLists.txt
git commit -m "feat(geo): add ENU Datum local tangent plane"
```

---

## Task 5: Core enums and IDs

Shared identifiers and enums used by `Measurement` and `Track`. Header-only.

**Files:**
- Create: `core/types/Ids.hpp`
- Test: `tests/types/test_ids.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/types/test_ids.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/types/Ids.hpp"

using navtracker::TrackId;
using navtracker::TrackStatus;
using navtracker::SensorKind;

TEST(Ids, TrackIdCompares) {
  EXPECT_EQ(TrackId{7}, TrackId{7});
  EXPECT_NE(TrackId{7}, TrackId{8});
  EXPECT_LT(TrackId{7}, TrackId{8});
}

TEST(Ids, DefaultsAreSafe) {
  EXPECT_EQ(TrackId{}.value, 0u);
  EXPECT_EQ(TrackStatus{}, TrackStatus::Tentative);
  EXPECT_EQ(SensorKind{}, SensorKind::Unknown);
}
```

- [ ] **Step 2: Add the test to the build**

In `CMakeLists.txt`, add `tests/types/test_ids.cpp` to `navtracker_tests`:

```cmake
add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/types/test_ids.cpp
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
)
```

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/types/Ids.hpp` not found.

- [ ] **Step 4: Write the implementation**

Create `core/types/Ids.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace navtracker {

// Which physical sensor class produced a measurement.
enum class SensorKind { Unknown, Ais, ArpaTtm, ArpaTll, EoIr, OwnShip, Lidar };

// Track lifecycle states (spec section 7). Default-constructs to Tentative.
enum class TrackStatus { Tentative, Confirmed, Coasting, Deleted };

// How a Measurement's value/covariance vectors are laid out.
enum class MeasurementModel { Position2D, PositionVelocity2D, RangeBearing2D };

// Stable internal track identity (spec invariant: primary key, never reused).
struct TrackId {
  std::uint64_t value{0};
  friend bool operator==(TrackId a, TrackId b) { return a.value == b.value; }
  friend bool operator!=(TrackId a, TrackId b) { return a.value != b.value; }
  friend bool operator<(TrackId a, TrackId b) { return a.value < b.value; }
};

}  // namespace navtracker
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Ids.*` tests pass.

- [ ] **Step 6: Commit**

```bash
git add core/types/Ids.hpp tests/types/test_ids.cpp CMakeLists.txt
git commit -m "feat(core): add SensorKind, TrackStatus, MeasurementModel, TrackId"
```

---

## Task 6: Measurement type

The normalized input handed to the core — the `ISensorAdapter` output type (spec section 5). Header-only; carries an Eigen value vector + covariance whose layout is described by `MeasurementModel`, plus association hints.

**Files:**
- Create: `core/types/Measurement.hpp`
- Test: `tests/types/test_measurement.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/types/test_measurement.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/types/Measurement.hpp"

using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;

TEST(Measurement, HoldsPosition2D) {
  Measurement m;
  m.time = Timestamp::fromSeconds(100.0);
  m.sensor = SensorKind::ArpaTll;
  m.source_id = "radar_fwd";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(1500.0, -300.0);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;

  EXPECT_EQ(m.dim(), 2);
  EXPECT_EQ(m.value.x(), 1500.0);
  EXPECT_EQ(m.covariance(0, 0), 25.0);
  EXPECT_FALSE(m.hints.mmsi.has_value());
}

TEST(Measurement, CarriesAssociationHints) {
  Measurement m;
  m.hints.mmsi = 211234560u;
  m.hints.sensor_track_id = 42;

  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 211234560u);
  ASSERT_TRUE(m.hints.sensor_track_id.has_value());
  EXPECT_EQ(*m.hints.sensor_track_id, 42);
}
```

- [ ] **Step 2: Add the test to the build**

In `CMakeLists.txt`, add `tests/types/test_measurement.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/types/Measurement.hpp` not found.

- [ ] **Step 4: Write the implementation**

Create `core/types/Measurement.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Opportunistic identity cues from a sensor; never the fusion key (spec D1).
struct AssociationHints {
  std::optional<std::uint32_t> mmsi;
  std::optional<std::int32_t> sensor_track_id;
};

// Normalized sensor output consumed by the tracker. `value` and `covariance`
// (R) are laid out according to `model`; e.g. Position2D -> [east, north] in
// the working ENU frame with a 2x2 R.
struct Measurement {
  Timestamp time;
  SensorKind sensor{SensorKind::Unknown};
  std::string source_id;
  MeasurementModel model{MeasurementModel::Position2D};
  Eigen::VectorXd value;
  Eigen::MatrixXd covariance;
  AssociationHints hints;

  int dim() const { return static_cast<int>(value.size()); }
};

}  // namespace navtracker
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Measurement.*` tests pass.

- [ ] **Step 6: Commit**

```bash
git add core/types/Measurement.hpp tests/types/test_measurement.cpp CMakeLists.txt
git commit -m "feat(core): add Measurement type"
```

---

## Task 7: Track type

The authoritative fused entity (spec section 5). Header-only. Stable `TrackId`, Eigen state + covariance (filled by the estimator in plan 2), lifecycle status, fused attributes, and provenance.

**Files:**
- Create: `core/types/Track.hpp`
- Test: `tests/types/test_track.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/types/test_track.cpp`:

```cpp
#include <gtest/gtest.h>
#include "core/types/Track.hpp"

using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackStatus;

TEST(Track, DefaultsToTentativeWithStableId) {
  Track t;
  EXPECT_EQ(t.status, TrackStatus::Tentative);
  EXPECT_EQ(t.id.value, 0u);
  EXPECT_TRUE(t.contributing_sources.empty());
  EXPECT_FALSE(t.attributes.mmsi.has_value());
}

TEST(Track, HoldsKinematicStateAndProvenance) {
  Track t;
  t.id = TrackId{17};
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(10.0, 20.0, 1.0, -2.0);   // [px, py, vx, vy]
  t.covariance = Eigen::Matrix4d::Identity();
  t.attributes.mmsi = 211234560u;
  t.attributes.name = "MV TEST";
  t.contributing_sources.push_back("ais");
  t.contributing_sources.push_back("radar_fwd");

  EXPECT_EQ(t.id, TrackId{17});
  EXPECT_EQ(t.state.size(), 4);
  EXPECT_EQ(t.state(2), 1.0);
  ASSERT_TRUE(t.attributes.name.has_value());
  EXPECT_EQ(*t.attributes.name, "MV TEST");
  EXPECT_EQ(t.contributing_sources.size(), 2u);
}
```

- [ ] **Step 2: Add the test to the build**

In `CMakeLists.txt`, add `tests/types/test_track.cpp` to `navtracker_tests`.

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build 2>&1 | head -20
```
Expected: FAIL — `core/types/Track.hpp` not found.

- [ ] **Step 4: Write the implementation**

Create `core/types/Track.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Fused, non-kinematic attributes. Every field is optional: a non-cooperative
// target without AIS still has a valid Track keyed by TrackId (spec invariant).
struct TrackAttributes {
  std::optional<std::uint32_t> mmsi;
  std::optional<std::string> name;
  std::optional<std::string> vessel_type;
  std::optional<double> length_m;
  std::optional<double> beam_m;
};

// The authoritative fused track. Kinematic state/covariance are populated by
// the estimator (plan 2); this plan defines the carrier type.
struct Track {
  TrackId id;
  Timestamp last_update;
  TrackStatus status{TrackStatus::Tentative};
  Eigen::VectorXd state;       // e.g. [px, py, vx, vy] in ENU
  Eigen::MatrixXd covariance;  // P
  TrackAttributes attributes;
  std::vector<std::string> contributing_sources;  // provenance
};

}  // namespace navtracker
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: PASS — all `Track.*` tests pass; full suite green.

- [ ] **Step 6: Commit**

```bash
git add core/types/Track.hpp tests/types/test_track.cpp CMakeLists.txt
git commit -m "feat(core): add Track type"
```

---

## Done criteria

- `cmake --build build && ctest --test-dir build --output-on-failure` is fully green.
- `navtracker_core` builds with zero I/O dependencies (only Eigen).
- Geodetic↔ENU conversion verified (known points + round-trips).
- `Timestamp`, `Measurement`, `Track`, and the shared enums/IDs exist and are tested.

## Roadmap (subsequent plans)

2. **Estimation** — `IMotionModel` (constant-velocity), `IEstimator` (EKF) with linear position + nonlinear range/bearing measurement models. Each documented Math/Assumptions/Rationale/Improve.
3. **Association + track management** — `IDataAssociator` (Mahalanobis gating + GNN), track lifecycle (initiate/confirm/coast/delete), stable-ID allocation.
4. **Pipeline + time** — time-ordered reorder buffer, tracker orchestration, `ISensorAdapter`/`ITrackSink` ports, deterministic replay test.
5. **Sensor adapters** — AIS, ARPA (TTM relative + TLL position), EO/IR (bearing-only), own-ship nav; normalization/geo-projection into ENU with per-sensor R.
6. **Scenario harness + metrics** — synthetic ground-truth scenarios, OSPA/track-accuracy metrics for comparing estimator/association choices.
