# Debug Visualization (MCAP / Foxglove) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline debug recorder that writes a Foxglove-compatible `.mcap` of the fusion pipeline — detections (with uncertainty), associations, gates, covariance ellipses, NIS, sensor bias, lifecycle, and CPA — openable/scrubbable in Lichtblick.

**Architecture:** A new output adapter `adapters/foxglove/` that implements the existing driven ports (`ITrackSnapshotSink`, `ITrackSink`, `IInnovationSink`, `ICollisionRiskSink`, `IDatumChangeSink`) plus input-side `recordMeasurement`/`recordOwnShip` taps wired from the `app/` composition root. Messages use Foxglove well-known schemas encoded as JSON (no protobuf). `navtracker_core` gains no dependency; `mcap` + `nlohmann_json` live behind a CMake option.

**Tech Stack:** C++17, CMake, Conan, GoogleTest, `mcap` (C++ writer/reader), `nlohmann_json` (header-only), Eigen.

**Spec:** `docs/superpowers/specs/2026-06-13-debug-visualization-foxglove-design.md`

---

## File Structure

```
adapters/foxglove/
  Geometry.hpp / Geometry.cpp       pure math: covariance-ellipse polyline,
                                    bearing wedge, color-by-sensor (no I/O, no json)
  FoxgloveJson.hpp / FoxgloveJson.cpp   navtracker types -> nlohmann::json for the
                                    well-known schemas (SceneUpdate / LocationFix /
                                    FrameTransform / Log) + /diag scalar objects
  Schemas.hpp                       embedded jsonschema text + schema names (constants)
  McapWriter.hpp / McapWriter.cpp   RAII wrapper over mcap::McapWriter: registerChannel
                                    + write(topic, time, json)
  FoxgloveDebugRecorder.hpp / .cpp  the sink/tap object; owns an McapWriter, a nullable
                                    const ISensorBiasProvider*, and a per-track S cache
adapters/foxglove/schemas/          vendored foxglove jsonschema .json files (Task 0)

app/example.cpp                     (modify) optional wiring demo behind a flag
adapters/sinks/TrackSnapshotFanout.hpp   small ITrackSnapshotSink multiplexer (Task 9)

tests/adapters/foxglove/
  test_geometry.cpp                 ellipse + wedge math
  test_foxglove_json.cpp            per-type json field assertions
  test_mcap_writer.cpp              round-trip write -> read
  test_recorder.cpp                 end-to-end sink -> mcap channels/messages
  test_recorder_determinism.cpp     same input twice -> identical payloads
```

Each `adapters/foxglove/*` unit has one responsibility and is testable in isolation: `Geometry` needs neither json nor mcap; `FoxgloveJson` needs neither mcap nor the recorder; `McapWriter` needs neither navtracker types nor the recorder.

---

## Task 0: Dependency + empty build target

**Files:**
- Modify: `conanfile.txt`
- Modify: `CMakeLists.txt`
- Create: `adapters/foxglove/schemas/` (vendored jsonschema files)
- Create: `adapters/foxglove/.keep` (placeholder so the dir exists; removed once real files land)

- [ ] **Step 1: Verify `mcap` + `nlohmann_json` exist in the configured Conan remote**

Run (sandbox disabled — writes `~/.conan2`, see build-env note):
```bash
conan search 'mcap/*' ; conan search 'nlohmann_json/*'
```
Expected: at least one `mcap/<ver>` and one `nlohmann_json/<ver>`. Pin the highest stable
versions you see (this plan assumes `mcap/1.4.0` and `nlohmann_json/3.11.3`; substitute the
actual versions if they differ). If `mcap` is absent from the remote, STOP and report —
the fallback (vendoring the single-header writer + lz4/zstd) is a separate decision, not
covered here.

- [ ] **Step 2: Add the deps to `conanfile.txt`**

```
[requires]
eigen/3.4.0
gtest/1.14.0
mcap/1.4.0
nlohmann_json/3.11.3

[generators]
CMakeDeps
CMakeToolchain
```

- [ ] **Step 3: Vendor the Foxglove jsonschema files**

Download the four well-known JSON schemas from the foxglove schemas repo
(`schemas/jsonschema/` at github.com/foxglove/foxglove-sdk; offline copies are fine) into
`adapters/foxglove/schemas/`:
`SceneUpdate.json`, `LocationFix.json`, `FrameTransform.json`, `Log.json`.
These are the schema *text* registered with each MCAP channel. (Foxglove also recognizes
well-known schemas by name, but registering the real schema text keeps the file
self-describing for any consumer.)

- [ ] **Step 4: Add the CMake option + library target**

In `CMakeLists.txt`, after the existing `find_package` lines (near line 9):
```cmake
option(NAVTRACKER_BUILD_FOXGLOVE "Build the Foxglove/MCAP debug recorder adapter" ON)
if(NAVTRACKER_BUILD_FOXGLOVE)
  find_package(mcap REQUIRED)
  find_package(nlohmann_json REQUIRED)
endif()
```
After the other `add_library(navtracker_* ...)` blocks (near line 118):
```cmake
if(NAVTRACKER_BUILD_FOXGLOVE)
  add_library(navtracker_foxglove
    adapters/foxglove/Geometry.cpp
    adapters/foxglove/FoxgloveJson.cpp
    adapters/foxglove/McapWriter.cpp
    adapters/foxglove/FoxgloveDebugRecorder.cpp)
  target_include_directories(navtracker_foxglove PUBLIC ${CMAKE_SOURCE_DIR})
  target_link_libraries(navtracker_foxglove
    PUBLIC navtracker_core Eigen3::Eigen
    PRIVATE mcap::mcap nlohmann_json::nlohmann_json)
  target_compile_definitions(navtracker_foxglove PRIVATE
    NAVTRACKER_FOXGLOVE_SCHEMA_DIR="${CMAKE_SOURCE_DIR}/adapters/foxglove/schemas")
endif()
```
Add the test sources + link to the existing `navtracker_tests` target (inside a guard so a
`-DNAVTRACKER_BUILD_FOXGLOVE=OFF` build still compiles). After the `add_executable(navtracker_tests ...)`
source list, append the foxglove test files conditionally is awkward in one call, so instead
add them via `target_sources`:
```cmake
if(NAVTRACKER_BUILD_FOXGLOVE)
  target_sources(navtracker_tests PRIVATE
    tests/adapters/foxglove/test_geometry.cpp
    tests/adapters/foxglove/test_foxglove_json.cpp
    tests/adapters/foxglove/test_mcap_writer.cpp
    tests/adapters/foxglove/test_recorder.cpp
    tests/adapters/foxglove/test_recorder_determinism.cpp)
  target_link_libraries(navtracker_tests PRIVATE navtracker_foxglove)
endif()
```

- [ ] **Step 5: Regenerate the build with the new deps**

Run (sandbox disabled for `conan install`):
```bash
conan install . -of=build -s build_type=Release -s compiler.cppstd=17 --build=missing
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
```
Expected: configure succeeds; `mcap` and `nlohmann_json` resolve. The library has no sources
yet — that's fine; CMake will error on the empty `add_library` only at build time, so do NOT
build yet. Create empty `.cpp` files in Step 6 first.

- [ ] **Step 6: Create empty translation units so the target links**

Create `adapters/foxglove/Geometry.cpp`, `FoxgloveJson.cpp`, `McapWriter.cpp`,
`FoxgloveDebugRecorder.cpp`, each containing only:
```cpp
// Implementation added in subsequent tasks.
```
And one stub test so the test target builds:
`tests/adapters/foxglove/test_geometry.cpp`:
```cpp
#include <gtest/gtest.h>
TEST(FoxgloveBuild, LinksAndRuns) { SUCCEED(); }
```

- [ ] **Step 7: Build + run to confirm wiring**

Run:
```bash
cmake --build build --target navtracker_tests 2>&1 | tail -5
ctest --test-dir build -R FoxgloveBuild --output-on-failure
```
Expected: build succeeds, `FoxgloveBuild.LinksAndRuns` passes.

- [ ] **Step 8: Commit**

```bash
git add conanfile.txt CMakeLists.txt adapters/foxglove tests/adapters/foxglove
git commit -m "Foxglove debug recorder: dependency + empty build target"
```

---

## Task 1: Geometry helpers (pure math)

**Files:**
- Modify: `adapters/foxglove/Geometry.cpp`
- Create: `adapters/foxglove/Geometry.hpp`
- Test: `tests/adapters/foxglove/test_geometry.cpp` (replace stub)

The ellipse polyline is the shared primitive for covariance ellipses (`P₂`) and gate
ellipses (`√γ · ellipse(S)`). `k` is the confidence multiplier (default 2 ≈ 2σ).

- [ ] **Step 1: Write the header**

`adapters/foxglove/Geometry.hpp`:
```cpp
#pragma once
#include <array>
#include <vector>
#include <Eigen/Core>
#include "core/types/Ids.hpp"

namespace navtracker::foxglove {

struct Pt { double x, y, z; };            // ENU, z held at 0 for plan-view geometry
struct Rgba { double r, g, b, a; };

// Polyline (closed loop) approximating the 1-σ·k ellipse of a symmetric 2x2
// covariance, centered at `center_enu`. `n` points around the loop.
std::vector<Pt> covarianceEllipse(const Eigen::Vector2d& center_enu,
                                  const Eigen::Matrix2d& cov,
                                  double k = 2.0, int n = 48);

// Bearing-only wedge: two rays from `sensor_enu` at angle alpha_rad ± k·sigma,
// out to `length_m`, returned as a 3-point open polyline [edge1, apex, edge2].
std::vector<Pt> bearingWedge(const Eigen::Vector2d& sensor_enu,
                             double alpha_rad, double sigma_rad,
                             double length_m, double k = 2.0);

// Stable per-sensor color (so the same source keeps its hue across frames).
Rgba colorForSensor(SensorKind sensor, const std::string& source_id);

}  // namespace navtracker::foxglove
```

- [ ] **Step 2: Write failing tests**

`tests/adapters/foxglove/test_geometry.cpp` (replace the stub file):
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "adapters/foxglove/Geometry.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

TEST(Geometry, EllipseAxesMatchEigenvaluesForDiagonalCov) {
  Eigen::Matrix2d cov;
  cov << 9.0, 0.0, 0.0, 4.0;            // sigma_x=3, sigma_y=2
  auto pts = covarianceEllipse(Eigen::Vector2d(10.0, -5.0), cov, /*k=*/2.0, /*n=*/4);
  // n=4 -> points at angles 0, 90, 180, 270 deg in eigenbasis.
  // Max |x-cx| should be k*sigma_x = 6, max |y-cy| = k*sigma_y = 4.
  double max_dx = 0, max_dy = 0;
  for (auto& p : pts) { max_dx = std::max(max_dx, std::abs(p.x - 10.0));
                        max_dy = std::max(max_dy, std::abs(p.y + 5.0)); }
  EXPECT_NEAR(max_dx, 6.0, 1e-9);
  EXPECT_NEAR(max_dy, 4.0, 1e-9);
  for (auto& p : pts) EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST(Geometry, EllipseIsClosedLoop) {
  auto pts = covarianceEllipse(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity(), 1.0, 16);
  ASSERT_GE(pts.size(), 2u);
  EXPECT_NEAR(pts.front().x, pts.back().x, 1e-9);
  EXPECT_NEAR(pts.front().y, pts.back().y, 1e-9);
}

TEST(Geometry, BearingWedgeApexAtSensorAndSpread) {
  auto w = bearingWedge(Eigen::Vector2d(1.0, 2.0), /*alpha=*/0.0, /*sigma=*/0.1,
                        /*length=*/100.0, /*k=*/2.0);
  ASSERT_EQ(w.size(), 3u);
  EXPECT_NEAR(w[1].x, 1.0, 1e-9);     // apex == sensor
  EXPECT_NEAR(w[1].y, 2.0, 1e-9);
  // alpha=0 is +east; edges at +/-0.2 rad. Edge1 y > sensor y, edge2 y < sensor y.
  EXPECT_GT(w[0].y, 2.0);
  EXPECT_LT(w[2].y, 2.0);
}

TEST(Geometry, ColorIsStablePerSource) {
  auto a = colorForSensor(SensorKind::EoIr, "cam-1");
  auto b = colorForSensor(SensorKind::EoIr, "cam-1");
  auto c = colorForSensor(SensorKind::EoIr, "cam-2");
  EXPECT_EQ(a.r, b.r); EXPECT_EQ(a.g, b.g); EXPECT_EQ(a.b, b.b);
  EXPECT_FALSE(a.r == c.r && a.g == c.g && a.b == c.b);
}
```

- [ ] **Step 3: Run tests to confirm they fail to compile/link**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -5`
Expected: FAIL — `Geometry.hpp` functions undefined.

- [ ] **Step 4: Implement `Geometry.cpp`**

```cpp
#include "adapters/foxglove/Geometry.hpp"
#include <cmath>
#include <functional>
#include <Eigen/Eigenvalues>

namespace navtracker::foxglove {

std::vector<Pt> covarianceEllipse(const Eigen::Vector2d& c, const Eigen::Matrix2d& cov,
                                  double k, int n) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  Eigen::Vector2d lam = es.eigenvalues().cwiseMax(0.0);   // clamp tiny negatives
  Eigen::Matrix2d V = es.eigenvectors();
  const double a = k * std::sqrt(lam(0)), b = k * std::sqrt(lam(1));
  std::vector<Pt> out;
  out.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {                          // <= n closes the loop
    const double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    Eigen::Vector2d e(a * std::cos(t), b * std::sin(t));  // in eigenbasis
    Eigen::Vector2d p = c + V * e;                        // back to ENU
    out.push_back({p.x(), p.y(), 0.0});
  }
  return out;
}

std::vector<Pt> bearingWedge(const Eigen::Vector2d& s, double alpha, double sigma,
                             double length, double k) {
  const double a1 = alpha + k * sigma, a2 = alpha - k * sigma;
  Eigen::Vector2d e1 = s + length * Eigen::Vector2d(std::cos(a1), std::sin(a1));
  Eigen::Vector2d e2 = s + length * Eigen::Vector2d(std::cos(a2), std::sin(a2));
  return {{e1.x(), e1.y(), 0.0}, {s.x(), s.y(), 0.0}, {e2.x(), e2.y(), 0.0}};
}

Rgba colorForSensor(SensorKind sensor, const std::string& source_id) {
  // Hash (kind, source_id) into a hue; fixed S/V. Deterministic across runs.
  const std::size_t h = std::hash<int>{}(static_cast<int>(sensor)) * 1000003u
                      ^ std::hash<std::string>{}(source_id);
  const double hue = static_cast<double>(h % 360u);       // degrees
  const double s = 0.65, v = 0.95, c = v * s, x = c * (1 - std::abs(std::fmod(hue / 60.0, 2.0) - 1));
  const double m = v - c;
  double r = 0, g = 0, bl = 0;
  if (hue < 60)      { r = c; g = x; }
  else if (hue < 120){ r = x; g = c; }
  else if (hue < 180){ g = c; bl = x; }
  else if (hue < 240){ g = x; bl = c; }
  else if (hue < 300){ r = x; bl = c; }
  else               { r = c; bl = x; }
  return {r + m, g + m, bl + m, 1.0};
}

}  // namespace navtracker::foxglove
```

- [ ] **Step 5: Run tests to confirm they pass**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R Geometry --output-on-failure`
Expected: all 4 `Geometry.*` tests PASS.

- [ ] **Step 6: Commit**

```bash
git add adapters/foxglove/Geometry.hpp adapters/foxglove/Geometry.cpp tests/adapters/foxglove/test_geometry.cpp
git commit -m "Foxglove: covariance-ellipse / bearing-wedge / sensor-color geometry"
```

---

## Task 2: Foxglove JSON builders

**Files:**
- Modify: `adapters/foxglove/FoxgloveJson.cpp`
- Create: `adapters/foxglove/FoxgloveJson.hpp`
- Create: `adapters/foxglove/Schemas.hpp`
- Test: `tests/adapters/foxglove/test_foxglove_json.cpp`

These build `nlohmann::json` objects matching the Foxglove well-known schemas. Timestamps
use `{sec, nsec}`. `Timestamp` in navtracker is nanoseconds since epoch (verify in
`core/types/Timestamp.hpp`; adjust the split if the unit differs).

- [ ] **Step 1: Write `Schemas.hpp` (channel + schema-name constants)**

```cpp
#pragma once
namespace navtracker::foxglove {
// Foxglove well-known schema names (recognized by Lichtblick for auto-render).
inline constexpr const char* kSceneUpdateSchema  = "foxglove.SceneUpdate";
inline constexpr const char* kLocationFixSchema  = "foxglove.LocationFix";
inline constexpr const char* kFrameTransformSchema = "foxglove.FrameTransform";
inline constexpr const char* kLogSchema          = "foxglove.Log";
// Custom diagnostic schema name (flat scalar object, plotted by field path).
inline constexpr const char* kDiagSchema         = "navtracker.Diag";
inline constexpr const char* kRootFrame          = "enu";
}  // namespace navtracker::foxglove
```

- [ ] **Step 2: Write `FoxgloveJson.hpp`**

```cpp
#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "adapters/foxglove/Geometry.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::foxglove {

nlohmann::json timeJson(Timestamp t);                       // {sec,nsec}

// A SceneUpdate "lines" primitive (type 0 = LINE_STRIP) of one polyline.
nlohmann::json lineEntity(const std::string& id, const std::vector<Pt>& pts,
                          const Rgba& color, double thickness = 1.0);
// A SceneUpdate "texts" label at a point.
nlohmann::json textEntity(const std::string& id, const Pt& at,
                          const std::string& text, const Rgba& color);
// A SceneUpdate "arrows" primitive from a->b.
nlohmann::json arrowEntity(const std::string& id, const Pt& a, const Pt& b,
                           const Rgba& color);

// Wrap a list of entity json objects into a SceneUpdate message at time t.
nlohmann::json sceneUpdate(Timestamp t, const std::vector<nlohmann::json>& entities);

nlohmann::json locationFix(Timestamp t, double lat_deg, double lon_deg,
                           const std::array<double, 9>& cov_row_major);
nlohmann::json frameTransform(Timestamp t, const std::string& parent,
                              const std::string& child,
                              double tx, double ty, double tz, double yaw_rad);
nlohmann::json logMsg(Timestamp t, int level, const std::string& name,
                      const std::string& message);

}  // namespace navtracker::foxglove
```

- [ ] **Step 3: Write failing tests**

`tests/adapters/foxglove/test_foxglove_json.cpp`:
```cpp
#include <gtest/gtest.h>
#include "adapters/foxglove/FoxgloveJson.hpp"
using namespace navtracker;
using namespace navtracker::foxglove;

TEST(FoxgloveJson, TimeSplitsNanos) {
  auto j = timeJson(Timestamp{1'500'000'003LL});   // 1.5s + 3ns  (adjust if Timestamp differs)
  EXPECT_EQ(j["sec"].get<long long>(), 1);
  EXPECT_EQ(j["nsec"].get<long long>(), 500'000'003);
}

TEST(FoxgloveJson, SceneUpdateHasEntityWithLine) {
  auto line = lineEntity("e1", {{0,0,0},{1,1,0}}, {1,0,0,1}, 2.0);
  auto su = sceneUpdate(Timestamp{0}, {line});
  ASSERT_EQ(su["entities"].size(), 1u);
  auto& ent = su["entities"][0];
  EXPECT_EQ(ent["frame_id"], "enu");
  ASSERT_EQ(ent["lines"].size(), 1u);
  EXPECT_EQ(ent["lines"][0]["points"].size(), 2u);
  EXPECT_DOUBLE_EQ(ent["lines"][0]["color"]["r"].get<double>(), 1.0);
}

TEST(FoxgloveJson, LocationFixCarriesLatLonAndCov) {
  std::array<double,9> cov{}; cov[0] = 4.0; cov[4] = 9.0;
  auto j = locationFix(Timestamp{0}, 59.9, 10.7, cov);
  EXPECT_DOUBLE_EQ(j["latitude"].get<double>(), 59.9);
  EXPECT_DOUBLE_EQ(j["longitude"].get<double>(), 10.7);
  ASSERT_EQ(j["position_covariance"].size(), 9u);
  EXPECT_DOUBLE_EQ(j["position_covariance"][0].get<double>(), 4.0);
}

TEST(FoxgloveJson, FrameTransformYawToQuaternion) {
  auto j = frameTransform(Timestamp{0}, "enu", "own_ship", 5, 6, 0, 0.0);
  EXPECT_DOUBLE_EQ(j["translation"]["x"].get<double>(), 5.0);
  EXPECT_NEAR(j["rotation"]["w"].get<double>(), 1.0, 1e-12);   // yaw 0 -> identity
  EXPECT_NEAR(j["rotation"]["z"].get<double>(), 0.0, 1e-12);
}
```

- [ ] **Step 4: Run tests to confirm they fail**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -5`
Expected: FAIL — functions undefined.

- [ ] **Step 5: Implement `FoxgloveJson.cpp`**

```cpp
#include "adapters/foxglove/FoxgloveJson.hpp"
#include <cmath>
#include "adapters/foxglove/Schemas.hpp"

namespace navtracker::foxglove {
using nlohmann::json;

json timeJson(Timestamp t) {
  const long long ns = static_cast<long long>(t.value);   // ns since epoch; adjust if needed
  return json{{"sec", ns / 1'000'000'000LL}, {"nsec", ns % 1'000'000'000LL}};
}

static json ptJson(const Pt& p) { return json{{"x", p.x}, {"y", p.y}, {"z", p.z}}; }
static json colorJson(const Rgba& c) {
  return json{{"r", c.r}, {"g", c.g}, {"b", c.b}, {"a", c.a}};
}
static json identityPose() {
  return json{{"position", {{"x",0},{"y",0},{"z",0}}},
              {"orientation", {{"x",0},{"y",0},{"z",0},{"w",1}}}};
}

json lineEntity(const std::string& id, const std::vector<Pt>& pts, const Rgba& color,
                double thickness) {
  json points = json::array();
  for (auto& p : pts) points.push_back(ptJson(p));
  json line{{"type", 0},                       // 0 = LINE_STRIP
            {"pose", identityPose()},
            {"thickness", thickness},
            {"scale_invariant", true},
            {"points", points},
            {"color", colorJson(color)},
            {"colors", json::array()},
            {"indices", json::array()}};
  return json{{"id", id}, {"frame_id", kRootFrame}, {"frame_locked", false},
              {"lifetime", {{"sec",0},{"nsec",0}}}, {"metadata", json::array()},
              {"arrows", json::array()}, {"cubes", json::array()},
              {"spheres", json::array()}, {"cylinders", json::array()},
              {"lines", json::array({line})}, {"triangles", json::array()},
              {"texts", json::array()}, {"models", json::array()}};
}

json textEntity(const std::string& id, const Pt& at, const std::string& text,
                const Rgba& color) {
  json txt{{"pose", {{"position", ptJson(at)}, {"orientation", {{"x",0},{"y",0},{"z",0},{"w",1}}}}},
           {"billboard", true}, {"font_size", 12.0}, {"scale_invariant", true},
           {"color", colorJson(color)}, {"text", text}};
  json e = lineEntity(id, {}, color);          // reuse the empty-arrays skeleton
  e["lines"] = json::array();
  e["texts"] = json::array({txt});
  return e;
}

json arrowEntity(const std::string& id, const Pt& a, const Pt& b, const Rgba& color) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len = std::sqrt(dx*dx + dy*dy);
  const double yaw = std::atan2(dy, dx);
  json arrow{{"pose", {{"position", ptJson(a)},
                       {"orientation", {{"x",0},{"y",0},{"z",std::sin(yaw/2)},{"w",std::cos(yaw/2)}}}}},
             {"shaft_length", len * 0.8}, {"shaft_diameter", 0.5},
             {"head_length", len * 0.2}, {"head_diameter", 1.0},
             {"color", colorJson(color)}};
  json e = lineEntity(id, {}, color);
  e["lines"] = json::array();
  e["arrows"] = json::array({arrow});
  return e;
}

json sceneUpdate(Timestamp t, const std::vector<json>& entities) {
  json ents = json::array();
  for (auto& e : entities) { json ec = e; ec["timestamp"] = timeJson(t); ents.push_back(ec); }
  return json{{"deletions", json::array()}, {"entities", ents}};
}

json locationFix(Timestamp t, double lat, double lon, const std::array<double,9>& cov) {
  return json{{"timestamp", timeJson(t)}, {"frame_id", ""},
              {"latitude", lat}, {"longitude", lon}, {"altitude", 0.0},
              {"position_covariance", cov}, {"position_covariance_type", 2}};  // 2 = DIAGONAL_KNOWN
}

json frameTransform(Timestamp t, const std::string& parent, const std::string& child,
                    double tx, double ty, double tz, double yaw) {
  return json{{"timestamp", timeJson(t)}, {"parent_frame_id", parent},
              {"child_frame_id", child},
              {"translation", {{"x",tx},{"y",ty},{"z",tz}}},
              {"rotation", {{"x",0},{"y",0},{"z",std::sin(yaw/2)},{"w",std::cos(yaw/2)}}}};
}

json logMsg(Timestamp t, int level, const std::string& name, const std::string& message) {
  return json{{"timestamp", timeJson(t)}, {"level", level}, {"message", message},
              {"name", name}, {"file", ""}, {"line", 0}};
}

}  // namespace navtracker::foxglove
```

- [ ] **Step 6: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R FoxgloveJson --output-on-failure
git add adapters/foxglove/FoxgloveJson.hpp adapters/foxglove/FoxgloveJson.cpp adapters/foxglove/Schemas.hpp tests/adapters/foxglove/test_foxglove_json.cpp
git commit -m "Foxglove: JSON builders for SceneUpdate/LocationFix/FrameTransform/Log"
```
Expected: 4 `FoxgloveJson.*` tests PASS.

---

## Task 3: McapWriter wrapper

**Files:**
- Modify: `adapters/foxglove/McapWriter.cpp`
- Create: `adapters/foxglove/McapWriter.hpp`
- Test: `tests/adapters/foxglove/test_mcap_writer.cpp`

- [ ] **Step 1: Write `McapWriter.hpp`**

```cpp
#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include "core/types/Timestamp.hpp"

namespace navtracker::foxglove {

// RAII wrapper over mcap::McapWriter. One channel per topic; schema text is
// loaded once per schema name from the vendored schemas dir.
class McapWriter {
 public:
  explicit McapWriter(const std::string& path);   // opens, zstd compression
  ~McapWriter();                                   // closes if open

  McapWriter(const McapWriter&) = delete;
  McapWriter& operator=(const McapWriter&) = delete;

  // Register (idempotent) a topic bound to a Foxglove schema name. schema_text
  // is the jsonschema (may be empty -> name-only recognition).
  void ensureChannel(const std::string& topic, const std::string& schema_name,
                     const std::string& schema_text);

  // Write a JSON payload to a previously-registered topic at log time t.
  void write(const std::string& topic, Timestamp t, const std::string& json_bytes);

  void close();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace navtracker::foxglove
```

- [ ] **Step 2: Write failing round-trip test**

`tests/adapters/foxglove/test_mcap_writer.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <mcap/reader.hpp>
#include "adapters/foxglove/McapWriter.hpp"
#include "adapters/foxglove/Schemas.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

TEST(McapWriter, RoundTripsOneMessage) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    McapWriter w(path);
    w.ensureChannel("/diag/test", kDiagSchema, "");
    w.write("/diag/test", Timestamp{42}, R"({"value":7})");
    w.close();
  }
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(path).ok());
  int count = 0; std::string topic; std::uint64_t logtime = 0;
  auto view = reader.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) {
    ++count; topic = it->channel->topic; logtime = it->message.logTime;
  }
  reader.close();
  std::remove(path.c_str());
  EXPECT_EQ(count, 1);
  EXPECT_EQ(topic, "/diag/test");
  EXPECT_EQ(logtime, 42u);
}
```

- [ ] **Step 3: Run to confirm failure**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -5`
Expected: FAIL — `McapWriter` undefined.

- [ ] **Step 4: Implement `McapWriter.cpp`**

```cpp
#include "adapters/foxglove/McapWriter.hpp"
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <stdexcept>

namespace navtracker::foxglove {

struct McapWriter::Impl {
  mcap::McapWriter writer;
  std::unordered_map<std::string, mcap::ChannelId> channels;     // topic -> id
  std::unordered_map<std::string, mcap::SchemaId> schemas;       // schema name -> id
  bool open = false;
  std::unordered_map<std::string, std::uint32_t> seq;            // per-topic sequence
};

McapWriter::McapWriter(const std::string& path) : impl_(new Impl) {
  mcap::McapWriterOptions opts("");                 // empty profile = generic
  opts.compression = mcap::Compression::Zstd;
  const auto status = impl_->writer.open(path, opts);
  if (!status.ok()) throw std::runtime_error("mcap open failed: " + status.message);
  impl_->open = true;
}

McapWriter::~McapWriter() { close(); delete impl_; }

void McapWriter::ensureChannel(const std::string& topic, const std::string& schema_name,
                               const std::string& schema_text) {
  if (impl_->channels.count(topic)) return;
  auto sit = impl_->schemas.find(schema_name);
  if (sit == impl_->schemas.end()) {
    mcap::Schema schema(schema_name, "jsonschema",
                        std::string_view(schema_text));   // empty ok: name-only
    impl_->writer.addSchema(schema);
    sit = impl_->schemas.emplace(schema_name, schema.id).first;
  }
  mcap::Channel channel(topic, "json", sit->second);
  impl_->writer.addChannel(channel);
  impl_->channels.emplace(topic, channel.id);
}

void McapWriter::write(const std::string& topic, Timestamp t, const std::string& bytes) {
  auto it = impl_->channels.find(topic);
  if (it == impl_->channels.end()) throw std::runtime_error("unknown topic: " + topic);
  mcap::Message msg;
  msg.channelId = it->second;
  msg.sequence = impl_->seq[topic]++;
  msg.logTime = static_cast<mcap::Timestamp>(t.value);
  msg.publishTime = msg.logTime;
  msg.data = reinterpret_cast<const std::byte*>(bytes.data());
  msg.dataSize = bytes.size();
  const auto status = impl_->writer.write(msg);
  if (!status.ok()) throw std::runtime_error("mcap write failed: " + status.message);
}

void McapWriter::close() {
  if (impl_ && impl_->open) { impl_->writer.close(); impl_->open = false; }
}

}  // namespace navtracker::foxglove
```
NOTE: `#define MCAP_IMPLEMENTATION` must appear in **exactly one** TU. This is it. Do not
repeat it in any other file. If `Timestamp.value` is not nanoseconds, scale here so logTime
is nanoseconds (Foxglove convention).

- [ ] **Step 5: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R McapWriter --output-on-failure
git add adapters/foxglove/McapWriter.hpp adapters/foxglove/McapWriter.cpp tests/adapters/foxglove/test_mcap_writer.cpp
git commit -m "Foxglove: McapWriter RAII wrapper with json channels + zstd"
```
Expected: `McapWriter.RoundTripsOneMessage` PASS.

---

## Task 4: Recorder — tracks, map, track-count diag

**Files:**
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.cpp`
- Create: `adapters/foxglove/FoxgloveDebugRecorder.hpp`
- Test: `tests/adapters/foxglove/test_recorder.cpp`

The recorder owns the writer, registers all channels up front, and converts each event.
Position uses the top-left 2×2 of `Track.covariance`; lat/lon via `toGeodeticWithCov`.

- [ ] **Step 1: Write `FoxgloveDebugRecorder.hpp`**

```cpp
#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include "ports/ITrackSnapshotSink.hpp"
#include "ports/ITrackSink.hpp"
#include "ports/IInnovationSink.hpp"
#include "ports/ICollisionRiskSink.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"     // OwnShipPose + IDatumChangeSink
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "adapters/foxglove/McapWriter.hpp"

namespace navtracker::foxglove {

struct RecorderConfig {
  double ellipse_k = 2.0;       // confidence multiplier for covariance ellipses
  double gate_gamma = 0.0;      // chi-square gate threshold; 0 disables /gates ellipses
};

class FoxgloveDebugRecorder final
    : public ITrackSnapshotSink, public ITrackSink, public IInnovationSink,
      public ICollisionRiskSink, public IDatumChangeSink {
 public:
  FoxgloveDebugRecorder(const std::string& path, const geo::Datum& datum,
                        const ISensorBiasProvider* bias = nullptr,
                        RecorderConfig cfg = {});
  ~FoxgloveDebugRecorder() override;

  // Input-side taps (called from app composition root).
  void recordMeasurement(const Measurement& m);
  void recordOwnShip(const OwnShipPose& pose);

  // ITrackSnapshotSink
  void onTracks(const std::vector<Track>& tracks, Timestamp now) override;
  // ITrackSink
  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  void onTrackDeleted(const TrackLifecycleEvent& e) override;
  // IInnovationSink
  void onInnovation(const InnovationEvent& e) override;
  // ICollisionRiskSink
  void onCollisionRisk(const CollisionRiskEvent& e) override;
  // IDatumChangeSink
  void onDatumRecentered(const geo::Datum& old_d, const geo::Datum& new_d) override;

  void close();

 private:
  void registerChannels();
  std::unique_ptr<McapWriter> w_;
  geo::Datum datum_;
  const ISensorBiasProvider* bias_;
  RecorderConfig cfg_;
  // Latest predicted innovation covariance per track (for /gates).
  std::unordered_map<std::uint64_t, Eigen::MatrixXd> last_S_;
};

}  // namespace navtracker::foxglove
```

- [ ] **Step 2: Write failing test (tracks + map + count)**

`tests/adapters/foxglove/test_recorder.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cstdio>
#include <map>
#include <string>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

static std::map<std::string,int> countByTopic(const std::string& path) {
  mcap::McapReader r; r.open(path);
  std::map<std::string,int> counts;
  auto view = r.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) counts[it->channel->topic]++;
  r.close();
  return counts;
}

static Track makeTrack(std::uint64_t id, double e, double n) {
  Track t; t.id = TrackId{id}; t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(e, n, 1.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 4.0;
  t.velocity_observed = true;
  return t;
}

TEST(Recorder, TracksEmitSceneMapAndCount) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    FoxgloveDebugRecorder rec(path, geo::Datum{59.9, 10.7});   // datum ctor: adjust to real sig
    std::vector<Track> tracks{makeTrack(1, 100, 200), makeTrack(2, -50, -50)};
    rec.onTracks(tracks, Timestamp{1000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/tracks"], 1);              // one SceneUpdate per onTracks call
  EXPECT_EQ(c["/map/tracks"], 2);          // one LocationFix per track
  EXPECT_EQ(c["/diag/track_count"], 1);
}
```

- [ ] **Step 3: Run to confirm failure**

Run: `cmake --build build --target navtracker_tests 2>&1 | tail -5`
Expected: FAIL — recorder undefined.

- [ ] **Step 4: Implement the recorder core + `onTracks`**

`adapters/foxglove/FoxgloveDebugRecorder.cpp`:
```cpp
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include <fstream>
#include <sstream>
#include "adapters/foxglove/FoxgloveJson.hpp"
#include "adapters/foxglove/Geometry.hpp"
#include "adapters/foxglove/Schemas.hpp"
#include "core/output/TrackOutput.hpp"

namespace navtracker::foxglove {
using nlohmann::json;

namespace {
std::string loadSchema(const char* file) {
  std::ifstream in(std::string(NAVTRACKER_FOXGLOVE_SCHEMA_DIR) + "/" + file);
  std::stringstream ss; ss << in.rdbuf(); return ss.str();   // empty if missing -> name-only
}
Eigen::Matrix2d pos2(const Eigen::MatrixXd& P) { return P.topLeftCorner<2,2>(); }
Eigen::Vector2d xy(const Eigen::VectorXd& s) { return s.head<2>(); }
}  // namespace

FoxgloveDebugRecorder::FoxgloveDebugRecorder(const std::string& path, const geo::Datum& datum,
                                             const ISensorBiasProvider* bias, RecorderConfig cfg)
    : w_(std::make_unique<McapWriter>(path)), datum_(datum), bias_(bias), cfg_(cfg) {
  registerChannels();
}
FoxgloveDebugRecorder::~FoxgloveDebugRecorder() { close(); }
void FoxgloveDebugRecorder::close() { if (w_) w_->close(); }

void FoxgloveDebugRecorder::registerChannels() {
  const std::string scene = loadSchema("SceneUpdate.json");
  const std::string loc   = loadSchema("LocationFix.json");
  const std::string tf    = loadSchema("FrameTransform.json");
  const std::string log   = loadSchema("Log.json");
  for (const char* t : {"/tracks","/detections","/associations","/gates","/cpa"})
    w_->ensureChannel(t, kSceneUpdateSchema, scene);
  for (const char* t : {"/map/tracks","/map/detections"})
    w_->ensureChannel(t, kLocationFixSchema, loc);
  w_->ensureChannel("/tf", kFrameTransformSchema, tf);
  w_->ensureChannel("/log", kLogSchema, log);
  for (const char* t : {"/diag/innovation","/diag/track_count","/diag/gate_ratio","/diag/bias"})
    w_->ensureChannel(t, kDiagSchema, "");
}

void FoxgloveDebugRecorder::onTracks(const std::vector<Track>& tracks, Timestamp now) {
  std::vector<json> entities;
  int confirmed = 0, tentative = 0;
  for (const auto& t : tracks) {
    if (t.state.size() < 2) continue;
    const Eigen::Vector2d p = xy(t.state);
    const Rgba col = (t.status == TrackStatus::Confirmed) ? Rgba{0.1,0.9,0.1,1.0}
                                                          : Rgba{0.9,0.9,0.1,1.0};
    const std::string base = "track-" + std::to_string(t.id.value);
    entities.push_back(lineEntity(base + "-cov",
        covarianceEllipse(p, pos2(t.covariance), cfg_.ellipse_k), col));
    entities.push_back(textEntity(base + "-label", {p.x(), p.y(), 0},
        std::to_string(t.id.value), col));
    if (t.velocity_observed && t.state.size() >= 4) {
      const Eigen::Vector2d v = t.state.segment<2>(2);
      entities.push_back(arrowEntity(base + "-vel", {p.x(),p.y(),0},
          {p.x()+v.x(), p.y()+v.y(), 0}, col));
    }
    (t.status == TrackStatus::Confirmed ? confirmed : tentative)++;
    // Map: lat/lon via the canonical helper.
    const auto geo = toGeodeticWithCov(p, pos2(t.covariance), datum_);
    std::array<double,9> cov{};
    cov[0] = geo.position_covariance_m2(0,0); cov[1] = geo.position_covariance_m2(0,1);
    cov[3] = geo.position_covariance_m2(1,0); cov[4] = geo.position_covariance_m2(1,1);
    w_->write("/map/tracks", now, locationFix(now, geo.lat_deg, geo.lon_deg, cov).dump());
  }
  w_->write("/tracks", now, sceneUpdate(now, entities).dump());
  json diag{{"time_ns", now.value}, {"confirmed", confirmed}, {"tentative", tentative},
            {"total", confirmed + tentative}};
  w_->write("/diag/track_count", now, diag.dump());
}

// Remaining sink methods implemented in later tasks (Tasks 5-8). Provide
// empty bodies now so the class is concrete and links:
void FoxgloveDebugRecorder::recordMeasurement(const Measurement&) {}
void FoxgloveDebugRecorder::recordOwnShip(const OwnShipPose&) {}
void FoxgloveDebugRecorder::onTrackInitiated(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackConfirmed(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackUpdated(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackDeleted(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onInnovation(const InnovationEvent&) {}
void FoxgloveDebugRecorder::onCollisionRisk(const CollisionRiskEvent&) {}
void FoxgloveDebugRecorder::onDatumRecentered(const geo::Datum&, const geo::Datum&) {}

}  // namespace navtracker::foxglove
```
NOTE: confirm the real `geo::Datum` constructor signature and `toGeodeticWithCov` return
fields (`PositionGeodeticWithCov.lat_deg/lon_deg/position_covariance_m2` per
`core/output/TrackOutput.hpp`). Adjust the test's `geo::Datum{...}` to match.

- [ ] **Step 5: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R "Recorder.Tracks" --output-on-failure
git add adapters/foxglove/FoxgloveDebugRecorder.hpp adapters/foxglove/FoxgloveDebugRecorder.cpp tests/adapters/foxglove/test_recorder.cpp
git commit -m "Foxglove recorder: tracks scene + map + track-count diag"
```
Expected: `Recorder.TracksEmitSceneMapAndCount` PASS.

---

## Task 5: Recorder — detections (per-model) + bias

**Files:**
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.cpp` (`recordMeasurement`)
- Test: `tests/adapters/foxglove/test_recorder.cpp` (add cases)

`recordMeasurement` branches on `Measurement.model`. Position models draw a marker + ellipse;
`Bearing2D` draws a wedge from `sensor_position_enu`. When a bias provider is present and the
estimate `is_published`, also draw the corrected marker and emit `/diag/bias`.

- [ ] **Step 1: Add failing tests**

Append to `tests/adapters/foxglove/test_recorder.cpp`:
```cpp
static Measurement posMeas(double e, double n) {
  Measurement m; m.time = Timestamp{2000}; m.sensor = SensorKind::Ais; m.source_id = "ais-1";
  m.model = MeasurementModel::Position2D; m.value = Eigen::Vector2d(e, n);
  m.covariance = Eigen::Matrix2d::Identity() * 9.0;
  return m;
}
static Measurement bearingMeas(double alpha) {
  Measurement m; m.time = Timestamp{2001}; m.sensor = SensorKind::EoIr; m.source_id = "cam-1";
  m.model = MeasurementModel::Bearing2D;
  m.value = Eigen::VectorXd::Constant(1, alpha);
  m.covariance = Eigen::MatrixXd::Constant(1,1, 0.01);
  m.sensor_position_enu = Eigen::Vector2d(0,0);
  return m;
}

TEST(Recorder, PositionAndBearingDetectionsEmit) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    FoxgloveDebugRecorder rec(path, geo::Datum{59.9,10.7});
    rec.recordMeasurement(posMeas(10, 20));
    rec.recordMeasurement(bearingMeas(0.0));
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/detections"], 2);          // one SceneUpdate per measurement
  EXPECT_EQ(c["/map/detections"], 1);      // only the position meas maps to lat/lon
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `ctest --test-dir build -R "Recorder.PositionAndBearing" --output-on-failure` (after build)
Expected: FAIL — counts are 0 (empty `recordMeasurement`).

- [ ] **Step 3: Implement `recordMeasurement`**

Replace the empty `recordMeasurement` body:
```cpp
void FoxgloveDebugRecorder::recordMeasurement(const Measurement& m) {
  const Rgba col = colorForSensor(m.sensor, m.source_id);
  std::vector<json> entities;
  const std::string base = "det-" + m.source_id + "-" + std::to_string(m.time.value);

  if (m.model == MeasurementModel::Bearing2D) {
    const double alpha = m.value(0);
    const double sigma = std::sqrt(m.covariance(0,0));
    entities.push_back(lineEntity(base + "-ray",
        bearingWedge(m.sensor_position_enu, alpha, sigma, /*length=*/2000.0, cfg_.ellipse_k), col));
    if (bias_) {
      const auto bb = bias_->bearingBias({m.sensor, m.source_id});
      if (bb.is_published) {
        Rgba c2 = col; c2.a = 0.4;
        entities.push_back(lineEntity(base + "-ray-corr",
            bearingWedge(m.sensor_position_enu, alpha + bb.bias_rad, sigma, 2000.0, cfg_.ellipse_k), c2));
        w_->write("/diag/bias", m.time,
                  json{{"time_ns", m.time.value}, {"sensor", static_cast<int>(m.sensor)},
                       {"source_id", m.source_id}, {"bearing_bias_rad", bb.bias_rad},
                       {"is_published", true}}.dump());
      }
    }
  } else {  // Position2D / PositionVelocity2D / RangeBearing2D: ENU point + ellipse
    const Eigen::Vector2d p = m.value.head<2>();
    entities.push_back(lineEntity(base + "-cov",
        covarianceEllipse(p, m.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k), col));
    if (bias_) {
      const auto pb = bias_->positionBias({m.sensor, m.source_id});
      if (pb.is_published) {
        const Eigen::Vector2d pc = p + pb.bias_enu_m;
        Rgba c2 = col; c2.a = 0.4;
        entities.push_back(lineEntity(base + "-cov-corr",
            covarianceEllipse(pc, m.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k), c2));
        entities.push_back(arrowEntity(base + "-bias", {p.x(),p.y(),0}, {pc.x(),pc.y(),0}, c2));
        w_->write("/diag/bias", m.time,
                  json{{"time_ns", m.time.value}, {"sensor", static_cast<int>(m.sensor)},
                       {"source_id", m.source_id}, {"bias_e_m", pb.bias_enu_m.x()},
                       {"bias_n_m", pb.bias_enu_m.y()}, {"is_published", true}}.dump());
      }
    }
    const auto geo = toGeodeticWithCov(p, m.covariance.topLeftCorner<2,2>(), datum_);
    std::array<double,9> cov{};
    cov[0]=geo.position_covariance_m2(0,0); cov[4]=geo.position_covariance_m2(1,1);
    w_->write("/map/detections", m.time, locationFix(m.time, geo.lat_deg, geo.lon_deg, cov).dump());
  }
  w_->write("/detections", m.time, sceneUpdate(m.time, entities).dump());
}
```
Add `#include <cmath>` if not already present.

- [ ] **Step 4: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R "Recorder.Position" --output-on-failure
git add adapters/foxglove/FoxgloveDebugRecorder.cpp tests/adapters/foxglove/test_recorder.cpp
git commit -m "Foxglove recorder: per-model detections + raw/corrected bias overlay"
```
Expected: `Recorder.PositionAndBearingDetectionsEmit` PASS.

---

## Task 6: Recorder — innovation diag + gates

**Files:**
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.cpp` (`onInnovation`, `onTracks` gate loop)
- Test: `tests/adapters/foxglove/test_recorder.cpp` (add case)

`onInnovation` emits `/diag/innovation` (NIS = νᵀS⁻¹ν) and caches `S` per track. `onTracks`,
when `cfg_.gate_gamma > 0` and a cached `S` exists for the track, draws the gate ellipse on
`/gates` centered on the track position with `k = √gamma`.

- [ ] **Step 1: Add failing test**

```cpp
TEST(Recorder, InnovationEmitsNisAndGate) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    RecorderConfig cfg; cfg.gate_gamma = 9.21;        // chi2 2dof 99%
    FoxgloveDebugRecorder rec(path, geo::Datum{59.9,10.7}, nullptr, cfg);
    InnovationEvent e; e.time = Timestamp{3000}; e.track_id = TrackId{1};
    e.sensor = SensorKind::Ais; e.source_id = "ais-1";
    e.residual = Eigen::Vector2d(1.0, 0.0);
    e.S = Eigen::Matrix2d::Identity() * 4.0; e.R = e.S; e.dim = 2;
    rec.onInnovation(e);                              // caches S for track 1
    rec.onTracks({makeTrack(1, 0, 0)}, Timestamp{3001});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/diag/innovation"], 1);
  EXPECT_EQ(c["/gates"], 1);                          // gate drawn because S cached + gamma>0
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `ctest --test-dir build -R "Recorder.Innovation" --output-on-failure` (after build)
Expected: FAIL — counts 0.

- [ ] **Step 3: Implement `onInnovation` + gate emission in `onTracks`**

Replace the empty `onInnovation`:
```cpp
void FoxgloveDebugRecorder::onInnovation(const InnovationEvent& e) {
  const double nis = e.residual.transpose() * e.S.ldlt().solve(e.residual);
  last_S_[e.track_id.value] = e.S;
  w_->write("/diag/innovation", e.time,
            json{{"time_ns", e.time.value}, {"track_id", e.track_id.value},
                 {"sensor", static_cast<int>(e.sensor)}, {"source_id", e.source_id},
                 {"nis", nis}, {"dim", e.dim}}.dump());
}
```
At the end of `onTracks`, BEFORE the final `/tracks` write, add a gate pass:
```cpp
  if (cfg_.gate_gamma > 0.0) {
    std::vector<json> gate_entities;
    for (const auto& t : tracks) {
      if (t.state.size() < 2) continue;
      auto sit = last_S_.find(t.id.value);
      if (sit == last_S_.end() || sit->second.rows() < 2) continue;
      const Eigen::Vector2d p = xy(t.state);
      gate_entities.push_back(lineEntity("gate-" + std::to_string(t.id.value),
          covarianceEllipse(p, sit->second.topLeftCorner<2,2>(), std::sqrt(cfg_.gate_gamma)),
          Rgba{0.4,0.4,1.0,0.6}));
    }
    w_->write("/gates", now, sceneUpdate(now, gate_entities).dump());
  }
```

- [ ] **Step 4: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R "Recorder.Innovation" --output-on-failure
git add adapters/foxglove/FoxgloveDebugRecorder.cpp tests/adapters/foxglove/test_recorder.cpp
git commit -m "Foxglove recorder: NIS diag + gate ellipse from cached innovation S"
```
Expected: `Recorder.InnovationEmitsNisAndGate` PASS.

---

## Task 7: Recorder — lifecycle log, CPA, own-ship/datum tf

**Files:**
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.cpp` (lifecycle, CPA, ownship, datum)
- Test: `tests/adapters/foxglove/test_recorder.cpp` (add case)

- [ ] **Step 1: Add failing test**

```cpp
TEST(Recorder, LifecycleCpaOwnshipEmit) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    FoxgloveDebugRecorder rec(path, geo::Datum{59.9,10.7});
    rec.onTrackConfirmed({TrackId{1}, Timestamp{4000}, TrackStatus::Confirmed});
    rec.onTrackDeleted({TrackId{1}, Timestamp{4500}, TrackStatus::Confirmed});
    CollisionRiskEvent ev; ev.transition = CollisionRiskTransition::Entered;
    ev.other = TrackId{1}; ev.time = Timestamp{4100};
    ev.prediction.cpa_distance_m = 50; ev.prediction.tcpa_seconds = 120;
    ev.prediction.probability_below_threshold = 0.8; ev.prediction.d_threshold_m = 100;
    rec.onCollisionRisk(ev);
    OwnShipPose pose; pose.time = Timestamp{4000}; pose.lat_deg = 59.9; pose.lon_deg = 10.7;
    pose.heading_true_deg = 90.0;
    rec.recordOwnShip(pose);
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_GE(c["/log"], 3);     // confirmed + deleted + cpa entered
  EXPECT_EQ(c["/cpa"], 1);
  EXPECT_EQ(c["/tf"], 1);
}
```

- [ ] **Step 2: Run to confirm failure** — `ctest --test-dir build -R "Recorder.Lifecycle"`; Expected FAIL.

- [ ] **Step 3: Implement the bodies**

```cpp
void FoxgloveDebugRecorder::onTrackInitiated(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 1, "lifecycle",
      "track " + std::to_string(e.id.value) + " initiated").dump());
}
void FoxgloveDebugRecorder::onTrackConfirmed(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 2, "lifecycle",
      "track " + std::to_string(e.id.value) + " confirmed").dump());
}
void FoxgloveDebugRecorder::onTrackUpdated(const TrackLifecycleEvent&) { /* high-volume: skip /log */ }
void FoxgloveDebugRecorder::onTrackDeleted(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 3, "lifecycle",
      "track " + std::to_string(e.id.value) + " deleted").dump());
}

void FoxgloveDebugRecorder::onCollisionRisk(const CollisionRiskEvent& e) {
  const char* kind = e.transition == CollisionRiskTransition::Entered ? "ENTERED"
                   : e.transition == CollisionRiskTransition::Exited  ? "EXITED" : "UPDATED";
  w_->write("/log", e.time, logMsg(e.time, 2, "cpa",
      std::string("CPA ") + kind + " track " + std::to_string(e.other.value) +
      " d=" + std::to_string(e.prediction.cpa_distance_m) +
      "m t=" + std::to_string(e.prediction.tcpa_seconds) + "s").dump());
  // Minimal CPA marker: a text at origin carrying the numbers (geometry can be
  // enriched later; keeps the channel populated + scrubbable).
  std::vector<json> ents{ textEntity("cpa-" + std::to_string(e.other.value),
      {0,0,0}, std::string(kind) + " d=" + std::to_string(e.prediction.cpa_distance_m),
      {1.0,0.3,0.3,1.0}) };
  w_->write("/cpa", e.time, sceneUpdate(e.time, ents).dump());
}

void FoxgloveDebugRecorder::recordOwnShip(const OwnShipPose& pose) {
  // ENU position of own-ship relative to the datum, via the same geodetic helper.
  // heading_true_deg is clockwise-from-north; ENU yaw (CCW-from-east) = 90 - hdg.
  const double yaw = (90.0 - pose.heading_true_deg) * M_PI / 180.0;
  const auto p = toEnu(pose.lat_deg, pose.lon_deg, datum_);   // see NOTE
  w_->write("/tf", pose.time, frameTransform(pose.time, kRootFrame, "own_ship",
      p.x(), p.y(), 0.0, yaw).dump());
}

void FoxgloveDebugRecorder::onDatumRecentered(const geo::Datum& /*old_d*/, const geo::Datum& new_d) {
  datum_ = new_d;
  // Mark the discontinuity in the log; geometry already written stays in the old frame.
  w_->write("/log", Timestamp{0}, logMsg(Timestamp{0}, 4, "datum",
      "datum recentered").dump());
}
```
NOTE: use the project's existing geodetic forward helper for `toEnu(lat,lon,datum)`. Find it
with `grep -rn "toEnu\|geodeticToEnu\|llToEnu\|forward" core/geo/`. If the helper returns a
3-vector, take `.head<2>()`. If `Timestamp{0}` is undesirable for the datum log, thread the
current time through `onTracks`/`recordOwnShip` into a member `last_time_` and use that.

- [ ] **Step 4: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R "Recorder.Lifecycle" --output-on-failure
git add adapters/foxglove/FoxgloveDebugRecorder.cpp tests/adapters/foxglove/test_recorder.cpp
git commit -m "Foxglove recorder: lifecycle log + CPA + own-ship/datum tf"
```
Expected: `Recorder.LifecycleCpaOwnshipEmit` PASS.

---

## Task 8: Associations layer (snapshot-driven)

**Files:**
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.cpp` (`onTracks` association pass)
- Test: `tests/adapters/foxglove/test_recorder.cpp` (add case)

For each track's `recent_contributions`, draw a line from the contributing detection to the
track position. Bearing-only touches (NaN-checked `alpha_rad`) anchor at the sensor position.

- [ ] **Step 1: Add failing test**

```cpp
TEST(Recorder, AssociationsLineFromTouchToTrack) {
  const std::string path = std::string(std::tmpnam(nullptr)) + ".mcap";
  {
    FoxgloveDebugRecorder rec(path, geo::Datum{59.9,10.7});
    Track t = makeTrack(1, 100, 100);
    Track::SourceTouch st; st.sensor = SensorKind::Ais; st.source_id = "ais-1";
    st.time = Timestamp{5000}; st.value_enu = Eigen::Vector2d(105, 98);
    t.recent_contributions.push_back(st);
    rec.onTracks({t}, Timestamp{5000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/associations"], 1);
}
```

- [ ] **Step 2: Run to confirm failure** — `ctest -R "Recorder.Associations"`; Expected FAIL (count 0).

- [ ] **Step 3: Implement the association pass**

Inside `onTracks`, after the gate pass and BEFORE the final `/tracks` write, add:
```cpp
  std::vector<json> assoc;
  for (const auto& t : tracks) {
    if (t.state.size() < 2) continue;
    const Eigen::Vector2d tp = xy(t.state);
    int k = 0;
    for (const auto& st : t.recent_contributions) {
      Eigen::Vector2d from = st.value_enu;
      if (std::isnan(st.alpha_rad) == false) {        // bearing-only touch: anchor at sensor
        from = st.sensor_position_enu;
      }
      assoc.push_back(lineEntity(
          "assoc-" + std::to_string(t.id.value) + "-" + std::to_string(k++),
          {{from.x(),from.y(),0},{tp.x(),tp.y(),0}},
          colorForSensor(st.sensor, st.source_id)));
    }
  }
  w_->write("/associations", now, sceneUpdate(now, assoc).dump());
```
Add `#include <cmath>` (for `std::isnan`) if not present.

- [ ] **Step 4: Run + commit**

```bash
cmake --build build --target navtracker_tests && ctest --test-dir build -R "Recorder.Associations" --output-on-failure
git add adapters/foxglove/FoxgloveDebugRecorder.cpp tests/adapters/foxglove/test_recorder.cpp
git commit -m "Foxglove recorder: association lines from recent_contributions"
```
Expected: `Recorder.AssociationsLineFromTouchToTrack` PASS.

---

## Task 9: App wiring, TrackSnapshotFanout, determinism + smoke test

**Files:**
- Create: `adapters/sinks/TrackSnapshotFanout.hpp`
- Modify: `app/example.cpp` (optional recorder wiring behind an env/arg flag)
- Test: `tests/adapters/foxglove/test_recorder_determinism.cpp`

- [ ] **Step 1: Write `TrackSnapshotFanout.hpp`**

```cpp
#pragma once
#include <vector>
#include "ports/ITrackSnapshotSink.hpp"
namespace navtracker {
// Fan one snapshot stream out to N sinks (keeps the single-setter API intact).
class TrackSnapshotFanout final : public ITrackSnapshotSink {
 public:
  void add(ITrackSnapshotSink* s) { sinks_.push_back(s); }
  void onTracks(const std::vector<Track>& tracks, Timestamp now) override {
    for (auto* s : sinks_) s->onTracks(tracks, now);
  }
 private:
  std::vector<ITrackSnapshotSink*> sinks_;
};
}  // namespace navtracker
```

- [ ] **Step 2: Write the determinism test**

`tests/adapters/foxglove/test_recorder_determinism.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <vector>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
using namespace navtracker;
using namespace navtracker::foxglove;

static std::vector<std::string> payloads(const std::string& path) {
  mcap::McapReader r; r.open(path);
  std::vector<std::string> out;
  auto view = r.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it)
    out.emplace_back(it->channel->topic + "|" + std::to_string(it->message.logTime) + "|" +
                     std::string(reinterpret_cast<const char*>(it->message.data), it->message.dataSize));
  r.close();
  return out;
}

static void run(const std::string& path) {
  FoxgloveDebugRecorder rec(path, geo::Datum{59.9,10.7});
  Track t; t.id = TrackId{1}; t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(100,200,1,0); t.covariance = Eigen::Matrix4d::Identity()*4.0;
  t.velocity_observed = true;
  rec.onTracks({t}, Timestamp{1000});
  rec.onTracks({t}, Timestamp{2000});
  rec.close();
}

TEST(RecorderDeterminism, SameInputIdenticalPayloads) {
  const std::string a = std::string(std::tmpnam(nullptr)) + ".mcap";
  const std::string b = std::string(std::tmpnam(nullptr)) + ".mcap";
  run(a); run(b);
  auto pa = payloads(a), pb = payloads(b);
  std::remove(a.c_str()); std::remove(b.c_str());
  EXPECT_EQ(pa, pb);
}
```

- [ ] **Step 3: Run to confirm it passes (no impl needed — determinism should already hold)**

Run: `cmake --build build --target navtracker_tests && ctest --test-dir build -R "RecorderDeterminism" --output-on-failure`
Expected: PASS. If it FAILS, the cause is unordered iteration (e.g. `last_S_` or a map drives
output order). Fix by ordering any map-driven emission (`std::map` or sort keys) so payload
order is input-determined — do NOT make the test tolerant.

- [ ] **Step 4: Wire the recorder into `app/example.cpp` behind a flag**

In `app/example.cpp`, guarded so the default core-only example is unchanged. Near the top:
```cpp
#ifdef NAVTRACKER_WITH_FOXGLOVE
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "adapters/sinks/TrackSnapshotFanout.hpp"
#endif
```
Where the tracker/manager/CPA are constructed, add (illustrative — match the example's actual
variable names):
```cpp
#ifdef NAVTRACKER_WITH_FOXGLOVE
  const char* mcap_path = std::getenv("NAVTRACKER_MCAP");
  std::unique_ptr<foxglove::FoxgloveDebugRecorder> recorder;
  foxglove::RecorderConfig rc; rc.gate_gamma = 9.21;
  if (mcap_path) {
    recorder = std::make_unique<foxglove::FoxgloveDebugRecorder>(mcap_path, datum, /*bias=*/nullptr, rc);
    mgr.setTrackSink(recorder.get());
    tracker.setInnovationSink(recorder.get());
    // snapshot/cpa/datum wiring as available in the example
  }
#endif
```
And in the measurement loop, tee BEFORE `tracker.process(m)`:
```cpp
#ifdef NAVTRACKER_WITH_FOXGLOVE
    if (recorder) recorder->recordMeasurement(m);
#endif
    tracker.process(m);
```
In `CMakeLists.txt`, build a second example variant when the option is on:
```cmake
if(NAVTRACKER_BUILD_FOXGLOVE)
  add_executable(navtracker_example_foxglove app/example.cpp)
  target_compile_definitions(navtracker_example_foxglove PRIVATE NAVTRACKER_WITH_FOXGLOVE)
  target_link_libraries(navtracker_example_foxglove PRIVATE
    navtracker_core navtracker_foxglove Eigen3::Eigen)
endif()
```

- [ ] **Step 5: Build the example + produce a sample mcap (smoke)**

Run:
```bash
cmake --build build --target navtracker_example_foxglove
NAVTRACKER_MCAP=$TMPDIR/navtracker_smoke.mcap ./build/navtracker_example_foxglove
ls -la $TMPDIR/navtracker_smoke.mcap
```
Expected: a non-empty `.mcap` is produced. (Open it in Lichtblick manually to eyeball.)

- [ ] **Step 6: Commit**

```bash
git add adapters/sinks/TrackSnapshotFanout.hpp app/example.cpp CMakeLists.txt tests/adapters/foxglove/test_recorder_determinism.cpp
git commit -m "Foxglove: app wiring + snapshot fanout + determinism/smoke tests"
```

---

## Task 10: Documentation

**Files:**
- Create: `docs/debug-visualization.md`
- Modify: `docs/learning/00-index.md`, the relevant KF/gating/NIS chapter, `docs/learning/19-glossary.md`
- Modify: `docs/learning/figures/generate.py` (add one `fig_*` function + call it)
- Modify: `adapters/foxglove/FoxgloveDebugRecorder.hpp` (doc comment cross-ref)

- [ ] **Step 1: Write `docs/debug-visualization.md`**

Cover: the channel/topic table (mirror the spec), the panel layout to recreate in Lichtblick
(3D panel on `enu` frame + Map panel + Plot panels for `/diag/*` + Log panel), the
`NAVTRACKER_MCAP` env var + `gate_gamma`/`ellipse_k` config, the `k`/confidence convention,
the gate-layer caveat (renders only for tracks with an evaluated hard-match that scan), and
the note that timestamps are source-event times so the file overlays the upstream raw-sensor
MCAPs. Cross-reference the spec.

- [ ] **Step 2: Add a learning figure**

In `docs/learning/figures/generate.py`, add a `fig_seeing_the_tracker()` that draws (matplotlib)
an annotated plan view: a track position, its `P` covariance ellipse, a larger gate ellipse,
two detections (one inside / one outside the gate), an association line, and a bearing wedge.
Call it from `main()`. Regenerate per `docs/learning/figures/README.md` (venv + run).

- [ ] **Step 3: Add the "seeing the tracker" section**

In the existing gating/NIS chapter (find via `grep -rl "gating\|NIS" docs/learning/*.md`), add a
"Seeing the tracker" section: how covariance vs gate ellipses differ visually, how association
lines and bearing wedges read, how NIS plots reveal an over/under-confident filter. Embed the
new figure. Add a glossary entry for "MCAP / Foxglove" in `19-glossary.md` and an index line in
`00-index.md`.

- [ ] **Step 4: Cross-reference from the header**

Add to the top doc-comment of `FoxgloveDebugRecorder.hpp`: a one-line pointer to
`docs/debug-visualization.md` and the learning section.

- [ ] **Step 5: Build docs sanity + commit**

Run: `python -c "import ast; ast.parse(open('docs/learning/figures/generate.py').read())"` (syntax ok)
```bash
git add docs/debug-visualization.md docs/learning adapters/foxglove/FoxgloveDebugRecorder.hpp
git commit -m "Docs: debug-visualization reference + learning 'seeing the tracker' + figure"
```

- [ ] **Step 6: Full test sweep**

Run: `ctest --test-dir build --output-on-failure`
Expected: entire suite green (including the pre-existing determinism/replay tests, confirming
the recorder is a pure observer that changed no tracking behavior).

---

## Self-Review Notes

- **Spec coverage:** detections/per-model (T5), tracks+cov (T4), associations (T8), gates (T6),
  map (T4/T5), tf+datum (T7), log+cpa (T7), diag innovation/count/bias (T4/T5/T6), bias raw vs
  corrected (T5), determinism (T9), docs+learning (T10), single CMake option + deps (T0). The
  spec's "associator accessor" is intentionally superseded — gates reuse `InnovationEvent.S` +
  configured `gamma` (no core change); the spec note will be updated to match.
- **Type consistency:** `covarianceEllipse(center, cov, k, n)` used identically in T4/T5/T6/T8;
  `lineEntity/textEntity/arrowEntity/sceneUpdate` signatures fixed in T2 and reused unchanged;
  `last_S_` keyed by `TrackId::value` written in T6, read in T6 gate pass.
- **Open verifications flagged inline (do at execution time):** exact `mcap`/`nlohmann_json`
  versions (T0); `Timestamp` unit (T2/T3); `geo::Datum` ctor + geodetic forward helper name
  (T4/T7); `app/example.cpp` actual variable names (T9).
