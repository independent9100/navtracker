# PMBM Land / Coastline Clutter-Prior Implementation Plan (Task A)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Suppress phantom PMBM births on/near shore using a continuous clutter prior computed from consumer-supplied coastline GeoJSON, fixing the philos near-shore-clutter over-count.

**Architecture:** A nullable core port `ILandModel::clutterPrior(enu)→double`. A pure `CoastlineGeometry` turns geodetic land polygons into a signed-distance shoreline ramp (≈0.5 at the waterline, plateau 1.0 only well inland, 0 offshore). `CoastlineModel` wraps it + the working datum, implements `ILandModel` **and** `IDatumChangeSink` (so a datum recenter just swaps the query datum). A GeoJSON adapter (uses the existing `nlohmann_json` dep) builds it. PMBM scales the adaptive-birth intensity by `(1−clutterPrior)` at the birth position and hard-drops births where the prior is on the inland plateau.

**Tech Stack:** C++17, Eigen 3.4, `nlohmann_json/3.11.3` (already a Conan dep), CMake+Conan, GoogleTest.

**Source spec:** `docs/superpowers/specs/2026-06-30-pmbm-land-clutter-prior-design.md` (all decisions settled; refined 2026-06-30 to inland-only hard gate).

## Global Constraints

- **C++17 only.** `core/` and `ports/` have ZERO I/O; GeoJSON parsing lives in `adapters/`.
- **Determinism:** no wall-clock, no RNG; the land model is a pure function of (polygons, datum); async coastline swap is applied at a deterministic, timestamp-ordered point (consumer responsibility) — never a wall-clock callback.
- **Back-compat / bit-identical:** the port is nullable; new Config flags default off (`use_land_model=false`); with no land model wired the birth path is unchanged.
- **Identity/hexagonal invariants** of the repo hold; follow existing port + nullable-setter patterns.
- **Injection acts on birth intensity, NOT λ_C:** Task 1's `birth_existence_target` decouples `r_new` from λ_C (verified), so raising λ_C is ineffective.
- **Hard gate is inland-only:** via the prior's ramp shape; never hard-refuse a water position (protects anchored near-shore vessels).
- **Build:** `cmake --build build -j`; **test:** `ctest --test-dir build --output-on-failure`. Do NOT run `conan install` (sandbox-readonly cache; deps already installed).
- **Determinism test stays green; full suite green except none** (the 2 former determinism failures were fixed — suite is now all-pass; any failure is a regression).
- **Docs standard:** four-section algorithm doc + `docs/learning/` chapter for the new algorithm.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `ports/ILandModel.hpp` | The port: `clutterPrior(enu)→double` | Create |
| `core/land/CoastlineGeometry.{hpp,cpp}` | Pure: land polygons (geodetic) → signed-distance shoreline-ramp prior | Create |
| `core/land/CoastlineModel.{hpp,cpp}` | `ILandModel` + `IDatumChangeSink`; wraps geometry + datum; `setCoastline` swap | Create |
| `adapters/land/GeoJsonCoastline.{hpp,cpp}` | Parse GeoJSON (nlohmann) → `CoastlineGeometry` | Create |
| `core/pmbm/PmbmTracker.hpp` | `setLandModel(const ILandModel*)`, member, Config flags | Modify |
| `core/pmbm/PmbmTracker.cpp` | Scale birth intensity by `(1−c)` / inland hard-drop in both candidate builders | Modify (`:235-310`, `:447-477`) |
| `core/benchmark/Config.cpp` | `imm_cv_ct_pmbm_coverage_land` config | Modify (`:~647-723`) |
| `core/benchmark/Sweep.cpp` | Build+wire `CoastlineModel` for the land config (philos) | Modify (`:266-313`) |
| `tests/land/test_coastline_geometry.cpp` | Geometry unit tests | Create |
| `tests/land/test_coastline_model.cpp` | Model + datum-recenter tests | Create |
| `tests/land/test_geojson_coastline.cpp` | Parser tests (+ boston.geojson smoke) | Create |
| `tests/pmbm/test_pmbm_land_model.cpp` | Birth-suppression integration tests | Create |
| `CMakeLists.txt` | Register new sources + test files | Modify |
| `docs/algorithms/pmbm-design.md`, `docs/learning/` | Four-section doc + chapter | Modify |

Coordinate convention: **GeoJSON coordinates are `[lon, lat]` in degrees**; store polygon vertices as `Eigen::Vector2d(lon_deg, lat_deg)`.

---

## Task 1: The `ILandModel` port

**Files:** Create `ports/ILandModel.hpp`; Test: (covered by Task 3's tests).

**Interfaces:**
- Produces: `class ILandModel { virtual double clutterPrior(const Eigen::Vector2d& enu_xy) const = 0; }`.

- [ ] **Step 1: Write the header.** Mirror the minimal style of `ports/ITrackSink.hpp`.

```cpp
#pragma once

#include <Eigen/Core>

namespace navtracker {

// Continuous spatial clutter/land prior, queried by the tracker at birth time.
// Pure, zero-I/O. Nullable in use: if no land model is wired, behaviour is
// exactly today's. See docs/superpowers/specs/2026-06-30-pmbm-land-clutter-prior-design.md
class ILandModel {
 public:
  virtual ~ILandModel() = default;

  // Prior that a detection at this ENU position (metres) is shore/structure
  // clutter rather than a real new vessel:
  //   0.0  = open water        (no birth suppression)
  //   ~0.5 = at the waterline  (soft suppression)
  //   1.0  = well inside land  (hard-gate region)
  // Positions outside the loaded coastline's coverage return 0.0.
  virtual double clutterPrior(const Eigen::Vector2d& enu_xy) const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 2: Build to confirm it compiles.** Run: `cmake --build build -j` → no errors. (No test yet; exercised in Task 3.)

- [ ] **Step 3: Commit.**
```bash
git add ports/ILandModel.hpp
git commit -m "Task A Step 1: ILandModel port (clutterPrior)"
```

---

## Task 2: `CoastlineGeometry` — signed-distance shoreline-ramp prior (pure)

**Files:** Create `core/land/CoastlineGeometry.{hpp,cpp}`; Test: `tests/land/test_coastline_geometry.cpp`; Modify `CMakeLists.txt`.

**Interfaces:**
- Produces:
  - `struct LandPolygon { std::vector<Eigen::Vector2d> outer; std::vector<std::vector<Eigen::Vector2d>> holes; };` (vertices = `(lon_deg, lat_deg)`).
  - `struct CoastlinePriorParams { double inland_halfwidth_m = 50.0; double offshore_halfwidth_m = 50.0; };`
  - `class CoastlineGeometry` with `CoastlineGeometry(std::vector<LandPolygon>, CoastlinePriorParams)`, `double priorAtGeodetic(double lat_deg, double lon_deg) const`, `bool empty() const`.

- [ ] **Step 1: Write failing tests.** Create `tests/land/test_coastline_geometry.cpp`. Use a square "island" from lon/lat (−71.06,42.36)–(−71.04,42.38) (~1.6 km box) with margins 50 m.

```cpp
#include <gtest/gtest.h>
#include "core/land/CoastlineGeometry.hpp"

using navtracker::CoastlineGeometry;
using navtracker::CoastlinePriorParams;
using navtracker::LandPolygon;

namespace {
CoastlineGeometry squareIsland() {
  LandPolygon p;
  p.outer = { {-71.06,42.36}, {-71.04,42.36}, {-71.04,42.38}, {-71.06,42.38}, {-71.06,42.36} };
  CoastlinePriorParams pr; pr.inland_halfwidth_m = 50.0; pr.offshore_halfwidth_m = 50.0;
  return CoastlineGeometry({p}, pr);
}
}  // namespace

TEST(CoastlineGeometry, WellInsideLandIsOne) {
  auto g = squareIsland();
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-9);  // center, deep inland
}
TEST(CoastlineGeometry, OpenWaterFarOutsideIsZero) {
  auto g = squareIsland();
  EXPECT_NEAR(g.priorAtGeodetic(42.300, -71.200), 0.0, 1e-9);  // km away
}
TEST(CoastlineGeometry, WaterlineIsAboutHalf) {
  auto g = squareIsland();
  // A point essentially ON the west edge (lon -71.06) at mid-latitude.
  const double c = g.priorAtGeodetic(42.370, -71.06);
  EXPECT_GT(c, 0.35);
  EXPECT_LT(c, 0.65);  // ~0.5 at the boundary with equal in/off margins
}
TEST(CoastlineGeometry, MonotonicAcrossShore) {
  auto g = squareIsland();
  // Walk west→east across the west edge: deep water, just-offshore, on-edge,
  // just-inland, deep-inland → prior must be non-decreasing.
  const double far_w  = g.priorAtGeodetic(42.370, -71.0630); // ~250 m offshore
  const double near_w = g.priorAtGeodetic(42.370, -71.0605); // ~40 m offshore
  const double inland = g.priorAtGeodetic(42.370, -71.0590); // ~80 m inland
  EXPECT_LE(far_w, near_w);
  EXPECT_LE(near_w, inland);
  EXPECT_NEAR(far_w, 0.0, 1e-6);
}
TEST(CoastlineGeometry, EmptyGeometryIsZero) {
  CoastlineGeometry g;
  EXPECT_TRUE(g.empty());
  EXPECT_NEAR(g.priorAtGeodetic(42.37, -71.05), 0.0, 1e-9);
}
```

- [ ] **Step 2: Run to verify it fails to compile.** Run: `cmake --build build -j 2>&1 | tail -5` → FAIL (`CoastlineGeometry.hpp` not found). (Register the test file in CMake first — Step 4 — so it compiles into the test target.)

- [ ] **Step 3: Implement the header.** Create `core/land/CoastlineGeometry.hpp`:

```cpp
#pragma once

#include <vector>
#include <Eigen/Core>

namespace navtracker {

struct LandPolygon {
  std::vector<Eigen::Vector2d> outer;                       // (lon_deg, lat_deg)
  std::vector<std::vector<Eigen::Vector2d>> holes;          // each ring (lon,lat)
};

struct CoastlinePriorParams {
  double inland_halfwidth_m = 50.0;     // W_in: ramp reaches 1.0 this far inland
  double offshore_halfwidth_m = 50.0;   // W_off: ramp reaches 0.0 this far offshore
};

// Pure geometry: land polygons (geodetic) -> signed-distance shoreline ramp.
// Prior c(d) with d = signed distance to nearest shore edge (d<0 inland):
//   c = clamp((W_off - d) / (W_off + W_in), 0, 1)
// => c=1 for d<=-W_in (plateau, hard-gate region), ~0.5 at d=0 (waterline),
//    c=0 for d>=+W_off (open water). Distance computed in local-equirectangular
//    metres about the query point.
class CoastlineGeometry {
 public:
  CoastlineGeometry() = default;
  CoastlineGeometry(std::vector<LandPolygon> polys, CoastlinePriorParams params);

  double priorAtGeodetic(double lat_deg, double lon_deg) const;
  bool empty() const { return polys_.empty(); }

 private:
  std::vector<LandPolygon> polys_;
  CoastlinePriorParams params_{};
};

}  // namespace navtracker
```

- [ ] **Step 4: Implement the .cpp + register in CMake.** Create `core/land/CoastlineGeometry.cpp`:

```cpp
#include "core/land/CoastlineGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegLat = 111320.0;

bool pointInRing(double lon, double lat, const std::vector<Eigen::Vector2d>& ring) {
  bool inside = false;
  const std::size_t n = ring.size();
  if (n < 3) return false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = ring[i].x(), yi = ring[i].y();
    const double xj = ring[j].x(), yj = ring[j].y();
    if (((yi > lat) != (yj > lat)) &&
        (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

// distance (m) from point (lon,lat) to segment a-b, in local equirectangular m.
double segDistM(double lon, double lat, const Eigen::Vector2d& a,
                const Eigen::Vector2d& b) {
  const double mlon = kMetersPerDegLat * std::cos(lat * kPi / 180.0);
  const double px = 0.0, py = 0.0;
  const double ax = (a.x() - lon) * mlon, ay = (a.y() - lat) * kMetersPerDegLat;
  const double bx = (b.x() - lon) * mlon, by = (b.y() - lat) * kMetersPerDegLat;
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = (len2 > 0.0) ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double cx = ax + t * dx, cy = ay + t * dy;
  return std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

double minEdgeDistM(double lon, double lat, const std::vector<Eigen::Vector2d>& ring) {
  double best = std::numeric_limits<double>::infinity();
  if (ring.size() < 2) return best;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    best = std::min(best, segDistM(lon, lat, ring[i], ring[i + 1]));
  }
  return best;
}

}  // namespace

CoastlineGeometry::CoastlineGeometry(std::vector<LandPolygon> polys,
                                     CoastlinePriorParams params)
    : polys_(std::move(polys)), params_(params) {}

double CoastlineGeometry::priorAtGeodetic(double lat_deg, double lon_deg) const {
  if (polys_.empty()) return 0.0;

  bool inside = false;
  double dist = std::numeric_limits<double>::infinity();
  for (const auto& p : polys_) {
    bool in_outer = pointInRing(lon_deg, lat_deg, p.outer);
    bool in_hole = false;
    for (const auto& h : p.holes)
      if (pointInRing(lon_deg, lat_deg, h)) { in_hole = true; break; }
    if (in_outer && !in_hole) inside = true;
    dist = std::min(dist, minEdgeDistM(lon_deg, lat_deg, p.outer));
    for (const auto& h : p.holes)
      dist = std::min(dist, minEdgeDistM(lon_deg, lat_deg, h));
  }

  const double d = inside ? -dist : dist;            // signed: <0 inland
  const double w = params_.offshore_halfwidth_m + params_.inland_halfwidth_m;
  if (w <= 0.0) return inside ? 1.0 : 0.0;
  const double c = (params_.offshore_halfwidth_m - d) / w;
  return std::clamp(c, 0.0, 1.0);
}

}  // namespace navtracker
```
Then add `core/land/CoastlineGeometry.cpp` to the `navtracker_core` target source list and `tests/land/test_coastline_geometry.cpp` to the test target in `CMakeLists.txt` (mirror how `core/pmbm/PmbmTracker.cpp` and `tests/pmbm/*.cpp` are listed). If sources are globbed, verify by building.

- [ ] **Step 5: Build & run; confirm PASS.**
Run: `cmake --build build -j && ctest --test-dir build -R CoastlineGeometry --output-on-failure` → 5 PASS.

- [ ] **Step 6: Commit.**
```bash
git add core/land/CoastlineGeometry.hpp core/land/CoastlineGeometry.cpp tests/land/test_coastline_geometry.cpp CMakeLists.txt
git commit -m "Task A Step 2: CoastlineGeometry signed-distance shoreline-ramp prior"
```

---

## Task 3: `CoastlineModel` — `ILandModel` + `IDatumChangeSink`

**Files:** Create `core/land/CoastlineModel.{hpp,cpp}`; Test: `tests/land/test_coastline_model.cpp`; Modify `CMakeLists.txt`.

**Interfaces:**
- Consumes: `ILandModel` (Task 1), `CoastlineGeometry` (Task 2), `geo::Datum`/`geo::Geodetic` (`core/geo/Datum.hpp`, `Wgs84.hpp`), `IDatumChangeSink` (`core/own_ship/OwnShipProvider.hpp`).
- Produces: `class CoastlineModel : public ILandModel, public IDatumChangeSink` with `CoastlineModel(CoastlineGeometry, geo::Datum)`, `clutterPrior(enu)`, `onDatumRecentered(old,new)`, `void setCoastline(CoastlineGeometry)`.

- [ ] **Step 1: Write failing tests.** Create `tests/land/test_coastline_model.cpp`.

```cpp
#include <gtest/gtest.h>
#include "core/land/CoastlineModel.hpp"
#include "core/geo/Datum.hpp"

using navtracker::CoastlineGeometry;
using navtracker::CoastlineModel;
using navtracker::CoastlinePriorParams;
using navtracker::LandPolygon;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {
CoastlineGeometry island() {
  LandPolygon p;
  p.outer = { {-71.06,42.36}, {-71.04,42.36}, {-71.04,42.38}, {-71.06,42.38}, {-71.06,42.36} };
  return CoastlineGeometry({p}, CoastlinePriorParams{});
}
}  // namespace

TEST(CoastlineModel, QueryConvertsEnuToGeodeticPrior) {
  Datum d{Geodetic{42.37, -71.05, 0.0}};            // datum inside the island
  CoastlineModel m{island(), d};
  // ENU origin == datum origin == deep inland -> ~1.0
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 1.0, 1e-6);
  // 5 km east -> far open water -> 0
  EXPECT_NEAR(m.clutterPrior({5000.0, 0.0}), 0.0, 1e-6);
}

TEST(CoastlineModel, DatumRecenterSwapsQueryFrameKeepsGeographicPrior) {
  Datum d0{Geodetic{42.37, -71.05, 0.0}};
  CoastlineModel m{island(), d0};
  const double before = m.clutterPrior({0.0, 0.0});   // origin = inland point
  // Recenter datum to a point 10 km away; the SAME geographic inland point is
  // now at a non-zero ENU; querying that ENU must still give the inland prior.
  Datum d1{Geodetic{42.46, -71.05, 0.0}};             // ~10 km north
  Eigen::Vector3d enu_of_old_origin = d1.toEnu(Geodetic{42.37, -71.05, 0.0});
  m.onDatumRecentered(d0, d1);
  const double after = m.clutterPrior({enu_of_old_origin.x(), enu_of_old_origin.y()});
  EXPECT_NEAR(before, after, 1e-6);
  // And the new ENU origin (10 km north, open water) is now 0.
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 0.0, 1e-6);
}

TEST(CoastlineModel, SetCoastlineSwapsGeometry) {
  Datum d{Geodetic{42.37, -71.05, 0.0}};
  CoastlineModel m{CoastlineGeometry{}, d};           // empty -> all water
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 0.0, 1e-9);
  m.setCoastline(island());
  EXPECT_NEAR(m.clutterPrior({0.0, 0.0}), 1.0, 1e-6);
}
```

- [ ] **Step 2: Run to verify it fails.** Run: `cmake --build build -j 2>&1 | tail -5` → FAIL (no `CoastlineModel.hpp`). (Register test in CMake in Step 4.)

- [ ] **Step 3: Implement the header.** Create `core/land/CoastlineModel.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "ports/ILandModel.hpp"

namespace navtracker {

// Concrete land model: holds coastline geometry (geodetic) + the working datum.
// clutterPrior converts the ENU query to geodetic via the current datum, so a
// datum recenter is just a datum swap (no polygon reprojection). Pure: no I/O,
// no wall-clock, no RNG. setCoastline / onDatumRecentered must be invoked at
// deterministic, timestamp-ordered points (see spec §5).
class CoastlineModel : public ILandModel, public IDatumChangeSink {
 public:
  CoastlineModel(CoastlineGeometry geom, geo::Datum datum)
      : geom_(std::move(geom)), datum_(datum) {}

  double clutterPrior(const Eigen::Vector2d& enu_xy) const override {
    const geo::Geodetic g =
        datum_.toGeodetic(Eigen::Vector3d(enu_xy.x(), enu_xy.y(), 0.0));
    return geom_.priorAtGeodetic(g.lat_deg, g.lon_deg);
  }

  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    datum_ = new_datum;
  }

  void setCoastline(CoastlineGeometry geom) { geom_ = std::move(geom); }

 private:
  CoastlineGeometry geom_;
  geo::Datum datum_;
};

}  // namespace navtracker
```
(Header-only is fine; if the project prefers a .cpp, move the bodies and add to CMake. Confirm `geo::Datum` is copy-assignable — it stores an origin + precomputed matrix; if it is NOT copy-assignable, store `geo::Datum` by value still works via copy-construct in `onDatumRecentered` using a `std::optional<geo::Datum>` member and `emplace`. Check `core/geo/Datum.hpp`; adjust if needed.)

- [ ] **Step 4: Register test in CMake; build & run.**
Add `tests/land/test_coastline_model.cpp` to the test target. Run:
`cmake --build build -j && ctest --test-dir build -R CoastlineModel --output-on-failure` → 3 PASS.

- [ ] **Step 5: Commit.**
```bash
git add core/land/CoastlineModel.hpp tests/land/test_coastline_model.cpp CMakeLists.txt
git commit -m "Task A Step 3: CoastlineModel (ILandModel + IDatumChangeSink)"
```

---

## Task 4: GeoJSON coastline adapter

**Files:** Create `adapters/land/GeoJsonCoastline.{hpp,cpp}`; Test: `tests/land/test_geojson_coastline.cpp`; Modify `CMakeLists.txt`.

**Interfaces:**
- Consumes: `CoastlineGeometry`, `LandPolygon`, `CoastlinePriorParams` (Task 2); `nlohmann::json`.
- Produces:
  - `CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text, CoastlinePriorParams params);`
  - `CoastlineGeometry loadCoastlineGeoJson(const std::string& path, CoastlinePriorParams params);`

- [ ] **Step 1: Write failing tests.** Create `tests/land/test_geojson_coastline.cpp`. Inline a tiny FeatureCollection + a smoke test on the real fixture (guarded if absent).

```cpp
#include <gtest/gtest.h>
#include <fstream>
#include "adapters/land/GeoJsonCoastline.hpp"

using navtracker::CoastlinePriorParams;
using navtracker::parseCoastlineGeoJson;
using navtracker::loadCoastlineGeoJson;

namespace {
const char* kTiny = R"({
  "type":"FeatureCollection",
  "features":[
    {"type":"Feature","properties":{},
     "geometry":{"type":"Polygon","coordinates":[
       [[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]}}
  ]})";
}

TEST(GeoJsonCoastline, ParsesPolygonAndPriorIsOneInside) {
  auto g = parseCoastlineGeoJson(kTiny, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);  // inside
  EXPECT_NEAR(g.priorAtGeodetic(42.300, -71.200), 0.0, 1e-6);  // far water
}

TEST(GeoJsonCoastline, ParsesMultiPolygon) {
  const char* mp = R"({"type":"FeatureCollection","features":[
    {"type":"Feature","properties":{},"geometry":{"type":"MultiPolygon","coordinates":[
      [[[-71.06,42.36],[-71.04,42.36],[-71.04,42.38],[-71.06,42.38],[-71.06,42.36]]]]}}]})";
  auto g = parseCoastlineGeoJson(mp, CoastlinePriorParams{});
  EXPECT_NEAR(g.priorAtGeodetic(42.370, -71.050), 1.0, 1e-6);
}

TEST(GeoJsonCoastline, BostonFixtureSmoke) {
  const std::string path = "tests/fixtures/philos/boston.geojson";
  std::ifstream f(path);
  if (!f.good()) GTEST_SKIP() << "boston.geojson fixture not present";
  auto g = loadCoastlineGeoJson(path, CoastlinePriorParams{});
  EXPECT_FALSE(g.empty());
  // Charlestown Navy Yard area = land; mid-harbour = water.
  EXPECT_GT(g.priorAtGeodetic(42.3730, -71.0535), 0.5);
  EXPECT_NEAR(g.priorAtGeodetic(42.340, -71.000), 0.0, 1e-6);
}
```

- [ ] **Step 2: Run to verify it fails.** Run: `cmake --build build -j 2>&1 | tail -5` → FAIL (no `GeoJsonCoastline.hpp`).

- [ ] **Step 3: Implement.** Create `adapters/land/GeoJsonCoastline.hpp`:

```cpp
#pragma once

#include <string>
#include "core/land/CoastlineGeometry.hpp"

namespace navtracker {
CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text,
                                        CoastlinePriorParams params);
CoastlineGeometry loadCoastlineGeoJson(const std::string& path,
                                       CoastlinePriorParams params);
}  // namespace navtracker
```
Create `adapters/land/GeoJsonCoastline.cpp` (follow `adapters/replay/AutoferryJsonReplay.cpp` for the `nlohmann::json` include + usage pattern):

```cpp
#include "adapters/land/GeoJsonCoastline.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace navtracker {
namespace {

std::vector<Eigen::Vector2d> ring(const nlohmann::json& coords) {
  std::vector<Eigen::Vector2d> r;
  r.reserve(coords.size());
  for (const auto& pt : coords) {
    if (pt.is_array() && pt.size() >= 2)
      r.emplace_back(pt[0].get<double>(), pt[1].get<double>());  // [lon,lat]
  }
  return r;
}

void addPolygon(const nlohmann::json& poly_coords, std::vector<LandPolygon>& out) {
  if (!poly_coords.is_array() || poly_coords.empty()) return;
  LandPolygon p;
  p.outer = ring(poly_coords[0]);
  for (std::size_t i = 1; i < poly_coords.size(); ++i)
    p.holes.push_back(ring(poly_coords[i]));
  if (p.outer.size() >= 3) out.push_back(std::move(p));
}

}  // namespace

CoastlineGeometry parseCoastlineGeoJson(const std::string& json_text,
                                        CoastlinePriorParams params) {
  const auto j = nlohmann::json::parse(json_text);
  std::vector<LandPolygon> polys;
  const auto& feats = j.contains("features") ? j["features"] : nlohmann::json::array();
  for (const auto& f : feats) {
    if (!f.contains("geometry") || f["geometry"].is_null()) continue;
    const auto& g = f["geometry"];
    const std::string type = g.value("type", "");
    if (type == "Polygon") {
      addPolygon(g["coordinates"], polys);
    } else if (type == "MultiPolygon") {
      for (const auto& poly : g["coordinates"]) addPolygon(poly, polys);
    }
  }
  return CoastlineGeometry(std::move(polys), params);
}

CoastlineGeometry loadCoastlineGeoJson(const std::string& path,
                                       CoastlinePriorParams params) {
  std::ifstream f(path);
  if (!f.good()) throw std::runtime_error("coastline geojson not found: " + path);
  std::stringstream ss; ss << f.rdbuf();
  return parseCoastlineGeoJson(ss.str(), params);
}

}  // namespace navtracker
```

- [ ] **Step 4: Register in CMake.** Add `adapters/land/GeoJsonCoastline.cpp` to the adapters/I-O target that already links `nlohmann_json` (the one compiling `adapters/replay/AutoferryJsonReplay.cpp` — find it in `CMakeLists.txt`), and `tests/land/test_geojson_coastline.cpp` to the test target (it must link that adapters target).

- [ ] **Step 5: Build & run; confirm PASS.**
Run: `cmake --build build -j && ctest --test-dir build -R GeoJsonCoastline --output-on-failure` → 3 PASS (Boston smoke passes since the fixture is present).

- [ ] **Step 6: Commit.**
```bash
git add adapters/land/ tests/land/test_geojson_coastline.cpp CMakeLists.txt
git commit -m "Task A Step 4: GeoJSON coastline adapter (nlohmann)"
```

---

## Task 5: PMBM birth integration

**Files:** Modify `core/pmbm/PmbmTracker.hpp` (setter, member, Config, helper), `core/pmbm/PmbmTracker.cpp` (both candidate builders); Test: `tests/pmbm/test_pmbm_land_model.cpp`; Modify `CMakeLists.txt`.

**Interfaces:**
- Consumes: `ILandModel` (Task 1).
- Produces: `PmbmTracker::setLandModel(const ILandModel*)`; Config `bool use_land_model=false`, `double land_birth_hard_gate=0.95`.

**Current code:** adaptive births in `buildAdaptiveBirthCandidates` set `cand.mean = t.state` (ENU birth state, `:447`), compute `lambda_birth` (`:462-475`), then `cand.rho_target = lambda_birth; cand.rho_total = lambda_birth + lambda_z` (`:476-477`). `r_new = rho_target/rho_total` is consumed at `:935`. The measurement-driven builder `buildNewTargetCandidates` sets `cand.mean` (`:300`) and `rho_target`/`rho_total` (`:279-280`).

**Mechanism (decision §9a):** scale the birth intensity `lambda_birth`/`rho_target` by `(1 − c)` and hard-drop (set to 0) when `c > land_birth_hard_gate`. Scaling the intensity is the single-site, downstream-consistent realization of the spec's `r_new *= (1−c)` (identical to first order for the small `r_new` of the philos regime; both `r_new` and the hypothesis weight follow `rho_target` naturally). Birth position = `cand.mean.head<2>()` (ENU).

- [ ] **Step 1: Write failing tests.** Create `tests/pmbm/test_pmbm_land_model.cpp`. Use a fake `ILandModel` returning a fixed prior by region, and the existing PMBM test-fixture style (see `tests/pmbm/test_pmbm_tracker_update.cpp`).

```cpp
#include <gtest/gtest.h>
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ILandModel.hpp"

using navtracker::EkfEstimator;
using navtracker::PmbmTracker;
using navtracker::Measurement;

namespace {
// prior = 1.0 east of x=500 (land), 0.0 west (water).
struct HalfPlaneLand : navtracker::ILandModel {
  double clutterPrior(const Eigen::Vector2d& p) const override {
    return p.x() > 500.0 ? 1.0 : 0.0;
  }
};
struct SoftLand : navtracker::ILandModel {       // constant mid prior
  double clutterPrior(const Eigen::Vector2d&) const override { return 0.5; }
};
Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = navtracker::SensorKind::ArpaTtm;
  m.model = navtracker::MeasurementModel::Position2D;
  m.time = navtracker::Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}
PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;   // philos winner; r_new independent of lambda_C
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  return c;
}
}  // namespace

TEST(PmbmLandModel, HardGateDropsBirthOnLand) {
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker tracker(ekf, cfg());
  HalfPlaneLand land; tracker.setLandModel(&land);
  PmbmTracker::Config c = cfg(); c.use_land_model = true;  // (re-make tracker with flag)
  PmbmTracker t2(ekf, c); t2.setLandModel(&land);
  t2.predict(navtracker::Timestamp::fromSeconds(0.0));
  t2.processBatch({posMeas(1000.0, 0.0, 0.0)});   // on land -> dropped
  // No confirmed/created Bernoulli with meaningful existence at that position.
  double maxr = 0.0;
  for (const auto& h : t2.density().mbm)
    for (const auto& b : h.bernoullis) maxr = std::max(maxr, b.existence_probability);
  EXPECT_LT(maxr, 1e-6);
}

TEST(PmbmLandModel, OpenWaterBirthUnchanged) {
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg(); c.use_land_model = true;
  PmbmTracker t(ekf, c); HalfPlaneLand land; t.setLandModel(&land);
  t.predict(navtracker::Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(0.0, 0.0, 0.0)});       // water -> normal birth ~0.1
  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis) maxr = std::max(maxr, b.existence_probability);
  EXPECT_NEAR(maxr, 0.1, 0.05);
}

TEST(PmbmLandModel, SoftPriorHalvesBirth) {
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg(); c.use_land_model = true;
  PmbmTracker t(ekf, c); SoftLand land; t.setLandModel(&land);
  t.predict(navtracker::Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(0.0, 0.0, 0.0)});       // prior 0.5
  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis) maxr = std::max(maxr, b.existence_probability);
  EXPECT_LT(maxr, 0.1);   // suppressed below the 0.1 open-water birth
  EXPECT_GT(maxr, 0.0);   // but not dropped
}

TEST(PmbmLandModel, NullLandModelBitIdentical) {
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();                  // use_land_model defaults false
  PmbmTracker t(ekf, c);                          // no setLandModel
  t.predict(navtracker::Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(1000.0, 0.0, 0.0)});
  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis) maxr = std::max(maxr, b.existence_probability);
  EXPECT_NEAR(maxr, 0.1, 0.05);                   // birth happens normally
}
```
(Adjust the existence-readout accessor to whatever the existing tests use — e.g. `tracker.density().mbm` as in `test_pmbm_tracker_update.cpp`. If the smart-birth gate interferes, leave `smart_birth_skip_existing` at its default off.)

- [ ] **Step 2: Run to verify it fails.** Run: `cmake --build build -j 2>&1 | tail -5` → FAIL (no `setLandModel` / `use_land_model`).

- [ ] **Step 3: Add setter, member, Config, helper (`PmbmTracker.hpp`).** Next to `setSensorActivity`:
```cpp
  void setLandModel(const ILandModel* m) { land_model_ = m; }
```
Member next to `sensor_activity_`:
```cpp
  const ILandModel* land_model_{nullptr};
```
Include `#include "ports/ILandModel.hpp"`. In `Config`, near `use_sensor_activity`:
```cpp
  // Task A: when true AND a land model is wired, scale adaptive-birth intensity
  // by (1 - clutterPrior(birth_pos)); drop the birth entirely when the prior
  // exceeds land_birth_hard_gate (inland plateau). Default off => bit-identical.
  bool use_land_model = false;
  double land_birth_hard_gate = 0.95;
```
Private helper:
```cpp
  // Returns the birth-intensity scale in [0,1] for a birth at ENU `pos`, or a
  // negative value meaning "hard-drop this birth". 1.0 when no land model.
  double landBirthScale(const Eigen::VectorXd& mean) const {
    if (!cfg_.use_land_model || land_model_ == nullptr || mean.size() < 2)
      return 1.0;
    const double c = land_model_->clutterPrior(mean.head<2>());
    if (c > cfg_.land_birth_hard_gate) return -1.0;   // hard-drop
    return 1.0 - c;                                    // soft scale
  }
```

- [ ] **Step 4: Apply in both candidate builders (`PmbmTracker.cpp`).** In `buildAdaptiveBirthCandidates`, replace the `cand.rho_target = lambda_birth; cand.rho_total = lambda_birth + lambda_z;` block (`:476-477`) with:
```cpp
    const double land_scale = landBirthScale(cand.mean);
    if (land_scale < 0.0) {            // inland hard gate -> no birth
      lambda_birth = 0.0;
    } else {
      lambda_birth *= land_scale;      // soft suppression
    }
    cand.rho_target = lambda_birth;
    cand.rho_total = lambda_birth + lambda_z;
```
In `buildNewTargetCandidates`, after `cand.mean` is finalized and `cand.rho_target`/`cand.rho_total` computed (`:279-301`), apply the same scaling to `cand.rho_target` and recompute `cand.rho_total = cand.rho_target + lambda_z;` (use the `lambda_z` already in scope). Guard so a hard-drop sets `cand.rho_target = 0.0`. Keep behaviour identical when `landBirthScale` returns 1.0.

- [ ] **Step 5: Register test, build & run; confirm PASS + suite green.**
Add `tests/pmbm/test_pmbm_land_model.cpp` to CMake. Run:
`cmake --build build -j && ctest --test-dir build -R "PmbmLandModel|Pmbm" --output-on-failure` → new tests PASS; existing PMBM suite unchanged. Then full `ctest --test-dir build` → all green.

- [ ] **Step 6: Commit.**
```bash
git add core/pmbm/PmbmTracker.hpp core/pmbm/PmbmTracker.cpp tests/pmbm/test_pmbm_land_model.cpp CMakeLists.txt
git commit -m "Task A Step 5: PMBM birth suppression via land clutter prior (soft + inland hard gate)"
```

---

## Task 6: Philos bench wiring + A/B (controller-run bench)

**Files:** Modify `core/benchmark/Config.cpp` (new config), `core/benchmark/Sweep.cpp` (wire the land model for the land config). Output CSV under `docs/baselines/`.

**Interfaces:**
- Consumes: `CoastlineModel`, `loadCoastlineGeoJson`, `PmbmTracker::setLandModel`, `OwnShipProvider::registerDatumSink`.

- [ ] **Step 1: Add the bench config.** In `core/benchmark/Config.cpp`, clone `imm_cv_ct_pmbm_coverage` as `imm_cv_ct_pmbm_coverage_land`: same pmbm_config plus `cfg.use_land_model = true;` (keep `land_birth_hard_gate` default). Add a flag on the bench `Config` struct, `bool use_land_model = false;`, set true for this config (mirrors `use_sensor_activity_model`). Update the config-count test.

- [ ] **Step 2: Wire the land model in `Sweep.cpp`.** In the PMBM branch, when `config.use_land_model` and the scenario is a replay with a known coastline fixture (philos), build:
```cpp
auto geom = loadCoastlineGeoJson("tests/fixtures/philos/boston.geojson",
                                 CoastlinePriorParams{});
auto land = std::make_shared<CoastlineModel>(std::move(geom), provider.datum());
provider.registerDatumSink(land.get());   // recenter-aware
tracker.setLandModel(land.get());
```
Keep `land` alive for the run (same lifetime handling as the activity provider). Only wire when the fixture exists; otherwise skip (no land model → coverage-only behaviour). Hexagonal: this is bench/adapter glue.

- [ ] **Step 3: Build.** Run: `cmake --build build -j --target navtracker_bench_baseline` → links.

- [ ] **Step 4: Run the philos A/B (controller).**
```bash
./build/bench/navtracker_bench_baseline --run-id 2026-06-30_philos_land_ab \
  --out docs/baselines/ --config-filter imm_cv_ct_pmbm --scenario-filter philos --seeds 1
```
Compare `imm_cv_ct_pmbm_coverage_land` vs `imm_cv_ct_pmbm_coverage`, `_bundle`, `_birthtarget` on `gospa_mean`, `card_err_mean`, `gospa_false`, `id_switches`. **Success = card_err / gospa_false collapse toward (or below) the birthtarget baseline** (the over-count is removed).

- [ ] **Step 5: Autoferry guard (controller).**
```bash
./build/bench/navtracker_bench_baseline --run-id 2026-06-30_autoferry_land_guard \
  --out docs/baselines/ --config-filter imm_cv_ct_pmbm --scenario-filter autoferry_scenario2 --seeds 1
```
Confirm `_coverage_land` does not regress autoferry vs `_coverage` (autoferry has no land fixture, so the land model is inert there — expect ≈ identical; verify).

- [ ] **Step 6: Record + commit.** Write the A/B tables + decision into `docs/algorithms/evaluation-log.md` and the Cl-3 row in `comparison-baselines.md`. Commit config, Sweep wiring, and the CSVs.
```bash
git add core/benchmark/Config.cpp core/benchmark/Sweep.cpp tests/benchmark/test_config.cpp \
  docs/algorithms/evaluation-log.md docs/algorithms/comparison-baselines.md docs/baselines/2026-06-30_*_ab.csv docs/baselines/2026-06-30_*_guard.csv
git commit -m "Task A Step 6: philos land-model A/B + decision"
```

---

## Task 7: Documentation

**Files:** Modify `docs/algorithms/pmbm-design.md`; create/extend `docs/learning/` chapter + `00-index.md` + glossary.

- [ ] **Step 1: Four-section algorithm doc.** Add a section to `pmbm-design.md` covering: Math (signed-ramp prior `c(d)`, birth-intensity scaling, inland-only hard gate, λ_C-decoupling rationale), Assumptions (consumer GeoJSON, soft band absorbs coarse waterline, deterministic async swap), Rationale (philos is spatial clutter — cite the 86%-on/near-land pre-check and the 185 phantom births; why births not λ_C; geodetic storage for recenter), Ways-to-improve (coverage-occlusion, on-land plausibility gating, online clutter learning, finer charts). Use the measured A/B numbers from Task 6.

- [ ] **Step 2: Learning chapter.** Add a plain-English chapter ("why we suppress tracks on land: the coastline clutter prior") with a diagram (the shoreline ramp: water 0 → waterline 0.5 → inland 1.0; and the GeoJSON→prior→birth-gate flow). Update `docs/learning/00-index.md` and the glossary (clutter prior, shoreline ramp, land mask). Cross-link from `pmbm-design.md`.

- [ ] **Step 3: Commit.**
```bash
git add docs/algorithms/pmbm-design.md docs/learning/
git commit -m "Task A Step 7: land clutter-prior algorithm doc + learning chapter"
```

---

## Self-Review notes

- **Spec coverage:** port §2→T1; ramp prior §6/§8→T2; datum-recenter + query §4→T3; GeoJSON + async swap §5→T4; birth injection (soft + inland hard gate, not λ_C) §3→T5; philos validation §10/§11→T6; docs §8→T7. Decisions §9(a-e) all mapped.
- **Type consistency:** `clutterPrior(const Eigen::Vector2d&)→double` everywhere; `LandPolygon`/`CoastlinePriorParams` shared T2→T4; `landBirthScale` returns `(1−c)` or `<0` for hard-drop; Config `use_land_model`/`land_birth_hard_gate` consistent T5/T6.
- **Determinism/back-compat:** nullable port + default-off flag; null path bit-identical (T5 `NullLandModelBitIdentical`); pure query, no wall-clock/RNG; async swap is consumer-ordered.
- **Known minor deviation from spec literal:** spec §3 wrote `r_new *= (1−c)`; implementation scales the birth *intensity* `rho_target` by `(1−c)` (single site, downstream-weight-consistent; equal to first order for small `r_new`). Documented in Task 5 and to be noted in the algorithm doc.
- **Open risk (spec §11):** coastline is coarse/administrative — soft waterline mitigates; anchored-against-shore non-AIS vessel inside the polygon is the accepted residual risk.
