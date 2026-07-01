# Static-Obstacle Branch — Stage 1a (Charted Input) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add charted static hazards (rocks, wrecks, pillars, dolphins, buoys) to navtracker as (a) a **vessel-birth suppression prior** — a phantom track cannot be born on a charted obstacle — and (b) an explicit **static-hazard output** (pull snapshot + push proximity alarm). Decision of record: ADR 0002; design sketch: design spec §14.10.

**Architecture:** A new estimator-adjacent branch that reuses the exact pattern the land model already proved. A pure `IStaticObstacleModel` port (separate from `ILandModel`, ADR 0002 decision 4) is queried at PMBM birth time via a combined `birthScale`; a `StaticHazardOutput` + `StaticHazardEvaluator` + `IStaticHazardSink` surface the hazards. A GeoJSON adapter loads charted obstacles (ENC / AIS AtoN Message 21 ingest later). Everything is **safe by construction**: model unwired or flag off → bit-identical to today.

**Tech Stack:** C++17, CMake, Conan, GoogleTest/GMock, Eigen, nlohmann/json (already a dep, used by `adapters/land/GeoJsonCoastline.cpp`).

**Scope boundary (explicit):** This plan is Stage **1a only** — the *charted* input. Stage **1b** (wiring PMBM to feed the clutter-map primitive via dominant-hypothesis `1−r` labeling and emitting *live* occupancy as a hazard) is deliberately **out of scope** here: it touches the PMBM association hot path and overlaps the Stage 2 occupancy-grid decision. Do not implement 1b in this plan.

## Global Constraints

- **Hexagonal invariant:** the domain core (`core/`) has zero I/O and zero sensor-format knowledge. Ports are interfaces (`ports/`). Concrete file parsing lives in `adapters/`. Source dependencies point inward.
- **Safe by construction:** with `use_static_obstacle_model=false` (default) OR no model wired, every code path must be **bit-identical** to current behaviour. This is a hard requirement, verified by an explicit null test in Task 2 and by the existing `test_pmbm_land_model.cpp` continuing to pass unchanged.
- **Never hard-gate a position a vessel could occupy.** Hard suppression (birth drop) fires ONLY inside an obstacle's physical footprint core. The keep-clear buffer is **soft-only** — a real vessel passing close still births. This is the anchored-vessel trap (ADR 0001/0002); the `soft_max < static_obstacle_hard_gate` invariant enforces it.
- **Determinism:** replay of a log twice → identical output. No wall-clock, no RNG in the new core code. Datum recenter handling must be deterministic (rebuild the ENU cache).
- **Units/frames:** ENU metres internally about the working datum; obstacle charted positions are geodetic (WGS84 degrees). Convert only at the model boundary. Distances in metres.
- **Static obstacles are SEPARATE from the coastline** (ADR 0002 decision 4): a distinct type, port, and data path — never folded into the land polygon.
- **Documentation standard (CLAUDE.md):** the new algorithmic component needs the four-part algorithm doc (math / assumptions / rationale / what-to-test) AND a `docs/learning/` chapter + PNG figure. Enforced in Task 7.
- **Combination semantics:** land prior and static-obstacle prior combine **multiplicatively** at the birth seam: `scale = (1 − c_land)·(1 − c_static)`; hard-drop if EITHER prior exceeds its own hard gate.

---

## File Structure

**New files:**
- `core/types/StaticObstacle.hpp` — the charted-hazard value type (ENC/S-101-aligned attributes). Header-only.
- `ports/IStaticObstacleModel.hpp` — pure query port (`birthSuppression` + `obstacles`). Header-only.
- `core/static/StaticObstacleModel.hpp` — concrete model (geodetic store + datum + ENU cache). Header-only (mirrors `CoastlineModel.hpp`).
- `ports/IStaticHazardSink.hpp` — push-based proximity observer. Header-only.
- `core/output/StaticHazardOutput.hpp` / `.cpp` — pull-style output struct + `toStaticHazardOutput` + `staticHazardId`.
- `core/collision/StaticHazardEvaluator.hpp` / `.cpp` — own-ship × obstacle keep-clear alarm with hysteresis.
- `adapters/static/GeoJsonStaticObstacles.hpp` / `.cpp` — GeoJSON → `std::vector<StaticObstacle>`.
- `tests/static/test_static_obstacle_model.cpp`
- `tests/pmbm/test_pmbm_static_obstacle.cpp`
- `tests/output/test_static_hazard_output.cpp`
- `tests/collision/test_static_hazard_evaluator.cpp`
- `tests/adapters/static/test_geojson_static_obstacles.cpp`
- `tests/fixtures/static/harbor_obstacles.geojson`
- `tests/integration/test_static_obstacle_pipeline.cpp`
- `docs/learning/26-static-obstacles.md` + a figure in `docs/learning/figures/generate.py`
- `docs/algorithms/static-obstacle-birth-prior.md`

**Modified files:**
- `core/pmbm/PmbmTracker.hpp` — config fields, member, setter, `landBirthScale`→`birthScale` (extended).
- `core/pmbm/PmbmTracker.cpp` — two birth call sites use `birthScale`.
- `core/benchmark/Config.hpp` — `use_static_obstacle_model` flag.
- `core/benchmark/Config.cpp` — new config `imm_cv_ct_pmbm_static`.
- `core/benchmark/ScenarioRun.hpp` — `static_obstacles_geojson_path` + `syntheticObstacles()` hook.
- `core/benchmark/Sweep.cpp` — wire the static-obstacle model (mirror the land block).
- `tests/benchmark/test_config.cpp` — config count 30→31 + label check.
- `CMakeLists.txt` — register new `.cpp` files and test files.
- `docs/output-contract.md`, `docs/adr/0002-...md`, design spec §14.10, `docs/algorithms/comparison-baselines.md`, `docs/algorithms/evaluation-log.md` — Task 7.

**Build/test commands** (use the repo's existing build dir — call it `build/`):
- Build tests: `cmake --build build -j --target navtracker_tests`
- Run a subset: `./build/navtracker_tests --gtest_filter='StaticObstacleModel.*'`
- Full suite: `ctest --test-dir build --output-on-failure` (or `./build/navtracker_tests`)
- (Reconfigure only if needed and only outside the sandbox: `conan install` / cmake configure fail inside the sandbox — see memory `reference-build-env`. Plain `cmake --build` / `ctest` work in-sandbox.)

---

### Task 1: `StaticObstacle` type + `IStaticObstacleModel` port + `StaticObstacleModel`

**Files:**
- Create: `core/types/StaticObstacle.hpp`
- Create: `ports/IStaticObstacleModel.hpp`
- Create: `core/static/StaticObstacleModel.hpp`
- Test: `tests/static/test_static_obstacle_model.cpp`
- Modify: `CMakeLists.txt` (add the test file to the `navtracker_tests` sources list)

**Interfaces:**
- Produces: `navtracker::StaticObstacle` (value type); `navtracker::IStaticObstacleModel` with `double birthSuppression(const Eigen::Vector2d&) const` and `const std::vector<StaticObstacle>& obstacles() const`; `navtracker::StaticObstacleModel(std::vector<StaticObstacle>, geo::Datum, StaticObstacleParams={})` implementing both `IStaticObstacleModel` and `IDatumChangeSink`.
- Consumes: `geo::Datum` (`core/geo/Datum.hpp`: `toEnu(Geodetic)→Vector3d`), `geo::Geodetic` (`core/geo/Wgs84.hpp`), `IDatumChangeSink` (`core/own_ship/OwnShipProvider.hpp`).

- [ ] **Step 1: Write the failing test** — `tests/static/test_static_obstacle_model.cpp`

```cpp
#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::AtoNRealism;
using navtracker::ObstacleCategory;
using navtracker::StaticObstacle;
using navtracker::StaticObstacleModel;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {

// A datum near Boston Harbor; the exact origin is irrelevant to the ramp math.
Datum datum() { return Datum(Geodetic{42.35, -71.05, 0.0}); }

// One obstacle placed at the datum origin (ENU ~ (0,0)), 15 m hard footprint,
// 100 m soft keep-clear buffer.
StaticObstacle originObstacle() {
  StaticObstacle o;
  o.position = Geodetic{42.35, -71.05, 0.0};
  o.footprint_radius_m = 15.0;
  o.keep_clear_radius_m = 100.0;
  o.category = ObstacleCategory::Pile;
  return o;
}

}  // namespace

// Inside the hard footprint → suppression 1.0 (hard-gate region).
TEST(StaticObstacleModel, InsideFootprintFullySuppressed) {
  StaticObstacleModel m({originObstacle()}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(5.0, 0.0)), 1.0, 1e-9);
}

// Just outside the footprint → soft, and STRICTLY below the 0.95 hard gate
// (soft_max default 0.9), so the keep-clear buffer never hard-drops a vessel.
TEST(StaticObstacleModel, JustOutsideFootprintIsSoftBelowGate) {
  StaticObstacleModel m({originObstacle()}, datum());
  const double c = m.birthSuppression(Eigen::Vector2d(16.0, 0.0));
  EXPECT_GT(c, 0.0);
  EXPECT_LT(c, 0.95);
}

// Mid-buffer ramps down; farther is less suppressed than nearer.
TEST(StaticObstacleModel, BufferRampsDownWithDistance) {
  StaticObstacleModel m({originObstacle()}, datum());
  const double near = m.birthSuppression(Eigen::Vector2d(40.0, 0.0));
  const double far = m.birthSuppression(Eigen::Vector2d(80.0, 0.0));
  EXPECT_GT(near, far);
  EXPECT_GT(far, 0.0);
}

// Beyond the keep-clear radius → clear water, no suppression.
TEST(StaticObstacleModel, BeyondKeepClearNoSuppression) {
  StaticObstacleModel m({originObstacle()}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(150.0, 0.0)), 0.0, 1e-9);
}

// No obstacles → always 0.0.
TEST(StaticObstacleModel, EmptyModelNeverSuppresses) {
  StaticObstacleModel m({}, datum());
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_TRUE(m.obstacles().empty());
}

// obstacles() returns the charted list verbatim.
TEST(StaticObstacleModel, ObstaclesAccessor) {
  StaticObstacleModel m({originObstacle()}, datum());
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_EQ(m.obstacles()[0].category, ObstacleCategory::Pile);
  EXPECT_DOUBLE_EQ(m.obstacles()[0].keep_clear_radius_m, 100.0);
}

// After a datum recenter, the obstacle's ENU cache is rebuilt so a query at
// the obstacle's true position is still fully suppressed. Use a datum shifted
// by a large offset; the obstacle's geodetic position is unchanged, so its ENU
// coordinate changes and the model must follow.
TEST(StaticObstacleModel, DatumRecenterRebuildsEnu) {
  StaticObstacleModel m({originObstacle()}, datum());
  // Query at the obstacle's ENU under the OLD datum is inside footprint.
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 1.0, 1e-9);

  Datum shifted(Geodetic{42.40, -71.05, 0.0});  // ~5.5 km north
  m.onDatumRecentered(datum(), shifted);
  // Under the NEW datum, the obstacle sits at ~(0, -5560) m; a query at (0,0)
  // is now far away (no suppression), while a query at the obstacle's new ENU
  // is fully suppressed.
  const Eigen::Vector3d obs_enu =
      shifted.toEnu(Geodetic{42.35, -71.05, 0.0});
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(m.birthSuppression(Eigen::Vector2d(obs_enu.x(), obs_enu.y())),
              1.0, 1e-9);
}
```

- [ ] **Step 2: Register the test in CMake and run to verify it fails**

Add `tests/static/test_static_obstacle_model.cpp` to the `navtracker_tests` source list in `CMakeLists.txt` (near the other `tests/pmbm/*` entries, e.g. after line 220). Run:
`cmake --build build -j --target navtracker_tests`
Expected: FAIL to compile (`core/static/StaticObstacleModel.hpp` / `core/types/StaticObstacle.hpp` not found).

- [ ] **Step 3: Create `core/types/StaticObstacle.hpp`**

```cpp
#pragma once

#include <limits>
#include <string>

#include "core/geo/Wgs84.hpp"  // geo::Geodetic

namespace navtracker {

// S-57/S-101-aligned category subset for charted hazards (CATOBS-ish).
enum class ObstacleCategory {
  Unknown, Rock, Wreck, Obstruction, Pile, Platform, Buoy, Beacon, Other
};

// S-57 WATLEV (water-level effect).
enum class WaterLevel {
  Unknown, AwashCoversUncovers, AlwaysSubmerged, AlwaysAboveWater, Floating
};

// AIS AtoN realism (Message 21): physical vs synthetic vs virtual.
enum class AtoNRealism { NotAtoN, Real, Synthetic, Virtual };

// A discrete charted static hazard: rock, wreck, pillar, dolphin, buoy, ...
// NOT coastline (ADR 0002 decision 4). Position is geodetic (chart frame).
// footprint_radius_m + position_uncertainty_m form the hard no-birth core;
// keep_clear_radius_m is the soft buffer and the operator keep-clear ring.
struct StaticObstacle {
  geo::Geodetic position;              // charted position (WGS84)
  double footprint_radius_m{0.0};      // physical extent (hard core)
  double keep_clear_radius_m{0.0};     // soft keep-clear buffer (>= footprint)
  double position_uncertainty_m{0.0};  // positional buffer, added to hard core
  ObstacleCategory category{ObstacleCategory::Unknown};
  WaterLevel water_level{WaterLevel::Unknown};
  double depth_m{std::numeric_limits<double>::quiet_NaN()};  // VALSOU (NaN=unknown)
  bool lit{false};
  AtoNRealism aton{AtoNRealism::NotAtoN};
  std::string source_id;               // provenance (chart id / AtoN MMSI)
};

}  // namespace navtracker
```

- [ ] **Step 4: Create `ports/IStaticObstacleModel.hpp`**

```cpp
#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/types/StaticObstacle.hpp"

namespace navtracker {

// Charted static-hazard model, queried by the tracker at birth time and by
// the hazard-output layer. Pure, zero-I/O. Nullable in use: if not wired,
// behaviour is exactly today's. Separate from ILandModel (ADR 0002 decision
// 4): discrete typed hazards, not a coastline suppression region.
class IStaticObstacleModel {
 public:
  virtual ~IStaticObstacleModel() = default;

  // Vessel-birth suppression prior in [0,1] at ENU position (metres):
  //   0.0 = clear water (no suppression)
  //   1.0 = inside a charted obstacle's hard footprint (hard-gate region)
  // No nearby obstacle -> 0.0.
  virtual double birthSuppression(const Eigen::Vector2d& enu_xy) const = 0;

  // The active charted obstacles (for the hazard output + proximity alarm).
  virtual const std::vector<StaticObstacle>& obstacles() const = 0;
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/static/StaticObstacleModel.hpp`**

```cpp
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "core/types/StaticObstacle.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

struct StaticObstacleParams {
  // Max soft suppression at the outer edge of the footprint. MUST be strictly
  // below the tracker's static_obstacle_hard_gate (default 0.95) so the
  // keep-clear buffer is soft-only: a real vessel passing close still births.
  // Only the hard footprint interior (c=1.0) triggers the hard gate.
  double soft_max = 0.9;
};

// Concrete static-obstacle model: holds charted obstacles (geodetic) + the
// working datum, precomputing their ENU positions for fast proximity queries.
// birthSuppression is a soft ramp c(d) over distance d to the nearest obstacle:
//   d <= R_hard            -> 1.0                              (footprint core)
//   R_hard < d <= R_soft   -> soft_max*(R_soft-d)/(R_soft-R_hard)  (buffer ramp)
//   d > R_soft             -> 0.0                              (clear water)
// with R_hard = footprint_radius_m + position_uncertainty_m and
//      R_soft  = max(keep_clear_radius_m, R_hard).
// Pure: no I/O, no wall-clock, no RNG. Mirrors CoastlineModel. On datum
// recenter the ENU cache is rebuilt (deterministic).
class StaticObstacleModel : public IStaticObstacleModel, public IDatumChangeSink {
 public:
  StaticObstacleModel(std::vector<StaticObstacle> obstacles, geo::Datum datum,
                      StaticObstacleParams params = {})
      : obstacles_(std::move(obstacles)), datum_(datum), params_(params) {
    rebuildEnu();
  }

  double birthSuppression(const Eigen::Vector2d& enu_xy) const override {
    double c = 0.0;
    for (std::size_t i = 0; i < enu_.size(); ++i) {
      const double d = (enu_[i] - enu_xy).norm();
      const double r_hard =
          obstacles_[i].footprint_radius_m + obstacles_[i].position_uncertainty_m;
      const double r_soft = std::max(obstacles_[i].keep_clear_radius_m, r_hard);
      double ci;
      if (d <= r_hard) {
        ci = 1.0;
      } else if (d <= r_soft && r_soft > r_hard) {
        ci = params_.soft_max * (r_soft - d) / (r_soft - r_hard);
      } else {
        ci = 0.0;
      }
      c = std::max(c, ci);
    }
    return c;
  }

  const std::vector<StaticObstacle>& obstacles() const override {
    return obstacles_;
  }

  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    datum_ = new_datum;
    rebuildEnu();
  }

 private:
  void rebuildEnu() {
    enu_.clear();
    enu_.reserve(obstacles_.size());
    for (const auto& o : obstacles_) {
      const Eigen::Vector3d e = datum_.toEnu(o.position);
      enu_.emplace_back(e.x(), e.y());
    }
  }

  std::vector<StaticObstacle> obstacles_;
  geo::Datum datum_;
  StaticObstacleParams params_;
  std::vector<Eigen::Vector2d> enu_;
};

}  // namespace navtracker
```

- [ ] **Step 6: Build and run to verify the test passes**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='StaticObstacleModel.*'`
Expected: PASS (7/7).

- [ ] **Step 7: Commit**

```bash
git add core/types/StaticObstacle.hpp ports/IStaticObstacleModel.hpp \
        core/static/StaticObstacleModel.hpp \
        tests/static/test_static_obstacle_model.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: StaticObstacle type + IStaticObstacleModel port + model"
```

---

### Task 2: PMBM birth-prior wiring (`birthScale`)

**Files:**
- Modify: `core/pmbm/PmbmTracker.hpp` (add include; config fields ~after line 393; setter ~after line 594; member ~after line 721; rename+extend `landBirthScale`→`birthScale` at lines 782-788)
- Modify: `core/pmbm/PmbmTracker.cpp` (two call sites: ~line 388 and ~line 495)
- Test: `tests/pmbm/test_pmbm_static_obstacle.cpp`
- Modify: `CMakeLists.txt` (add the test)

**Interfaces:**
- Consumes: `IStaticObstacleModel` (Task 1); the existing land seam.
- Produces: `PmbmTracker::setStaticObstacleModel(const IStaticObstacleModel*)`; `PmbmTracker::Config::use_static_obstacle_model` (bool, default false), `::static_obstacle_hard_gate` (double, default 0.95).
- **Invariant:** with `use_static_obstacle_model=false` (default) or no model wired, `birthScale` returns exactly what `landBirthScale` returned. The existing `tests/pmbm/test_pmbm_land_model.cpp` MUST pass unchanged.

- [ ] **Step 1: Write the failing test** — `tests/pmbm/test_pmbm_static_obstacle.cpp`

```cpp
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::ObstacleCategory;
using navtracker::SensorKind;
using navtracker::StaticObstacle;
using navtracker::StaticObstacleModel;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;
using navtracker::pmbm::PmbmTracker;

namespace {

Datum datum() { return Datum(Geodetic{42.35, -71.05, 0.0}); }

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  return c;
}

// Obstacle at ENU origin: 15 m hard footprint, 100 m soft keep-clear.
StaticObstacleModel originObstacleModel() {
  StaticObstacle o;
  o.position = Geodetic{42.35, -71.05, 0.0};
  o.footprint_radius_m = 15.0;
  o.keep_clear_radius_m = 100.0;
  o.category = ObstacleCategory::Pile;
  return StaticObstacleModel({o}, datum());
}

double maxExistence(const PmbmTracker& t) {
  double maxr = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      maxr = std::max(maxr, b.existence_probability);
  return maxr;
}

}  // namespace

// A would-be phantom born INSIDE the obstacle footprint is hard-dropped.
TEST(PmbmStaticObstacle, HardGateDropsBirthOnObstacle) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  StaticObstacleModel m = originObstacleModel();
  t.setStaticObstacleModel(&m);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(5.0, 0.0, 0.0)});  // inside 15 m footprint
  EXPECT_LT(maxExistence(t), 1e-6);
}

// A vessel in the soft keep-clear buffer is suppressed below the open-water
// 0.1 birth but still births (> 0) — the anchored-vessel protection.
TEST(PmbmStaticObstacle, SoftBufferSuppressesButStillBirths) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  StaticObstacleModel m = originObstacleModel();
  t.setStaticObstacleModel(&m);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(60.0, 0.0, 0.0)});  // 60 m: inside 100 m buffer
  const double r = maxExistence(t);
  EXPECT_LT(r, 0.1);
  EXPECT_GT(r, 0.0);
}

// Clear water beyond the keep-clear radius → normal ~0.1 birth.
TEST(PmbmStaticObstacle, ClearWaterBirthUnchanged) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  StaticObstacleModel m = originObstacleModel();
  t.setStaticObstacleModel(&m);
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(300.0, 0.0, 0.0)});  // 300 m: clear water
  EXPECT_NEAR(maxExistence(t), 0.1, 0.05);
}

// Null model / flag off → bit-identical: a birth ON the obstacle happens
// normally at ~0.1.
TEST(PmbmStaticObstacle, NullModelBitIdentical) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();  // use_static_obstacle_model defaults false
  PmbmTracker t(ekf, c);          // no setStaticObstacleModel
  t.predict(Timestamp::fromSeconds(0.0));
  t.processBatch({posMeas(5.0, 0.0, 0.0)});
  EXPECT_NEAR(maxExistence(t), 0.1, 0.05);
}
```

- [ ] **Step 2: Register the test and run to verify it fails**

Add `tests/pmbm/test_pmbm_static_obstacle.cpp` to `CMakeLists.txt` (after line 220). Run:
`cmake --build build -j --target navtracker_tests`
Expected: FAIL to compile (`setStaticObstacleModel` / `use_static_obstacle_model` do not exist).

- [ ] **Step 3: Add the include, config fields, member, and setter to `core/pmbm/PmbmTracker.hpp`**

At the top with the other port includes, add:
```cpp
#include "ports/IStaticObstacleModel.hpp"
```
In `struct Config`, immediately after `double land_birth_hard_gate = 0.95;` (line 393), add:
```cpp
    // Stage 1 static-obstacle branch (ADR 0002): when true AND a static-
    // obstacle model is wired via setStaticObstacleModel(), the adaptive-
    // birth intensity is additionally scaled by (1 − birthSuppression(pos)),
    // and births inside an obstacle's hard footprint (suppression >
    // static_obstacle_hard_gate) are dropped. Combines multiplicatively with
    // the land model. Default false → bit-identical to today's behaviour.
    bool use_static_obstacle_model = false;
    // Hard-gate threshold for the static-obstacle prior; symmetric with
    // land_birth_hard_gate. Only the footprint interior (suppression = 1.0)
    // exceeds the default 0.95, so the keep-clear buffer stays soft-only.
    // Must be in (0, 1]. Ignored when use_static_obstacle_model = false.
    double static_obstacle_hard_gate = 0.95;
```
Immediately after the `setLandModel` definition (line 594), add:
```cpp
  // Stage 1 (ADR 0002): optional charted static-obstacle prior. Null (default)
  // or use_static_obstacle_model == false → bit-identical to today's.
  void setStaticObstacleModel(const IStaticObstacleModel* m) {
    obstacle_model_ = m;
  }
```
Immediately after the `land_model_` member (line 721), add:
```cpp
  const IStaticObstacleModel* obstacle_model_{nullptr};
```

- [ ] **Step 4: Rename and extend `landBirthScale` → `birthScale`** in `core/pmbm/PmbmTracker.hpp` (lines 777-788)

Replace the whole `landBirthScale` method with:
```cpp
  // Returns the birth-intensity scale in [0, 1] for a birth at ENU position
  // `mean.head<2>()`, or a negative value meaning "hard-drop this birth".
  // Combines the land prior and the static-obstacle prior multiplicatively:
  //   scale = (1 − c_land) · (1 − c_static)
  // and hard-drops when EITHER prior exceeds its own hard gate. With no models
  // wired both priors are 0 → scale 1.0 (bit-identical to pre-Stage-1).
  double birthScale(const Eigen::VectorXd& mean) const {
    if (mean.size() < 2) return 1.0;
    double c_land = 0.0;
    if (cfg_.use_land_model && land_model_ != nullptr)
      c_land = land_model_->clutterPrior(mean.head<2>());
    double c_static = 0.0;
    if (cfg_.use_static_obstacle_model && obstacle_model_ != nullptr)
      c_static = obstacle_model_->birthSuppression(mean.head<2>());
    if (c_land > cfg_.land_birth_hard_gate ||
        c_static > cfg_.static_obstacle_hard_gate)
      return -1.0;  // hard-drop
    return (1.0 - c_land) * (1.0 - c_static);
  }
```

- [ ] **Step 5: Update the two call sites in `core/pmbm/PmbmTracker.cpp`**

At the measurement-driven site (~line 388) and the adaptive-birth site (~line 495), rename the call and the local variable for clarity:
- Replace `const double land_scale = landBirthScale(cand.mean);` with `const double birth_scale = birthScale(cand.mean);`
- Replace subsequent uses of `land_scale` in that block with `birth_scale`.

(Both sites: the surrounding `if (birth_scale < 0.0) { ... } else if (birth_scale < 1.0) { ... }` logic is unchanged — only the call and variable name change.)

- [ ] **Step 6: Build and run — new test + the land-model regression**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='PmbmStaticObstacle.*:PmbmLandModel.*'`
Expected: PASS (4 new + 4 existing land tests all green — proves the rename kept land behaviour bit-identical).

- [ ] **Step 7: Commit**

```bash
git add core/pmbm/PmbmTracker.hpp core/pmbm/PmbmTracker.cpp \
        tests/pmbm/test_pmbm_static_obstacle.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: PMBM birth-prior wiring (birthScale combines land + obstacle)"
```

---

### Task 3: `StaticHazardOutput` + `toStaticHazardOutput` + `staticHazardId`

**Files:**
- Create: `core/output/StaticHazardOutput.hpp` / `core/output/StaticHazardOutput.cpp`
- Test: `tests/output/test_static_hazard_output.cpp`
- Modify: `CMakeLists.txt` (add `core/output/StaticHazardOutput.cpp` to `navtracker_core` sources after line 79; add the test after line 289)

**Interfaces:**
- Produces: `struct StaticHazardOutput`; `std::uint64_t staticHazardId(const StaticObstacle&)`; `StaticHazardOutput toStaticHazardOutput(const StaticObstacle&)`.
- Consumes: `StaticObstacle`, `geo::Geodetic`.

- [ ] **Step 1: Write the failing test** — `tests/output/test_static_hazard_output.cpp`

```cpp
#include <gtest/gtest.h>

#include "core/geo/Wgs84.hpp"
#include "core/output/StaticHazardOutput.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::AtoNRealism;
using navtracker::ObstacleCategory;
using navtracker::StaticHazardOutput;
using navtracker::StaticObstacle;
using navtracker::WaterLevel;
using navtracker::staticHazardId;
using navtracker::toStaticHazardOutput;
using navtracker::geo::Geodetic;

namespace {

StaticObstacle wreck() {
  StaticObstacle o;
  o.position = Geodetic{42.351, -71.052, 0.0};
  o.footprint_radius_m = 20.0;
  o.keep_clear_radius_m = 120.0;
  o.category = ObstacleCategory::Wreck;
  o.water_level = WaterLevel::AlwaysSubmerged;
  o.depth_m = 3.5;
  o.lit = true;
  o.aton = AtoNRealism::Real;
  o.source_id = "ENC:US5MA...";
  return o;
}

}  // namespace

// Conversion copies charted attributes verbatim; is_charted true.
TEST(StaticHazardOutput, ConversionCopiesAttributes) {
  const StaticHazardOutput o = toStaticHazardOutput(wreck());
  EXPECT_DOUBLE_EQ(o.position.lat_deg, 42.351);
  EXPECT_DOUBLE_EQ(o.position.lon_deg, -71.052);
  EXPECT_DOUBLE_EQ(o.keep_clear_radius_m, 120.0);
  EXPECT_DOUBLE_EQ(o.footprint_radius_m, 20.0);
  EXPECT_EQ(o.category, ObstacleCategory::Wreck);
  EXPECT_EQ(o.water_level, WaterLevel::AlwaysSubmerged);
  EXPECT_DOUBLE_EQ(o.depth_m, 3.5);
  EXPECT_TRUE(o.lit);
  EXPECT_EQ(o.aton, AtoNRealism::Real);
  EXPECT_TRUE(o.is_charted);
  EXPECT_EQ(o.source_id, "ENC:US5MA...");
}

// Id is deterministic and order-independent (function of position + category).
TEST(StaticHazardOutput, IdDeterministicAndStable) {
  EXPECT_EQ(staticHazardId(wreck()), staticHazardId(wreck()));
  EXPECT_EQ(toStaticHazardOutput(wreck()).hazard_id, staticHazardId(wreck()));
}

// Distinct positions → distinct ids.
TEST(StaticHazardOutput, DistinctObstaclesDistinctIds) {
  StaticObstacle a = wreck();
  StaticObstacle b = wreck();
  b.position.lat_deg += 0.01;  // ~1.1 km away
  EXPECT_NE(staticHazardId(a), staticHazardId(b));
}
```

- [ ] **Step 2: Register and run to verify it fails**

Add both CMake entries. Run `cmake --build build -j --target navtracker_tests`.
Expected: FAIL to compile (`StaticHazardOutput.hpp` not found).

- [ ] **Step 3: Create `core/output/StaticHazardOutput.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <string>

#include "core/geo/Wgs84.hpp"
#include "core/types/StaticObstacle.hpp"

namespace navtracker {

// Drainable output for one static hazard (pull-style, parallel to
// TrackOutput). Position is geodetic — obstacles are charted in WGS84, so no
// datum rotation is needed (unlike a kinematic track). is_charted separates
// Stage-1 chart hazards from Stage-2 live-detected occupancy.
struct StaticHazardOutput {
  std::uint64_t hazard_id{0};
  geo::Geodetic position{};
  double keep_clear_radius_m{0.0};
  double footprint_radius_m{0.0};
  double position_uncertainty_m{0.0};
  ObstacleCategory category{ObstacleCategory::Unknown};
  WaterLevel water_level{WaterLevel::Unknown};
  double depth_m{std::numeric_limits<double>::quiet_NaN()};  // VALSOU: NaN = unknown
  bool lit{false};
  AtoNRealism aton{AtoNRealism::NotAtoN};
  bool is_charted{true};
  std::string source_id;
};

// Deterministic stable id from an obstacle's charted position + category.
// Order-independent (not a list index). Rounds lat/lon to ~1 m before hashing
// so numeric jitter does not change the id.
std::uint64_t staticHazardId(const StaticObstacle& obs);

// Build a StaticHazardOutput from a charted obstacle (attributes verbatim,
// id via staticHazardId, is_charted = true).
StaticHazardOutput toStaticHazardOutput(const StaticObstacle& obs);

}  // namespace navtracker
```

- [ ] **Step 4: Create `core/output/StaticHazardOutput.cpp`**

```cpp
#include "core/output/StaticHazardOutput.hpp"

#include <cmath>

namespace navtracker {

std::uint64_t staticHazardId(const StaticObstacle& obs) {
  // Round to ~1e-5 deg (~1 m) so tiny numeric jitter does not change the id.
  const long long lat = std::llround(obs.position.lat_deg * 1e5);
  const long long lon = std::llround(obs.position.lon_deg * 1e5);
  std::uint64_t h = 14695981039346656037ULL;  // FNV-1a 64-bit offset basis
  auto mix = [&h](std::uint64_t v) {
    h ^= v;
    h *= 1099511628211ULL;  // FNV prime
  };
  mix(static_cast<std::uint64_t>(lat));
  mix(static_cast<std::uint64_t>(lon));
  mix(static_cast<std::uint64_t>(obs.category));
  return h;
}

StaticHazardOutput toStaticHazardOutput(const StaticObstacle& obs) {
  StaticHazardOutput o;
  o.hazard_id = staticHazardId(obs);
  o.position = obs.position;
  o.keep_clear_radius_m = obs.keep_clear_radius_m;
  o.footprint_radius_m = obs.footprint_radius_m;
  o.position_uncertainty_m = obs.position_uncertainty_m;
  o.category = obs.category;
  o.water_level = obs.water_level;
  o.depth_m = obs.depth_m;
  o.lit = obs.lit;
  o.aton = obs.aton;
  o.is_charted = true;
  o.source_id = obs.source_id;
  return o;
}

}  // namespace navtracker
```

- [ ] **Step 5: Build and run to verify it passes**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='StaticHazardOutput.*'`
Expected: PASS (3/3).

- [ ] **Step 6: Commit**

```bash
git add core/output/StaticHazardOutput.hpp core/output/StaticHazardOutput.cpp \
        tests/output/test_static_hazard_output.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: StaticHazardOutput + toStaticHazardOutput + staticHazardId"
```

---

### Task 4: `IStaticHazardSink` + `StaticHazardEvaluator`

**Files:**
- Create: `ports/IStaticHazardSink.hpp`
- Create: `core/collision/StaticHazardEvaluator.hpp` / `.cpp`
- Test: `tests/collision/test_static_hazard_evaluator.cpp`
- Modify: `CMakeLists.txt` (add `core/collision/StaticHazardEvaluator.cpp` to `navtracker_core` after line 79; add the test after line 188)

**Interfaces:**
- Produces: `enum StaticHazardTransition`, `struct StaticHazardEvent`, `class IStaticHazardSink`; `class StaticHazardEvaluator` with `StaticHazardEvaluator(const IStaticObstacleModel*, Config={})`, `setSink(IStaticHazardSink*)`, `evaluate(const Eigen::Vector2d& own_ship_enu, const geo::Datum&, Timestamp)`.
- Consumes: `IStaticObstacleModel`, `staticHazardId` (Task 3), `geo::Datum`, `Timestamp`.

- [ ] **Step 1: Write the failing test** — `tests/collision/test_static_hazard_evaluator.cpp`

```cpp
#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "core/collision/StaticHazardEvaluator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/types/StaticObstacle.hpp"
#include "ports/IStaticHazardSink.hpp"

using navtracker::IStaticHazardSink;
using navtracker::StaticHazardEvaluator;
using navtracker::StaticHazardEvent;
using navtracker::StaticHazardTransition;
using navtracker::StaticObstacle;
using navtracker::StaticObstacleModel;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;

namespace {

Datum datum() { return Datum(Geodetic{42.35, -71.05, 0.0}); }

StaticObstacleModel originModel() {
  StaticObstacle o;
  o.position = Geodetic{42.35, -71.05, 0.0};  // ENU ~ (0,0)
  o.footprint_radius_m = 15.0;
  o.keep_clear_radius_m = 100.0;
  return StaticObstacleModel({o}, datum());
}

struct Recorder : IStaticHazardSink {
  std::vector<StaticHazardEvent> events;
  void onStaticHazard(const StaticHazardEvent& e) override {
    events.push_back(e);
  }
};

}  // namespace

// Approaching within the keep-clear radius fires exactly one Entered; leaving
// past the hysteresis radius fires exactly one Exited.
TEST(StaticHazardEvaluator, EnterThenExitWithHysteresis) {
  StaticObstacleModel m = originModel();
  StaticHazardEvaluator ev(&m);  // default exit_hysteresis = 1.1
  Recorder rec;
  ev.setSink(&rec);

  ev.evaluate(Eigen::Vector2d(300.0, 0.0), datum(), Timestamp::fromSeconds(0));  // far
  ev.evaluate(Eigen::Vector2d(80.0, 0.0), datum(), Timestamp::fromSeconds(1));   // inside 100
  ev.evaluate(Eigen::Vector2d(90.0, 0.0), datum(), Timestamp::fromSeconds(2));   // still inside, < 110
  ev.evaluate(Eigen::Vector2d(120.0, 0.0), datum(), Timestamp::fromSeconds(3));  // beyond 110 → exit

  ASSERT_EQ(rec.events.size(), 2u);
  EXPECT_EQ(rec.events[0].transition, StaticHazardTransition::Entered);
  EXPECT_EQ(rec.events[1].transition, StaticHazardTransition::Exited);
  EXPECT_NEAR(rec.events[0].distance_m, 80.0, 1.0);
  EXPECT_DOUBLE_EQ(rec.events[0].keep_clear_m, 100.0);
}

// No sink wired → no crash, no effect.
TEST(StaticHazardEvaluator, NullSinkSafe) {
  StaticObstacleModel m = originModel();
  StaticHazardEvaluator ev(&m);
  ev.evaluate(Eigen::Vector2d(0.0, 0.0), datum(), Timestamp::fromSeconds(0));
  SUCCEED();
}

// Staying outside never fires.
TEST(StaticHazardEvaluator, StaysOutsideNoEvent) {
  StaticObstacleModel m = originModel();
  StaticHazardEvaluator ev(&m);
  Recorder rec;
  ev.setSink(&rec);
  ev.evaluate(Eigen::Vector2d(300.0, 0.0), datum(), Timestamp::fromSeconds(0));
  ev.evaluate(Eigen::Vector2d(250.0, 0.0), datum(), Timestamp::fromSeconds(1));
  EXPECT_TRUE(rec.events.empty());
}
```

- [ ] **Step 2: Register and run to verify it fails**

Add CMake entries. `cmake --build build -j --target navtracker_tests`.
Expected: FAIL to compile.

- [ ] **Step 3: Create `ports/IStaticHazardSink.hpp`**

```cpp
#pragma once

#include <cstdint>

#include "core/types/Timestamp.hpp"

namespace navtracker {

// Push-based static-hazard proximity observer. StaticHazardEvaluator emits
// these per (own-ship × charted-obstacle) on keep-clear crossings, with
// hysteresis. Distinct from ICollisionRiskSink: a static-geometry range check,
// not a trajectory CPA (ADR 0002 — obstacles have no velocity).
enum class StaticHazardTransition { Entered, Exited, Updated };

struct StaticHazardEvent {
  StaticHazardTransition transition;
  std::uint64_t hazard_id;
  Timestamp time;
  double distance_m;    // own-ship to obstacle centre (m)
  double keep_clear_m;  // the obstacle's keep-clear radius (m)
};

class IStaticHazardSink {
 public:
  virtual ~IStaticHazardSink() = default;
  virtual void onStaticHazard(const StaticHazardEvent& event) = 0;
};

}  // namespace navtracker
```

- [ ] **Step 4: Create `core/collision/StaticHazardEvaluator.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <map>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IStaticHazardSink.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

// Emits static-hazard proximity events per (own-ship × charted-obstacle) with
// hysteresis. Static geometry: distance from own-ship to obstacle centre vs
// the obstacle's keep-clear radius. Mirrors CpaEvaluator's per-pair state +
// hysteresis, minus the trajectory/CPA math.
class StaticHazardEvaluator {
 public:
  struct Config {
    // Exit when distance exceeds keep_clear * exit_hysteresis (> 1.0), so a
    // vessel loitering at the boundary does not flap Entered/Exited.
    double exit_hysteresis = 1.1;
    // Emit an Updated event each cycle while inside the keep-clear ring.
    bool emit_updates = false;
  };

  explicit StaticHazardEvaluator(const IStaticObstacleModel* model,
                                 Config cfg = {})
      : model_(model), cfg_(cfg) {}

  void setSink(IStaticHazardSink* s) { sink_ = s; }

  // Evaluate all obstacles against the current own-ship ENU position (metres,
  // in `datum`'s frame). Fires Entered/Exited/Updated with hysteresis.
  void evaluate(const Eigen::Vector2d& own_ship_enu, const geo::Datum& datum,
                Timestamp t);

 private:
  const IStaticObstacleModel* model_{nullptr};
  IStaticHazardSink* sink_{nullptr};
  Config cfg_;
  std::map<std::uint64_t, bool> inside_;  // hazard_id -> currently inside ring
};

}  // namespace navtracker
```

- [ ] **Step 5: Create `core/collision/StaticHazardEvaluator.cpp`**

```cpp
#include "core/collision/StaticHazardEvaluator.hpp"

#include "core/output/StaticHazardOutput.hpp"  // staticHazardId

namespace navtracker {

void StaticHazardEvaluator::evaluate(const Eigen::Vector2d& own_ship_enu,
                                     const geo::Datum& datum, Timestamp t) {
  if (model_ == nullptr) return;
  for (const auto& obs : model_->obstacles()) {
    const Eigen::Vector3d e = datum.toEnu(obs.position);
    const double d = (Eigen::Vector2d(e.x(), e.y()) - own_ship_enu).norm();
    const std::uint64_t id = staticHazardId(obs);
    const double enter_r = obs.keep_clear_radius_m;
    const double exit_r = obs.keep_clear_radius_m * cfg_.exit_hysteresis;

    auto it = inside_.find(id);
    const bool was_inside = (it != inside_.end()) ? it->second : false;
    bool now_inside = was_inside;
    if (!was_inside && d < enter_r) {
      now_inside = true;
    } else if (was_inside && d > exit_r) {
      now_inside = false;
    }

    if (sink_ != nullptr) {
      if (now_inside && !was_inside) {
        sink_->onStaticHazard({StaticHazardTransition::Entered, id, t, d,
                               obs.keep_clear_radius_m});
      } else if (!now_inside && was_inside) {
        sink_->onStaticHazard({StaticHazardTransition::Exited, id, t, d,
                               obs.keep_clear_radius_m});
      } else if (now_inside && cfg_.emit_updates) {
        sink_->onStaticHazard({StaticHazardTransition::Updated, id, t, d,
                               obs.keep_clear_radius_m});
      }
    }
    inside_[id] = now_inside;
  }
}

}  // namespace navtracker
```

- [ ] **Step 6: Build and run to verify it passes**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='StaticHazardEvaluator.*'`
Expected: PASS (3/3).

- [ ] **Step 7: Commit**

```bash
git add ports/IStaticHazardSink.hpp core/collision/StaticHazardEvaluator.hpp \
        core/collision/StaticHazardEvaluator.cpp \
        tests/collision/test_static_hazard_evaluator.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: IStaticHazardSink + StaticHazardEvaluator (keep-clear alarm)"
```

---

### Task 5: GeoJSON adapter + fixture

**Files:**
- Create: `adapters/static/GeoJsonStaticObstacles.hpp` / `.cpp`
- Create: `tests/fixtures/static/harbor_obstacles.geojson`
- Test: `tests/adapters/static/test_geojson_static_obstacles.cpp`
- Modify: `CMakeLists.txt` (add `adapters/static/GeoJsonStaticObstacles.cpp` to the `navtracker_land` library sources at line 108-109; add the test to `navtracker_tests` after line 233; ensure `navtracker_tests` links `navtracker_land` — it already does for the coastline path, verify)

**Interfaces:**
- Produces: `std::vector<StaticObstacle> parseStaticObstaclesGeoJson(const std::string& json_text)`; `std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path)`.
- Consumes: `StaticObstacle`; `nlohmann/json` (mirror `adapters/land/GeoJsonCoastline.cpp` — `#include <nlohmann/json.hpp>`).

**Adapter behaviour (spec):** parse a GeoJSON `FeatureCollection`. Each `Feature` with a `Point` geometry (`[lon, lat]`) becomes one `StaticObstacle`. `properties` (all optional, defaults in brackets):
- `category` (string → `ObstacleCategory`; case-insensitive: `rock`,`wreck`,`obstruction`,`pile`,`platform`,`buoy`,`beacon`,`other` → matching enum; anything else → `Unknown`) [`Unknown`]
- `watlev` (string → `WaterLevel`: `awash`/`covers`→`AwashCoversUncovers`, `submerged`→`AlwaysSubmerged`, `above`→`AlwaysAboveWater`, `floating`→`Floating`; else `Unknown`) [`Unknown`]
- `aton` (string → `AtoNRealism`: `real`,`synthetic`,`virtual`; else `NotAtoN`) [`NotAtoN`]
- `depth_m` (number) [`NaN`]
- `lit` (bool) [`false`]
- `footprint_radius_m`, `keep_clear_radius_m`, `position_uncertainty_m` (numbers) [`0.0`]
- `source_id` (string) [`""`]

Features whose geometry is null / not a `Point` / lacks ≥2 finite coordinates are skipped (mirror `GeoJsonCoastline`'s skip-invalid behaviour). `loadStaticObstaclesGeoJson` throws `std::runtime_error` if the file cannot be opened (mirror `loadCoastlineGeoJson`).

- [ ] **Step 1: Create the fixture** — `tests/fixtures/static/harbor_obstacles.geojson`

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": { "type": "Point", "coordinates": [-71.0520, 42.3510] },
      "properties": {
        "category": "wreck",
        "watlev": "submerged",
        "depth_m": 3.5,
        "lit": true,
        "aton": "real",
        "footprint_radius_m": 20.0,
        "keep_clear_radius_m": 120.0,
        "position_uncertainty_m": 5.0,
        "source_id": "ENC:wreck-1"
      }
    },
    {
      "type": "Feature",
      "geometry": { "type": "Point", "coordinates": [-71.0500, 42.3500] },
      "properties": {
        "category": "pile"
      }
    }
  ]
}
```

- [ ] **Step 2: Write the failing test** — `tests/adapters/static/test_geojson_static_obstacles.cpp`

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "adapters/static/GeoJsonStaticObstacles.hpp"
#include "core/types/StaticObstacle.hpp"

using navtracker::AtoNRealism;
using navtracker::ObstacleCategory;
using navtracker::StaticObstacle;
using navtracker::WaterLevel;
using navtracker::loadStaticObstaclesGeoJson;
using navtracker::parseStaticObstaclesGeoJson;

// Full-attribute feature parses into all fields.
TEST(GeoJsonStaticObstacles, ParsesFullAttributes) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.052,42.351]},
      "properties":{"category":"wreck","watlev":"submerged","depth_m":3.5,
      "lit":true,"aton":"real","footprint_radius_m":20.0,
      "keep_clear_radius_m":120.0,"position_uncertainty_m":5.0,
      "source_id":"ENC:wreck-1"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_DOUBLE_EQ(obs[0].position.lat_deg, 42.351);
  EXPECT_DOUBLE_EQ(obs[0].position.lon_deg, -71.052);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Wreck);
  EXPECT_EQ(obs[0].water_level, WaterLevel::AlwaysSubmerged);
  EXPECT_DOUBLE_EQ(obs[0].depth_m, 3.5);
  EXPECT_TRUE(obs[0].lit);
  EXPECT_EQ(obs[0].aton, AtoNRealism::Real);
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 20.0);
  EXPECT_DOUBLE_EQ(obs[0].keep_clear_radius_m, 120.0);
  EXPECT_DOUBLE_EQ(obs[0].position_uncertainty_m, 5.0);
  EXPECT_EQ(obs[0].source_id, "ENC:wreck-1");
}

// Sparse feature uses defaults (NaN depth, zero radii, Unknown category).
TEST(GeoJsonStaticObstacles, MissingPropertiesUseDefaults) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"pile"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Pile);
  EXPECT_TRUE(std::isnan(obs[0].depth_m));
  EXPECT_DOUBLE_EQ(obs[0].footprint_radius_m, 0.0);
  EXPECT_FALSE(obs[0].lit);
  EXPECT_EQ(obs[0].aton, AtoNRealism::NotAtoN);
}

// Non-Point / null geometry features are skipped.
TEST(GeoJsonStaticObstacles, SkipsInvalidGeometry) {
  const std::string json = R"({
    "type":"FeatureCollection","features":[
     {"type":"Feature","geometry":null,"properties":{}},
     {"type":"Feature","geometry":{"type":"LineString",
      "coordinates":[[-71.05,42.35],[-71.06,42.36]]},"properties":{}},
     {"type":"Feature","geometry":{"type":"Point","coordinates":[-71.05,42.35]},
      "properties":{"category":"rock"}}]})";
  const std::vector<StaticObstacle> obs = parseStaticObstaclesGeoJson(json);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].category, ObstacleCategory::Rock);
}

// The on-disk fixture loads two obstacles. Use NAVTRACKER_SOURCE_DIR (a
// compile-time absolute path, defined on navtracker_tests) so the test is
// working-directory-independent and passes under ctest — matching
// tests/land/test_geojson_coastline.cpp.
TEST(GeoJsonStaticObstacles, LoadsFixtureFromDisk) {
  const std::vector<StaticObstacle> obs = loadStaticObstaclesGeoJson(
      std::string(NAVTRACKER_SOURCE_DIR) +
      "/tests/fixtures/static/harbor_obstacles.geojson");
  ASSERT_EQ(obs.size(), 2u);
}

// Missing file throws.
TEST(GeoJsonStaticObstacles, MissingFileThrows) {
  EXPECT_THROW(loadStaticObstaclesGeoJson("tests/fixtures/static/nope.geojson"),
               std::runtime_error);
}
```

- [ ] **Step 3: Register the test in CMake and run to verify it fails**

Add the test. Run `cmake --build build -j --target navtracker_tests`.
Expected: FAIL to compile (`adapters/static/GeoJsonStaticObstacles.hpp` not found).

- [ ] **Step 4: Create `adapters/static/GeoJsonStaticObstacles.hpp`**

```cpp
#pragma once

#include <string>
#include <vector>

#include "core/types/StaticObstacle.hpp"

namespace navtracker {

/// Parse a GeoJSON FeatureCollection of Point features into StaticObstacles.
/// See docs/superpowers/plans/2026-07-01-static-obstacle-stage1.md, Task 5 for
/// the property schema and defaults. Features without a valid Point geometry
/// (>= 2 finite coordinates) are skipped.
std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text);

/// Load a GeoJSON file from disk and parse it.
/// Throws std::runtime_error if the file cannot be opened.
std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path);

}  // namespace navtracker
```

- [ ] **Step 5: Create `adapters/static/GeoJsonStaticObstacles.cpp`**

Implement using `nlohmann/json` (mirror `adapters/land/GeoJsonCoastline.cpp` structure: file `#include`s, `parse...` on a string, `load...` opening an `ifstream` and throwing on failure). Provide small case-insensitive string→enum helpers for `category`, `watlev`, `aton`. Read each optional property with `contains()` + type check, applying the defaults from the schema. Coordinates are `[lon, lat]`; require both finite (`std::isfinite`), else skip the feature. Complete implementation:

```cpp
#include "adapters/static/GeoJsonStaticObstacles.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace navtracker {
namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

ObstacleCategory toCategory(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "rock") return ObstacleCategory::Rock;
  if (s == "wreck") return ObstacleCategory::Wreck;
  if (s == "obstruction") return ObstacleCategory::Obstruction;
  if (s == "pile") return ObstacleCategory::Pile;
  if (s == "platform") return ObstacleCategory::Platform;
  if (s == "buoy") return ObstacleCategory::Buoy;
  if (s == "beacon") return ObstacleCategory::Beacon;
  if (s == "other") return ObstacleCategory::Other;
  return ObstacleCategory::Unknown;
}

WaterLevel toWaterLevel(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "awash" || s == "covers") return WaterLevel::AwashCoversUncovers;
  if (s == "submerged") return WaterLevel::AlwaysSubmerged;
  if (s == "above") return WaterLevel::AlwaysAboveWater;
  if (s == "floating") return WaterLevel::Floating;
  return WaterLevel::Unknown;
}

AtoNRealism toAton(const std::string& raw) {
  const std::string s = lower(raw);
  if (s == "real") return AtoNRealism::Real;
  if (s == "synthetic") return AtoNRealism::Synthetic;
  if (s == "virtual") return AtoNRealism::Virtual;
  return AtoNRealism::NotAtoN;
}

double numberOr(const nlohmann::json& props, const char* key, double dflt) {
  if (props.contains(key) && props[key].is_number())
    return props[key].get<double>();
  return dflt;
}

std::string stringOr(const nlohmann::json& props, const char* key) {
  if (props.contains(key) && props[key].is_string())
    return props[key].get<std::string>();
  return std::string{};
}

}  // namespace

std::vector<StaticObstacle> parseStaticObstaclesGeoJson(
    const std::string& json_text) {
  std::vector<StaticObstacle> out;
  const nlohmann::json root = nlohmann::json::parse(json_text);
  if (!root.contains("features") || !root["features"].is_array()) return out;

  for (const auto& feat : root["features"]) {
    if (!feat.contains("geometry") || feat["geometry"].is_null()) continue;
    const auto& geom = feat["geometry"];
    if (!geom.contains("type") || geom["type"] != "Point") continue;
    if (!geom.contains("coordinates") || !geom["coordinates"].is_array() ||
        geom["coordinates"].size() < 2)
      continue;
    const double lon = geom["coordinates"][0].get<double>();
    const double lat = geom["coordinates"][1].get<double>();
    if (!std::isfinite(lon) || !std::isfinite(lat)) continue;

    StaticObstacle o;
    o.position = geo::Geodetic{lat, lon, 0.0};
    const nlohmann::json props =
        feat.contains("properties") && feat["properties"].is_object()
            ? feat["properties"]
            : nlohmann::json::object();
    o.category = toCategory(stringOr(props, "category"));
    o.water_level = toWaterLevel(stringOr(props, "watlev"));
    o.aton = toAton(stringOr(props, "aton"));
    o.depth_m = numberOr(props, "depth_m",
                         std::numeric_limits<double>::quiet_NaN());
    o.lit = props.contains("lit") && props["lit"].is_boolean()
                ? props["lit"].get<bool>()
                : false;
    o.footprint_radius_m = numberOr(props, "footprint_radius_m", 0.0);
    o.keep_clear_radius_m = numberOr(props, "keep_clear_radius_m", 0.0);
    o.position_uncertainty_m = numberOr(props, "position_uncertainty_m", 0.0);
    o.source_id = stringOr(props, "source_id");
    out.push_back(std::move(o));
  }
  return out;
}

std::vector<StaticObstacle> loadStaticObstaclesGeoJson(const std::string& path) {
  std::ifstream in(path);
  if (!in.good())
    throw std::runtime_error("cannot open static-obstacle GeoJSON: " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return parseStaticObstaclesGeoJson(ss.str());
}

}  // namespace navtracker
```
(Add `#include <limits>` if `<cmath>` does not bring in `std::numeric_limits` — it comes from `<limits>`, so add it.)

- [ ] **Step 6: Build and run to verify it passes**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='GeoJsonStaticObstacles.*'`
Expected: PASS (5/5). If `LoadsFixtureFromDisk` fails on path, confirm tests run from the project root (they do — the coastline test uses the same relative-path convention).

- [ ] **Step 7: Commit**

```bash
git add adapters/static/GeoJsonStaticObstacles.hpp adapters/static/GeoJsonStaticObstacles.cpp \
        tests/fixtures/static/harbor_obstacles.geojson \
        tests/adapters/static/test_geojson_static_obstacles.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: GeoJSON static-obstacle adapter + fixture"
```

---

### Task 6: Benchmark wiring + synthetic integration scenario + config count

**Files:**
- Modify: `core/benchmark/Config.hpp` (add `bool use_static_obstacle_model{false};` after line 100)
- Modify: `core/benchmark/ScenarioRun.hpp` (add `#include "core/types/StaticObstacle.hpp"` + `#include <optional>`; add `std::string static_obstacles_geojson_path;` after line 64; add `syntheticObstacles()` virtual after line 92)
- Modify: `core/benchmark/Sweep.cpp` (add includes; wire the static-obstacle model in the PMBM branch, mirroring the land block at lines 341-363)
- Modify: `core/benchmark/Config.cpp` (add config `imm_cv_ct_pmbm_static`)
- Modify: `tests/benchmark/test_config.cpp` (count 30→31 + label check)
- Test: `tests/integration/test_static_obstacle_pipeline.cpp`
- Modify: `CMakeLists.txt` (add the integration test)

**Interfaces:**
- Consumes: everything from Tasks 1-5.
- Produces: bench config label `imm_cv_ct_pmbm_static`; `ScenarioDescriptor::static_obstacles_geojson_path`; `ScenarioRun::syntheticObstacles()`.

- [ ] **Step 1: Write the failing integration test** — `tests/integration/test_static_obstacle_pipeline.cpp`

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/collision/StaticHazardEvaluator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/static/StaticObstacleModel.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/StaticObstacle.hpp"
#include "ports/IStaticHazardSink.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::ObstacleCategory;
using navtracker::SensorKind;
using navtracker::StaticHazardEvaluator;
using navtracker::StaticHazardEvent;
using navtracker::StaticHazardTransition;
using navtracker::StaticObstacle;
using navtracker::StaticObstacleModel;
using navtracker::Timestamp;
using navtracker::geo::Datum;
using navtracker::geo::Geodetic;
using navtracker::pmbm::PmbmTracker;

namespace {

Datum datum() { return Datum(Geodetic{42.35, -71.05, 0.0}); }

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

StaticObstacleModel obstacleModel() {
  StaticObstacle o;
  o.position = Geodetic{42.35, -71.05, 0.0};  // ENU ~ (0,0)
  o.footprint_radius_m = 15.0;
  o.keep_clear_radius_m = 100.0;
  o.category = ObstacleCategory::Pile;
  return StaticObstacleModel({o}, datum());
}

struct HazardRecorder : navtracker::IStaticHazardSink {
  std::vector<StaticHazardEvent> events;
  void onStaticHazard(const StaticHazardEvent& e) override {
    events.push_back(e);
  }
};

PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  c.use_static_obstacle_model = true;
  return c;
}

}  // namespace

// End-to-end Stage 1a: a persistent clutter source inside a charted obstacle's
// footprint is suppressed (no lingering high-existence track there), while a
// real vessel transiting through the keep-clear buffer is still tracked; and
// the keep-clear evaluator fires when own-ship enters the ring.
TEST(StaticObstaclePipeline, SuppressesPhantomKeepsRealVesselAndAlarms) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.5);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker t(ekf, cfg());
  StaticObstacleModel m = obstacleModel();
  t.setStaticObstacleModel(&m);

  // A real vessel TRANSITS the 100 m keep-clear buffer: west→east at y=60 m
  // from x=-70 to x=+70 (closest approach 60 m to the obstacle centre — inside
  // the 100 m soft buffer, outside the 15 m hard footprint), 10 m/s, one scan
  // per second. It even BIRTHS inside the buffer (its birth is soft-suppressed)
  // yet must still be tracked — the passing/anchored-vessel protection.
  double maxExistenceNearObstacle = 0.0;
  double maxExistenceOnVessel = 0.0;
  for (int k = 0; k < 15; ++k) {
    const double tt = static_cast<double>(k);
    t.predict(Timestamp::fromSeconds(tt));
    const double vx = -70.0 + 10.0 * tt;  // vessel x this scan (-70..+70, in buffer)
    std::vector<Measurement> scan;
    scan.push_back(posMeas(5.0, 3.0, tt));      // clutter inside the hard footprint
    scan.push_back(posMeas(vx, 60.0, tt));      // the real vessel, inside the buffer
    t.processBatch(scan);

    for (const auto& h : t.density().mbm) {
      for (const auto& b : h.bernoullis) {
        const Eigen::Vector2d p = b.state.mean.head<2>();
        const double r = b.existence_probability;
        if ((p - Eigen::Vector2d(5.0, 3.0)).norm() < 20.0)
          maxExistenceNearObstacle = std::max(maxExistenceNearObstacle, r);
        if (std::abs(p.y() - 60.0) < 30.0)
          maxExistenceOnVessel = std::max(maxExistenceOnVessel, r);
      }
    }
  }

  // Phantom at the obstacle never accumulates confidence.
  EXPECT_LT(maxExistenceNearObstacle, 0.2);
  // The real vessel in the buffer is tracked with real confidence.
  EXPECT_GT(maxExistenceOnVessel, 0.6);

  // Keep-clear alarm: own-ship steams from far toward the obstacle.
  StaticHazardEvaluator ev(&m);
  HazardRecorder rec;
  ev.setSink(&rec);
  ev.evaluate(Eigen::Vector2d(300.0, 0.0), datum(), Timestamp::fromSeconds(0));
  ev.evaluate(Eigen::Vector2d(50.0, 0.0), datum(), Timestamp::fromSeconds(1));
  ASSERT_FALSE(rec.events.empty());
  EXPECT_EQ(rec.events.front().transition, StaticHazardTransition::Entered);
}
```

> **Implementer note:** verify the PMBM Bernoulli state accessor used above
> (`b.state.mean.head<2>()`) against `core/pmbm/PmbmTypes.hpp` / how
> `test_pmbm_scenario.cpp` reads Bernoulli means, and adjust the field access if
> the type differs. The existence field is `b.existence_probability` (confirmed
> in `test_pmbm_land_model.cpp`). If the CV process-noise or exact thresholds
> make the vessel confidence marginal, tune the scan count / noise — the
> assertions (phantom suppressed, vessel tracked, alarm fires) are the spec, not
> the exact constants.

- [ ] **Step 2: Register and run to verify it fails**

Add the test to `CMakeLists.txt`. Run `cmake --build build -j --target navtracker_tests`.
Expected: FAIL (compiles once Tasks 1-4 are in; the assertions are the target).

- [ ] **Step 3: Add the benchmark `Config` flag** — `core/benchmark/Config.hpp`

After `bool use_land_model{false};` (line 100), add:
```cpp
  // Stage 1 static-obstacle branch (ADR 0002): when true, Sweep builds a
  // StaticObstacleModel from the scenario's synthetic obstacles (preferred)
  // or its static_obstacles_geojson_path, and calls
  // tracker.setStaticObstacleModel(). Only meaningful when tracker_kind ==
  // Pmbm and the PMBM config sets use_static_obstacle_model. Scenarios with no
  // obstacles silently skip wiring (model stays null → bit-identical).
  bool use_static_obstacle_model{false};
```

- [ ] **Step 4: Extend `ScenarioRun.hpp`**

Add includes at the top: `#include <optional>` (already present) and `#include "core/types/StaticObstacle.hpp"`. After `std::string coastline_geojson_path;` (line 64), add:
```cpp
  // Optional path to a GeoJSON file of charted static obstacles (ADR 0002
  // Stage 1). Non-empty signals a chart fixture; Sweep checks this + file
  // existence before building a StaticObstacleModel. Relative to the cwd.
  std::string static_obstacles_geojson_path;
```
After the `syntheticCoastline()` method (line 92), add:
```cpp
  // Optional in-memory charted obstacles for synthetic scenarios. Default =
  // none (every existing scenario untouched). When present AND
  // config.use_static_obstacle_model AND Scenario.datum is set, Sweep builds a
  // StaticObstacleModel from these (in preference to
  // static_obstacles_geojson_path).
  virtual std::optional<std::vector<StaticObstacle>> syntheticObstacles() const {
    return std::nullopt;
  }
```

- [ ] **Step 5: Wire the model in `Sweep.cpp`**

Add includes near the coastline includes: `#include "core/static/StaticObstacleModel.hpp"` and `#include "adapters/static/GeoJsonStaticObstacles.hpp"`. Immediately after the land-model block (after line 363), add the parallel block:
```cpp
          // Stage 1 static-obstacle model (ADR 0002), same lifetime/datum
          // rules as the land model above. Prefer in-memory synthetic
          // obstacles; else a GeoJSON fixture path. Null → bit-identical.
          std::shared_ptr<StaticObstacleModel> obstacles;
          if (config.use_static_obstacle_model && scen.datum.has_value()) {
            std::optional<std::vector<StaticObstacle>> synth =
                scenario_ptr->syntheticObstacles();
            if (synth.has_value()) {
              obstacles = std::make_shared<StaticObstacleModel>(
                  std::move(*synth), *scen.datum);
              tracker.setStaticObstacleModel(obstacles.get());
            } else if (!desc.static_obstacles_geojson_path.empty()) {
              std::ifstream probe(desc.static_obstacles_geojson_path);
              if (probe.good()) {
                try {
                  auto obs = loadStaticObstaclesGeoJson(
                      desc.static_obstacles_geojson_path);
                  obstacles = std::make_shared<StaticObstacleModel>(
                      std::move(obs), *scen.datum);
                  tracker.setStaticObstacleModel(obstacles.get());
                } catch (const std::exception&) {
                  // GeoJSON parse failure — proceed without obstacles.
                }
              }
            }
          }
```

- [ ] **Step 6: Add the `imm_cv_ct_pmbm_static` config in `Config.cpp`**

Find the `imm_cv_ct_pmbm_land` config block (~line 619). Add a new config that is a copy of it with the static-obstacle flags turned on:
- Set the benchmark-level `c.use_static_obstacle_model = true;` (keep `c.use_land_model = true;`).
- Inside its `pmbm_config` lambda, after the land settings, add `cfg.use_static_obstacle_model = true;`.
- Label: `"imm_cv_ct_pmbm_static"`.
Everything else (estimator/associator/bias-estimator wiring, `adaptive_birth`, `k_best`, `lambda_birth`, gate) is copied verbatim from `imm_cv_ct_pmbm_land`. This is the honest land-vs-land+static ablation; with no obstacle fixture it is bit-identical to `imm_cv_ct_pmbm_land`.

- [ ] **Step 7: Update the config-count test** — `tests/benchmark/test_config.cpp`

Change the size assertion from `30u` to `31u` (line 16). Add, next to the existing land label check (line 30):
```cpp
  EXPECT_EQ(labels.count("imm_cv_ct_pmbm_static"), 1u);
```

- [ ] **Step 8: Build and run**

`cmake --build build -j --target navtracker_tests && ./build/navtracker_tests --gtest_filter='StaticObstaclePipeline.*:*Config*'`
Expected: PASS (integration test + all config tests). Then run the full suite once: `ctest --test-dir build --output-on-failure` — expect all green (proves no bit-identical regression elsewhere).

- [ ] **Step 9: Commit**

```bash
git add core/benchmark/Config.hpp core/benchmark/Config.cpp core/benchmark/ScenarioRun.hpp \
        core/benchmark/Sweep.cpp tests/benchmark/test_config.cpp \
        tests/integration/test_static_obstacle_pipeline.cpp CMakeLists.txt
git commit -m "Static-obstacle Stage 1a: bench wiring (config/sweep/scenario) + integration scenario"
```

---

### Task 7: Documentation (learning chapter + figure, algorithm doc, contract, ADR/spec/baselines status)

**Files:**
- Create: `docs/learning/26-static-obstacles.md`
- Modify: `docs/learning/00-index.md`, `docs/learning/19-glossary.md`
- Modify: `docs/learning/figures/generate.py` (add one `fig_static_obstacle_*()` function + call it from `main()`); regenerate the PNG
- Create: `docs/algorithms/static-obstacle-birth-prior.md` (four-part: math / assumptions / rationale / what-to-test)
- Modify: `docs/output-contract.md` (add a `StaticHazardOutput` section)
- Modify: `docs/adr/0002-static-objects-track-vessels-map-environment.md` (Staging §1: mark "charted `StaticObstacle` input" as SHIPPED 2026-07-01; note the clutter-map reframe remains open)
- Modify: `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md` §14.10 (Staging: charted-input part shipped)
- Modify: `docs/algorithms/comparison-baselines.md` (split the Stage 1 row: charted input shipped; clutter-map reframe still open)
- Modify: `docs/algorithms/evaluation-log.md` (dated entry: what shipped, the safe-by-construction guarantee, the soft/hard gate design, that no bench measurement yet exists because no real fixture has charted obstacles — a follow-up)

**No production code changes in this task.** These are docs; the "test" is internal consistency + the figure regenerating cleanly.

- [ ] **Step 1: Write the algorithm reference** — `docs/algorithms/static-obstacle-birth-prior.md`

Four required sections (CLAUDE.md standard):
- **Math:** `birthSuppression` ramp `c(d)` (footprint core → 1.0; buffer → `soft_max·(R_soft−d)/(R_soft−R_hard)`; else 0); combined birth scale `(1−c_land)(1−c_static)`; hard-drop if either prior exceeds its gate. Keep-clear alarm: `d(own_ship, obstacle) < keep_clear` with exit hysteresis. Frames: geodetic chart store → ENU cache via datum; distance in ENU metres.
- **Assumptions:** charted position accuracy bounded by `position_uncertainty_m`; obstacles static; keep-clear radius encodes required clearance; `soft_max < static_obstacle_hard_gate` so the buffer never hard-drops a vessel; a real vessel gets repeated detections and re-births through the soft ramp.
- **Rationale:** why a separate port not a subclass (ADR 0002 decision 4); why multiplicative combination; why hard-gate only in the footprint (anchored-vessel trap); why geodetic store + ENU cache (moving platform — cross-reference the §14.10 fixed-vs-moving caveat and Herrmann 2025).
- **Ways to improve / test next:** Stage 1b (live clutter-map→hazard); Stage 2 evidential occupancy grid + stationary IMM mode; ENC/AIS-AtoN ingest; a real fixture with charted obstacles for an A/B bench measurement; sensor-aware near-shore birth.
Cross-link to the learning chapter.

- [ ] **Step 2: Write the learning chapter** — `docs/learning/26-static-obstacles.md`

Plain English (per CLAUDE.md tone rules + memory `feedback-easy-language`: short sentences, everyday words, no legal/medical/finance metaphors). Cover: the two meanings of "static" (a stopped boat is still a boat; a pier is the environment); why feeding a pier to a boat-tracker makes fake boats (the philos over-count); what a charted obstacle is; the hard core vs the soft keep-clear ring, with the passing-vessel protection; the keep-clear alarm; and a one-line pointer that "learning static from the sensors" (uncharted) is the next stage. Reference the figure. Add the chapter to `00-index.md` and add glossary entries (static obstacle, keep-clear radius, footprint, AtoN) to `19-glossary.md`.

- [ ] **Step 3: Add the figure**

Add a `fig_static_obstacle_zones()` function to `docs/learning/figures/generate.py` that draws: an obstacle with a red hard-footprint disc, an amber soft keep-clear ring, a green vessel track passing through the ring (annotated "still tracked"), and an ✗ phantom at the obstacle centre (annotated "suppressed"). Call it from `main()`. Regenerate per `docs/learning/figures/README.md` (matplotlib venv). Do not hand-edit the PNG. If the venv/network is unavailable in-sandbox, surface it as a blocker (do not fake the PNG).

- [ ] **Step 4: Update the contract, ADR, spec, baselines, eval-log**

- `docs/output-contract.md`: a `StaticHazardOutput` section — geodetic position (no NED rotation, and why: charted, not kinematic), keep-clear/footprint radii, attributes, `is_charted`, `hazard_id` stability, and that the keep-clear alarm is a static range check, not a CPA.
- ADR 0002 Staging §1 + design spec §14.10 Staging: mark the charted-input part shipped 2026-07-01 (reference this plan); clutter-map reframe still open.
- `comparison-baselines.md`: split the "Static-obstacle Stage 1" row into "Stage 1a charted input — **shipped**" and "Stage 1b clutter-map→hazard reframe — open".
- `evaluation-log.md`: dated entry summarising the ship, the safe-by-construction guarantee, the soft/hard-gate design, and that no A/B bench number exists yet (no real fixture has charted obstacles — a measurement follow-up; do not claim an improvement we haven't measured).

- [ ] **Step 5: Verify docs render + figure regenerated, then commit**

Confirm the PNG regenerated and the chapter/index/glossary cross-links resolve. Commit:
```bash
git add docs/learning/26-static-obstacles.md docs/learning/00-index.md docs/learning/19-glossary.md \
        docs/learning/figures/generate.py docs/learning/figures/*.png \
        docs/algorithms/static-obstacle-birth-prior.md docs/output-contract.md \
        docs/adr/0002-static-objects-track-vessels-map-environment.md \
        docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md \
        docs/algorithms/comparison-baselines.md docs/algorithms/evaluation-log.md
git commit -m "Static-obstacle Stage 1a: learning chapter + figure + algorithm doc + contract/ADR/baselines"
```

---

## Post-plan self-review (writing-plans)

- **Spec coverage:** ADR 0002 Stage 1 charted-input = Tasks 1-6; four-part algorithm doc + learning chapter + figure (CLAUDE.md) = Task 7; safe-by-construction = Task 2 null test + full-suite run in Task 6; ENC/S-101 attributes = Task 1 type + Task 5 adapter; separate-from-coastline = separate type/port/adapter throughout. Stage 1b explicitly deferred.
- **Type consistency:** `birthScale` (renamed from `landBirthScale`) used at both call sites; `StaticObstacle`/`IStaticObstacleModel`/`StaticObstacleModel`/`StaticHazardOutput`/`StaticHazardEvaluator`/`IStaticHazardSink` names consistent across tasks; `staticHazardId` defined in Task 3 and reused in Task 4's evaluator; benchmark flag `use_static_obstacle_model` (Config.hpp) distinct from and paired with the PMBM `Config::use_static_obstacle_model` (both required, mirroring land).
- **Placeholder scan:** all new headers, the adapter `.cpp`, the output `.cpp`, the evaluator `.cpp`, and every test carry complete code; the two `.cpp` bodies that live inside large existing files (PMBM call sites) are exact edits; the scenario/Config.cpp edits reference the exact mirror block to copy.
- **Known soft spots flagged for the implementer:** the PMBM Bernoulli state accessor in Task 6's integration test (verify against `PmbmTypes.hpp`); the figure venv availability; the `navtracker_tests` link to `navtracker_land`.
