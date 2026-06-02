# Heading Errors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire §14.9 end-to-end — let SimBus inject own-ship heading bias / drift / white noise, let ARPA and EO/IR adapters inflate the bearing variance by σ_heading, and quantify the tracker's degradation across the existing bus scenarios with and without R-inflation.

**Architecture:** Single math change in `projectRangeBearingToEnu` (add `sigma_heading_rad` parameter; combine angular variances in quadrature). Two adapter config structs (`ArpaAdapterConfig`, `EoIrAdapterConfig`) thread σ_heading through to the helper. `OwnShipEmitterConfig` gains `heading_noise_std_deg` and the emitter draws one Gaussian per HDT tick (bias + drift hooks already exist). Investigation lives in two new test files — a 24-cell sweep and a small bias/drift probe.

**Tech Stack:** C++17, Eigen, gtest, CMake/Conan.

**Spec:** `docs/superpowers/specs/2026-06-02-heading-errors-design.md` (especially §3 architecture, §4 components, §10 decision table).

**Context for the implementer:**
- All adapter constructor signatures get a new defaulted parameter so existing callers compile unchanged. Verify after each task that you haven't broken any of the 216 currently passing tests.
- `projectRangeBearingToEnu` adds `sigma_heading_rad` *before* `own_ship_pos_enu` — see Task 1 for the exact signature. Both call sites (`ArpaAdapter::ingest` TTM branch and `EoIrAdapter::ingest`) are updated in their respective tasks.
- ARPA TLL (absolute lat/lon) and `AisAdapter` do not consume own-ship heading and are not modified.
- The four existing bus comparisons (`test_bus_{jpda,imm3,pf,mht}_comparison.cpp`) and the regression tests (`test_bus_regression.cpp`) must continue to pass under default-zero σ_heading.
- Build via `cd build && cmake --build . -j`. Run filter via `./navtracker_tests --gtest_filter='…'`.

---

## File Structure

**Modify (core, source):**
- `adapters/util/Projection.hpp` / `.cpp` — add `sigma_heading_rad` parameter to `projectRangeBearingToEnu` (Task 1)
- `adapters/arpa/ArpaAdapter.hpp` / `.cpp` — add `ArpaAdapterConfig`, pass σ_h to projection helper on TTM-R path (Task 2)
- `adapters/eoir/EoIrAdapter.hpp` / `.cpp` — add `EoIrAdapterConfig`, pass σ_h to projection helper (Task 3)
- `sim/OwnShipEmitter.hpp` / `.cpp` — add `heading_noise_std_deg`, draw Gaussian per tick (Task 4)
- `tests/sim/BusComparisonHelpers.hpp` — extend the three scenario factories with a `HeadingSweepKnob` so the sweep can re-use them (Task 5)

**Create (tests):**
- `tests/adapters/util/test_projection.cpp` — append one new test (Task 1)
- `tests/adapters/arpa/test_arpa_adapter.cpp` — append one new test (Task 2)
- `tests/adapters/eoir/test_eoir_adapter.cpp` — append one new test (Task 3)
- `tests/sim/test_own_ship_emitter.cpp` — append one new test (Task 4)
- `tests/sim/test_bus_heading_sweep.cpp` — the 24-cell sweep (Task 6)
- `tests/sim/test_bus_heading_bias_drift_probe.cpp` — bias-only and drift-only single-seed probes (Task 7)

**Modify (docs):**
- `docs/algorithms/evaluation-log.md` — append "Heading error sweep (2026-06-02)" section (Task 8)

**CMakeLists.txt:** add the two new sweep test files. The append-only tests for projection, ARPA, EO/IR, and emitter live inside files already registered.

---

### Task 1: Add `sigma_heading_rad` to `projectRangeBearingToEnu`

Centralised math change. Both adapters route through this helper; combining the angular variances here keeps the rule in one place. Existing two unit tests stay green; one new test pins the new behaviour.

**Files:**
- Modify: `adapters/util/Projection.hpp`
- Modify: `adapters/util/Projection.cpp`
- Modify: `tests/adapters/util/test_projection.cpp` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/adapters/util/test_projection.cpp`:

```cpp
TEST(Projection, SigmaHeadingInflatesCrossTrackVarianceInQuadrature) {
  // Same configuration as the existing CovarianceAnisotropyMatchesPolarJacobian
  // test (1 km east of own-ship, 5 m range σ, 0.01 rad bearing σ), but
  // adds 0.02 rad of heading uncertainty. The expected effect is that the
  // total angular variance becomes 0.01² + 0.02² rad², and the ENU
  // covariance's "cross-track" eigenvalue grows proportionally.
  const Eigen::Vector2d own(0.0, 0.0);
  const double range = 1000.0;
  const double range_std = 5.0;
  const double bearing_std = 0.01;
  const double sigma_heading = 0.02;

  const auto zero_h = projectRangeBearingToEnu(
      range, kPi / 2.0, range_std, bearing_std, 0.0, own);
  const auto with_h = projectRangeBearingToEnu(
      range, kPi / 2.0, range_std, bearing_std, sigma_heading, own);

  // East-pointing target: along-range axis is east, cross-track axis is
  // north. So cov(1,1) (north–north) is the cross-track variance.
  const double expected_xtrack_no_h =
      (range * bearing_std) * (range * bearing_std);
  const double expected_xtrack_with_h =
      (range * range) * (bearing_std * bearing_std + sigma_heading * sigma_heading);

  EXPECT_NEAR(zero_h.cov(1, 1), expected_xtrack_no_h, 1e-6);
  EXPECT_NEAR(with_h.cov(1, 1), expected_xtrack_with_h, 1e-6);

  // Along-range variance (east–east) is unchanged.
  EXPECT_NEAR(zero_h.cov(0, 0), with_h.cov(0, 0), 1e-9);
}
```

If `kPi` is not in scope inside the test file, add `constexpr double kPi = 3.14159265358979323846;` at the top of the file's anonymous namespace (or match the existing convention — read the file first).

- [ ] **Step 2: Run to confirm it fails to compile**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -5
```

Expected: compile error on the `projectRangeBearingToEnu(..., 0.0, own)` call because the function takes 5 args, not 6.

- [ ] **Step 3: Add the parameter to the header**

Edit `adapters/util/Projection.hpp` to:

```cpp
#pragma once

#include <Eigen/Core>

namespace navtracker {

struct PointAndCov2D {
  Eigen::Vector2d pos_enu;
  Eigen::Matrix2d cov;
};

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       const Eigen::Vector2d& own_ship_pos_enu);

}  // namespace navtracker
```

- [ ] **Step 4: Update the implementation**

Edit `adapters/util/Projection.cpp` to:

```cpp
#include "adapters/util/Projection.hpp"

#include <cmath>

namespace navtracker {

PointAndCov2D projectRangeBearingToEnu(double range_m,
                                       double bearing_true_rad,
                                       double range_std_m,
                                       double bearing_std_rad,
                                       double sigma_heading_rad,
                                       const Eigen::Vector2d& own_ship_pos_enu) {
  const double sb = std::sin(bearing_true_rad);
  const double cb = std::cos(bearing_true_rad);

  PointAndCov2D out;
  out.pos_enu.x() = own_ship_pos_enu.x() + range_m * sb;
  out.pos_enu.y() = own_ship_pos_enu.y() + range_m * cb;

  Eigen::Matrix2d J;
  J << sb,  range_m * cb,
       cb, -range_m * sb;

  // Heading uncertainty adds to the angular component in quadrature: the
  // total angular variance is the sensor's intrinsic bearing variance plus
  // the own-ship heading variance, since both rotate the line of sight.
  const double angular_var =
      bearing_std_rad * bearing_std_rad +
      sigma_heading_rad * sigma_heading_rad;
  Eigen::Matrix2d R;
  R << range_std_m * range_std_m, 0.0,
       0.0, angular_var;
  out.cov = J * R * J.transpose();
  return out;
}

}  // namespace navtracker
```

- [ ] **Step 5: Update the two call sites so the build is green again**

In `adapters/arpa/ArpaAdapter.cpp`, find the `projectRangeBearingToEnu` call inside the TTM branch. It currently looks like:

```cpp
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad, 50.0, 1.0 * kDeg2Rad, own_xy);
```

Insert `0.0` between the `1.0 * kDeg2Rad` and `own_xy` so the new parameter is supplied:

```cpp
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad,
                             50.0, 1.0 * kDeg2Rad,
                             0.0,  // σ_heading; wired in Task 2
                             own_xy);
```

In `adapters/eoir/EoIrAdapter.cpp`, the call currently looks like:

```cpp
const PointAndCov2D out = projectRangeBearingToEnu(
    d.range_m, bearing_true_rad, d.range_std_m, d.bearing_std_deg * kDeg2Rad, own_xy);
```

Update to:

```cpp
const PointAndCov2D out = projectRangeBearingToEnu(
    d.range_m, bearing_true_rad,
    d.range_std_m, d.bearing_std_deg * kDeg2Rad,
    0.0,  // σ_heading; wired in Task 3
    own_xy);
```

- [ ] **Step 6: Build and run all projection tests**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='Projection.*' 2>&1 | tail -10
```

Expected: all three Projection tests PASS (the two existing plus the new one).

- [ ] **Step 7: Run the full suite to confirm zero regressions**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 217/217 (216 prior + 1 new) PASS.

- [ ] **Step 8: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add adapters/util/Projection.hpp adapters/util/Projection.cpp adapters/arpa/ArpaAdapter.cpp adapters/eoir/EoIrAdapter.cpp tests/adapters/util/test_projection.cpp && git -c commit.gpgsign=false commit -m "feat(projection): add sigma_heading_rad parameter to projectRangeBearingToEnu"
```

---

### Task 2: Plumb σ_heading through `ArpaAdapter`

Add a config struct so the adapter can be configured to inflate R by a non-zero σ_h. Backward-compatible: default `{}` matches today's behaviour.

**Files:**
- Modify: `adapters/arpa/ArpaAdapter.hpp`
- Modify: `adapters/arpa/ArpaAdapter.cpp`
- Modify: `tests/adapters/arpa/test_arpa_adapter.cpp` (append one test)

- [ ] **Step 1: Write the failing test**

Append to `tests/adapters/arpa/test_arpa_adapter.cpp`:

```cpp
TEST(ArpaAdapter, HeadingStdInflatesTtmCovariance) {
  // Same own-ship + target geometry as the existing TTM tests in this
  // file: own-ship at the datum origin heading 0°, ARPA target at range
  // 1 km on relative bearing 90° (so east in the ENU frame).
  navtracker::geo::Datum datum({53.5, 8.0, 0.0});
  navtracker::OwnShipProvider provider;
  navtracker::OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  pose.time = navtracker::Timestamp::fromSeconds(0.0);
  provider.update(pose);

  // Build two adapters: one with σ_h=0, one with σ_h=2 deg.
  navtracker::ArpaAdapter a0(datum, provider, navtracker::ArpaAdapterConfig{});
  navtracker::ArpaAdapter a2(datum, provider,
                             navtracker::ArpaAdapterConfig{/*heading_std_deg=*/2.0});

  const std::string ttm =
      "$RATTM,01,0.5399568,090.0,R,0.0,0.0,T,000.0,000.0,N,,A,*00";
  a0.ingest(ttm, navtracker::Timestamp::fromSeconds(1.0));
  a2.ingest(ttm, navtracker::Timestamp::fromSeconds(1.0));

  const auto m0 = a0.poll();
  const auto m2 = a2.poll();
  ASSERT_EQ(m0.size(), 1u);
  ASSERT_EQ(m2.size(), 1u);

  // East-pointing measurement: north–north (cov(1,1)) is the cross-track
  // variance. Inflation should make it strictly larger; east–east
  // (cov(0,0), along-range) should be unchanged.
  EXPECT_GT(m2[0].covariance(1, 1), m0[0].covariance(1, 1));
  EXPECT_NEAR(m0[0].covariance(0, 0), m2[0].covariance(0, 0), 1e-3);
}
```

If the `$RATTM` sentence above doesn't match the existing tests' format byte-for-byte, adopt whatever pattern those tests use (read the file first; the goal is one valid TTM sentence pointing east at 1 km range, not a specific bytes-on-the-wire).

Add necessary includes (`#include "adapters/own_ship/OwnShipProvider.hpp"`, etc.) at the top of the file if not already present.

- [ ] **Step 2: Build to confirm it fails to compile**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -5
```

Expected: error — no member `ArpaAdapterConfig` in `navtracker`, or constructor doesn't accept the third argument.

- [ ] **Step 3: Extend the header**

Edit `adapters/arpa/ArpaAdapter.hpp` to:

```cpp
#pragma once

#include <string_view>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

struct ArpaAdapterConfig {
  double heading_std_deg{0.0};
};

class ArpaAdapter : public ISensorAdapter {
 public:
  ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              ArpaAdapterConfig cfg = {});

  bool ingest(std::string_view line, Timestamp t);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  ArpaAdapterConfig cfg_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
```

- [ ] **Step 4: Wire the config into the implementation**

In `adapters/arpa/ArpaAdapter.cpp`:

Update the constructor (currently `ArpaAdapter::ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship) : datum_(std::move(datum)), own_ship_(own_ship) {}`) to:

```cpp
ArpaAdapter::ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
                         ArpaAdapterConfig cfg)
    : datum_(std::move(datum)), own_ship_(own_ship), cfg_(cfg) {}
```

Update the TTM-branch `projectRangeBearingToEnu` call (which Task 1 left at `0.0`) to use the config:

```cpp
const PointAndCov2D out =
    projectRangeBearingToEnu(range_m, bearing_true_rad,
                             50.0, 1.0 * kDeg2Rad,
                             cfg_.heading_std_deg * kDeg2Rad,
                             own_xy);
```

The TLL branch is unchanged.

- [ ] **Step 5: Build and run the ARPA tests**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='ArpaAdapter.*' 2>&1 | tail -10
```

Expected: all ARPA tests PASS (existing + new).

- [ ] **Step 6: Run the full suite**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 218/218 PASS.

- [ ] **Step 7: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add adapters/arpa/ArpaAdapter.hpp adapters/arpa/ArpaAdapter.cpp tests/adapters/arpa/test_arpa_adapter.cpp && git -c commit.gpgsign=false commit -m "feat(arpa): ArpaAdapterConfig.heading_std_deg threads sigma_h into TTM R"
```

---

### Task 3: Plumb σ_heading through `EoIrAdapter`

Same shape as Task 2 but for `EoIrAdapter`.

**Files:**
- Modify: `adapters/eoir/EoIrAdapter.hpp`
- Modify: `adapters/eoir/EoIrAdapter.cpp`
- Modify: `tests/adapters/eoir/test_eoir_adapter.cpp` (append one test)

- [ ] **Step 1: Write the failing test**

Append to `tests/adapters/eoir/test_eoir_adapter.cpp`:

```cpp
TEST(EoIrAdapter, HeadingStdInflatesCrossTrackCovariance) {
  // Own-ship at datum origin, heading 0°. Camera detection at relative
  // bearing 90° (east), range 1 km, bearing σ 0.5°, range σ 10 m.
  navtracker::geo::Datum datum({53.5, 8.0, 0.0});
  navtracker::OwnShipProvider provider;
  navtracker::OwnShipPose pose;
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  pose.time = navtracker::Timestamp::fromSeconds(0.0);
  provider.update(pose);

  navtracker::EoIrAdapter a0(datum, provider, navtracker::EoIrAdapterConfig{});
  navtracker::EoIrAdapter a2(datum, provider,
                             navtracker::EoIrAdapterConfig{/*heading_std_deg=*/2.0});

  navtracker::CameraDetection d;
  d.time = navtracker::Timestamp::fromSeconds(1.0);
  d.bearing_relative_deg = 90.0;
  d.range_m = 1000.0;
  d.bearing_std_deg = 0.5;
  d.range_std_m = 10.0;

  a0.ingest(d);
  a2.ingest(d);
  const auto m0 = a0.poll();
  const auto m2 = a2.poll();
  ASSERT_EQ(m0.size(), 1u);
  ASSERT_EQ(m2.size(), 1u);

  // East-pointing measurement: north–north (cov(1,1)) is the cross-track
  // variance — should grow. East–east (cov(0,0)) along-range should not.
  EXPECT_GT(m2[0].covariance(1, 1), m0[0].covariance(1, 1));
  EXPECT_NEAR(m0[0].covariance(0, 0), m2[0].covariance(0, 0), 1e-3);
}
```

Add missing includes at the top of the file if needed (`#include "adapters/own_ship/OwnShipProvider.hpp"` and friends).

- [ ] **Step 2: Build to confirm failure**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -5
```

Expected: compile error on `EoIrAdapterConfig`.

- [ ] **Step 3: Extend the header**

Edit `adapters/eoir/EoIrAdapter.hpp` to:

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

struct CameraDetection {
  Timestamp time;
  double bearing_relative_deg{0.0};
  double range_m{0.0};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  std::optional<std::int32_t> sensor_track_id;
  std::string source_id{"eo_ir"};
};

struct EoIrAdapterConfig {
  double heading_std_deg{0.0};
};

class EoIrAdapter : public ISensorAdapter {
 public:
  EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              EoIrAdapterConfig cfg = {});

  void ingest(const CameraDetection& d);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  EoIrAdapterConfig cfg_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
```

- [ ] **Step 4: Wire the config into the implementation**

In `adapters/eoir/EoIrAdapter.cpp`:

Update the constructor:

```cpp
EoIrAdapter::EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship,
                         EoIrAdapterConfig cfg)
    : datum_(std::move(datum)), own_ship_(own_ship), cfg_(cfg) {}
```

Update the projection call (left at `0.0` in Task 1):

```cpp
const PointAndCov2D out = projectRangeBearingToEnu(
    d.range_m, bearing_true_rad,
    d.range_std_m, d.bearing_std_deg * kDeg2Rad,
    cfg_.heading_std_deg * kDeg2Rad,
    own_xy);
```

- [ ] **Step 5: Build and run EO/IR tests**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='EoIrAdapter.*' 2>&1 | tail -10
```

Expected: all EoIrAdapter tests PASS.

- [ ] **Step 6: Run the full suite**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 219/219 PASS.

- [ ] **Step 7: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add adapters/eoir/EoIrAdapter.hpp adapters/eoir/EoIrAdapter.cpp tests/adapters/eoir/test_eoir_adapter.cpp && git -c commit.gpgsign=false commit -m "feat(eoir): EoIrAdapterConfig.heading_std_deg threads sigma_h into R"
```

---

### Task 4: Add heading noise to `OwnShipEmitter`

The bias and drift hooks already exist and are applied. We add white noise on top.

**Files:**
- Modify: `sim/OwnShipEmitter.hpp`
- Modify: `sim/OwnShipEmitter.cpp`
- Modify: `tests/sim/test_own_ship_emitter.cpp` (append one test)

- [ ] **Step 1: Write the failing test**

Append to `tests/sim/test_own_ship_emitter.cpp`:

```cpp
TEST(OwnShipEmitter, HeadingNoiseShowsUpAsExpectedStddev) {
  using namespace navtracker;
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 0.0;
  cfg.heading_true_deg = 0.0;
  cfg.heading_noise_std_deg = 2.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/42);

  // Pull a few hundred 1-Hz HDT ticks and accumulate the parsed heading.
  std::vector<double> samples;
  for (int k = 0; k < 400; ++k) {
    sim::EmitContext ctx;
    ctx.now = Timestamp::fromSeconds(static_cast<double>(k));
    emitter.emit(ctx);
    ASSERT_TRUE(provider.latest().has_value());
    double h = provider.latest()->heading_true_deg;
    // Heading wraps to [0, 360). Re-centre near 0 by mapping (180, 360) to
    // (-180, 0) so stats around the nominal 0° aren't broken by the wrap.
    if (h > 180.0) h -= 360.0;
    samples.push_back(h);
  }

  double mean = 0.0;
  for (double s : samples) mean += s;
  mean /= static_cast<double>(samples.size());
  double sse = 0.0;
  for (double s : samples) sse += (s - mean) * (s - mean);
  const double stddev =
      std::sqrt(sse / static_cast<double>(samples.size() - 1));

  EXPECT_NEAR(mean, 0.0, 0.3);          // sample mean near 0 with N=400
  EXPECT_NEAR(stddev, 2.0, 0.3);        // empirical stddev near σ
}
```

You will need `#include <cmath>` and `#include <vector>` if not already present.

- [ ] **Step 2: Build to confirm it fails to compile**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -5
```

Expected: error — no member `heading_noise_std_deg` in `OwnShipEmitterConfig`.

- [ ] **Step 3: Extend the config struct**

Edit `sim/OwnShipEmitter.hpp` to add the new field:

```cpp
struct OwnShipEmitterConfig {
  double dt_s{1.0};
  double gps_pos_std_m{5.0};
  double heading_true_deg{0.0};
  // §14.9 hooks (default zero — deferred per spec).
  double heading_bias_deg{0.0};
  double heading_drift_deg_per_s{0.0};
  double heading_noise_std_deg{0.0};
};
```

- [ ] **Step 4: Implement the noise draw**

Edit `sim/OwnShipEmitter.cpp` to include the heading noise. Inside `emit`, the HDT block computes `hdg` from nominal+bias+drift. Add a noise term before the wrap to [0, 360):

```cpp
// HDT — heading written via integer arithmetic to avoid locale dependence
// of snprintf %f.
{
  const double dt = next_emit_.secondsSince(t0_);
  double hdg = cfg_.heading_true_deg + cfg_.heading_bias_deg +
               cfg_.heading_drift_deg_per_s * dt;
  if (cfg_.heading_noise_std_deg > 0.0) {
    std::normal_distribution<double> h_noise(0.0, cfg_.heading_noise_std_deg);
    hdg += h_noise(rng_);
  }
  double hdg_norm = std::fmod(hdg, 360.0);
  if (hdg_norm < 0.0) hdg_norm += 360.0;
  // … rest unchanged …
```

The existing `noise_` member is the GPS-position distribution and stays in use for `nx`/`ny` above. We construct the heading distribution on the fly each tick — cheap, and avoids needing a second member field. Same `rng_` substream is fine: tick count is deterministic given the trajectory and dt, so reproducibility holds.

- [ ] **Step 5: Build and run OwnShipEmitter tests**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='OwnShipEmitter.*' 2>&1 | tail -15
```

Expected: all OwnShipEmitter tests PASS, including the new one.

- [ ] **Step 6: Run the full suite**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 220/220 PASS.

- [ ] **Step 7: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add sim/OwnShipEmitter.hpp sim/OwnShipEmitter.cpp tests/sim/test_own_ship_emitter.cpp && git -c commit.gpgsign=false commit -m "feat(sim): heading_noise_std_deg in OwnShipEmitter"
```

---

### Task 5: Extend bus comparison helpers with a `HeadingSweepKnob`

Add a single new variant function per scenario that accepts a knob struct, so the sweep test in Task 6 can call them. We keep the existing zero-arg helpers (and their callers in the four bus comparison tests) unchanged.

**Files:**
- Modify: `tests/sim/BusComparisonHelpers.hpp` (append the new struct + three new factory variants)

- [ ] **Step 1: Append the new types and helpers**

Append to the end of `tests/sim/BusComparisonHelpers.hpp` (inside `namespace navtracker_test`):

```cpp
struct HeadingSweepKnob {
  double sigma_heading_deg{0.0};        // per-tick white noise on HDT
  double bias_deg{0.0};                 // constant offset
  double drift_deg_per_s{0.0};          // linear-in-time offset
  bool   r_inflation_on{false};         // pass σ_h through to adapter cfg
};

inline navtracker::Scenario runBusClutterCrossingWithHeading(
    std::uint32_t seed, int clutter_per_rotation,
    const HeadingSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);

  ArpaAdapterConfig arpa_cfg_adapter;
  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) {
    arpa_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
    eo_cfg_adapter.heading_std_deg   = knob.sigma_heading_deg;
  }
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 30.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-200.0,  5.0), Eigen::Vector2d(15.0, 0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d( 200.0, -5.0), Eigen::Vector2d(-15.0, 0.0),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_emitter_cfg;
  arpa_emitter_cfg.targets.push_back({1, 1});
  arpa_emitter_cfg.targets.push_back({2, 2});
  arpa_emitter_cfg.clutter_per_rotation = clutter_per_rotation;
  bus.attachArpa(arpa_adapter, arpa_emitter_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.targets.push_back({2, 2});
  eo_emitter_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  return bus.run();
}

inline navtracker::Scenario runBusBearingOnlyMovingWithHeading(
    std::uint32_t seed, const HeadingSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);

  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) eo_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
  EoIrAdapter eo_adapter(datum, provider, eo_cfg_adapter);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 60.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0), Eigen::Vector2d(0.0, 10.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(1500.0, 0.0), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.fov_deg = 360.0;
  eo_emitter_cfg.range_mode = sim::EoIrEmitterConfig::RangeMode::BearingOnly;
  eo_emitter_cfg.bearing_std_deg = 1.5;
  eo_emitter_cfg.dt_s = 1.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  return bus.run();
}

inline navtracker::Scenario runBusManeuveringWithHeading(
    std::uint32_t seed, const HeadingSweepKnob& knob) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter ais_adapter(datum);

  ArpaAdapterConfig arpa_cfg_adapter;
  EoIrAdapterConfig eo_cfg_adapter;
  if (knob.r_inflation_on) {
    arpa_cfg_adapter.heading_std_deg = knob.sigma_heading_deg;
    eo_cfg_adapter.heading_std_deg   = knob.sigma_heading_deg;
  }
  ArpaAdapter arpa_adapter(datum, provider, arpa_cfg_adapter);
  EoIrAdapter eo_adapter (datum, provider, eo_cfg_adapter);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 15.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ManeuveringTrajectory>(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      /*straight=*/5.0, /*turn=*/5.0, /*omega=*/0.2,
      Timestamp::fromSeconds(0.0)));

  sim::OwnShipEmitterConfig own_cfg;
  own_cfg.heading_bias_deg = knob.bias_deg;
  own_cfg.heading_drift_deg_per_s = knob.drift_deg_per_s;
  own_cfg.heading_noise_std_deg = knob.sigma_heading_deg;
  bus.attachOwnShip(own_adapter, own_cfg);

  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  bus.attachAis(ais_adapter, ais_cfg);

  sim::ArpaEmitterConfig arpa_emitter_cfg;
  arpa_emitter_cfg.targets.push_back({1, 1});
  bus.attachArpa(arpa_adapter, arpa_emitter_cfg);

  sim::EoIrEmitterConfig eo_emitter_cfg;
  eo_emitter_cfg.targets.push_back({1, 1});
  eo_emitter_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_emitter_cfg);

  return bus.run();
}
```

Add any missing includes at the top of the file: the new helpers reference `ArpaAdapterConfig`, `EoIrAdapterConfig`, and the existing adapter constructors are already included.

- [ ] **Step 2: Build to confirm the file compiles**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3
```

Expected: build succeeds. (No tests exercise the new helpers yet.)

- [ ] **Step 3: Run the full suite to confirm no collateral damage**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 220/220 PASS (unchanged from Task 4).

- [ ] **Step 4: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add tests/sim/BusComparisonHelpers.hpp && git -c commit.gpgsign=false commit -m "test(sim): HeadingSweepKnob + sweep-friendly scenario helpers"
```

---

### Task 6: Heading sweep test (24 cells × 20 seeds)

Builds the headline experiment. EKF + GNN throughout, since the goal is to characterise the error model, not redo the algorithm bake-off.

**Files:**
- Create: `tests/sim/test_bus_heading_sweep.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the sweep test**

Create `tests/sim/test_bus_heading_sweep.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

// Run a single (scenario, knob) cell with EKF+GNN, return one RunStats.
RunStats runStandardCell(
    navtracker::Scenario&& s, double cutoff, int confirm, int del,
    double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

RunStats runBatchedCell(
    navtracker::Scenario&& s, double cutoff, int confirm, int del,
    double miss_timeout, double q_proc) {
  auto motion = std::make_shared<ConstantVelocity2D>(q_proc);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(confirm, del);
  Tracker tracker(ekf, gnn, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

struct Cell {
  const char* scenario_name;
  double sigma_heading_deg;
  bool r_inflation_on;
  AggStats agg;
};

void printCellsTable(const char* scenario_name,
                     const std::vector<Cell>& cells) {
  std::fprintf(stderr,
      "\n[Bus Heading Sweep on %s, %d seeds]\n"
      "  sigma_h_deg | R_inflate | per-window OSPA mean   | id_sw_mean\n",
      scenario_name, kNumSeeds);
  for (const Cell& c : cells) {
    std::fprintf(stderr,
        "  %10.2f  | %-9s | %7.4f +/- %6.4f m | %.2f\n",
        c.sigma_heading_deg,
        c.r_inflation_on ? "on" : "off",
        c.agg.mean_ospa, c.agg.std_ospa, c.agg.mean_id_sw);
  }
}

}  // namespace

TEST(BusHeadingSweep, ClutterCrossing) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runBatchedCell(
            runBusClutterCrossingWithHeading(seed, /*clutter=*/5, knob),
            /*cutoff=*/50.0, /*confirm=*/2, /*del=*/4, /*miss=*/30.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"ClutterCrossing", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("ClutterCrossing", cells);
  SUCCEED();
}

TEST(BusHeadingSweep, BearingOnlyMoving) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runStandardCell(
            runBusBearingOnlyMovingWithHeading(seed, knob),
            /*cutoff=*/500.0, /*confirm=*/1, /*del=*/8, /*miss=*/90.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"BearingOnlyMoving", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("BearingOnlyMoving", cells);
  SUCCEED();
}

TEST(BusHeadingSweep, Maneuvering) {
  const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
  const bool   r_on[]  = {false, true};

  std::vector<Cell> cells;
  for (double sh : sigmas) {
    for (bool ron : r_on) {
      std::vector<RunStats> runs;
      for (int k = 0; k < kNumSeeds; ++k) {
        const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
        HeadingSweepKnob knob;
        knob.sigma_heading_deg = sh;
        knob.r_inflation_on = ron;
        runs.push_back(runStandardCell(
            runBusManeuveringWithHeading(seed, knob),
            /*cutoff=*/100.0, /*confirm=*/1, /*del=*/5, /*miss=*/10.0,
            /*q=*/0.1));
      }
      cells.push_back(Cell{"Maneuvering", sh, ron, aggregate(runs)});
    }
  }
  printCellsTable("Maneuvering", cells);
  SUCCEED();
}
```

- [ ] **Step 2: Register the test in `CMakeLists.txt`**

Add the new source file to the `add_executable(navtracker_tests …)` list, alongside the other `tests/sim/test_bus_*` entries. Specifically, after the line `tests/sim/test_bus_mht_comparison.cpp`, insert:

```cmake
  tests/sim/test_bus_heading_sweep.cpp
```

- [ ] **Step 3: Build and run the sweep**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='BusHeadingSweep.*' 2>&1 | tail -60
```

Expected: build succeeds; all three `BusHeadingSweep.*` tests PASS (soft `SUCCEED()` — they always pass). Capture and save the three printed tables.

- [ ] **Step 4: Run the full suite**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 223/223 PASS (220 + 3 new sweep tests).

- [ ] **Step 5: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add tests/sim/test_bus_heading_sweep.cpp CMakeLists.txt && git -c commit.gpgsign=false commit -m "test(sim): heading-error sweep across three bus scenarios"
```

---

### Task 7: Bias / drift probe

A small, single-seed test confirming bias and drift propagate. Not a sweep — just "yes the wires are connected."

**Files:**
- Create: `tests/sim/test_bus_heading_bias_drift_probe.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the probe**

Create `tests/sim/test_bus_heading_bias_drift_probe.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/Metrics.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

double runOneBearingOnly(std::uint32_t seed, const HeadingSweepKnob& knob) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  GnnAssociator gnn(20.0);
  TrackManager mgr(1, 8);
  Tracker tracker(ekf, gnn, mgr, 90.0);
  const ScenarioResult r = runScenario(
      runBusBearingOnlyMovingWithHeading(seed, knob),
      tracker, mgr, /*cutoff=*/500.0);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return pw.mean;
}

}  // namespace

TEST(BusHeadingProbe, ConstantBiasShiftsOspa) {
  // Single seed, σ=0, R-inflation off. A 1° constant bias on the HDT
  // should cause the projected bearing-only track to sit ~range·sin(1°)
  // ≈ 26 m to one side of truth at 1.5 km range. Pre-check is just
  // "OSPA goes up", which is the wired-up signal.
  HeadingSweepKnob no_err;
  HeadingSweepKnob with_bias;
  with_bias.bias_deg = 1.0;
  const double ospa_clean = runOneBearingOnly(201u, no_err);
  const double ospa_biased = runOneBearingOnly(201u, with_bias);
  std::fprintf(stderr,
      "\n[Bus Heading Probe: BearingOnlyMoving, seed=201]\n"
      "  no error   : per-window OSPA mean = %.4f m\n"
      "  bias 1 deg : per-window OSPA mean = %.4f m\n",
      ospa_clean, ospa_biased);
  EXPECT_GT(ospa_biased, ospa_clean);
}

TEST(BusHeadingProbe, LinearDriftShiftsOspa) {
  // 0.01 deg/s drift over the 60 s scenario yields a 0.6° final heading
  // offset; expect the OSPA mean to rise relative to the no-error case.
  HeadingSweepKnob no_err;
  HeadingSweepKnob with_drift;
  with_drift.drift_deg_per_s = 0.01;
  const double ospa_clean = runOneBearingOnly(201u, no_err);
  const double ospa_drift = runOneBearingOnly(201u, with_drift);
  std::fprintf(stderr,
      "\n[Bus Heading Probe: BearingOnlyMoving, seed=201]\n"
      "  no error     : per-window OSPA mean = %.4f m\n"
      "  drift 0.01/s : per-window OSPA mean = %.4f m\n",
      ospa_clean, ospa_drift);
  EXPECT_GT(ospa_drift, ospa_clean);
}
```

- [ ] **Step 2: Register in CMakeLists.txt**

Add to the `add_executable(navtracker_tests …)` list, alongside the heading sweep entry:

```cmake
  tests/sim/test_bus_heading_bias_drift_probe.cpp
```

- [ ] **Step 3: Build and run the probe**

```bash
cd /home/andreas/workspace/navtracker/build && cmake --build . -j 2>&1 | tail -3 && ./navtracker_tests --gtest_filter='BusHeadingProbe.*' 2>&1 | tail -20
```

Expected: both probes PASS; capture the printed OSPA numbers.

If a probe FAILS (`bias_ospa <= clean_ospa`), the most likely cause is that the scenario's bearing-only-at-1.5km isn't sensitive enough at this bias magnitude to overcome noise on this single seed. In that case, bump the bias from 1° to 3° and report; do not silently suppress the failure.

- [ ] **Step 4: Full suite**

```bash
cd /home/andreas/workspace/navtracker/build && ./navtracker_tests 2>&1 | tail -3
```

Expected: 225/225 PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add tests/sim/test_bus_heading_bias_drift_probe.cpp CMakeLists.txt && git -c commit.gpgsign=false commit -m "test(sim): bias/drift propagation probe through bus + tracker"
```

---

### Task 8: Append "Heading error sweep" to eval-log

Record the findings in the lasting documentation.

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

- [ ] **Step 1: Collect the printed tables**

You captured three sweep tables in Task 6 (ClutterCrossing, BearingOnlyMoving, Maneuvering) and two probe lines in Task 7 (bias, drift). Have them at hand.

- [ ] **Step 2: Append the new section**

Append to `docs/algorithms/evaluation-log.md`:

```markdown
## Heading error sweep (2026-06-02)

§14.9 wired end-to-end. Own-ship HDT now carries injected bias / drift /
white noise; `ArpaAdapter` and `EoIrAdapter` accept a `heading_std_deg`
that propagates through `projectRangeBearingToEnu` into the bearing
variance (combined in quadrature with the sensor's intrinsic σ).

Sweep: EKF + GNN, 20 seeds (201..220), σ_h ∈ {0°, 0.5°, 1°, 2°},
R-inflation off vs on. Three scenarios re-used from the bus comparison
helpers.

### ClutterCrossing (targets at ~200 m range)

<paste the table printed by BusHeadingSweep.ClutterCrossing>

### BearingOnlyMoving (target at 1.5 km range — headline)

<paste the table printed by BusHeadingSweep.BearingOnlyMoving>

### Maneuvering (single target, 15 s scenario)

<paste the table printed by BusHeadingSweep.Maneuvering>

### Bias / drift propagation probe

<paste the BusHeadingProbe.ConstantBiasShiftsOspa and
BusHeadingProbe.LinearDriftShiftsOspa numbers>

### Verdict

<write 4–8 sentences interpreting the tables. Cover:
- Does the bearing-only case show the expected ~range·σ_h cross-track
  degradation when R-inflation is off?
- Does turning R-inflation on recover most of the lost accuracy?
- Are the short-range scenarios (Crossing / Maneuvering) measurably
  affected, or is the short range protective?
- Does the bias/drift probe show the expected directional shift?>

### Methodology notes

- Per-window OSPA at 1 s windows (truth-tick clock).
- Heading noise is white (per-tick i.i.d. Gaussian). No process model.
- Bias and drift held at 0 during the sweep; they get a separate probe.
- Sweep uses one canonical tracker per scenario (EKF + GNN). The
  comparison vs other estimators / associators is intentionally not
  re-run; the question here is the error model, not the algorithm.
- Determinism: each seed produces a byte-identical Scenario.
- Tests live at `tests/sim/test_bus_heading_sweep.cpp` and
  `tests/sim/test_bus_heading_bias_drift_probe.cpp`.
```

Replace each `<paste …>` and `<write …>` block with the captured / interpreted content. Don't leave any angle-bracket placeholders.

- [ ] **Step 3: Commit**

```bash
cd /home/andreas/workspace/navtracker && git add docs/algorithms/evaluation-log.md && git -c commit.gpgsign=false commit -m "docs(eval): heading error sweep verdict"
```

---

## Self-review

**Spec coverage:**
- §4.1 OwnShipEmitter heading noise → Task 4. ✓
- §4.2 projectRangeBearingToEnu σ_h parameter → Task 1. ✓
- §4.3 ArpaAdapterConfig → Task 2. ✓
- §4.4 EoIrAdapterConfig → Task 3. ✓
- §4.5 sweep harness + bias/drift probe → Tasks 5, 6, 7. ✓
- §6 assumptions (white noise, no parameter noise) → encoded in test designs. ✓
- §7 backward-compatibility (default 0) → preserved by Task 2/3 default cfg parameters. ✓
- §9 testing strategy bullets → unit tests in Tasks 1–4 + scenario tests in Tasks 6–7. ✓
- §10 decision table all rows → addressed across tasks. ✓
- Eval-log entry → Task 8. ✓

**Placeholder scan:** §4.5-style `<knob>` placeholders appear only in Task 8 inside the eval-log template, where they explicitly direct the implementer to substitute captured run output. No prescriptive code or commands left as TBD. The `<paste …>` and `<write …>` brackets are flagged in-line as substitution sites.

**Type consistency:** `ArpaAdapterConfig` / `EoIrAdapterConfig` introduced in Tasks 2/3, used in Task 5/6/7. `HeadingSweepKnob` introduced in Task 5, used in Tasks 6/7. `RunStats` / `AggStats` / `aggregate` / `kNumSeeds` / `kWindowDtS` are existing types in `BusComparisonHelpers.hpp` and reused unchanged. `projectRangeBearingToEnu` signature is consistent between Task 1 (added) and Tasks 2/3 (used with config values).
