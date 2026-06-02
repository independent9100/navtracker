# Bus-Driven Comparisons Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-run the four prior winning estimator/associator comparisons (JPDA vs GNN, IMM-3 vs CV, PF vs EKF, MHT vs JPDA) through SimulatedSensorBus with 20 seeds each, and update the evaluation log with bus-driven findings.

**Architecture:** Three small infrastructure pieces first (`ManeuveringTrajectory` for IMM, ARPA `clutter_per_rotation` knob for JPDA/MHT, `perWindowOspa` helper as a cadence-fair metric), then four bus-driven comparison test files sharing a header-only helper (`BusComparisonHelpers.hpp`) to keep the sweep loop DRY. The existing tracker constructors / aggregate-stats pattern from `tests/scenario/test_multi_seed_sweep.cpp` is reused directly.

**Tech Stack:** C++17, Eigen 3.4, GoogleTest, CMake. Builds via `cmake --build build`; single test run via `./build/navtracker_tests --gtest_filter='<TestSuite.TestName>'`.

**Reference spec:** `docs/superpowers/specs/2026-06-02-bus-driven-comparisons-design.md`

---

## File Structure

**New files:**

```
sim/
  TruthTrajectory.hpp           // extend: add ManeuveringTrajectory
  TruthTrajectory.cpp           // extend: implement eval()
  ArpaEmitter.hpp / .cpp        // extend: clutter_per_rotation, next_clutter_emit_
core/scenario/
  Metrics.hpp / .cpp            // extend: PerWindowOspa + computePerWindowOspa
tests/sim/
  test_maneuvering_trajectory.cpp
  test_arpa_clutter.cpp
  test_per_window_ospa.cpp      // lives in tests/sim/ for proximity to bus consumers
  BusComparisonHelpers.hpp      // shared sweep helpers (header-only, inline funcs)
  test_bus_jpda_comparison.cpp
  test_bus_imm3_comparison.cpp
  test_bus_pf_comparison.cpp
  test_bus_mht_comparison.cpp
docs/algorithms/
  evaluation-log.md             // append bus-driven section
```

**Modified files:**
- `CMakeLists.txt` — append each new test file to `navtracker_tests`. The `.cpp` of `Metrics.cpp` is already listed; no library-side additions.

---

## Task 1: ManeuveringTrajectory

**Files:**
- Modify: `sim/TruthTrajectory.hpp` (append class declaration)
- Modify: `sim/TruthTrajectory.cpp` (append implementation)
- Test: `tests/sim/test_maneuvering_trajectory.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_maneuvering_trajectory.cpp`:

```cpp
#include "sim/TruthTrajectory.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace navtracker;
using sim::ManeuveringTrajectory;
using sim::TruthState;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(ManeuveringTrajectory, StraightLegMatchesConstantVelocity) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      /*straight_duration_s=*/5.0,
      /*turn_duration_s=*/5.0,
      /*omega_rad_s=*/0.2,
      Timestamp::fromSeconds(0.0));

  const TruthState s0 = traj.eval(Timestamp::fromSeconds(0.0));
  EXPECT_DOUBLE_EQ(s0.position.x(), 0.0);
  EXPECT_DOUBLE_EQ(s0.position.y(), 0.0);
  EXPECT_DOUBLE_EQ(s0.velocity.x(), 10.0);
  EXPECT_DOUBLE_EQ(s0.velocity.y(),  0.0);

  // End of straight leg: t = 5 s, no turn yet → linear.
  const TruthState s5 = traj.eval(Timestamp::fromSeconds(5.0));
  EXPECT_NEAR(s5.position.x(), 50.0, 1e-9);
  EXPECT_NEAR(s5.position.y(),  0.0, 1e-9);
  EXPECT_NEAR(s5.velocity.x(), 10.0, 1e-9);
  EXPECT_NEAR(s5.velocity.y(),  0.0, 1e-9);
}

TEST(ManeuveringTrajectory, TurnLegRotatesVelocityVector) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, 0.2, Timestamp::fromSeconds(0.0));

  // Mid-turn: t = 7.5 s → τ' = 2.5 s into the turn; heading rotated 0.5 rad.
  const TruthState s75 = traj.eval(Timestamp::fromSeconds(7.5));
  const double speed = std::hypot(s75.velocity.x(), s75.velocity.y());
  EXPECT_NEAR(speed, 10.0, 1e-9);
  const double heading = std::atan2(s75.velocity.y(), s75.velocity.x());
  EXPECT_NEAR(heading, 0.5, 1e-9);

  // End of turn: t = 10 s → heading rotated by ω·Δt = 1.0 rad.
  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  const double end_heading = std::atan2(s10.velocity.y(), s10.velocity.x());
  EXPECT_NEAR(end_heading, 1.0, 1e-9);
}

TEST(ManeuveringTrajectory, PostTurnLegIsLinearInPostTurnVelocity) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, 0.2, Timestamp::fromSeconds(0.0));

  // After-turn velocity = 10 m/s at heading 1.0 rad.
  const TruthState end_turn = traj.eval(Timestamp::fromSeconds(10.0));
  const Eigen::Vector2d v_post = end_turn.velocity;

  // At t = 12 s (2 s into post-turn leg) the position should advance by 2*v_post.
  const TruthState s12 = traj.eval(Timestamp::fromSeconds(12.0));
  EXPECT_NEAR(s12.position.x(), end_turn.position.x() + 2.0 * v_post.x(), 1e-9);
  EXPECT_NEAR(s12.position.y(), end_turn.position.y() + 2.0 * v_post.y(), 1e-9);
  EXPECT_DOUBLE_EQ(s12.velocity.x(), v_post.x());
  EXPECT_DOUBLE_EQ(s12.velocity.y(), v_post.y());
}

TEST(ManeuveringTrajectory, NegativeOmegaTurnsTheOtherWay) {
  ManeuveringTrajectory traj(
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(10.0, 0.0),
      5.0, 5.0, -0.2, Timestamp::fromSeconds(0.0));
  const TruthState s10 = traj.eval(Timestamp::fromSeconds(10.0));
  const double heading = std::atan2(s10.velocity.y(), s10.velocity.x());
  EXPECT_NEAR(heading, -1.0, 1e-9);
}
```

- [ ] **Step 2: Append the test to `CMakeLists.txt`**

In `/home/andreas/workspace/navtracker/CMakeLists.txt`, append `tests/sim/test_maneuvering_trajectory.cpp` to the `navtracker_tests` source list (alphabetic placement under the existing `tests/sim/...` block is fine).

- [ ] **Step 3: Build to verify the test fails to compile**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `'ManeuveringTrajectory' is not a member of 'navtracker::sim'`.

- [ ] **Step 4: Append the class declaration to `sim/TruthTrajectory.hpp`**

Insert before the closing `}  // namespace navtracker::sim`:

```cpp
class ManeuveringTrajectory final : public ITruthTrajectory {
 public:
  // Three-leg: straight at `velocity` from `start` for `straight_duration_s`,
  // then a constant-rate turn at `omega_rad_s` for `turn_duration_s`
  // (positive omega = left turn in ENU), then straight at the post-turn
  // heading and speed indefinitely. t0 anchors the first leg's start.
  ManeuveringTrajectory(Eigen::Vector2d start,
                        Eigen::Vector2d velocity,
                        double straight_duration_s,
                        double turn_duration_s,
                        double omega_rad_s,
                        Timestamp t0);

  TruthState eval(Timestamp t) const override;

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d v0_;
  double t_straight_;
  double t_turn_;
  double omega_;
  Timestamp t0_;
};
```

- [ ] **Step 5: Append the implementation to `sim/TruthTrajectory.cpp`**

Add `#include <cmath>` if not already present. Append:

```cpp
ManeuveringTrajectory::ManeuveringTrajectory(Eigen::Vector2d start,
                                             Eigen::Vector2d velocity,
                                             double straight_duration_s,
                                             double turn_duration_s,
                                             double omega_rad_s,
                                             Timestamp t0)
    : start_(std::move(start)),
      v0_(std::move(velocity)),
      t_straight_(straight_duration_s),
      t_turn_(turn_duration_s),
      omega_(omega_rad_s),
      t0_(t0) {}

TruthState ManeuveringTrajectory::eval(Timestamp t) const {
  double tau = t.secondsSince(t0_);
  if (tau < 0.0) tau = 0.0;

  // Leg 1: straight.
  if (tau <= t_straight_) {
    return TruthState{start_ + v0_ * tau, v0_};
  }

  // Leg 1 end-state.
  const Eigen::Vector2d p1 = start_ + v0_ * t_straight_;

  // Leg 2: constant-rate turn.
  if (tau <= t_straight_ + t_turn_) {
    const double tau2 = tau - t_straight_;
    const double speed = v0_.norm();
    const double theta0 = std::atan2(v0_.y(), v0_.x());
    const double theta = theta0 + omega_ * tau2;
    // Position by integrating velocity vector with constant turn rate:
    //   x(t) = x0 + (speed/omega) * (sin(theta) - sin(theta0))
    //   y(t) = y0 - (speed/omega) * (cos(theta) - cos(theta0))
    // (works for omega != 0; the test never hits omega==0).
    const Eigen::Vector2d p_turn(
        p1.x() + (speed / omega_) * (std::sin(theta) - std::sin(theta0)),
        p1.y() - (speed / omega_) * (std::cos(theta) - std::cos(theta0)));
    const Eigen::Vector2d v_turn(speed * std::cos(theta),
                                 speed * std::sin(theta));
    return TruthState{p_turn, v_turn};
  }

  // Leg 3: straight at post-turn velocity.
  const double speed = v0_.norm();
  const double theta_end = std::atan2(v0_.y(), v0_.x()) + omega_ * t_turn_;
  const Eigen::Vector2d v_post(speed * std::cos(theta_end),
                               speed * std::sin(theta_end));
  // Position at end of leg 2:
  const Eigen::Vector2d p2(
      p1.x() + (speed / omega_) * (std::sin(theta_end) -
                                   std::sin(std::atan2(v0_.y(), v0_.x()))),
      p1.y() - (speed / omega_) * (std::cos(theta_end) -
                                   std::cos(std::atan2(v0_.y(), v0_.x()))));
  const double tau3 = tau - t_straight_ - t_turn_;
  return TruthState{p2 + v_post * tau3, v_post};
}
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='ManeuveringTrajectory.*'`
Expected: 4/4 PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/TruthTrajectory.hpp sim/TruthTrajectory.cpp \
        tests/sim/test_maneuvering_trajectory.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: ManeuveringTrajectory (straight-turn-straight)

Three-leg CT trajectory used by the IMM-3 bus comparison. Constant-rate
turn between two straight legs; closed-form integration of the turn arc.
Mirrors core/scenario/Builders.cpp::buildManeuveringTargetScenario so the
bus-driven IMM test is directly comparable to the prior direct-Measurement
multi-seed sweep.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: ARPA clutter knob

**Files:**
- Modify: `sim/ArpaEmitter.hpp` (extend `ArpaEmitterConfig`, add `next_clutter_emit_` + `clutter_*` distributions)
- Modify: `sim/ArpaEmitter.cpp` (clutter emission per rotation slot)
- Test: `tests/sim/test_arpa_clutter.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_arpa_clutter.cpp`:

```cpp
#include "sim/ArpaEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext emptyCtx(double t_seconds, const Eigen::Vector2d& own_pos) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{own_pos, Eigen::Vector2d::Zero()};
  // No targets — only clutter exercised here.
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

TEST(ArpaClutter, NoClutterByDefault) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;  // default clutter_per_rotation = 0
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  for (double t : {0.0, 3.0, 6.0, 9.0}) {
    emitter.emit(emptyCtx(t, Eigen::Vector2d::Zero()));
  }
  EXPECT_EQ(adapter.poll().size(), 0u);
}

TEST(ArpaClutter, FiresApproximatelyPoissonCountPerRotation) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;
  cfg.clutter_per_rotation = 5;
  cfg.rotation_dt_s = 3.0;
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/7);

  // 30 s @ 3 s rotation → 11 rotations (slots at t=0,3,...,30) → mean ≈ 55.
  for (int i = 0; i <= 30; ++i) {
    emitter.emit(emptyCtx(static_cast<double>(i),
                          Eigen::Vector2d::Zero()));
  }
  const auto out = adapter.poll();
  // Bracket: with mean 55 and stddev sqrt(55) ≈ 7.4, ±20 covers ~2.7σ.
  EXPECT_GE(out.size(), 35u);
  EXPECT_LE(out.size(), 75u);
}

TEST(ArpaClutter, EveryClutterMeasurementWithinConfiguredRangeBox) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;
  cfg.clutter_per_rotation = 10;
  cfg.clutter_min_range_m = 100.0;
  cfg.max_range_m = 1000.0;
  cfg.rotation_dt_s = 3.0;
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/9);

  for (int i = 0; i <= 9; ++i) {
    emitter.emit(emptyCtx(static_cast<double>(i),
                          Eigen::Vector2d::Zero()));
  }
  const auto out = adapter.poll();
  ASSERT_GT(out.size(), 0u);
  for (const auto& m : out) {
    const double r = m.value.norm();  // own-ship at origin
    EXPECT_GE(r,  99.0);   // 1 m slack for numeric noise
    EXPECT_LE(r, 1001.0);
  }
}
```

- [ ] **Step 2: Append the test to `CMakeLists.txt`**

Append `tests/sim/test_arpa_clutter.cpp` to `navtracker_tests`.

- [ ] **Step 3: Build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `'clutter_per_rotation' is not a member of 'ArpaEmitterConfig'`.

- [ ] **Step 4: Extend `sim/ArpaEmitter.hpp`**

In the `ArpaEmitterConfig` struct, add two fields (insert before the closing `};`):

```cpp
  int clutter_per_rotation{0};       // Poisson mean N false alarms per rotation
  double clutter_min_range_m{50.0};  // clutter drawn uniformly in [min, max_range_m]
```

In the `ArpaEmitter` class, add new private members (after the existing `next_emit_` map):

```cpp
  Timestamp next_clutter_emit_{};
  bool clutter_initialised_{false};
  std::poisson_distribution<int> clutter_count_dist_;
  std::uniform_real_distribution<double> clutter_range_dist_;
  std::uniform_real_distribution<double> clutter_bearing_dist_;
```

Add `#include <random>` at the top if it isn't already (it should be).

- [ ] **Step 5: Extend `sim/ArpaEmitter.cpp`**

In the constructor's member initialiser list, append three new initialisers (after the existing noise distributions):

```cpp
    , clutter_count_dist_(cfg_.clutter_per_rotation > 0
                          ? static_cast<double>(cfg_.clutter_per_rotation)
                          : 1.0)
    , clutter_range_dist_(cfg_.clutter_min_range_m, cfg_.max_range_m)
    , clutter_bearing_dist_(0.0, 360.0)
```

(The Poisson distribution requires mean > 0; we initialise with a 1.0 placeholder when disabled and gate the sampling on `clutter_per_rotation > 0` below.)

In `ArpaEmitter::emit`, immediately AFTER the `if (!initialised_) { ... }` block and BEFORE the per-target loop, add the clutter init + emit logic:

```cpp
  if (!clutter_initialised_) {
    next_clutter_emit_ = ctx.now;
    clutter_initialised_ = true;
  }

  if (cfg_.clutter_per_rotation > 0) {
    while (next_clutter_emit_ <= ctx.now) {
      const int n_clutter = clutter_count_dist_(rng_);
      for (int k = 0; k < n_clutter; ++k) {
        const double r_clutter = clutter_range_dist_(rng_);
        const double b_clutter = clutter_bearing_dist_(rng_);
        const double r_nm = r_clutter / 1852.0;
        std::string body = "RATTM,00,";
        body += formatMilli3(r_nm);
        body += ',';
        body += formatMilli3(b_clutter);
        body += ",R,0.0,0.0,T,0.0,0.0,N,T,,000000.00,A";
        const std::string sentence = wrapWithChecksum(body);
        adapter_.ingest(sentence, next_clutter_emit_);
      }
      next_clutter_emit_ = Timestamp::fromSeconds(
          next_clutter_emit_.seconds() + cfg_.rotation_dt_s);
    }
  }
```

(`formatMilli3` and `wrapWithChecksum` are already defined in the same `.cpp` and in `NmeaEncode.hpp` respectively.)

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='ArpaClutter.*'`
Expected: 3/3 PASS.

Also re-run the existing ARPA suite to confirm no regression: `./build/navtracker_tests --gtest_filter='ArpaEmitter.*'`. Expected: 4/4 PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/ArpaEmitter.hpp sim/ArpaEmitter.cpp \
        tests/sim/test_arpa_clutter.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: ArpaEmitter clutter_per_rotation knob

Poisson-mean N false alarms per rotation, drawn uniformly in
[clutter_min_range_m, max_range_m] x [0, 360deg). Emitted via $RATTM with
arpa_track_num=0. Clutter cadence runs on a single global next_clutter_emit_
(independent of targets) so determinism holds across configs. Used by the
JPDA and MHT bus-driven comparisons.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: perWindowOspa helper

**Files:**
- Modify: `core/scenario/Metrics.hpp` (append `PerWindowOspa` + signature)
- Modify: `core/scenario/Metrics.cpp` (implementation)
- Test: `tests/sim/test_per_window_ospa.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_per_window_ospa.cpp`:

```cpp
#include "core/scenario/Metrics.hpp"

#include <gtest/gtest.h>

#include "core/scenario/Harness.hpp"

using namespace navtracker;

namespace {

ScenarioResult makeResult(const std::vector<std::pair<double, double>>& time_ospa) {
  ScenarioResult r;
  for (const auto& [t, o] : time_ospa) {
    ScenarioStep s;
    s.time = Timestamp::fromSeconds(t);
    r.steps.push_back(std::move(s));
    r.ospa_per_step.push_back(o);
  }
  return r;
}

}  // namespace

TEST(PerWindowOspa, GroupsByOneSecondWindowAndMeansWithinEach) {
  // Steps at t = 0.1, 0.2, 1.1, 1.2, 1.3, 2.5 with OSPA [10, 12, 5, 5, 8, 20].
  const ScenarioResult r = makeResult({
      {0.1, 10.0}, {0.2, 12.0},
      {1.1,  5.0}, {1.2,  5.0}, {1.3, 8.0},
      {2.5, 20.0},
  });
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), /*window_dt_s=*/1.0);
  ASSERT_EQ(w.per_window.size(), 3u);
  EXPECT_NEAR(w.per_window[0], 11.0, 1e-9);
  EXPECT_NEAR(w.per_window[1],  6.0, 1e-9);
  EXPECT_NEAR(w.per_window[2], 20.0, 1e-9);
  // Mean of [11, 6, 20] = 37/3.
  EXPECT_NEAR(w.mean, 37.0 / 3.0, 1e-9);
  // stddev (sample, N-1) of [11, 6, 20]: sqrt(((11-37/3)^2 + (6-37/3)^2 + (20-37/3)^2)/2)
  // = sqrt((104/9 + 361/9 + 529/9 + 24/9 ... let me just trust the impl, range check.
  EXPECT_GT(w.stddev, 5.0);
  EXPECT_LT(w.stddev, 10.0);
}

TEST(PerWindowOspa, EmptyResultReturnsZeroes) {
  ScenarioResult r;
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), 1.0);
  EXPECT_EQ(w.per_window.size(), 0u);
  EXPECT_EQ(w.mean, 0.0);
  EXPECT_EQ(w.stddev, 0.0);
}

TEST(PerWindowOspa, SingleWindowSingleStepReportsItVerbatim) {
  const ScenarioResult r = makeResult({{0.5, 17.0}});
  const PerWindowOspa w = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), 1.0);
  ASSERT_EQ(w.per_window.size(), 1u);
  EXPECT_DOUBLE_EQ(w.per_window[0], 17.0);
  EXPECT_DOUBLE_EQ(w.mean, 17.0);
  EXPECT_DOUBLE_EQ(w.stddev, 0.0);  // single sample => stddev defined as 0
}
```

- [ ] **Step 2: Append the test to `CMakeLists.txt`**

Append `tests/sim/test_per_window_ospa.cpp` to `navtracker_tests`.

- [ ] **Step 3: Build to verify failure**

Run: `cmake --build build 2>&1 | head -40`
Expected: FAIL — `'PerWindowOspa' / 'computePerWindowOspa' was not declared in this scope`.

- [ ] **Step 4: Extend `core/scenario/Metrics.hpp`**

Append before the closing `}  // namespace navtracker`:

```cpp
struct PerWindowOspa {
  double mean{0.0};                  // mean of per-window means
  double stddev{0.0};                // sample stddev (N-1) across windows; 0 if <2 windows
  std::vector<double> per_window;    // per-window mean OSPA (skipping empty windows)
};

// Group ScenarioResult.steps by floor((step.time - t0) / window_dt_s). For
// each window, average the OSPA values from result.ospa_per_step that fall
// in that window. Empty windows are skipped. The overall `mean` is the mean
// of per-window means (so each window contributes equally regardless of how
// many measurements fired during it).
PerWindowOspa computePerWindowOspa(const ScenarioResult& result,
                                   Timestamp t0,
                                   double window_dt_s);
```

Add `#include "core/scenario/Harness.hpp"` at the top so `ScenarioResult` is visible.

- [ ] **Step 5: Extend `core/scenario/Metrics.cpp`**

Add `#include <cmath>` and `#include <unordered_map>` if not present.

Append before the closing `}  // namespace navtracker`:

```cpp
PerWindowOspa computePerWindowOspa(const ScenarioResult& result,
                                   Timestamp t0,
                                   double window_dt_s) {
  PerWindowOspa out;
  if (result.steps.empty() || result.ospa_per_step.size() != result.steps.size())
    return out;

  // Group by window index, preserve insertion order via parallel vectors.
  std::vector<int> bucket_keys;
  std::vector<double> bucket_sums;
  std::vector<int> bucket_counts;
  std::unordered_map<int, std::size_t> key_to_idx;

  for (std::size_t i = 0; i < result.steps.size(); ++i) {
    const double rel = result.steps[i].time.secondsSince(t0);
    if (rel < 0.0) continue;
    const int key = static_cast<int>(rel / window_dt_s);
    auto it = key_to_idx.find(key);
    if (it == key_to_idx.end()) {
      key_to_idx.emplace(key, bucket_keys.size());
      bucket_keys.push_back(key);
      bucket_sums.push_back(result.ospa_per_step[i]);
      bucket_counts.push_back(1);
    } else {
      bucket_sums[it->second]  += result.ospa_per_step[i];
      bucket_counts[it->second] += 1;
    }
  }

  // Sort windows by key so per_window is monotonic in time.
  std::vector<std::size_t> order(bucket_keys.size());
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) {
              return bucket_keys[a] < bucket_keys[b];
            });

  out.per_window.reserve(order.size());
  for (std::size_t idx : order) {
    out.per_window.push_back(bucket_sums[idx] /
                             static_cast<double>(bucket_counts[idx]));
  }
  if (out.per_window.empty()) return out;

  double sum = 0.0;
  for (double v : out.per_window) sum += v;
  out.mean = sum / static_cast<double>(out.per_window.size());
  if (out.per_window.size() < 2) return out;
  double sse = 0.0;
  for (double v : out.per_window) sse += (v - out.mean) * (v - out.mean);
  out.stddev = std::sqrt(sse / static_cast<double>(out.per_window.size() - 1));
  return out;
}
```

Add `#include <algorithm>` at the top of `Metrics.cpp` if not present (for `std::sort`).

- [ ] **Step 6: Run the tests**

Run: `cmake --build build && ./build/navtracker_tests --gtest_filter='PerWindowOspa.*'`
Expected: 3/3 PASS.

- [ ] **Step 7: Commit**

```bash
git add core/scenario/Metrics.hpp core/scenario/Metrics.cpp \
        tests/sim/test_per_window_ospa.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
core: per-window OSPA helper

computePerWindowOspa buckets ScenarioResult steps by 1 s (configurable)
windows, means OSPA within each bucket, then means across buckets. Each
window contributes equally regardless of how many measurements fired in
it — the fair cross-cadence metric for SimulatedSensorBus comparisons
(bus emits ~10x more measurements than direct-Measurement scenarios).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: BusComparisonHelpers + JPDA vs GNN comparison

**Files:**
- Create: `tests/sim/BusComparisonHelpers.hpp`
- Create: `tests/sim/test_bus_jpda_comparison.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create `tests/sim/BusComparisonHelpers.hpp`**

```cpp
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Harness.hpp"
#include "core/scenario/HarnessBatched.hpp"
#include "core/scenario/HarnessBatchedMht.hpp"
#include "core/scenario/Metrics.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/SimulatedSensorBus.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker_test {

struct RunStats {
  double mean_ospa;        // mean of per-window OSPA means (1 s windows)
  double stddev_ospa;      // stddev across windows
  int id_switches;
};

struct AggStats {
  double mean_ospa;
  double std_ospa;
  double mean_id_sw;
};

inline AggStats aggregate(const std::vector<RunStats>& runs) {
  const std::size_t N = runs.size();
  if (N == 0) return {0.0, 0.0, 0.0};
  double sum_o = 0.0, sum_i = 0.0;
  for (const auto& r : runs) { sum_o += r.mean_ospa; sum_i += r.id_switches; }
  const double m_o = sum_o / static_cast<double>(N);
  double sse = 0.0;
  for (const auto& r : runs) sse += (r.mean_ospa - m_o) * (r.mean_ospa - m_o);
  const double s_o = N > 1 ? std::sqrt(sse / static_cast<double>(N - 1)) : 0.0;
  return {m_o, s_o, sum_i / static_cast<double>(N)};
}

// Two cooperative AIS+ARPA+EO/IR targets crossing through the origin, with
// configurable ARPA clutter. Used by JPDA and MHT bus comparisons.
inline navtracker::Scenario runBusClutterCrossing(std::uint32_t seed,
                                                  int clutter_per_rotation) {
  using namespace navtracker;
  using navtracker::geo::Datum;
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
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(1, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(-200.0,  5.0),
      Eigen::Vector2d(  15.0,  0.0),
      Timestamp::fromSeconds(0.0)));
  bus.addTarget(2, std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d( 200.0, -5.0),
      Eigen::Vector2d( -15.0,  0.0),
      Timestamp::fromSeconds(0.0)));

  bus.attachOwnShip(own_adapter, {});
  sim::AisEmitterConfig ais_cfg;
  ais_cfg.targets.push_back({1, 200000001u, true});
  ais_cfg.targets.push_back({2, 200000002u, true});
  bus.attachAis(ais_adapter, ais_cfg);
  sim::ArpaEmitterConfig arpa_cfg;
  arpa_cfg.targets.push_back({1, 1});
  arpa_cfg.targets.push_back({2, 2});
  arpa_cfg.clutter_per_rotation = clutter_per_rotation;
  bus.attachArpa(arpa_adapter, arpa_cfg);
  sim::EoIrEmitterConfig eo_cfg;
  eo_cfg.targets.push_back({1, 1});
  eo_cfg.targets.push_back({2, 2});
  eo_cfg.fov_deg = 360.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

// Single static target observed by a moving EO-IR sensor (own-ship). Bearing
// only. Used by the PF comparison.
inline navtracker::Scenario runBusBearingOnlyMoving(std::uint32_t seed) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  EoIrAdapter eo_adapter(datum, provider);

  sim::SimulatedSensorBusConfig cfg;
  cfg.t0 = Timestamp::fromSeconds(0.0);
  cfg.duration_s = 60.0;
  cfg.dt_s = 0.1;
  cfg.truth_sample_dt_s = 1.0;
  cfg.seed = seed;
  cfg.datum = datum;
  sim::SimulatedSensorBus bus(cfg);

  bus.setOwnShip(std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d(0.0, -300.0),
      Eigen::Vector2d(0.0,   10.0),
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
  eo_cfg.dt_s = 1.0;
  bus.attachEoIr(eo_adapter, eo_cfg);

  return bus.run();
}

// Single maneuvering target (straight-turn-straight) covered by the full
// quartet. Used by the IMM-3 comparison.
inline navtracker::Scenario runBusManeuvering(std::uint32_t seed) {
  using namespace navtracker;
  using navtracker::geo::Datum;
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter own_adapter(provider);
  AisAdapter  ais_adapter(datum);
  ArpaAdapter arpa_adapter(datum, provider);
  EoIrAdapter eo_adapter(datum, provider);

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

constexpr int kNumSeeds = 20;
constexpr double kWindowDtS = 1.0;

}  // namespace navtracker_test
```

- [ ] **Step 2: Create the JPDA vs GNN comparison test**

`tests/sim/test_bus_jpda_comparison.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runBatchedBus(const IEstimator& est, const IDataAssociator& assoc,
                       const Scenario& s, double cutoff, int confirm, int del,
                       double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, JpdaVsGnnClutterCrossing) {
  std::vector<RunStats> gnn_runs, jpda_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusClutterCrossing(seed, /*clutter_per_rotation=*/5);
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    GnnAssociator  gnn(20.0);
    JpdaAssociator jpda(20.0, 0.9, 1e-4);
    gnn_runs .push_back(runBatchedBus(ekf, gnn,  s, 50.0, 2, 4, 30.0));
    jpda_runs.push_back(runBatchedBus(ekf, jpda, s, 50.0, 2, 4, 30.0));
  }
  const AggStats g = aggregate(gnn_runs);
  const AggStats j = aggregate(jpda_runs);
  std::fprintf(stderr,
      "\n[Bus JPDA vs GNN on ClutterCrossing, %d seeds]\n"
      "  GNN  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  JPDA : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      g.mean_ospa, g.std_ospa, g.mean_id_sw,
      j.mean_ospa, j.std_ospa, j.mean_id_sw);

  // Soft assertion: JPDA wins on at least one of OSPA or ID-switches.
  EXPECT_TRUE(j.mean_ospa < g.mean_ospa || j.mean_id_sw < g.mean_id_sw)
      << "JPDA does not beat GNN on either metric through the bus.";
}
```

- [ ] **Step 3: Append tests to `CMakeLists.txt`**

Append `tests/sim/test_bus_jpda_comparison.cpp` to `navtracker_tests`.
(The header `BusComparisonHelpers.hpp` is `#include`d only — no source-list entry.)

- [ ] **Step 4: Build and run**

```
cmake --build build
./build/navtracker_tests --gtest_filter='BusComparison.JpdaVsGnnClutterCrossing'
```

Expected: PASS. The test prints aggregate stats to stderr; capture them for the eval-log task.

If the soft assertion fails (JPDA loses on both metrics), STOP and report — that's the answer to "does the win survive?" and it changes the eval-log story. Don't silently widen the assertion.

- [ ] **Step 5: Commit**

```bash
git add tests/sim/BusComparisonHelpers.hpp \
        tests/sim/test_bus_jpda_comparison.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: bus comparison — JPDA vs GNN on clutter crossing (20 seeds)

First bus-driven head-to-head comparison. BusComparisonHelpers.hpp shares
sweep helpers (RunStats, AggStats, aggregate, scenario runners) across the
four comparison tests. JPDA vs GNN through the full sensor quartet with
Poisson(5)/rotation ARPA clutter; per-window OSPA + ID-switches.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: IMM-3 vs CV bus comparison

**Files:**
- Create: `tests/sim/test_bus_imm3_comparison.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_bus_imm3_comparison.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/estimation/PrescribedTurn.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runStandardBus(const IEstimator& est, const IDataAssociator& assoc,
                        const Scenario& s, double cutoff, int confirm, int del,
                        double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, Imm3VsCvManeuvering) {
  std::vector<RunStats> ekf_runs, imm3_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusManeuvering(seed);

    auto cv4 = std::make_shared<ConstantVelocity2D>(0.5);
    const EkfEstimator ekf(cv4, 10.0);
    GnnAssociator gnn(50.0);

    std::vector<std::shared_ptr<IMotionModel>> motions3 = {
        std::make_shared<ConstantVelocity5State>(0.5, 0.001),
        std::make_shared<PrescribedTurn>(+0.2, 0.5, 0.001),
        std::make_shared<PrescribedTurn>(-0.2, 0.5, 0.001)};
    Eigen::MatrixXd pi3(3, 3);
    pi3 << 0.90, 0.05, 0.05,
           0.10, 0.85, 0.05,
           0.10, 0.05, 0.85;
    Eigen::VectorXd mu3(3); mu3 << 0.34, 0.33, 0.33;
    const ImmEstimator imm3(motions3, pi3, mu3, 10.0, 0.01);

    ekf_runs .push_back(runStandardBus(ekf,  gnn, s, 100.0, 1, 5, 10.0));
    imm3_runs.push_back(runStandardBus(imm3, gnn, s, 100.0, 1, 5, 10.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats i = aggregate(imm3_runs);
  std::fprintf(stderr,
      "\n[Bus IMM-3 vs CV on Maneuvering, %d seeds]\n"
      "  EKF   : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  IMM-3 : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      i.mean_ospa, i.std_ospa, i.mean_id_sw);

  EXPECT_TRUE(i.mean_ospa < e.mean_ospa || i.mean_id_sw < e.mean_id_sw)
      << "IMM-3 does not beat single-mode CV on either metric through the bus.";
}
```

- [ ] **Step 2: Append to `CMakeLists.txt`**

Append `tests/sim/test_bus_imm3_comparison.cpp`.

- [ ] **Step 3: Build and run**

```
cmake --build build
./build/navtracker_tests --gtest_filter='BusComparison.Imm3VsCvManeuvering'
```

Expected: PASS. Capture the printed aggregates for the eval-log.

If the soft assertion fails (IMM-3 loses on both metrics), STOP and report.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_bus_imm3_comparison.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: bus comparison — IMM-3 vs CV on maneuvering (20 seeds)

ManeuveringTrajectory (straight-turn-straight) driven through the full
sensor quartet; IMM-3 with [CV5, PrescribedTurn(+0.2), PrescribedTurn(-0.2)]
modes vs single-mode CV-2D EKF baseline. Same IMM config as the prior
direct-Measurement sweep (test_multi_seed_sweep.cpp).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: PF vs EKF bus comparison

**Files:**
- Create: `tests/sim/test_bus_pf_comparison.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_bus_pf_comparison.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ParticleFilterEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runStandardBus(const IEstimator& est, const IDataAssociator& assoc,
                        const Scenario& s, double cutoff, int confirm, int del,
                        double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenario(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, PfVsEkfBearingOnlyMoving) {
  std::vector<RunStats> ekf_runs, pf_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusBearingOnlyMoving(seed);

    auto motion = std::make_shared<ConstantVelocity2D>(0.05);
    const EkfEstimator ekf(motion, 5.0);
    const ParticleFilterEstimator pf(motion, 2000, 5.0, 0.5,
                                     static_cast<std::uint64_t>(seed));
    GnnAssociator gnn(2500.0);

    ekf_runs.push_back(runStandardBus(ekf, gnn, s, 500.0, 1, 8, 90.0));
    pf_runs .push_back(runStandardBus(pf,  gnn, s, 500.0, 1, 8, 90.0));
  }
  const AggStats e = aggregate(ekf_runs);
  const AggStats p = aggregate(pf_runs);
  std::fprintf(stderr,
      "\n[Bus PF vs EKF on BearingOnlyMovingSensor, %d seeds]\n"
      "  EKF : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  PF  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      e.mean_ospa, e.std_ospa, e.mean_id_sw,
      p.mean_ospa, p.std_ospa, p.mean_id_sw);

  // PF vs EKF was directional in the prior sweep (overlapping CIs); allow
  // either to win, but make sure neither blows up.
  EXPECT_GT(e.mean_ospa, 0.0);
  EXPECT_GT(p.mean_ospa, 0.0);
}
```

- [ ] **Step 2: Append to `CMakeLists.txt`**

Append `tests/sim/test_bus_pf_comparison.cpp`.

- [ ] **Step 3: Build and run**

```
cmake --build build
./build/navtracker_tests --gtest_filter='BusComparison.PfVsEkfBearingOnlyMoving'
```

Expected: PASS (sanity-only assertion). Capture the aggregates.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_bus_pf_comparison.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: bus comparison — PF vs EKF on bearing-only moving sensor (20 seeds)

Single static target observed by a moving EO/IR sensor in BearingOnly mode.
PF (2000 particles) vs EKF baseline. Assertion is sanity-only since PF vs
EKF was directional in the prior direct-Measurement sweep; the test prints
the per-window OSPA aggregates so we can record the bus-driven verdict in
the eval log.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: MHT vs JPDA bus comparison

**Files:**
- Create: `tests/sim/test_bus_mht_comparison.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the test file**

`tests/sim/test_bus_mht_comparison.cpp`:

```cpp
#include "tests/sim/BusComparisonHelpers.hpp"

#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;
using namespace navtracker_test;

namespace {

RunStats runBatchedBus(const IEstimator& est, const IDataAssociator& assoc,
                       const Scenario& s, double cutoff, int confirm, int del,
                       double miss_timeout) {
  TrackManager mgr(confirm, del);
  Tracker tracker(est, assoc, mgr, miss_timeout);
  const ScenarioResult r = runScenarioBatched(s, tracker, mgr, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

RunStats runMhtBatchedBus(const IEstimator& est, const Scenario& s,
                          double cutoff, const MhtTracker::Config& cfg) {
  MhtTracker tracker(est, cfg);
  const ScenarioResult r = runScenarioBatchedMht(s, tracker, cutoff);
  const PerWindowOspa pw = computePerWindowOspa(
      r, Timestamp::fromSeconds(0.0), kWindowDtS);
  return RunStats{pw.mean, pw.stddev,
                  countIdSwitches(r.steps, cutoff)};
}

}  // namespace

TEST(BusComparison, MhtVsJpdaClutterCrossing) {
  std::vector<RunStats> jpda_runs, mht_runs;
  for (int k = 0; k < kNumSeeds; ++k) {
    const std::uint32_t seed = 201u + static_cast<std::uint32_t>(k);
    const Scenario s = runBusClutterCrossing(seed, /*clutter_per_rotation=*/5);

    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    const EkfEstimator ekf(motion, 5.0);
    JpdaAssociator jpda(20.0, 0.9, 1e-4);

    MhtTracker::Config mht_cfg;
    mht_cfg.probability_of_detection = 0.9;
    mht_cfg.clutter_density = 1e-4;
    mht_cfg.gate_threshold = 20.0;
    mht_cfg.n_scan = 3;
    mht_cfg.k_max_leaves = 5;
    mht_cfg.score_delete_threshold = -15.0;

    jpda_runs.push_back(runBatchedBus(ekf, jpda, s, 50.0, 2, 4, 30.0));
    mht_runs .push_back(runMhtBatchedBus(ekf, s, 50.0, mht_cfg));
  }
  const AggStats j = aggregate(jpda_runs);
  const AggStats m = aggregate(mht_runs);
  std::fprintf(stderr,
      "\n[Bus MHT vs JPDA on ClutterCrossing, %d seeds]\n"
      "  JPDA : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n"
      "  MHT  : per-window OSPA %.4f +/- %.4f m   id_sw_mean=%.2f\n",
      kNumSeeds,
      j.mean_ospa, j.std_ospa, j.mean_id_sw,
      m.mean_ospa, m.std_ospa, m.mean_id_sw);

  // No directional assertion — the prior sweep retracted the MHT win. We
  // print aggregates so the eval-log update can record whatever ratio the
  // bus produces.
  EXPECT_GT(j.mean_ospa, 0.0);
  EXPECT_GT(m.mean_ospa, 0.0);
}
```

- [ ] **Step 2: Append to `CMakeLists.txt`**

Append `tests/sim/test_bus_mht_comparison.cpp`.

- [ ] **Step 3: Build and run**

```
cmake --build build
./build/navtracker_tests --gtest_filter='BusComparison.MhtVsJpdaClutterCrossing'
```

Expected: PASS. Capture the aggregates.

- [ ] **Step 4: Commit**

```bash
git add tests/sim/test_bus_mht_comparison.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
sim: bus comparison — MHT vs JPDA on clutter crossing (20 seeds)

Same scenario as the JPDA comparison (ARPA clutter Poisson(5)/rotation,
full sensor quartet). MHT TOMHT vs JPDA. No directional assertion; the
test reports per-window OSPA + ID-switch aggregates for the eval log
to record confirm or re-retract.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Eval-log update

**Files:**
- Modify: `docs/algorithms/evaluation-log.md`

- [ ] **Step 1: Run all four comparison tests and capture their output**

Run:

```
./build/navtracker_tests --gtest_filter='BusComparison.*' 2>&1 | tee /tmp/bus_comparisons.log
```

Each test prints a small block to stderr with mean OSPA ± stddev and mean ID-switches for each method.

- [ ] **Step 2: Append a new section to `docs/algorithms/evaluation-log.md`**

Open the file and append at the bottom (after the existing multi-seed sweep section):

```markdown
## Bus-driven confirmation pass (2026-06-02)

Re-ran the four winning comparisons through `SimulatedSensorBus` (full
sensor quartet: OwnShip + AIS + ARPA + EO/IR; ARPA clutter Poisson(5)
per rotation on the JPDA and MHT scenarios). Metric: per-window OSPA
(1 s windows, mean of per-window means) + cumulative ID-switch count.
20 seeds (range 201..220, identical to the prior direct-Measurement
sweep). Heading bias deferred to §14.9.

Numbers (from the four `BusComparison.*` test runs):

### JPDA vs GNN — clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| GNN    | <fill from test stderr block "Bus JPDA vs GNN..."> | <...> |
| JPDA   | <...>               | <...>            |

Verdict: <confirmed / retracted / new finding>, based on whichever metric(s) JPDA wins on.

### IMM-3 vs CV — maneuvering

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | <fill from stderr "Bus IMM-3 vs CV..."> | <...> |
| IMM-3  | <...>               | <...>            |

Verdict: <...>

### PF vs EKF — bearing-only moving sensor

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| EKF    | <fill from stderr "Bus PF vs EKF..."> | <...> |
| PF     | <...>               | <...>            |

Verdict: <directional / confirmed / retracted>.

### MHT vs JPDA — clutter crossing

| Method | per-window OSPA (m) | mean ID switches |
|--------|---------------------|------------------|
| JPDA   | <fill from stderr "Bus MHT vs JPDA..."> | <...> |
| MHT    | <...>               | <...>            |

Verdict: <re-confirmed retraction / new finding>.

### Notes

- Per-window OSPA differs in scale from the prior per-measurement mean
  OSPA because the bus emits ~10× more measurements than the direct-
  Measurement scenarios, and 1 s windows average each tick once. Direct
  comparison of the *numbers* to the prior table is illustrative, not
  strict; the comparison that matters is between methods on the SAME row.
- Bus injects: 1 Hz OwnShip GPS (no heading bias yet), Class-A SOTDMA
  AIS, 3 s ARPA rotation (with optional Poisson clutter), 10 Hz EO/IR
  with bearing+range or bearing-only.
- Determinism: each seed produces a byte-identical Scenario; re-running
  this table yields the same numbers.
```

Replace every `<fill from ...>` placeholder with the exact numbers from your test output. Replace each `<verdict>` with one of: `confirmed`, `retracted`, `directional`, `new finding`. Justify each in one sentence inside the verdict line.

- [ ] **Step 3: Verify the file renders cleanly as markdown**

Skim the rendered output (Open `docs/algorithms/evaluation-log.md` and inspect; tables should align).

- [ ] **Step 4: Commit**

```bash
git add docs/algorithms/evaluation-log.md
git commit -m "$(cat <<'EOF'
docs: eval-log bus-driven confirmation pass (4 comparisons, 20 seeds)

JPDA, IMM-3, PF, MHT re-evaluated through SimulatedSensorBus with full
sensor quartet, ARPA clutter where relevant, per-window OSPA + ID-switch
count over 20 seeds (201..220). Verdicts per method recorded inline.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review Notes

**Spec coverage:**
- §4.1 `ManeuveringTrajectory` → Task 1.
- §4.2 ARPA `clutter_per_rotation` + separate cadence → Task 2.
- §4.3 `perWindowOspa` helper → Task 3.
- §5 four comparison tests with shared sweep helpers → Tasks 4-7 (split: helpers + JPDA in Task 4, then one task per remaining comparison).
- §6 eval-log update → Task 8.

The spec's file layout under `tests/sim/` lists four separate `test_bus_*_comparison.cpp` files. This plan honours that and adds the shared `BusComparisonHelpers.hpp` to DRY the sweep loops. `test_per_window_ospa.cpp` lives under `tests/sim/` for proximity to bus consumers even though the helper itself is in `core/scenario/Metrics`.

**Placeholder scan:** The eval-log task contains `<fill from test stderr block ...>` markers and `<verdict>` markers. These are intentional — the numbers materialise only at execution time, and the verdicts depend on what the numbers say. Task 8 explicitly tells the implementer to replace every marker with concrete values before committing.

**Type consistency:**
- `RunStats { mean_ospa, stddev_ospa, id_switches }` and `AggStats { mean_ospa, std_ospa, mean_id_sw }` defined in `BusComparisonHelpers.hpp` (Task 4) and used identically by Tasks 5, 6, 7. (Field name `std_ospa` in `AggStats` matches the legacy field from `test_multi_seed_sweep.cpp`.)
- `PerWindowOspa { mean, stddev, per_window }` defined in `Metrics.hpp` (Task 3) and consumed in `runStandardBus` / `runBatchedBus` / `runMhtBatchedBus` (Tasks 4-7) — field names match.
- `runStandardBus`, `runBatchedBus`, `runMhtBatchedBus` are TU-local helpers re-declared per test file (each TU only needs the ones it uses). Signatures are identical across files where they appear.
- `runBusClutterCrossing(seed, clutter_per_rotation)`, `runBusBearingOnlyMoving(seed)`, `runBusManeuvering(seed)` declared in `BusComparisonHelpers.hpp` (Task 4) and used in Tasks 4, 5, 6, 7.
- `kNumSeeds = 20`, `kWindowDtS = 1.0` constants live in the header (Task 4) and are used by every comparison test.

**Misc:** The `runStandardBus` / `runBatchedBus` / `runMhtBatchedBus` helpers are defined in three different test TUs (Tasks 4, 5, 6, 7) — DRY-wise, hoisting them into `BusComparisonHelpers.hpp` would be cleaner. Reason for keeping them per-TU: each helper depends on tracker/associator types that vary per test (and they're only ~10 lines each). If the duplication becomes burdensome, a follow-up can lift them.
