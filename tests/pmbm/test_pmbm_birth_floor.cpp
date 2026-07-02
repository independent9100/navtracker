#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/StaticObstacle.hpp"
#include "ports/ILandModel.hpp"
#include "ports/IStaticObstacleModel.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::ILandModel;
using navtracker::IStaticObstacleModel;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::StaticObstacle;
using navtracker::Timestamp;
using navtracker::pmbm::PmbmTracker;

namespace {

// Fakes that return a fixed suppression regardless of position, so a test can
// dial c_land / c_static directly.
struct FakeLand : ILandModel {
  double c;
  explicit FakeLand(double v) : c(v) {}
  double clutterPrior(const Eigen::Vector2d&) const override { return c; }
};
struct FakeObstacle : IStaticObstacleModel {
  double c;
  std::vector<StaticObstacle> none;
  explicit FakeObstacle(double v) : c(v) {}
  double birthSuppression(const Eigen::Vector2d&) const override { return c; }
  const std::vector<StaticObstacle>& obstacles() const override { return none; }
};

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.sensor = SensorKind::ArpaTtm;
  m.model = MeasurementModel::Position2D;
  m.time = Timestamp::fromSeconds(t);
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

// birth_existence_target = 0.1, floor = 0.05: an unsuppressed birth (r_new =
// 0.1) passes the floor; a heavily suppressed one (scale ≤ 0.05) lands ~0.005.
PmbmTracker::Config cfg() {
  PmbmTracker::Config c;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.1;
  c.min_new_bernoulli_existence = 0.05;
  c.probability_of_detection = 0.9;
  c.survival_probability = 1.0;
  return c;
}

double maxExistence(const PmbmTracker& t) {
  double m = 0.0;
  for (const auto& h : t.density().mbm)
    for (const auto& b : h.bernoullis)
      m = std::max(m, b.existence_probability);
  return m;
}

}  // namespace

// R1: land (c=0.5) + obstacle (c=0.9) compose to scale 0.05. The suppressed
// r_new (~0.005) is below the 0.05 floor, but the UNSUPPRESSED r_new (0.1)
// passes — so the Bernoulli must be created with the tiny suppressed existence,
// not silently killed every scan (the hard no-birth zone ADR 0002 forbids).
TEST(PmbmBirthFloor, LandPlusObstacleCreatesSuppressedBernoulli) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeLand land(0.5);
  FakeObstacle obs(0.9);
  t.setLandModel(&land);
  t.setStaticObstacleModel(&obs);

  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});

  const double r = maxExistence(t);
  EXPECT_GT(r, 0.0);    // NOT hard-dropped by the floor (was killed pre-R1)
  EXPECT_LT(r, 0.05);   // but suppressed below the birth floor
}

// R1: a stationary target inside both a soft shore band and a keep-clear buffer
// still initiates and, through repeated observation, climbs to Confirmed — the
// suppressed birth (r ~0.005) is above r_min (1e-3) so it survives pruning and
// accumulates evidence rather than being reborn from scratch each scan.
TEST(PmbmBirthFloor, StationaryTargetInBufferBirthsAndConfirms) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeLand land(0.5);
  FakeObstacle obs(0.9);
  t.setLandModel(&land);
  t.setStaticObstacleModel(&obs);

  for (int k = 0; k < 30; ++k) {
    t.predict(Timestamp::fromSeconds(k));
    t.processBatch({posMeas(100.0, 0.0, k)});
  }
  EXPECT_GT(maxExistence(t), c.output_existence_floor);  // reached Confirmed
}

// R1 scope: land-only suppression is NOT relaxed — ADR 0001's deliberate
// near-shore no-birth zone (coverage_land, floor == birth_existence_target) is
// preserved. A birth suppressed by LAND alone below the floor is still dropped,
// unlike the land×obstacle composition case above. This is the fix that keeps
// R1 byte-identical on the philos coverage_land guard.
TEST(PmbmBirthFloor, LandOnlySuppressionPreservesAdr0001NoBirthZone) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.min_new_bernoulli_existence = 0.1;  // floor == target (coverage_land regime)
  c.use_land_model = true;              // land only — no obstacle model wired
  PmbmTracker t(ekf, c);
  FakeLand land(0.5);  // drives the suppressed r_new below the 0.1 floor
  t.setLandModel(&land);
  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_NEAR(maxExistence(t), 0.0, 1e-9);  // still a no-birth zone (ADR 0001)
}

// R1 finding #3: in the land×obstacle OVERLAP the floor must relax the OBSTACLE
// suppression only — land keeps its ADR-0001 near-shore no-birth role. A spot
// with heavy land (c=0.9, land-only r_new ≈ 0.011 < 0.05 floor) and a mild
// obstacle (c=0.5) must NOT birth: the obstacle presence must not strip land's
// gate. Pre-fix the gate used the FULLY unsuppressed r_new (0.1 > floor) and
// wrongly birthed here, disabling the land zone wherever a keep-clear ring
// overlaps the shore band.
TEST(PmbmBirthFloor, ObstacleOverlapDoesNotStripLandZone) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();  // floor 0.05, target 0.1
  c.use_land_model = true;
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeLand land(0.9);    // land-only r_new ≈ 0.011 < 0.05 floor → no-birth zone
  FakeObstacle obs(0.5);  // mild obstacle present (overlap), soft
  t.setLandModel(&land);
  t.setStaticObstacleModel(&obs);

  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_NEAR(maxExistence(t), 0.0, 1e-9);  // land zone preserved in overlap
}

// R1 finding #3 counterpart: with MILD land (c=0.5, land-only r_new ≈ 0.053 >
// 0.05 floor) the obstacle relaxation still lets a legitimate near-obstacle
// birth through — the fix relaxes the obstacle, not land, so a vessel next to a
// charted pier in clear-of-shore water still initiates.
TEST(PmbmBirthFloor, ObstacleOverlapStillBirthsWhereLandIsMild) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_land_model = true;
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeLand land(0.5);    // land-only r_new ≈ 0.053 > 0.05 floor → admits
  FakeObstacle obs(0.9);  // heavy obstacle suppression, relaxed by the gate
  t.setLandModel(&land);
  t.setStaticObstacleModel(&obs);

  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_GT(maxExistence(t), 0.0);   // admitted (obstacle relaxed)
  EXPECT_LT(maxExistence(t), 0.05);  // but materialised at the suppressed r_new
}

// R1 finding #1 (characterisation, NOT a bug): a birth admitted past the
// phantom-birth gate still materialises at its SUPPRESSED r_new. When land×
// obstacle composition drives that below r_min (1e-3) the r_min pruner removes
// it the same scan — so a target this deep in a keep-clear ring over a shore
// band is NOT trackable from position alone. This is by design: injecting r_min
// existence to force persistence would seed phantom tracks in clutter (the exact
// over-count the philos work fights). The real cure is sensor-aware suppression
// (ADR 0001 A3: EO/IR/AIS corroboration). This test locks the boundary in.
TEST(PmbmBirthFloor, DeepCompositionNotTrackablePositionOnly) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.min_new_bernoulli_existence = 0.0;  // no phantom gate → admission always
  c.use_land_model = true;
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeLand land(0.9);     // composed scale 0.1 * 0.05 = 0.005
  FakeObstacle obs(0.95);  // soft (== hard gate, not >) → r_new ≈ 5.5e-4 < r_min
  t.setLandModel(&land);
  t.setStaticObstacleModel(&obs);

  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_NEAR(maxExistence(t), 0.0, 1e-9);  // pruned by r_min (A3 territory)
}

// R1 guard: with no models wired (default), the birth floor is unchanged — a
// lone measurement births at the target existence (0.1), bit-identical to
// pre-R1 behaviour.
TEST(PmbmBirthFloor, NoModelsBitIdentical) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker t(ekf, cfg());  // use_land_model / use_static_obstacle_model false
  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_NEAR(maxExistence(t), 0.1, 1e-6);
}

// R1 guard: a hard-gate suppression (c_static 0.99 > 0.95) still kills the birth
// entirely — the pre-suppression floor relaxation must not resurrect hard-drops.
TEST(PmbmBirthFloor, HardGateStillKills) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf{motion, 5.0};
  PmbmTracker::Config c = cfg();
  c.use_static_obstacle_model = true;
  PmbmTracker t(ekf, c);
  FakeObstacle obs(0.99);  // > static_obstacle_hard_gate (0.95) → hard-drop
  t.setStaticObstacleModel(&obs);
  t.predict(Timestamp::fromSeconds(0));
  t.processBatch({posMeas(100.0, 0.0, 0)});
  EXPECT_NEAR(maxExistence(t), 0.0, 1e-9);
}
