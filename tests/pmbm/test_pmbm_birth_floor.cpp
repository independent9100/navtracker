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
