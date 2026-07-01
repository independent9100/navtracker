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
