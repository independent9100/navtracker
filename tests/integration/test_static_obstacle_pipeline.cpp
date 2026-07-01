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

  // A real vessel crosses west→east at y=60 m (60 m from obstacle centre, in
  // the 100 m soft buffer), 5 m/s, one scan per second.
  double maxExistenceNearObstacle = 0.0;
  double maxExistenceOnVessel = 0.0;
  for (int k = 0; k < 20; ++k) {
    const double tt = static_cast<double>(k);
    t.predict(Timestamp::fromSeconds(tt));
    const double vx = -200.0 + 5.0 * tt;  // vessel x at this scan
    std::vector<Measurement> scan;
    scan.push_back(posMeas(5.0, 3.0, tt));      // clutter inside footprint
    scan.push_back(posMeas(vx, 60.0, tt));      // the real vessel
    t.processBatch(scan);

    for (const auto& h : t.density().mbm) {
      for (const auto& b : h.bernoullis) {
        const Eigen::Vector2d p = b.mean.head<2>();
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
