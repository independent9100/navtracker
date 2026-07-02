#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "core/collision/StaticHazardEvaluator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/output/StaticHazardOutput.hpp"  // staticHazardId
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

// R7.3: two obstacles at the same rounded position + category collide in
// staticHazardId. Their hysteresis state must NOT be shared — each must fire
// its own Entered and Exited. Keyed by hazard_id, the second obstacle's Enter
// is masked by the first's inside-state; keyed by index, both fire.
TEST(StaticHazardEvaluator, ColocatedObstaclesHaveIndependentHysteresis) {
  StaticObstacle a;
  a.position = Geodetic{42.35, -71.05, 0.0};  // ENU ~ (0,0)
  a.footprint_radius_m = 15.0;
  a.keep_clear_radius_m = 100.0;
  a.category = navtracker::ObstacleCategory::Wreck;
  StaticObstacle b = a;  // identical position + category -> same hazard_id
  StaticObstacleModel m({a, b}, datum());
  StaticHazardEvaluator ev(&m);
  Recorder rec;
  ev.setSink(&rec);

  ev.evaluate(Eigen::Vector2d(300.0, 0.0), datum(), Timestamp::fromSeconds(0));  // far
  ev.evaluate(Eigen::Vector2d(50.0, 0.0), datum(), Timestamp::fromSeconds(1));   // inside
  ev.evaluate(Eigen::Vector2d(300.0, 0.0), datum(), Timestamp::fromSeconds(2));  // far

  int entered = 0, exited = 0;
  for (const auto& e : rec.events) {
    if (e.transition == StaticHazardTransition::Entered) ++entered;
    if (e.transition == StaticHazardTransition::Exited) ++exited;
  }
  EXPECT_EQ(entered, 2);
  EXPECT_EQ(exited, 2);
}

// R7.3 (enhancement): distinct source_ids give distinct hazard ids even when
// position + category collide (co-located ENC records: wreck vs obstruction).
TEST(StaticHazardOutput, SourceIdDisambiguatesColocatedHazardId) {
  StaticObstacle a;
  a.position = Geodetic{42.35, -71.05, 0.0};
  a.category = navtracker::ObstacleCategory::Wreck;
  a.source_id = "ENC:wreck-1";
  StaticObstacle b = a;
  b.source_id = "ENC:obstruction-7";
  EXPECT_NE(navtracker::staticHazardId(a), navtracker::staticHazardId(b));
}
