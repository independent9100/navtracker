#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/collision/CpaEvaluator.hpp"
#include "core/geo/Datum.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ICollisionRiskSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public ICollisionRiskSink {
 public:
  std::vector<CollisionRiskEvent> events;
  void onCollisionRisk(const CollisionRiskEvent& e) override {
    events.push_back(e);
  }
  std::size_t countOf(CollisionRiskTransition tr) const {
    std::size_t n = 0;
    for (const auto& e : events) if (e.transition == tr) ++n;
    return n;
  }
};

OwnShipPose makePoseAtDatum(double t_s,
                            const Eigen::Vector2d& velocity) {
  OwnShipPose p;
  p.time = Timestamp::fromSeconds(t_s);
  p.lat_deg = 0.0;
  p.lon_deg = 0.0;
  p.alt_m = 0.0;
  p.position_std_m = 1.0;
  p.velocity_enu = velocity;
  p.velocity_std_m_per_s = 0.1;
  p.velocity_is_valid = true;
  return p;
}

Track makeTrackAt(Eigen::Vector2d position, Eigen::Vector2d velocity,
                  double t_s) {
  Track t;
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::VectorXd(4);
  t.state << position.x(), position.y(), velocity.x(), velocity.y();
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  t.covariance(0, 0) = 4.0; t.covariance(1, 1) = 4.0;
  t.covariance(2, 2) = 0.04; t.covariance(3, 3) = 0.04;
  t.last_update = Timestamp::fromSeconds(t_s);
  return t;
}

// Helper: build a manager+provider with a confirmed inbound target.
struct Scene {
  geo::Datum datum{geo::Geodetic{0.0, 0.0, 0.0}};
  OwnShipProvider provider{datum};
  TrackManager mgr{1, 4};

  TrackId addInbound() {
    Track t = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                          Eigen::Vector2d(-5.0, 0.0), 0.0);
    const TrackId id = mgr.add(t, Timestamp::fromSeconds(0.0));
    mgr.recordHit(id);  // confirm (confirm_hits_=1 so single hit confirms)
    return id;
  }
};

}  // namespace

TEST(CpaEvaluator, NoOwnShipIsNoop) {
  OwnShipProvider provider;
  TrackManager mgr(1, 4);
  CpaEvaluator eval(mgr, provider);
  RecordingSink sink;
  eval.setSink(&sink);
  eval.evaluate(Timestamp::fromSeconds(1.0));
  EXPECT_TRUE(sink.events.empty());
}

TEST(CpaEvaluator, EnteredFiresOnHighRisk) {
  Scene s;
  s.provider.update(makePoseAtDatum(0.0, Eigen::Vector2d(5.0, 0.0)));
  s.addInbound();

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 200.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(s.mgr, s.provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  eval.evaluate(Timestamp::fromSeconds(0.0));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Entered), 1u);
}

TEST(CpaEvaluator, NoRefireWhileRisky) {
  Scene s;
  s.provider.update(makePoseAtDatum(0.0, Eigen::Vector2d(5.0, 0.0)));
  s.addInbound();

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 200.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(s.mgr, s.provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  eval.evaluate(Timestamp::fromSeconds(0.0));
  const std::size_t after_first = sink.countOf(CollisionRiskTransition::Entered);
  eval.evaluate(Timestamp::fromSeconds(0.01));
  eval.evaluate(Timestamp::fromSeconds(0.02));
  EXPECT_EQ(sink.countOf(CollisionRiskTransition::Entered), after_first);
}

TEST(CpaEvaluator, DeletedTrackFiresExited) {
  geo::Datum datum{geo::Geodetic{0.0, 0.0, 0.0}};
  OwnShipProvider provider(datum);
  TrackManager mgr(1, 1);  // delete after 1 miss
  provider.update(makePoseAtDatum(0.0, Eigen::Vector2d(5.0, 0.0)));
  Track t = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                        Eigen::Vector2d(-5.0, 0.0), 0.0);
  const TrackId id = mgr.add(t, Timestamp::fromSeconds(0.0));
  mgr.recordHit(id);

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 200.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  eval.evaluate(Timestamp::fromSeconds(0.0));
  ASSERT_GE(sink.countOf(CollisionRiskTransition::Entered), 1u);
  mgr.recordMiss(id);  // immediate delete (delete_misses_=1)
  eval.evaluate(Timestamp::fromSeconds(1.0));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Exited), 1u);
}

TEST(CpaEvaluator, EmitUpdatesGated) {
  Scene s;
  s.provider.update(makePoseAtDatum(0.0, Eigen::Vector2d(5.0, 0.0)));
  s.addInbound();

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 200.0;
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  cfg.emit_updates = true;
  CpaEvaluator eval(s.mgr, s.provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);

  eval.evaluate(Timestamp::fromSeconds(0.0));
  eval.evaluate(Timestamp::fromSeconds(0.01));
  eval.evaluate(Timestamp::fromSeconds(0.02));
  EXPECT_GE(sink.countOf(CollisionRiskTransition::Updated), 1u);
}

TEST(CpaEvaluator, TentativeSkippedByDefault) {
  geo::Datum datum{geo::Geodetic{0.0, 0.0, 0.0}};
  OwnShipProvider provider(datum);
  TrackManager mgr(10, 4);  // confirm_hits_ high so stays Tentative
  provider.update(makePoseAtDatum(0.0, Eigen::Vector2d(5.0, 0.0)));
  Track t = makeTrackAt(Eigen::Vector2d(100.0, 0.0),
                        Eigen::Vector2d(-5.0, 0.0), 0.0);
  t.status = TrackStatus::Tentative;
  mgr.add(t, Timestamp::fromSeconds(0.0));
  // No further hits — stays Tentative.

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 200.0;
  CpaEvaluator eval(mgr, provider, cfg);
  RecordingSink sink;
  eval.setSink(&sink);
  eval.evaluate(Timestamp::fromSeconds(0.0));
  EXPECT_EQ(sink.events.size(), 0u);
}
