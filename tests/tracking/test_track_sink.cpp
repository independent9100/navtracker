#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ITrackSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public ITrackSink {
 public:
  std::vector<TrackLifecycleEvent> initiated;
  std::vector<TrackLifecycleEvent> confirmed;
  std::vector<TrackLifecycleEvent> updated;
  std::vector<TrackLifecycleEvent> deleted;
  void onTrackInitiated(const TrackLifecycleEvent& e) override { initiated.push_back(e); }
  void onTrackConfirmed(const TrackLifecycleEvent& e) override { confirmed.push_back(e); }
  void onTrackUpdated(const TrackLifecycleEvent& e) override { updated.push_back(e); }
  void onTrackDeleted(const TrackLifecycleEvent& e) override { deleted.push_back(e); }
};

Measurement positionAt(double x, double y, double t_s) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;
  m.source_id = "test";
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return m;
}

Track makeBareTrack() {
  Track t;
  t.state = Eigen::VectorXd::Zero(4);
  t.covariance = Eigen::MatrixXd::Identity(4, 4);
  return t;
}

}  // namespace

TEST(TrackSink, AddFiresInitiated) {
  TrackManager mgr(2, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  mgr.add(makeBareTrack(), Timestamp::fromSeconds(1.0));
  ASSERT_EQ(sink.initiated.size(), 1u);
  EXPECT_EQ(sink.initiated[0].status, TrackStatus::Tentative);
  EXPECT_DOUBLE_EQ(sink.initiated[0].time.seconds(), 1.0);
}

TEST(TrackSink, RecordHitFiresConfirmedOnce) {
  TrackManager mgr(2, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  const TrackId id = mgr.add(makeBareTrack(), Timestamp::fromSeconds(1.0));
  mgr.recordHit(id);
  ASSERT_EQ(sink.confirmed.size(), 1u);
  mgr.recordHit(id);
  mgr.recordHit(id);
  EXPECT_EQ(sink.confirmed.size(), 1u);
}

TEST(TrackSink, RecordMissDeleteFiresDeletedBeforeErase) {
  TrackManager mgr(1, 2);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  const TrackId id = mgr.add(makeBareTrack(), Timestamp::fromSeconds(1.0));
  mgr.recordMiss(id);
  EXPECT_EQ(sink.deleted.size(), 0u);
  mgr.recordMiss(id);
  ASSERT_EQ(sink.deleted.size(), 1u);
  EXPECT_EQ(sink.deleted[0].id, id);
}

TEST(TrackSink, TrackerProcessFiresUpdated) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(100.0);
  TrackManager mgr(1, 4);
  RecordingSink sink;
  mgr.setTrackSink(&sink);
  Tracker tracker(est, assoc, mgr, 60.0);

  tracker.process(positionAt(100.0, 0.0, 1.0));
  tracker.process(positionAt(101.0, 0.0, 2.0));

  EXPECT_GT(sink.updated.size(), 0u);
}

TEST(TrackSink, NullSinkIsSafe) {
  TrackManager mgr(2, 4);
  const TrackId id = mgr.add(makeBareTrack(), Timestamp::fromSeconds(1.0));
  mgr.recordHit(id);
  mgr.recordHit(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);
  mgr.recordMiss(id);
  SUCCEED();
}
