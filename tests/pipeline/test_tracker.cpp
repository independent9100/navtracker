#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::GnnAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::Tracker;
using navtracker::TrackManager;
using navtracker::TrackStatus;

namespace {
Measurement positionAt(double t, double x, double y, const std::string& src) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity();
  return z;
}
}  // namespace

TEST(Tracker, InitiatesAndUpdatesSingleTarget) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(2, 3);
  Tracker tracker(estimator, associator, manager, 10.0);

  tracker.process(positionAt(0.0, 10.0, 0.0, "ais"));
  tracker.process(positionAt(1.0, 12.0, 0.0, "ais"));
  tracker.process(positionAt(2.0, 14.0, 0.0, "ais"));

  ASSERT_EQ(manager.size(), 1u);
  EXPECT_EQ(manager.tracks()[0].status, TrackStatus::Confirmed);
  EXPECT_NEAR(manager.tracks()[0].state(0), 14.0, 1.0);
  EXPECT_NEAR(manager.tracks()[0].state(1), 0.0, 1.0);
}

TEST(Tracker, StaleTrackTimesOutAndIsDeleted) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager manager(1, 2);
  Tracker tracker(estimator, associator, manager, 0.5);

  tracker.process(positionAt(0.0, 0.0, 0.0, "s"));
  ASSERT_EQ(manager.size(), 1u);
  tracker.process(positionAt(1.0, 1000.0, 0.0, "s"));
  ASSERT_EQ(manager.size(), 2u);
  tracker.process(positionAt(2.0, 1000.5, 0.0, "s"));
  EXPECT_EQ(manager.size(), 1u);
}

TEST(Tracker, ProcessBatchHandlesMultipleMeasurementsAtSameTime) {
  auto motion = std::make_shared<navtracker::ConstantVelocity2D>(0.1);
  const navtracker::EkfEstimator ekf(motion, 5.0);
  navtracker::GnnAssociator gnn(50.0);
  navtracker::TrackManager mgr(1, 5);
  navtracker::Tracker tr(ekf, gnn, mgr, 10.0);

  navtracker::Measurement z1;
  z1.time = navtracker::Timestamp::fromSeconds(1.0);
  z1.model = navtracker::MeasurementModel::Position2D;
  z1.value = Eigen::Vector2d(100.0, 0.0);
  z1.covariance = Eigen::Matrix2d::Identity() * 4.0;
  z1.source_id = "t";

  navtracker::Measurement z2 = z1;
  z2.value = Eigen::Vector2d(0.0, 100.0);

  tr.processBatch({z1, z2});
  // Two distinct measurements far apart -> two new tracks initiated.
  EXPECT_EQ(mgr.size(), 2u);
}

TEST(Tracker, RecordsRecentContributionsOnFusion) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(50.0);
  TrackManager manager(1, 3);
  Tracker tracker(estimator, associator, manager, 10.0);

  // Seed a track with an AIS measurement so a track exists.
  Measurement ais;
  ais.time = Timestamp::fromSeconds(0.0);
  ais.sensor = SensorKind::Ais;
  ais.source_id = "ais";
  ais.model = MeasurementModel::Position2D;
  ais.value = Eigen::Vector2d(100.0, 0.0);
  ais.covariance = Eigen::Matrix2d::Identity();
  tracker.process(ais);

  // Now feed an AIS and ARPA measurement at the same time, both within
  // the gate of the existing track. They should fuse into it.
  Measurement ais2 = ais;
  ais2.time = Timestamp::fromSeconds(1.0);
  ais2.value = Eigen::Vector2d(100.5, 0.0);

  Measurement arpa;
  arpa.time = Timestamp::fromSeconds(1.0);
  arpa.sensor = SensorKind::ArpaTtm;
  arpa.source_id = "arpa";
  arpa.model = MeasurementModel::Position2D;
  arpa.value = Eigen::Vector2d(100.6, 0.1);
  arpa.covariance = Eigen::Matrix2d::Identity();

  tracker.process(ais2);
  tracker.process(arpa);

  ASSERT_EQ(manager.size(), 1u);
  const auto& tr = manager.tracks()[0];
  ASSERT_GE(tr.recent_contributions.size(), 2u);
  std::set<std::string> source_ids;
  for (const auto& t : tr.recent_contributions) {
    source_ids.insert(t.source_id);
  }
  EXPECT_TRUE(source_ids.count("ais") > 0);
  EXPECT_TRUE(source_ids.count("arpa") > 0);
}

TEST(Tracker, ReplayIsDeterministic) {
  const std::vector<Measurement> stream{
      positionAt(0.0, 0.0, 0.0, "a"),
      positionAt(1.0, 1.0, 0.0, "a"),
      positionAt(2.0, 2.0, 0.0, "a"),
      positionAt(3.0, 3.0, 0.0, "a"),
  };

  auto run = [&stream]() {
    auto motion = std::make_shared<ConstantVelocity2D>(0.1);
    EkfEstimator estimator(motion, 10.0);
    GnnAssociator associator(20.0);
    TrackManager manager(2, 3);
    Tracker tracker(estimator, associator, manager, 10.0);
    for (const auto& z : stream) tracker.process(z);
    return manager.tracks();
  };

  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].id.value, b[i].id.value);
    ASSERT_EQ(a[i].state.size(), b[i].state.size());
    for (int k = 0; k < a[i].state.size(); ++k) {
      EXPECT_DOUBLE_EQ(a[i].state(k), b[i].state(k));
    }
  }
}

TEST(Tracker, PropagatesOwnPositionStdToSourceTouch) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(50.0);
  TrackManager manager(1, 3);
  Tracker tracker(estimator, associator, manager, 10.0);

  // Seed a track with an AIS measurement so a track exists.
  Measurement ais;
  ais.time = Timestamp::fromSeconds(0.0);
  ais.sensor = SensorKind::Ais;
  ais.source_id = "ais";
  ais.model = MeasurementModel::Position2D;
  ais.value = Eigen::Vector2d(100.0, 0.0);
  ais.covariance = Eigen::Matrix2d::Identity();
  tracker.process(ais);

  // Now feed an ARPA measurement at the same target, with a non-zero
  // sensor_position_std_m: the tracker must copy it onto SourceTouch.
  Measurement arpa;
  arpa.time = Timestamp::fromSeconds(1.0);
  arpa.sensor = SensorKind::ArpaTtm;
  arpa.source_id = "arpa";
  arpa.model = MeasurementModel::Position2D;
  arpa.value = Eigen::Vector2d(100.6, 0.1);
  arpa.covariance = Eigen::Matrix2d::Identity();
  arpa.sensor_position_std_m = 4.2;
  tracker.process(arpa);

  ASSERT_EQ(manager.size(), 1u);
  const auto& tr = manager.tracks()[0];
  bool found_arpa = false;
  for (const auto& t : tr.recent_contributions) {
    if (t.source_id == "arpa") {
      found_arpa = true;
      EXPECT_DOUBLE_EQ(t.own_position_std_m, 4.2);
    }
  }
  EXPECT_TRUE(found_arpa);
}

// --- Stale-input guard (default ON) -----------------------------------
// The engine is time-driven: a measurement older than the high-water
// mark would be applied against newer state and rewind last_update,
// silently corrupting the track. Default behaviour is to drop and count
// it; the ReorderBuffer is the tool for *recovering* late data.

TEST(Tracker, DropsStaleMeasurementByDefaultAndCounts) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);

  // Twin A: clean in-order feed.
  TrackManager mgr_clean(2, 3);
  Tracker clean(estimator, associator, mgr_clean, 100.0);
  clean.process(positionAt(123.0, 10.0, 0.0, "s1"));
  clean.process(positionAt(126.0, 16.0, 0.0, "s2"));

  // Twin B: same feed with a stale measurement injected. It gates to
  // the track, so without the guard it would corrupt state and rewind
  // last_update.
  TrackManager mgr(2, 3);
  Tracker tracker(estimator, associator, mgr, 100.0);
  tracker.process(positionAt(123.0, 10.0, 0.0, "s1"));
  tracker.process(positionAt(115.0, 9.0, 0.0, "s1"));   // stale
  tracker.process(positionAt(126.0, 16.0, 0.0, "s2"));

  EXPECT_EQ(tracker.staleDropped(), 1u);
  EXPECT_EQ(clean.staleDropped(), 0u);
  ASSERT_EQ(mgr.size(), 1u);
  ASSERT_EQ(mgr_clean.size(), 1u);
  // Identical to the clean run, bit for bit.
  EXPECT_TRUE(mgr.tracks()[0].state == mgr_clean.tracks()[0].state);
  EXPECT_EQ(mgr.tracks()[0].last_update.nanos(),
            mgr_clean.tracks()[0].last_update.nanos());
}

TEST(Tracker, EqualTimestampIsNotStale) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager mgr(2, 3);
  Tracker tracker(estimator, associator, mgr, 100.0);

  tracker.process(positionAt(123.0, 10.0, 0.0, "s1"));
  tracker.process(positionAt(123.0, 10.5, 0.0, "s2"));  // same instant: OK
  EXPECT_EQ(tracker.staleDropped(), 0u);
}

TEST(Tracker, StaleGuardCanBeOptedOut) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager mgr(2, 3);
  Tracker tracker(estimator, associator, mgr, 100.0);
  tracker.setRejectStaleMeasurements(false);

  tracker.process(positionAt(123.0, 10.0, 0.0, "s1"));
  // Far away: with the guard off this seeds a second track (legacy
  // behaviour); with the guard on it would have been dropped.
  tracker.process(positionAt(115.0, 1000.0, 0.0, "s1"));
  EXPECT_EQ(tracker.staleDropped(), 0u);
  EXPECT_EQ(mgr.size(), 2u);
}

TEST(Tracker, ProcessBatchDropsStaleScan) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator estimator(motion, 10.0);
  const GnnAssociator associator(20.0);
  TrackManager mgr(2, 3);
  Tracker tracker(estimator, associator, mgr, 100.0);

  tracker.processBatch({positionAt(123.0, 10.0, 0.0, "s1")});
  tracker.processBatch({positionAt(115.0, 9.0, 0.0, "s1"),
                        positionAt(115.0, 1000.0, 0.0, "s1")});
  EXPECT_EQ(tracker.staleDropped(), 2u);
  EXPECT_EQ(mgr.size(), 1u);  // stale far-away measurement seeded nothing
}
