// Review #4: the JPDA soft-update path must populate contributing_sources
// (provenance) just like the hard/GNN path does. Before the fix the soft
// branch updated recent_contributions (bias history) but left
// contributing_sources empty, so TrackOutput under-reported the fusing
// sensors whenever the tracker ran in JPDA mode.

#include <algorithm>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/JpdaAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

using namespace navtracker;

namespace {

Measurement posMeas(double x, double y, double t, const std::string& src) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 25.0;  // 5 m std
  m.source_id = src;
  m.sensor = SensorKind::Ais;
  return m;
}

bool hasSource(const Track& t, const std::string& src) {
  return std::find(t.contributing_sources.begin(),
                   t.contributing_sources.end(),
                   src) != t.contributing_sources.end();
}

}  // namespace

TEST(JpdaContributingSources, SoftUpdateRecordsEachFusingSource) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  JpdaAssociator jpda(20.0, /*P_D*/ 0.9, /*lambda_C*/ 1e-4);
  TrackManager mgr(/*confirm*/ 1, /*delete*/ 5);
  Tracker tracker(ekf, jpda, mgr, /*miss_timeout*/ 30.0);

  // Batch 1: a single AIS report initiates the track.
  tracker.processBatch({posMeas(0.0, 0.0, 0.0, "ais")});
  ASSERT_EQ(mgr.size(), 1u);
  ASSERT_TRUE(hasSource(mgr.tracks()[0], "ais"));

  // Batch 2: a radar report near the predicted position soft-updates the
  // same track (JPDA β > 0). Its source must now appear in provenance.
  tracker.processBatch({posMeas(1.0, 0.5, 1.0, "radar")});
  ASSERT_EQ(mgr.size(), 1u);
  const Track& t = mgr.tracks()[0];
  EXPECT_TRUE(hasSource(t, "ais"));
  EXPECT_TRUE(hasSource(t, "radar"));

  // No duplicate entries: a second radar report keeps the list deduped.
  tracker.processBatch({posMeas(2.0, 1.0, 2.0, "radar")});
  const Track& t2 = mgr.tracks()[0];
  const long radar_count = std::count(t2.contributing_sources.begin(),
                                      t2.contributing_sources.end(), "radar");
  EXPECT_EQ(radar_count, 1);
}
