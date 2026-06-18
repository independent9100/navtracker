// Regression tests for the 2026-06-18 code-review fixes #1-#3:
//   #1  MHT pipeline propagates fused identity (MMSI + contributing_sources)
//   #2  MHT pipeline shifts internal tree state on datum recenter
//   #3  single-hypothesis Tracker bounds recent_contributions to a window
#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {
namespace {

Measurement aisMeas(double x, double y, double t, std::uint32_t mmsi,
                    const std::string& src) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.sensor = SensorKind::Ais;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.hints.mmsi = mmsi;
  return m;
}

// --- #1: identity propagation in the MHT pipeline ---------------------
TEST(ReviewFix1, MhtPropagatesMmsiAndContributingSources) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  // A static-ish AIS target with a stable MMSI over several scans.
  for (int i = 1; i <= 6; ++i) {
    mht.processBatch({aisMeas(static_cast<double>(i) * 2.0, 0.0,
                              static_cast<double>(i), 244000001u, "ais")});
  }

  ASSERT_EQ(mht.tracks().size(), 1u);
  const Track& tr = mht.tracks().front();
  ASSERT_TRUE(tr.attributes.mmsi.has_value());
  EXPECT_EQ(*tr.attributes.mmsi, 244000001u);
  ASSERT_EQ(tr.contributing_sources.size(), 1u);
  EXPECT_EQ(tr.contributing_sources.front(), "ais");
}

TEST(ReviewFix1, MhtIdentitySurvivesBirthFromAis) {
  // Even if a tree is born from AIS and never hit again on the very next
  // scan, the birth-time capture must populate identity.
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);
  mht.processBatch({aisMeas(0.0, 0.0, 1.0, 999u, "ais")});
  ASSERT_EQ(mht.tracks().size(), 1u);
  ASSERT_TRUE(mht.tracks().front().attributes.mmsi.has_value());
  EXPECT_EQ(*mht.tracks().front().attributes.mmsi, 999u);
}

// --- #2: MHT datum-recenter shift -------------------------------------
TEST(ReviewFix2, MhtOnDatumRecenteredPreservesGeodeticPosition) {
  const geo::Datum old_datum(geo::Geodetic{53.5, 8.0, 0.0});
  const geo::Datum new_datum(geo::Geodetic{53.5, 9.0, 0.0});  // ~66 km east

  // A real-world target a little north-east of the old origin.
  const double target_lat = 53.55, target_lon = 8.10;
  const auto enu_old = old_datum.toEnu({target_lat, target_lon, 0.0});

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  // Settle a track on the target (static point) in the OLD datum frame.
  for (int i = 1; i <= 6; ++i) {
    mht.processBatch({aisMeas(enu_old.x(), enu_old.y(),
                              static_cast<double>(i), 1u, "ais")});
  }
  ASSERT_EQ(mht.tracks().size(), 1u);

  mht.onDatumRecentered(old_datum, new_datum);

  // The track state must now describe the SAME geodetic point, expressed
  // in the new datum's ENU frame.
  const Track& tr = mht.tracks().front();
  const auto enu_new_expected = new_datum.toEnu({target_lat, target_lon, 0.0});
  EXPECT_NEAR(tr.state(0), enu_new_expected.x(), 2.0);  // within 2 m
  EXPECT_NEAR(tr.state(1), enu_new_expected.y(), 2.0);

  // And it must NOT still be sitting at the old-frame coordinates.
  EXPECT_GT(std::abs(tr.state(0) - enu_old.x()), 1000.0);
}

// --- #3: single-hypothesis Tracker bounds recent_contributions --------
TEST(ReviewFix3, TrackerPrunesRecentContributions) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 10.0);
  const GnnAssociator gnn(50.0);
  TrackManager mgr(2, 3);
  Tracker tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/1000.0);

  // 200 scans, 1 s apart, single moving target — without pruning
  // recent_contributions would hold ~200 entries; with the 2 s window it
  // must stay tiny.
  for (int i = 0; i < 200; ++i) {
    tracker.process(aisMeas(static_cast<double>(i) * 5.0, 0.0,
                            static_cast<double>(i), 7u, "ais"));
  }
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_LE(mgr.tracks().front().recent_contributions.size(), 4u);
  EXPECT_GE(mgr.tracks().front().recent_contributions.size(), 1u);
}

}  // namespace
}  // namespace navtracker
