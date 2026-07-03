// Unit tests for LiveOccupancyModel (Stage 1b-i): a datum-stable occupancy
// grid that learns persistent + extended structure from the PMBM per-scan
// (position, 1 − r) feed and suppresses births there only — boats (compact)
// and uniform clutter (transient) are never suppressed.
#include "core/static/LiveOccupancyModel.hpp"

#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {
namespace {

geo::Datum anchorDatum() {
  return geo::Datum(geo::Geodetic{63.0, 10.0, 0.0});
}

// Build a single-sensor scan feed: each (position, weight) becomes a
// clutter-labeled return, as the feed_clutter_map producer emits them.
std::vector<ISensorDetectionModel::ScanObservation> feed(
    const std::vector<std::pair<Eigen::Vector2d, double>>& returns,
    double t_sec) {
  ISensorDetectionModel::ScanObservation obs;
  obs.sensor = SensorKind::ArpaTtm;
  obs.model = MeasurementModel::Position2D;
  obs.num_unassociated = 0;
  obs.time = Timestamp::fromSeconds(t_sec);
  for (const auto& [p, w] : returns) {
    obs.clutter_positions.push_back(p);
    obs.clutter_position_weights.push_back(w);
  }
  return {obs};
}

// A pier: six contiguous cell centres along x (25 m cells) at y = 0.
std::vector<std::pair<Eigen::Vector2d, double>> pierReturns() {
  std::vector<std::pair<Eigen::Vector2d, double>> r;
  for (int k = 0; k < 6; ++k)
    r.emplace_back(Eigen::Vector2d(12.5 + 25.0 * k, 0.0), 1.0);
  return r;
}

// A single anchored boat, one cell, far from the pier.
std::pair<Eigen::Vector2d, double> boatReturn() {
  return {Eigen::Vector2d(1012.5, 1012.5), 1.0};
}

LiveOccupancyParams testParams() {
  LiveOccupancyParams p;
  p.cell_size_m = 25.0;
  p.ewma_alpha = 0.3;
  p.persistence_bar = 0.5;
  p.extended_cells_min = 4;
  p.suppression_max = 0.9;
  p.suppression_radius_m = 25.0;
  return p;
}

// Persistent + extended structure (a pier) suppresses births there; a compact
// persistent region (a boat) and empty water do not.
TEST(LiveOccupancyModel, PersistentExtendedStructureSuppressesBirths) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  auto returns = pierReturns();
  returns.push_back(boatReturn());
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));

  // Pier → suppressed (near the soft-max cap once fully persistent).
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.5);
  // Boat → compact → never suppressed.
  EXPECT_DOUBLE_EQ(m.birthSuppression(Eigen::Vector2d(1012.5, 1012.5)), 0.0);
  // Open water far from any return → 0.
  EXPECT_DOUBLE_EQ(m.birthSuppression(Eigen::Vector2d(5000.0, 5000.0)), 0.0);
  // Only the pier is emitted as a (live, uncharted) structure hazard.
  EXPECT_EQ(m.obstacles().size(), 1u);
}

// Uniform, non-repeating clutter never becomes persistent, so it is never
// suppressed — this is the dense_clutter no-regression guarantee.
TEST(LiveOccupancyModel, UniformTransientClutterNeverSuppresses) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  // Each scan touches a fresh, distinct far-apart cell (never revisited).
  for (int scan = 0; scan < 40; ++scan) {
    Eigen::Vector2d p(2000.0 + 100.0 * scan, -3000.0 - 100.0 * scan);
    m.observe(feed({{p, 1.0}}, scan));
    EXPECT_DOUBLE_EQ(m.birthSuppression(p), 0.0);
  }
  EXPECT_TRUE(m.obstacles().empty());
}

// A structure that stops being fed decays below the persistence bar and stops
// suppressing (EWMA forgetting).
TEST(LiveOccupancyModel, AbandonedStructureDecaysAndStopsSuppressing) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.5);
  // Feed unrelated far clutter for many scans — the pier is no longer touched.
  for (int scan = 10; scan < 40; ++scan)
    m.observe(feed({{Eigen::Vector2d(9000.0 + scan, 9000.0), 1.0}}, scan));
  EXPECT_DOUBLE_EQ(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
}

// Suppression is capped strictly below the tracker's default hard gate (0.95):
// the live layer is soft-only, never a hard no-birth kill (R3 safety — a
// mislabeled large vessel must still be able to birth and confirm).
TEST(LiveOccupancyModel, SuppressionIsSoftOnly) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  auto returns = pierReturns();
  for (int scan = 0; scan < 100; ++scan) m.observe(feed(returns, scan));
  EXPECT_LE(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.9);
}

// Datum recenter re-anchors: the same GEOGRAPHIC point keeps its suppression
// even though its ENU coordinates change under the new datum.
TEST(LiveOccupancyModel, DatumRecenterKeepsGeographicSuppression) {
  const geo::Datum anchor = anchorDatum();
  LiveOccupancyModel m(anchor, testParams());
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));

  const Eigen::Vector2d q_old(62.5, 0.0);
  const double s_old = m.birthSuppression(q_old);
  ASSERT_GT(s_old, 0.5);

  // New datum shifted ~2 km east of the anchor.
  const geo::Datum shifted(
      anchor.toGeodetic(Eigen::Vector3d(2000.0, 0.0, 0.0)));
  m.onDatumRecentered(anchor, shifted);

  // Same geographic point, expressed in the NEW datum's ENU frame.
  const geo::Geodetic q_geo =
      anchor.toGeodetic(Eigen::Vector3d(q_old.x(), q_old.y(), 0.0));
  const Eigen::Vector3d q_new3 = shifted.toEnu(q_geo);
  const double s_new =
      m.birthSuppression(Eigen::Vector2d(q_new3.x(), q_new3.y()));
  EXPECT_NEAR(s_new, s_old, 1e-6);
}

// Identical feed sequences produce identical suppression (determinism).
TEST(LiveOccupancyModel, DeterministicAcrossIdenticalRuns) {
  auto run = []() {
    LiveOccupancyModel m(anchorDatum(), testParams());
    auto returns = pierReturns();
    returns.push_back(boatReturn());
    for (int scan = 0; scan < 15; ++scan) m.observe(feed(returns, scan));
    return m.birthSuppression(Eigen::Vector2d(62.5, 0.0));
  };
  EXPECT_DOUBLE_EQ(run(), run());
}

}  // namespace
}  // namespace navtracker
