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

// Introspection: a fed pier is counted as one structure; querying its
// suppressed region increments the hit counter; a boat-only feed classifies
// no structure and a query there records no hit.
TEST(LiveOccupancyModel, IntrospectionTracksStructureAndHits) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(pierReturns(), scan));
  EXPECT_EQ(m.peakStructureCount(), 1);
  EXPECT_GT(m.peakPersistence(), testParams().persistence_bar);
  const long before = m.suppressionHits();
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
  EXPECT_EQ(m.suppressionHits(), before + 1);  // the query landed in structure

  LiveOccupancyModel boat(anchorDatum(), testParams());
  for (int scan = 0; scan < 20; ++scan)
    boat.observe(feed({boatReturn()}, scan));
  EXPECT_EQ(boat.peakStructureCount(), 0);  // compact → never structure
  EXPECT_DOUBLE_EQ(boat.birthSuppression(Eigen::Vector2d(1012.5, 1012.5)), 0.0);
  EXPECT_EQ(boat.suppressionHits(), 0);
}

// CONSERVATION INVARIANT (ADR 0002 amendment 2026-07-03): birth suppression at
// a location is legal ONLY if that location is emitted as a static hazard.
// Every query with birthSuppression > 0 must lie inside some emitted hazard's
// keep_clear ring. A 2D block exposes the failure a single centroid hazard has:
// a per-cell-suppressed corner point can fall outside the centroid's keep-clear.
TEST(LiveOccupancyModel, SuppressionImpliesAnEmittedHazardCoversIt) {
  const geo::Datum anchor = anchorDatum();
  LiveOccupancyModel m(anchor, testParams());
  // A 3x3 block of cells (extended → classified structure).
  std::vector<std::pair<Eigen::Vector2d, double>> block;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      block.emplace_back(Eigen::Vector2d(12.5 + 25.0 * i, 12.5 + 25.0 * j), 1.0);
  for (int scan = 0; scan < 12; ++scan) m.observe(feed(block, scan));
  ASSERT_FALSE(m.obstacles().empty());

  // Precompute each hazard's anchor-ENU centre + keep-clear radius.
  std::vector<std::pair<Eigen::Vector2d, double>> rings;
  for (const auto& o : m.obstacles()) {
    const Eigen::Vector3d e = anchor.toEnu(o.position);
    rings.emplace_back(Eigen::Vector2d(e.x(), e.y()), o.keep_clear_radius_m);
  }

  // Sweep a fine grid over the structure's neighbourhood.
  for (double x = -80.0; x <= 160.0; x += 4.0) {
    for (double y = -80.0; y <= 160.0; y += 4.0) {
      const Eigen::Vector2d q(x, y);
      if (m.birthSuppression(q) <= 0.0) continue;
      bool covered = false;
      for (const auto& r : rings)
        if ((q - r.first).norm() <= r.second + 1e-6) {
          covered = true;
          break;
        }
      EXPECT_TRUE(covered)
          << "suppressed q=(" << x << "," << y << ") not in any hazard ring";
    }
  }
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
