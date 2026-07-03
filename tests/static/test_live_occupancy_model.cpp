// Unit tests for LiveOccupancyModel (Stage 1b-i): a datum-stable occupancy
// grid that learns persistent + extended structure from the PMBM per-scan
// (position, 1 − r) feed and suppresses births there only — boats (compact)
// and uniform clutter (transient) are never suppressed.
#include "core/static/LiveOccupancyModel.hpp"

#include <cmath>
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

// Single-sensor feed with a coverage footprint attached (R8.4 coverage-aware
// decay). Absent this, feed() leaves coverage.valid == false ⇒ full coverage.
std::vector<ISensorDetectionModel::ScanObservation> feedCov(
    const std::vector<std::pair<Eigen::Vector2d, double>>& returns, double t_sec,
    const ISensorDetectionModel::CoverageSector& cov) {
  auto bundle = feed(returns, t_sec);
  bundle[0].coverage = cov;
  return bundle;
}

// A coverage disc about `sensor` of radius `range` (full azimuth).
ISensorDetectionModel::CoverageSector disc(const Eigen::Vector2d& sensor,
                                           double range) {
  ISensorDetectionModel::CoverageSector c;
  c.valid = true;
  c.sensor_enu = sensor;
  c.max_range_m = range;
  return c;  // sector_width_rad defaults to full circle
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

// CLUTTER SAFETY (detector mode): the clutter-adaptive bar rejects dense uniform
// clutter that an absolute bar would classify — preventing the dense_clutter
// death-spiral (false hazards everywhere + suppressed births). Uniform cells fed
// at a sub-structure rate stay below the adaptive bar; a structure cell fed every
// scan stays above it.
TEST(LiveOccupancyModel, ClutterAdaptiveBarRejectsDenseUniformClutter) {
  auto drive = [](LiveOccupancyModel& m) {
    const Eigen::Vector2d structure(50.0, 50.0);
    const std::vector<Eigen::Vector2d> clutter = {
        {3000.0, 0.0}, {3200.0, 0.0}, {3400.0, 0.0}, {3600.0, 0.0}, {3800.0, 0.0}};
    for (int scan = 0; scan < 30; ++scan) {
      std::vector<std::pair<Eigen::Vector2d, double>> r = {{structure, 1.0}};
      if (scan % 3 == 0)  // uniform cells revisited ~1/3 of scans
        for (const auto& c : clutter) r.emplace_back(c, 1.0);
      m.observe(feed(r, scan));
    }
  };
  LiveOccupancyParams base;
  base.cell_size_m = 100.0;
  base.ewma_alpha = 0.3;
  base.persistence_bar = 0.2;
  base.extended_cells_min = 1;  // detector: a single persistent cell classifies

  // Absolute bar (clutter-adaptive OFF): the 1/3-rate clutter cells cross 0.2 →
  // wrongly classified alongside the structure cell.
  LiveOccupancyModel abs_m(anchorDatum(), base);
  drive(abs_m);
  EXPECT_GT(abs_m.peakStructureCount(), 1)
      << "absolute bar wrongly classifies uniform clutter cells";

  // Clutter-adaptive ON: bar rises above the uniform-clutter persistence at this
  // density → only the every-scan structure cell survives.
  LiveOccupancyParams adapt = base;
  adapt.clutter_adaptive = true;      // estimate clutter background from the feed
  adapt.clutter_reject_factor = 2.0;  // bar ≈ 2×median > clutter, < structure
  LiveOccupancyModel ad_m(anchorDatum(), adapt);
  drive(ad_m);
  EXPECT_EQ(ad_m.peakStructureCount(), 1)
      << "adaptive bar should keep ONLY the structure cell, reject clutter";
}

// RECOVERY / bounded latency (ADR 0002 amendment rule 3): a static object that
// starts moving must stop being suppressed within a bounded number of scans, so
// the mover can birth normally (no permanent "static" pin). At the detector's
// low extent floor a compact anchored boat DOES classify + suppress (accepted
// degraded mode); when its returns leave, the vacated cell must forget quickly.
TEST(LiveOccupancyModel, VacatedCellsRecoverWithinBoundedLatency) {
  LiveOccupancyParams p = testParams();
  p.extended_cells_min = 1;  // detector mode: a compact boat classifies
  LiveOccupancyModel m(anchorDatum(), p);
  const Eigen::Vector2d spot(1012.5, 1012.5);
  // Anchored long enough to be classified + suppressed.
  for (int scan = 0; scan < 15; ++scan) m.observe(feed({{spot, 1.0}}, scan));
  ASSERT_GT(m.birthSuppression(spot), 0.0) << "boat should suppress while anchored";

  // It gets underway: its returns now appear at fresh, never-revisited cells
  // (a mover never dwells). Count scans until the vacated spot stops suppressing.
  int latency = -1;
  for (int k = 0; k < 20; ++k) {
    Eigen::Vector2d moving(1200.0 + 60.0 * k, 1200.0 + 60.0 * k);
    m.observe(feed({{moving, 1.0}}, 15 + k));
    // The mover's own current cell must never be suppressed (it never dwells).
    EXPECT_DOUBLE_EQ(m.birthSuppression(moving), 0.0) << "a mover must never suppress";
    if (m.birthSuppression(spot) <= 0.0) { latency = k + 1; break; }
  }
  ASSERT_GE(latency, 0) << "vacated spot never recovered";
  EXPECT_LE(latency, 5) << "recovery latency " << latency << " scans exceeds bound";
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

// COVERAGE-AWARE DECAY (R8.4 / increment 6): a cell forgets ONLY when it was
// observable (inside the scan's coverage footprint) and returned empty. A cell
// OUTSIDE coverage must NOT decay — absence of returns where the sensor did not
// look is not evidence of vacancy. This is what distinguishes a DEPARTED vessel
// (returns cease while the cell is still in view — the sunset_cruise loiterer at
// t≈94) from a cell that merely fell out of coverage this scan.
TEST(LiveOccupancyModel, OutOfRangeCellsDoNotDecay) {
  LiveOccupancyParams p = testParams();
  p.extended_cells_min = 1;  // detector mode: a compact cell classifies
  LiveOccupancyModel m(anchorDatum(), p);
  const Eigen::Vector2d spot(1012.5, 1012.5);  // |spot| ≈ 1431 m from origin
  for (int scan = 0; scan < 15; ++scan) m.observe(feed({{spot, 1.0}}, scan));
  const double s0 = m.birthSuppression(spot);
  ASSERT_GT(s0, 0.0);

  // The sensor now looks only within 500 m of the origin — spot is out of range.
  // Feed a return inside the disc so scans are non-empty. spot must not decay.
  const auto cov = disc(Eigen::Vector2d(0.0, 0.0), 500.0);
  for (int k = 0; k < 25; ++k)
    m.observe(feedCov({{Eigen::Vector2d(100.0, 0.0), 1.0}}, 15 + k, cov));

  EXPECT_DOUBLE_EQ(m.birthSuppression(spot), s0)
      << "a cell outside the sensor's coverage range must not decay";
}

// The azimuth SECTOR matters (philos per-burst sweeps — the reason a disc-only
// model is wrong): a cell IN RANGE but OUTSIDE the swept sector is unobserved
// and must not decay.
TEST(LiveOccupancyModel, InRangeButOutOfSectorDoesNotDecay) {
  LiveOccupancyParams p = testParams();
  p.extended_cells_min = 1;
  LiveOccupancyModel m(anchorDatum(), p);
  const Eigen::Vector2d spot(1012.5, 1012.5);  // bearing ≈ 45° from origin
  for (int scan = 0; scan < 15; ++scan) m.observe(feed({{spot, 1.0}}, scan));
  const double s0 = m.birthSuppression(spot);
  ASSERT_GT(s0, 0.0);

  // Sensor at origin, range covers spot, but the sector points WEST (180°),
  // 90° wide → [135°, 225°], which excludes spot's 45°.
  ISensorDetectionModel::CoverageSector cov = disc(Eigen::Vector2d(0.0, 0.0), 5000.0);
  cov.sector_center_rad = M_PI;       // west
  cov.sector_width_rad = M_PI / 2.0;  // 90°
  for (int k = 0; k < 25; ++k)
    m.observe(feedCov({{Eigen::Vector2d(-100.0, 0.0), 1.0}}, 15 + k, cov));

  EXPECT_DOUBLE_EQ(m.birthSuppression(spot), s0)
      << "a cell in range but outside the swept sector must not decay";
}

// Recovery still works: an OBSERVED-but-empty cell DOES decay (else a departed
// vessel's cells would pin forever). Same setup, coverage now INCLUDES spot.
TEST(LiveOccupancyModel, ObservedEmptyCellsStillDecay) {
  LiveOccupancyParams p = testParams();
  p.extended_cells_min = 1;
  LiveOccupancyModel m(anchorDatum(), p);
  const Eigen::Vector2d spot(1012.5, 1012.5);
  for (int scan = 0; scan < 15; ++scan) m.observe(feed({{spot, 1.0}}, scan));
  ASSERT_GT(m.birthSuppression(spot), 0.0);

  const auto cov = disc(Eigen::Vector2d(0.0, 0.0), 5000.0);  // covers spot
  int latency = -1;
  for (int k = 0; k < 20; ++k) {
    m.observe(feedCov({{Eigen::Vector2d(100.0, 0.0), 1.0}}, 15 + k, cov));
    if (m.birthSuppression(spot) <= 0.0) { latency = k + 1; break; }
  }
  EXPECT_GE(latency, 0)
      << "observed-empty cell never decayed — a departed vessel would pin forever";
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
