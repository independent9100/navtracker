// Unit tests for LiveOccupancyModel (Stage 1b-i): a datum-stable occupancy
// grid that learns persistent + extended structure from the PMBM per-scan
// (position, 1 − r) feed and suppresses births there only — boats (compact)
// and uniform clutter (transient) are never suppressed.
#include "core/static/LiveOccupancyModel.hpp"

#include <cmath>
#include <memory>
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

// A charted structure point at anchor-ENU `enu` (charts are geodetic).
StaticObstacle chartAt(const Eigen::Vector2d& enu) {
  StaticObstacle o;
  o.position = anchorDatum().toGeodetic(Eigen::Vector3d(enu.x(), enu.y(), 0.0));
  o.source_id = "chart";
  return o;
}

// Chart corroboration (increment 6): a live-structure hazard whose centroid
// coincides with a charted structure point is CONFIRMED. Label only — the
// emitted hazard and its suppression are unchanged.
TEST(LiveOccupancyModel, ChartCoincidentStructureIsCorroborated) {
  LiveOccupancyParams p = testParams();
  p.chart_corroboration_radius_m = 50.0;
  LiveOccupancyModel m(anchorDatum(), p);
  m.setChartedStructure({chartAt({75.0, 12.5})});  // on the pier centroid
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_TRUE(m.obstacleCorroborated(0));
  EXPECT_EQ(m.chartCorroboratedCount(), 1);
}

// A charted point far from the live structure does NOT corroborate it — the
// uncorroborated hazard is the eviction candidate (increment 8).
TEST(LiveOccupancyModel, ChartDistantFromStructureIsNotCorroborated) {
  LiveOccupancyParams p = testParams();
  p.chart_corroboration_radius_m = 50.0;
  LiveOccupancyModel m(anchorDatum(), p);
  m.setChartedStructure({chartAt({5000.0, 5000.0})});  // far from the pier
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_FALSE(m.obstacleCorroborated(0));
  EXPECT_EQ(m.chartCorroboratedCount(), 0);
}

// Inert-by-default: with no charts set, every hazard reports uncorroborated
// and behaviour is bit-identical to the no-chart model.
TEST(LiveOccupancyModel, NoChartsMeansNoCorroboration) {
  LiveOccupancyModel m(anchorDatum(), testParams());
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_FALSE(m.obstacleCorroborated(0));
  EXPECT_EQ(m.chartCorroboratedCount(), 0);
}

// A live camera frame looking toward the pier, with a detection elsewhere in
// the FOV (frame live) but none at the pier bearing.
LiveOccupancyModel::CameraObservation emptyPierFrame(double t) {
  LiveOccupancyModel::CameraObservation f;
  f.t_unix = t;
  f.sensor_enu = Eigen::Vector2d(0.0, 0.0);
  f.fov_center_rad = 0.165;        // toward the pier centroid (~9.5 deg)
  f.fov_half_width_rad = 0.4;      // ~23 deg half-HFOV
  f.match_tolerance_rad = 0.05;
  f.detection_bearings_rad = {0.5};  // live, but away from any pier cell
  return f;
}

// Camera corroboration (increment 6): a hazard whose cell is continuously
// camera-observed-empty (in FOV, live frame, no detection at its bearing) for
// >= the sustain window is flagged — the departed-vessel signal.
TEST(LiveOccupancyModel, CameraObservedEmptyFlagsSustainedEmptyCell) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_EQ(m.obstacles().size(), 1u);
  ASSERT_FALSE(m.obstacleCameraObservedEmpty(0));  // no camera fed yet

  for (double t = 100.0; t <= 102.0; t += 0.5) m.observeCamera(emptyPierFrame(t));
  m.observe(feed(returns, 103));  // recompute reads the streak, flags the hazard
  ASSERT_EQ(m.obstacles().size(), 1u);
  EXPECT_TRUE(m.obstacleCameraObservedEmpty(0));
  EXPECT_EQ(m.cameraObservedEmptyCount(), 1);
}

// A detection AT the hazard's bearing resets the streak — something IS there, so
// it is not observed-empty.
TEST(LiveOccupancyModel, CameraDetectionAtBearingKeepsHazardUnflagged) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  for (double t = 100.0; t <= 102.0; t += 0.5) {
    auto f = emptyPierFrame(t);
    // Pier centroid cell (3,0) centre (87.5,12.5) has bearing atan2(12.5,87.5).
    f.detection_bearings_rad = {std::atan2(12.5, 87.5)};
    m.observeCamera(f);
  }
  m.observe(feed(returns, 103));
  EXPECT_FALSE(m.obstacleCameraObservedEmpty(0));
}

// A hazard OUTSIDE the camera FOV is never flagged — absence there is not
// evidence of absence (the coverage-aware-decay principle, camera modality).
TEST(LiveOccupancyModel, CameraOutOfFovNeverFlags) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  for (double t = 100.0; t <= 102.0; t += 0.5) {
    auto f = emptyPierFrame(t);
    f.fov_center_rad = 2.5;  // pointing away; the pier is out of FOV
    m.observeCamera(f);
  }
  m.observe(feed(returns, 103));
  EXPECT_FALSE(m.obstacleCameraObservedEmpty(0));
}

// ── Increment (ii): camera EVICTION as behaviour ────────────────────────────
//
// A departed vessel leaves a FROZEN pin — its radar returns cease and coverage-
// aware decay holds the persistence (the cell is outside the swept sector, so
// radar cannot clear it; the 6c corroboration wall). Camera-observed-empty +
// chart-UNconfirmed EVICTS it: the cell's accumulated persistence is SPENT
// (erased), so the frozen evidence cannot re-emit it. Eviction OFF (increment i)
// leaves the frozen pin in place. Conservation-safe: the previously-suppressed
// location's birthSuppression drops to exactly 0 (a birth there is now free).
TEST(LiveOccupancyModel, CameraEvictionRemovesFrozenDepartedPin) {
  auto build = [](bool evict) {
    LiveOccupancyParams p = testParams();
    p.camera_empty_sustain_s = 1.0;
    p.evict_camera_empty = evict;
    auto m = std::make_unique<LiveOccupancyModel>(anchorDatum(), p);
    auto returns = pierReturns();
    for (int scan = 0; scan < 10; ++scan) m->observe(feed(returns, scan));
    // Returns cease; the pier is now OUT of coverage every scan (a distant sweep)
    // → coverage-aware decay freezes it (the departed-vessel pin radar can't clear).
    const auto away = disc(Eigen::Vector2d(5000.0, 5000.0), 500.0);
    for (int scan = 10; scan < 15; ++scan)
      m->observe(feedCov({{Eigen::Vector2d(5000.0, 5000.0), 1.0}}, scan, away));
    return m;
  };

  // Baseline (eviction off): the frozen pin persists — the corroboration wall.
  auto off = build(false);
  ASSERT_GT(off->birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);

  // Eviction on: camera observes the pier bearing empty (matured), then one more
  // frozen scan runs the eviction pre-pass and spends the refuted persistence.
  auto on = build(true);
  ASSERT_GT(on->birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
  for (double t = 15.0; t <= 17.0; t += 0.5) on->observeCamera(emptyPierFrame(t));
  const auto away = disc(Eigen::Vector2d(5000.0, 5000.0), 500.0);
  on->observe(feedCov({{Eigen::Vector2d(5000.0, 5000.0), 1.0}}, 17.5, away));
  EXPECT_TRUE(on->obstacles().empty()) << "frozen departed pin evicted by camera";
  EXPECT_DOUBLE_EQ(on->birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);

  // No blink: a further frozen scan does NOT re-emit (persistence was spent, not
  // just the hazard dropped — coverage-aware freeze cannot resurrect it).
  on->observe(feedCov({{Eigen::Vector2d(5000.0, 5000.0), 1.0}}, 18.0, away));
  EXPECT_TRUE(on->obstacles().empty()) << "evicted pin must not re-emit (no blink)";
}

// Evidence precedence: a chart-CONFIRMED component is HELD regardless of camera
// (chart-confirmed → hold > camera-empty → evict). The same frozen, camera-empty
// pier that Test-above evicts is retained here because a charted point coincides.
TEST(LiveOccupancyModel, ChartConfirmedStructureHeldAgainstCameraEviction) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  p.evict_camera_empty = true;
  p.chart_corroboration_radius_m = 50.0;
  LiveOccupancyModel m(anchorDatum(), p);
  m.setChartedStructure({chartAt({75.0, 12.5})});  // on the pier centroid
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  const auto away = disc(Eigen::Vector2d(5000.0, 5000.0), 500.0);
  for (int scan = 10; scan < 15; ++scan)
    m.observe(feedCov({{Eigen::Vector2d(5000.0, 5000.0), 1.0}}, scan, away));
  for (double t = 15.0; t <= 17.0; t += 0.5) m.observeCamera(emptyPierFrame(t));
  m.observe(feedCov({{Eigen::Vector2d(5000.0, 5000.0), 1.0}}, 17.5, away));
  ASSERT_EQ(m.obstacles().size(), 1u)
      << "chart-confirmed structure held regardless of camera";
  EXPECT_TRUE(m.obstacleCorroborated(0));
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
}

// THE DECOUPLING GUARANTEE (the increment-ii design steer): camera-empty evidence
// is keyed by CELL and accrues even while the cell is NOT in the structure set;
// eviction fires the instant a flickering cell RE-ENTERS structure while holding
// matured, recent evidence. (Increment i only reported the flag when the cell
// happened to be emitted — the loiterer's coincidence requirement. Here the pier
// leaves structure, matures its evidence off-stage, and is evicted on re-entry.)
TEST(LiveOccupancyModel, CameraEvictionKeyedByCellFiresOnStructureReentry) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  p.evict_camera_empty = true;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  ASSERT_EQ(m.obstacles().size(), 1u);

  // The pier drops OUT of the structure set: observed-empty scans (coverage
  // covers it) decay it below the bar. Meanwhile the camera keeps observing its
  // bearing empty — evidence accrues per cell though nothing is emitted.
  const auto cover = disc(Eigen::Vector2d(0.0, 0.0), 5000.0);  // covers the pier
  for (double t = 10.0; t <= 12.0; t += 1.0) {
    m.observe(feedCov({{Eigen::Vector2d(300.0, 300.0), 1.0}}, t, cover));
    m.observeCamera(emptyPierFrame(t));
  }
  ASSERT_TRUE(m.obstacles().empty()) << "pier decayed below the bar — not structure now";

  // The pier RE-ENTERS the structure set (returns resume). Eviction must fire on
  // re-entry using the evidence matured while it was off-stage — NOT re-emit it.
  m.observe(feed(returns, 13));
  EXPECT_TRUE(m.obstacles().empty())
      << "re-entered cell evicted on the spot from matured off-stage evidence";
}

// A stale streak (the camera stopped looking long ago) must NOT evict — evidence
// has a recency window. Guards a real vessel that re-pins a cell the camera
// observed empty in the distant past.
TEST(LiveOccupancyModel, CameraEvictionIgnoresStaleEvidence) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  p.camera_empty_recency_window_s = 5.0;
  p.evict_camera_empty = true;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  // Mature the streak early (t = 10..12).
  for (double t = 10.0; t <= 12.0; t += 0.5) m.observeCamera(emptyPierFrame(t));
  // Time passes with the pier kept alive by returns but NO further camera obs.
  // The scan clock advances far beyond the streak's last frame (12) → stale.
  for (int scan = 30; scan < 40; ++scan) m.observe(feed(returns, scan));
  EXPECT_EQ(m.obstacles().size(), 1u) << "stale evidence must not evict";
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
}

// Eviction is OFF by default: matured camera-empty on an uncharted hazard FLAGS
// it (increment i) but does NOT remove it. The config flag is the sole switch.
TEST(LiveOccupancyModel, CameraEvictionOffKeepsFlaggedHazard) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;  // evict_camera_empty stays false (default)
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  for (double t = 10.0; t <= 12.0; t += 0.5) m.observeCamera(emptyPierFrame(t));
  m.observe(feed(returns, 13));
  ASSERT_EQ(m.obstacles().size(), 1u) << "label-only: hazard remains when eviction off";
  EXPECT_TRUE(m.obstacleCameraObservedEmpty(0)) << "still flagged";
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
}

// A hazard the camera never looks at (out of FOV) is NEVER evicted — absence
// outside the FOV is not evidence of absence (the camera-blind-region guarantee,
// the eviction analogue of CameraOutOfFovNeverFlags).
TEST(LiveOccupancyModel, CameraEvictionSparesBlindRegion) {
  LiveOccupancyParams p = testParams();
  p.camera_empty_sustain_s = 1.0;
  p.evict_camera_empty = true;
  LiveOccupancyModel m(anchorDatum(), p);
  auto returns = pierReturns();
  for (int scan = 0; scan < 10; ++scan) m.observe(feed(returns, scan));
  for (double t = 10.0; t <= 12.0; t += 0.5) {
    auto f = emptyPierFrame(t);
    f.fov_center_rad = 2.5;  // looking away — the pier is out of FOV
    m.observeCamera(f);
  }
  m.observe(feed(returns, 13));
  EXPECT_EQ(m.obstacles().size(), 1u) << "camera-blind hazard must never evict";
  EXPECT_GT(m.birthSuppression(Eigen::Vector2d(62.5, 0.0)), 0.0);
}

// A six-cell pier starting at (x0, y) along +x (25 m cells).
std::vector<std::pair<Eigen::Vector2d, double>> pierAt(double x0, double y) {
  std::vector<std::pair<Eigen::Vector2d, double>> r;
  for (int k = 0; k < 6; ++k) r.emplace_back(Eigen::Vector2d(x0 + 25.0 * k, y), 1.0);
  return r;
}

// A live camera frame aimed at `bearing` (rad, ENU math), narrow FOV, with a
// live detection half a radian away (so the frame is live but the aimed cell is
// observed empty).
LiveOccupancyModel::CameraObservation emptyFrameToward(double t, double bearing) {
  LiveOccupancyModel::CameraObservation f;
  f.t_unix = t;
  f.sensor_enu = Eigen::Vector2d(0.0, 0.0);
  f.fov_center_rad = bearing;
  f.fov_half_width_rad = 0.3;
  f.match_tolerance_rad = 0.05;
  f.detection_bearings_rad = {bearing + 0.5};  // live, away from the aimed cell
  return f;
}

// ── Increment (ii) PROMOTION GATE (synthetic scenario) ───────────────────────
//
// The circularity rule: the mechanism is DEMONSTRATED on real philos (the ferry/
// loiterer replay), but PROMOTION gates on synthetic truth. This mini-world holds
// three frozen structures at once — a departed vessel (uncharted, camera sees its
// bearing empty), a chart-confirmed structure (camera also sees empty), and a
// camera-blind structure (out of the FOV) — and toggles eviction. Ground truth:
// ONLY the departed vessel should lose its pin. The two held structures'
// suppression must be BYTE-IDENTICAL with eviction on vs off ("tracks_on_keep
// flat"); the departed vessel's suppression must lift to exactly 0 (departed-
// evicts + conservation-safe). Co-presence also proves no cross-talk: evicting
// one structure does not disturb the others.
TEST(LiveOccupancyModel, EvictionSceneDepartedEvictsHeldStructuresStayFlat) {
  auto runScene = [](bool evict) {
    LiveOccupancyParams p = testParams();  // extent_min 4, bar 0.5, non-adaptive
    p.camera_empty_sustain_s = 1.0;
    p.chart_corroboration_radius_m = 50.0;
    p.evict_camera_empty = evict;
    auto m = std::make_unique<LiveOccupancyModel>(anchorDatum(), p);
    m->setChartedStructure({chartAt({2075.0, 12.5})});  // on held_charted centroid

    auto all = pierAt(12.5, 0.0);                        // departed   (bearing ~0, in FOV)
    for (const auto& r : pierAt(2012.5, 0.0)) all.push_back(r);   // held_charted (in FOV)
    for (const auto& r : pierAt(12.5, 300.0)) all.push_back(r);   // held_blind  (65–88°, out of FOV)
    for (int s = 0; s < 10; ++s) m->observe(feed(all, s));

    // Returns cease; a distant sweep covers none of them → all three FREEZE
    // (coverage-aware decay holds them — the corroboration wall).
    const auto away = disc(Eigen::Vector2d(10000.0, 10000.0), 500.0);
    for (int s = 10; s < 15; ++s)
      m->observe(feedCov({{Eigen::Vector2d(10000.0, 10000.0), 1.0}}, s, away));
    // The camera observes the bearing-0 corridor empty (departed + held_charted
    // are in FOV; held_blind is not). Matures the per-cell streaks.
    for (double t = 15.0; t <= 17.0; t += 0.5) m->observeCamera(emptyPierFrame(t));
    m->observe(feedCov({{Eigen::Vector2d(10000.0, 10000.0), 1.0}}, 17.5, away));
    return m;
  };

  const Eigen::Vector2d q_departed(62.5, 12.5);    // inside departed
  const Eigen::Vector2d q_charted(2075.0, 12.5);   // inside held_charted
  const Eigen::Vector2d q_blind(62.5, 312.5);      // inside held_blind

  auto off = runScene(false);
  ASSERT_EQ(off->obstacles().size(), 3u) << "all three frozen structures held (baseline)";
  ASSERT_GT(off->birthSuppression(q_departed), 0.0);
  ASSERT_GT(off->birthSuppression(q_charted), 0.0);
  ASSERT_GT(off->birthSuppression(q_blind), 0.0);

  auto on = runScene(true);
  EXPECT_EQ(on->obstacles().size(), 2u) << "only the departed vessel is evicted";
  // departed-evicts + conservation: the pin lifts to exactly 0 (a birth is free).
  EXPECT_DOUBLE_EQ(on->birthSuppression(q_departed), 0.0);
  // tracks_on_keep flat: the held structures are byte-identical to eviction-off.
  EXPECT_DOUBLE_EQ(on->birthSuppression(q_charted), off->birthSuppression(q_charted))
      << "chart-confirmed structure held identically (evidence precedence)";
  EXPECT_DOUBLE_EQ(on->birthSuppression(q_blind), off->birthSuppression(q_blind))
      << "camera-blind structure held identically (absence outside FOV isn't absence)";
}

// The FLICKER regression (the increment-i finding, now a deterministic gate): a
// frozen departed pin blinks in and out of the structure set as the clutter-
// ADAPTIVE bar moves with the live-cell population. Eviction must fire on
// re-entry from evidence matured while the cell was off-stage — the exact
// pathology that made the loiterer under-flag under label-only increment (i).
//
// Construction: the pin (frozen) and two companion cells all sit at ~0.9, so the
// median (hence the adaptive bar) starts ABOVE the pin → the pin is OUT of the
// structure set (a busy scene suppresses it). The camera matures its observed-
// empty streak while it is off-stage. Then the companions decay (the scene
// quiets) → the bar falls below the pin → the pin RE-ENTERS structure — and
// eviction spends it on that re-entry, so the phantom is killed before it can
// suppress a single birth.
TEST(LiveOccupancyModel, CameraEvictionSurvivesAdaptiveBarFlicker) {
  LiveOccupancyParams p;
  p.cell_size_m = 25.0;
  p.ewma_alpha = 0.3;
  p.persistence_bar = 0.1;        // low floor; the adaptive bar dominates
  p.extended_cells_min = 1;       // detector: a single cell classifies
  p.suppression_max = 0.9;
  p.suppression_radius_m = 25.0;
  p.clutter_adaptive = true;      // bar = max(0.1, 1.5×median) — moves with the pop
  p.clutter_reject_factor = 1.5;
  p.camera_empty_sustain_s = 1.0;
  p.evict_camera_empty = true;
  p.camera_empty_recency_window_s = 100.0;  // isolate the flicker variable
  LiveOccupancyModel m(anchorDatum(), p);

  const Eigen::Vector2d departed(1012.5, 1012.5);  // bearing 0.785 (45°), in FOV
  const double brg = std::atan2(departed.y(), departed.x());
  // Companion high cells OUT of the camera FOV (negative quadrant) whose decay
  // lowers the median/bar. Low cells (fed weight 0.3) keep a floor population.
  const Eigen::Vector2d compA(-1012.5, -1012.5), compB(-1037.5, -1012.5);
  const Eigen::Vector2d lowA(-1062.5, -1012.5), lowB(-1087.5, -1012.5);
  // Coverage over the negative-quadrant cells only → the departed pin is out of
  // coverage every scan (frozen: its returns have ceased).
  const auto negCover = disc(Eigen::Vector2d(-1050.0, -1012.5), 300.0);

  // 1) Build the pin AND the companions to ~0.9. With three ~0.9 cells the median
  //    is 0.9 → bar 1.35 → NOTHING classifies (the pin is suppressed out of a busy
  //    scene). departed is merely present in persistence, frozen.
  double t = 0.0;
  for (int s = 0; s < 10; ++s, t += 1.0)
    m.observe(feed({{departed, 1.0}, {compA, 1.0}, {compB, 1.0}}, t));
  ASSERT_TRUE(m.obstacles().empty()) << "busy scene: high bar suppresses the pin (OUT)";

  // 2) Mature the camera streak on the pin while it is OFF-STAGE (not structure).
  m.observeCamera(emptyFrameToward(10.0, brg));
  m.observeCamera(emptyFrameToward(11.0, brg));  // span 1.0 ≥ sustain → matured

  // 3) The scene quiets: the companions decay (in coverage, no longer fed) while
  //    the pin stays frozen out of coverage. As the median falls, the bar drops
  //    below the pin → it RE-ENTERS structure — and eviction spends it on that
  //    re-entry, so it never emits a hazard.
  for (int s = 0; s < 12; ++s, t += 1.0) {
    m.observe(feedCov({{lowA, 0.3}, {lowB, 0.3}}, t, negCover));
    m.observeCamera(emptyFrameToward(t, brg));
  }

  // 4) Prove the pin was EVICTED (persistence spent), not merely below the bar:
  //    drive the bar to its floor with the pin still frozen out of coverage. A
  //    pin still holding 0.9 would re-emit as a hazard; an evicted one stays gone.
  for (int s = 0; s < 3; ++s, t += 1.0)
    m.observe(feedCov({{lowA, 0.3}, {lowB, 0.3}}, t, negCover));
  EXPECT_DOUBLE_EQ(m.birthSuppression(departed), 0.0)
      << "eviction spent the pin — it does not re-emit when the bar falls (no blink)";
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
