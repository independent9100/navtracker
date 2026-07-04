#pragma once

#include <map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "core/types/StaticObstacle.hpp"
#include "ports/ILiveOccupancyFeed.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

/**
 * Tuning for the live occupancy/structure layer. Defaults chosen so a single
 * anchored boat's watch circle never qualifies as structure (extent gate) while
 * a pier/breakwater does. See docs/algorithms/live-static-occupancy.md.
 */
struct LiveOccupancyParams {
  double cell_size_m = 25.0;      // metric grid resolution
  double ewma_alpha = 0.3;        // persistence EWMA rate (per fed scan)
  double persistence_bar = 0.5;   // a cell is "persistent" at/above this EWMA
  int extended_cells_min = 4;     // connected persistent cells to be "structure"
  double suppression_max = 0.9;   // cap < tracker hard gate (0.95): soft-only
  double suppression_radius_m = 25.0;  // ramp distance beyond a structure cell
  double erase_floor = 1e-3;      // drop cells whose EWMA decays below this
  // Clutter-adaptive persistence bar (detector mode). Uniform clutter reaches a
  // per-cell persistence set by its density; structure exceeds its OWN clutter
  // background even where absolute persistence does not separate them (philos
  // structure 0.30 ≫ its clutter; dense clutter 0.28 ≯ its 0.28). The background
  // is ESTIMATED from the feed — the median persistence of live cells, since the
  // clutter-map feed is dominated by clutter, not structure — so no external λ_C
  // is needed (works on real data). Effective bar = max(persistence_bar,
  // clutter_reject_factor × median). false ⇒ absolute bar only (1b-i behaviour).
  bool clutter_adaptive = false;
  double clutter_reject_factor = 1.5;
  // Chart corroboration (increment 6): an emitted live-structure hazard whose
  // centroid lies within this radius of a charted structure point is CONFIRMED
  // as structure (label only — suppression is unchanged; the label feeds
  // operator confidence and the increment-8 eviction-by-evidence policy: an
  // uncorroborated pin is the eviction candidate, a corroborated one is
  // retained). ~100 m ≈ one coarse cell, so a hazard whose ~100 m-cell centroid
  // sits within a cell of charted structure counts as coincident (the centroid
  // can be up to a half-diagonal off the true structure). Only active when
  // setChartedStructure() has been called with a non-empty set.
  double chart_corroboration_radius_m = 100.0;
  // Camera corroboration (increment 6, camera): an occupied cell is flagged
  // "camera-observed-empty" once it has been continuously observed-empty by a
  // live camera frame — IN the frame's field of view, no detection within the
  // frame's match tolerance of the cell's bearing — for at least this long. The
  // sustain guards against a single dropped detection; absence is evidence only
  // when the camera was actually looking (the coverage-aware-decay principle in
  // the camera modality). A flagged, chart-UNconfirmed hazard is the increment-8
  // eviction candidate (a departed vessel). Only active when observeCamera() is
  // fed; label only (suppression/hazards unchanged).
  double camera_empty_sustain_s = 2.0;
  // Camera EVICTION as behaviour (increment ii). When true, a structure cell
  // that is camera-observed-empty (its per-cell streak matured AND still recent,
  // below) and is NOT chart-confirmed is EVICTED: its accumulated persistence is
  // SPENT (erased), not merely its hazard dropped. This matters because coverage-
  // aware decay FREEZES the persistence of an unobserved cell (a departed
  // vessel's radar returns cease while the cell is outside the swept sector — the
  // 6c corroboration wall); dropping the hazard alone would let that frozen
  // persistence re-emit it next scan (a blinker), so eviction erases it and the
  // cell starts over from fresh returns. Evidence is keyed by CELL and accrues
  // even while the cell is NOT emitted, so eviction fires the instant a flickering
  // cell re-enters the structure set. Evidence precedence: a chart-confirmed
  // component is HELD regardless of camera. Conservation-safe by construction —
  // suppression is re-derived from the post-eviction persistence, so lifting it
  // can only free a birth, never orphan one. Default false ⇒ increment-(i) label-
  // only behaviour, bit-identical.
  bool evict_camera_empty = false;
  // A camera-empty streak evicts only if its most recent observed-empty frame is
  // within this window of the current scan time. A streak from long ago (the
  // camera stopped looking, or a real vessel has since re-pinned the cell) is
  // STALE and must not evict now — the recency guard that makes decoupled per-cell
  // evidence safe. Only consulted when evict_camera_empty is true.
  double camera_empty_recency_window_s = 5.0;
  // Corroboration suppression VETO (increment 6 / R9 item 1b): a birth is NEVER
  // suppressed within this radius of a RECENT AIS/cooperative vessel fix — an
  // AIS/cooperative-known platform must remain track-eligible (the strongest
  // vessel discriminator under the ADR-0002 amendment: "where we CAN tell, a
  // vessel must track, never be suppressed"). ~100 m ≈ one coarse cell, covering
  // the vessel + fix uncertainty. Only active when observeVesselFix() is fed; the
  // veto only REDUCES suppression to 0, so the conservation invariant is preserved.
  double veto_radius_m = 100.0;
  // A vessel fix vetoes only while it is within this window of the current scan
  // time; a stale fix (the vessel's feed went quiet) is pruned, so an anchored
  // AIS-silent vessel may fall back to the accepted static-hazard degraded mode
  // until its next fix re-asserts the veto. Matches typical AIS/cooperative cadence.
  double veto_window_s = 60.0;
};

/**
 * A datum-stable live occupancy grid that learns persistent + spatially
 * extended structure (piers, breakwaters, shoreline) from the PMBM per-scan
 * (position, 1 − r) feed and suppresses vessel births there — and ONLY there.
 * Compact persistent regions (an anchored boat's watch circle) and transient
 * uniform clutter are never suppressed. See the Stage 1b-i design.
 *
 * Two faces on one object:
 *   * IStaticObstacleModel  — birthSuppression()/obstacles(): the birth-channel
 *     suppression + hazard output, wired via PmbmTracker::setStaticObstacleModel.
 *   * ILiveOccupancyFeed    — observe(): the per-scan accumulation feed, wired
 *     via PmbmTracker::setLiveOccupancyFeed. Deliberately NOT a detection model
 *     → the learned map never touches λ_C / p_D (Stage 1b design requirement).
 *
 * Frame: the grid is anchored to a fixed datum (construction). Incoming ENU
 * (about the tracker's *current* datum) is transformed to the anchor frame at
 * accumulate/query time; IDatumChangeSink updates the current-datum transform on
 * recenter, so persistence stays attached to geography (mirrors CoastlineModel /
 * StaticObstacleModel). A single fixed datum (the bench) ⇒ identity transform.
 *
 * Pure: no I/O, no wall-clock, no RNG. Deterministic (ordered maps + key-sorted
 * connected-component labeling).
 */
class LiveOccupancyModel : public IStaticObstacleModel,
                           public ILiveOccupancyFeed,
                           public IDatumChangeSink {
 public:
  /** Anchor the grid to a fixed datum; `current_` tracks the tracker's datum. */
  explicit LiveOccupancyModel(geo::Datum anchor, LiveOccupancyParams params = {})
      : anchor_(anchor), current_(anchor), params_(params) {}

  /** ILiveOccupancyFeed: accumulate one scan's clutter-labeled returns. */
  void observe(const std::vector<ISensorDetectionModel::ScanObservation>&
                   by_sensor) override;

  /** IStaticObstacleModel: soft birth suppression, structure-only. */
  double birthSuppression(const Eigen::Vector2d& enu_xy) const override;
  /** The structure hazards learned so far (index-aligned with corroboration). */
  const std::vector<StaticObstacle>& obstacles() const override {
    return obstacles_;
  }

  /** IDatumChangeSink: re-anchor the ENU↔anchor transform (grid data untouched). */
  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    current_ = new_datum;
  }

  /**
   * Chart corroboration input (increment 6): the charted STRUCTURE point cloud
   * (piers/wharves densified to points offline — see the philos wiring). Cached
   * in the fixed anchor frame at set time; charts are static, so no datum-sink
   * handling is needed (anchor_ never moves). Empty ⇒ corroboration inert, all
   * emitted hazards report uncorroborated (bit-identical to no-chart behaviour).
   */
  void setChartedStructure(const std::vector<StaticObstacle>& charted);

  // One LIVE camera frame's evidence, grouped by timestamp (a frame present in
  // the feed produced >= 1 detection, so it IS live — the frame-live guard of
  // the absence-asymmetry rule). Bearings are absolute ENU "math" bearings
  // (atan2(dN, dE)), the same convention CameraBearingCsvReader emits;
  // fov_center_rad is the camera optical axis in that convention and
  // fov_half_width_rad its half-HFOV. sensor_enu is own-ship in the tracker's
  // current datum (re-expressed to the anchor frame internally).
  struct CameraObservation {
    double t_unix{0.0};
    Eigen::Vector2d sensor_enu{Eigen::Vector2d::Zero()};
    std::vector<double> detection_bearings_rad;  // absolute ENU math bearings
    double fov_center_rad{0.0};
    double fov_half_width_rad{0.0};
    double match_tolerance_rad{0.0};
  };

  /**
   * Camera corroboration feed (increment 6): advance each occupied cell's
   * observed-empty streak from one live camera frame. A cell IN the FOV with no
   * detection within tolerance of its bearing extends its streak; a matching
   * detection resets it (something is there); a cell OUT of the FOV is left
   * untouched (not observed — never evidence of absence). Consumed through this
   * dedicated API, NOT the clutter feed (the occupancy/coverage path is
   * untouched; bearing-only camera measurements cannot pollute it). Inert until
   * first called.
   */
  void observeCamera(const CameraObservation& frame);

  /**
   * One AIS/cooperative vessel fix (suppression veto input, R9 item 1b). Position
   * is in the tracker's CURRENT-datum ENU (re-expressed to the anchor frame
   * internally, like a camera sensor position); t_unix is its timestamp for the
   * recency window. The WIRING selects which measurements are positional anchors
   * (SensorKind::Ais / Cooperative — `isNonScanningSource`) and feeds only those,
   * so the model stays sensor-kind agnostic. Inert until first fed.
   */
  struct VesselFix {
    double t_unix{0.0};
    Eigen::Vector2d position_enu{Eigen::Vector2d::Zero()};
  };
  void observeVesselFix(const VesselFix& fix);

  /** Number of currently-active (unpruned, recent) vessel-fix vetoes. */
  std::size_t vesselFixCount() const { return vessel_fixes_.size(); }

  /**
   * Introspection (tests / diagnostics): the most structure components ever
   * classified simultaneously, and the highest per-cell persistence ever
   * reached, across the model's lifetime. Peak (not current) because a fed
   * structure that later decays would otherwise be invisible at end-of-run.
   */
  int peakStructureCount() const { return peak_structure_count_; }
  double peakPersistence() const { return peak_persistence_; }
  /**
   * Number of birthSuppression() queries that landed in a suppressed region
   * (returned > 0). Zero ⇒ the birth path never queried classified structure.
   */
  long suppressionHits() const { return suppression_hits_; }

  /**
   * Chart-corroboration introspection (index-aligned with obstacles()). An
   * emitted live hazard is corroborated when a charted structure point lies
   * within params.chart_corroboration_radius_m of its centroid. Uncorroborated
   * hazards are the eviction candidates (increment 8) — a departed vessel that
   * pinned a cell (no chart, no AIS, no camera) reports false here.
   */
  bool obstacleCorroborated(std::size_t i) const {
    return i < obstacle_corroborated_.size() && obstacle_corroborated_[i];
  }
  int chartCorroboratedCount() const {
    int n = 0;
    for (bool c : obstacle_corroborated_)
      if (c) ++n;
    return n;
  }

  /**
   * Camera-observed-empty introspection (index-aligned with obstacles()). True
   * once the hazard's centroid cell has been continuously camera-observed-empty
   * for >= camera_empty_sustain_s. A flagged, chart-UNconfirmed hazard is the
   * departed-vessel eviction candidate (increment 8).
   */
  bool obstacleCameraObservedEmpty(std::size_t i) const {
    return i < obstacle_camera_empty_.size() && obstacle_camera_empty_[i];
  }
  int cameraObservedEmptyCount() const {
    int n = 0;
    for (bool c : obstacle_camera_empty_)
      if (c) ++n;
    return n;
  }

 private:
  using Cell = std::pair<int, int>;

  Eigen::Vector2d toAnchorEnu(const Eigen::Vector2d& enu_current) const;
  Cell cellOf(const Eigen::Vector2d& anchor_enu) const;
  Eigen::Vector2d cellCenter(const Cell& c) const;
  // Persistent + extended structure components (effective bar + 4-connected flood
  // fill), each a key-sorted cell list. Shared by recomputeStructure() (emit) and
  // evictCameraRefutedCells() (the eviction pre-pass), so the bar/connectivity
  // definition lives in exactly one place.
  std::vector<std::vector<Cell>> structureComponents() const;
  void recomputeStructure();
  // Increment-ii eviction pre-pass: spend (erase) the persistence of structure
  // cells the camera has refuted (matured + recent observed-empty streak) in
  // chart-UNconfirmed components, so recomputeStructure() below cannot re-emit
  // them. `now_s` is the current scan time for the recency test.
  void evictCameraRefutedCells(double now_s);

  geo::Datum anchor_;   // fixed grid frame
  geo::Datum current_;  // tracker's current datum (== anchor_ until a recenter)
  LiveOccupancyParams params_;

  std::map<Cell, double> persistence_;      // EWMA persistence per touched cell
  // Emitted hazards + the geometry birthSuppression() is DERIVED from, so
  // suppression > 0 ⇒ inside some hazard's keep-clear ring (the ADR-0002
  // conservation invariant, structural). All three vectors are index-aligned.
  std::vector<StaticObstacle> obstacles_;        // one per structure component
  std::vector<Eigen::Vector2d> obstacle_center_; // anchor-ENU centroid per hazard
  std::vector<double> obstacle_conf_;            // suppression confidence per hazard
  std::vector<bool> obstacle_corroborated_;      // chart-confirmed? (index-aligned)
  std::vector<Eigen::Vector2d> charted_enu_;     // charted structure pts (anchor ENU)
  std::vector<bool> obstacle_camera_empty_;      // camera-observed-empty? (index-aligned)
  // Per-cell camera observed-empty streak: (start, last) unix times of the
  // current unbroken streak of in-FOV, no-detection observations. Absent ⇒ not
  // currently observed-empty (just seen, or never observed). Out-of-FOV frames
  // are neutral (neither extend `last` nor reset), so an unobserved gap cannot
  // inflate the sustained span; a matching detection erases the entry. Keyed in
  // the anchor frame like the occupancy grid.
  std::map<Cell, std::pair<double, double>> camera_empty_streak_;
  // Recent AIS/cooperative vessel fixes (position in the anchor frame, + t_unix),
  // each vetoing birth suppression within veto_radius_m. Pruned to veto_window_s
  // in observe(). Empty ⇒ no veto (bit-identical to no-fix behaviour).
  std::vector<VesselFix> vessel_fixes_;
  int peak_structure_count_ = 0;
  double peak_persistence_ = 0.0;
  mutable long suppression_hits_ = 0;
};

}  // namespace navtracker
