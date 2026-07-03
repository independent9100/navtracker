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

// Tuning for the live occupancy/structure layer. Defaults chosen so a single
// anchored boat's watch circle never qualifies as structure (extent gate) while
// a pier/breakwater does. See docs/algorithms/live-static-occupancy.md.
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
};

// A datum-stable live occupancy grid that learns persistent + spatially
// extended structure (piers, breakwaters, shoreline) from the PMBM per-scan
// (position, 1 − r) feed and suppresses vessel births there — and ONLY there.
// Compact persistent regions (an anchored boat's watch circle) and transient
// uniform clutter are never suppressed. See the Stage 1b-i design.
//
// Two faces on one object:
//   * IStaticObstacleModel  — birthSuppression()/obstacles(): the birth-channel
//     suppression + hazard output, wired via PmbmTracker::setStaticObstacleModel.
//   * ILiveOccupancyFeed    — observe(): the per-scan accumulation feed, wired
//     via PmbmTracker::setLiveOccupancyFeed. Deliberately NOT a detection model
//     → the learned map never touches λ_C / p_D (Stage 1b design requirement).
//
// Frame: the grid is anchored to a fixed datum (construction). Incoming ENU
// (about the tracker's *current* datum) is transformed to the anchor frame at
// accumulate/query time; IDatumChangeSink updates the current-datum transform on
// recenter, so persistence stays attached to geography (mirrors CoastlineModel /
// StaticObstacleModel). A single fixed datum (the bench) ⇒ identity transform.
//
// Pure: no I/O, no wall-clock, no RNG. Deterministic (ordered maps + key-sorted
// connected-component labeling).
class LiveOccupancyModel : public IStaticObstacleModel,
                           public ILiveOccupancyFeed,
                           public IDatumChangeSink {
 public:
  explicit LiveOccupancyModel(geo::Datum anchor, LiveOccupancyParams params = {})
      : anchor_(anchor), current_(anchor), params_(params) {}

  // ILiveOccupancyFeed: accumulate one scan's clutter-labeled returns.
  void observe(const std::vector<ISensorDetectionModel::ScanObservation>&
                   by_sensor) override;

  // IStaticObstacleModel: soft birth suppression, structure-only.
  double birthSuppression(const Eigen::Vector2d& enu_xy) const override;
  const std::vector<StaticObstacle>& obstacles() const override {
    return obstacles_;
  }

  // IDatumChangeSink: re-anchor the ENU↔anchor transform (grid data untouched).
  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    current_ = new_datum;
  }

  // Introspection (tests / diagnostics): the most structure components ever
  // classified simultaneously, and the highest per-cell persistence ever
  // reached, across the model's lifetime. Peak (not current) because a fed
  // structure that later decays would otherwise be invisible at end-of-run.
  int peakStructureCount() const { return peak_structure_count_; }
  double peakPersistence() const { return peak_persistence_; }
  // Number of birthSuppression() queries that landed in a suppressed region
  // (returned > 0). Zero ⇒ the birth path never queried classified structure.
  long suppressionHits() const { return suppression_hits_; }

 private:
  using Cell = std::pair<int, int>;

  Eigen::Vector2d toAnchorEnu(const Eigen::Vector2d& enu_current) const;
  Cell cellOf(const Eigen::Vector2d& anchor_enu) const;
  Eigen::Vector2d cellCenter(const Cell& c) const;
  void recomputeStructure();

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
  int peak_structure_count_ = 0;
  double peak_persistence_ = 0.0;
  mutable long suppression_hits_ = 0;
};

}  // namespace navtracker
