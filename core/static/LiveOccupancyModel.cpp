#include "core/static/LiveOccupancyModel.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace navtracker {

Eigen::Vector2d LiveOccupancyModel::toAnchorEnu(
    const Eigen::Vector2d& enu_current) const {
  // Fast path: no recenter has happened (the bench's single-datum case), so the
  // incoming ENU is already in the anchor frame — exact, no round-trip drift.
  const geo::Geodetic& a = anchor_.origin();
  const geo::Geodetic& c = current_.origin();
  if (a.lat_deg == c.lat_deg && a.lon_deg == c.lon_deg && a.alt_m == c.alt_m)
    return enu_current;
  const geo::Geodetic g =
      current_.toGeodetic(Eigen::Vector3d(enu_current.x(), enu_current.y(), 0.0));
  const Eigen::Vector3d e = anchor_.toEnu(g);
  return Eigen::Vector2d(e.x(), e.y());
}

LiveOccupancyModel::Cell LiveOccupancyModel::cellOf(
    const Eigen::Vector2d& anchor_enu) const {
  return {static_cast<int>(std::floor(anchor_enu.x() / params_.cell_size_m)),
          static_cast<int>(std::floor(anchor_enu.y() / params_.cell_size_m))};
}

Eigen::Vector2d LiveOccupancyModel::cellCenter(const Cell& c) const {
  return Eigen::Vector2d((c.first + 0.5) * params_.cell_size_m,
                         (c.second + 0.5) * params_.cell_size_m);
}

void LiveOccupancyModel::observe(
    const std::vector<ISensorDetectionModel::ScanObservation>& by_sensor) {
  const double a = params_.ewma_alpha;

  // 1) COVERAGE-AWARE EWMA decay: a cell forgets only when it was OBSERVABLE
  //    this scan (inside some bundle's coverage footprint) and returned empty.
  //    Absence of returns where no sensor looked is not evidence of vacancy —
  //    that is exactly what separates a departed vessel (returns cease while the
  //    cell is still in coverage) from a cell that merely left coverage. If no
  //    bundle carries a valid footprint, full coverage is assumed → every cell
  //    decays (the legacy behaviour, bit-identical for synthetic/unwired feeds).
  //    Footprints arrive in the tracker's current-datum ENU; re-express the
  //    sensor position in the grid's anchor frame (sector angles are datum-
  //    relative — exact for a fixed datum, negligible inter-datum rotation).
  std::vector<ISensorDetectionModel::CoverageSector> cover;
  for (const auto& obs : by_sensor) {
    if (!obs.coverage.valid) continue;
    ISensorDetectionModel::CoverageSector cs = obs.coverage;
    cs.sensor_enu = toAnchorEnu(cs.sensor_enu);
    cover.push_back(cs);
  }
  const bool have_cover = !cover.empty();
  for (auto& kv : persistence_) {
    bool observable = !have_cover;  // no footprint ⇒ full coverage
    if (have_cover) {
      const Eigen::Vector2d center = cellCenter(kv.first);  // anchor ENU
      for (const auto& cs : cover)
        if (cs.covers(center)) {
          observable = true;
          break;
        }
    }
    if (observable) kv.second *= (1.0 - a);
  }

  // 2) Largest clutter weight touching each cell this scan (order-independent).
  std::map<Cell, double> touched;
  for (const auto& obs : by_sensor) {
    const auto& pos = obs.clutter_positions;
    const auto& w = obs.clutter_position_weights;
    for (std::size_t i = 0; i < pos.size(); ++i) {
      // Contract: an empty weight vector means weight 1.0 per return.
      const double weight = w.empty() ? 1.0 : w[i];
      if (weight <= 0.0) continue;
      const Cell c = cellOf(toAnchorEnu(pos[i]));
      auto it = touched.find(c);
      if (it == touched.end())
        touched.emplace(c, weight);
      else
        it->second = std::max(it->second, weight);
    }
  }

  // 3) EWMA update for touched cells: p ← (1−α)p + α·w (decay already applied).
  for (const auto& kv : touched) persistence_[kv.first] += a * kv.second;

  // 4) Drop numerically-dead cells to bound memory (deterministic).
  for (auto it = persistence_.begin(); it != persistence_.end();) {
    if (it->second < params_.erase_floor)
      it = persistence_.erase(it);
    else
      ++it;
  }

  // 5) Re-classify structure (persistent AND extended) → suppression + hazards.
  recomputeStructure();

  // Introspection peaks.
  peak_structure_count_ =
      std::max(peak_structure_count_, static_cast<int>(obstacles_.size()));
  for (const auto& kv : persistence_)
    peak_persistence_ = std::max(peak_persistence_, kv.second);
}

void LiveOccupancyModel::recomputeStructure() {
  obstacles_.clear();
  obstacle_center_.clear();
  obstacle_conf_.clear();

  // Effective persistence bar. In detector mode, raise it above the estimated
  // uniform-clutter background (median live-cell persistence — the clutter-map
  // feed is clutter-dominated) so dense clutter is rejected relative to its own
  // density (no death-spiral) while sparse structure, which sits far above its
  // own clutter, still classifies.
  double bar = params_.persistence_bar;
  if (params_.clutter_adaptive && !persistence_.empty()) {
    std::vector<double> vals;
    vals.reserve(persistence_.size());
    for (const auto& kv : persistence_) vals.push_back(kv.second);
    std::sort(vals.begin(), vals.end());
    const double median = vals[vals.size() / 2];
    bar = std::max(bar, params_.clutter_reject_factor * median);
  }

  // Persistent cells (ordered → deterministic component labeling).
  std::map<Cell, double> persistent;
  for (const auto& kv : persistence_)
    if (kv.second >= bar) persistent.emplace(kv);

  std::map<Cell, bool> visited;
  for (const auto& seed : persistent) {
    if (visited[seed.first]) continue;
    // 4-connected flood fill from this seed.
    std::vector<Cell> component;
    std::queue<Cell> q;
    q.push(seed.first);
    visited[seed.first] = true;
    while (!q.empty()) {
      const Cell cur = q.front();
      q.pop();
      component.push_back(cur);
      const Cell nbrs[4] = {{cur.first + 1, cur.second},
                            {cur.first - 1, cur.second},
                            {cur.first, cur.second + 1},
                            {cur.first, cur.second - 1}};
      for (const Cell& n : nbrs) {
        if (persistent.count(n) && !visited[n]) {
          visited[n] = true;
          q.push(n);
        }
      }
    }

    if (static_cast<int>(component.size()) < params_.extended_cells_min)
      continue;  // compact → a boat / small feature → never suppressed

    // Structure component → ONE synthesized live (uncharted) hazard whose
    // footprint COVERS every cell in the component, so the birth suppression
    // derived from it (below) can never exceed the emitted hazard's keep-clear
    // ring (the conservation invariant). conf = the component's peak
    // persistence (capped), used as the suppression strength.
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    double conf = 0.0;
    for (const Cell& c : component) {
      centroid += cellCenter(c);
      conf = std::max(conf, std::min(1.0, persistent.at(c)));
    }
    centroid /= static_cast<double>(component.size());

    // Footprint = the max cell-centre reach from the centroid + a half-cell, so
    // the hard core encloses all persistent cells; keep-clear adds the ramp.
    double reach = 0.0;
    for (const Cell& c : component)
      reach = std::max(reach, (cellCenter(c) - centroid).norm());
    const double footprint = reach + 0.5 * params_.cell_size_m;

    StaticObstacle o;
    o.position = anchor_.toGeodetic(Eigen::Vector3d(centroid.x(), centroid.y(),
                                                    0.0));
    o.footprint_radius_m = footprint;
    o.keep_clear_radius_m = footprint + params_.suppression_radius_m;
    o.category = ObstacleCategory::Obstruction;
    o.water_level = WaterLevel::AlwaysAboveWater;
    o.source_id = "live_occupancy";
    obstacles_.push_back(std::move(o));
    obstacle_center_.push_back(centroid);
    obstacle_conf_.push_back(conf);
  }
}

double LiveOccupancyModel::birthSuppression(
    const Eigen::Vector2d& enu_xy) const {
  if (obstacles_.empty()) return 0.0;
  const Eigen::Vector2d q = toAnchorEnu(enu_xy);
  // Derived ENTIRELY from the emitted hazards: ramp 1.0 inside footprint,
  // linear to 0 at keep-clear. best > 0 ⇒ q is inside some hazard's keep-clear
  // ring ⇒ that hazard is emitted (the conservation invariant, by construction).
  double best = 0.0;
  for (std::size_t i = 0; i < obstacles_.size(); ++i) {
    const double d = (obstacle_center_[i] - q).norm();
    const double fr = obstacles_[i].footprint_radius_m;
    const double kc = obstacles_[i].keep_clear_radius_m;
    double ramp;
    if (d <= fr)
      ramp = 1.0;                         // inside the structure footprint
    else if (d <= kc && kc > fr)
      ramp = (kc - d) / (kc - fr);        // soft ramp to the keep-clear edge
    else
      continue;                           // outside this hazard's influence
    best = std::max(best, params_.suppression_max * obstacle_conf_[i] * ramp);
  }
  if (best > 0.0) ++suppression_hits_;
  return best;
}

}  // namespace navtracker
