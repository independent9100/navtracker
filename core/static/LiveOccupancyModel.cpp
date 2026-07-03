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

  // 1) EWMA decay of every known cell toward 0 (untouched cells forget).
  for (auto& kv : persistence_) kv.second *= (1.0 - a);

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
  structure_conf_.clear();
  obstacles_.clear();

  // Persistent cells (ordered → deterministic component labeling).
  std::map<Cell, double> persistent;
  for (const auto& kv : persistence_)
    if (kv.second >= params_.persistence_bar) persistent.emplace(kv);

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

    // Structure component: record per-cell suppression confidence and one
    // synthesized live (uncharted) hazard at the component centroid.
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    int imin = component.front().first, imax = imin;
    int jmin = component.front().second, jmax = jmin;
    for (const Cell& c : component) {
      structure_conf_[c] = std::min(1.0, persistent.at(c));
      centroid += cellCenter(c);
      imin = std::min(imin, c.first);
      imax = std::max(imax, c.first);
      jmin = std::min(jmin, c.second);
      jmax = std::max(jmax, c.second);
    }
    centroid /= static_cast<double>(component.size());

    const double span_x = (imax - imin + 1) * params_.cell_size_m;
    const double span_y = (jmax - jmin + 1) * params_.cell_size_m;
    StaticObstacle o;
    o.position = anchor_.toGeodetic(Eigen::Vector3d(centroid.x(), centroid.y(),
                                                    0.0));
    o.footprint_radius_m = 0.5 * std::max(span_x, span_y);
    o.keep_clear_radius_m = o.footprint_radius_m + params_.suppression_radius_m;
    o.category = ObstacleCategory::Obstruction;
    o.water_level = WaterLevel::AlwaysAboveWater;
    o.source_id = "live_occupancy";
    obstacles_.push_back(std::move(o));
  }
}

double LiveOccupancyModel::birthSuppression(
    const Eigen::Vector2d& enu_xy) const {
  if (structure_conf_.empty()) return 0.0;
  const Eigen::Vector2d q = toAnchorEnu(enu_xy);
  const double half = 0.5 * params_.cell_size_m;
  const double R = params_.suppression_radius_m;
  double best = 0.0;
  for (const auto& kv : structure_conf_) {
    const double d = (cellCenter(kv.first) - q).norm();
    double ramp;
    if (d <= half)
      ramp = 1.0;                         // inside the structure cell
    else if (d <= half + R)
      ramp = (half + R - d) / R;          // soft ramp to the buffer edge
    else
      continue;                           // outside this cell's influence
    best = std::max(best, params_.suppression_max * kv.second * ramp);
  }
  if (best > 0.0) ++suppression_hits_;
  return best;
}

}  // namespace navtracker
