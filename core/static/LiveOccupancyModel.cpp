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

void LiveOccupancyModel::setChartedStructure(
    const std::vector<StaticObstacle>& charted) {
  charted_enu_.clear();
  charted_enu_.reserve(charted.size());
  for (const StaticObstacle& o : charted) {
    const Eigen::Vector3d e = anchor_.toEnu(o.position);
    charted_enu_.emplace_back(e.x(), e.y());
  }
}

void LiveOccupancyModel::observeCamera(const CameraObservation& frame) {
  constexpr double kTwoPi = 6.283185307179586476925287;
  // Own-ship in the grid's anchor frame; cell centres are already anchor ENU, so
  // bearings are computed consistently (exact for a fixed datum).
  const Eigen::Vector2d sensor = toAnchorEnu(frame.sensor_enu);
  for (const auto& kv : persistence_) {
    const Eigen::Vector2d d = cellCenter(kv.first) - sensor;
    const double brg = std::atan2(d.y(), d.x());  // absolute ENU math bearing
    // In this frame's FOV? Absence outside it is never evidence of absence.
    if (std::abs(std::remainder(brg - frame.fov_center_rad, kTwoPi)) >
        frame.fov_half_width_rad)
      continue;
    // A detection within tolerance of the cell bearing ⇒ something is there.
    bool matched = false;
    for (double db : frame.detection_bearings_rad)
      if (std::abs(std::remainder(brg - db, kTwoPi)) <=
          frame.match_tolerance_rad) {
        matched = true;
        break;
      }
    auto it = camera_empty_streak_.find(kv.first);
    if (matched) {
      if (it != camera_empty_streak_.end()) camera_empty_streak_.erase(it);
    } else if (it == camera_empty_streak_.end()) {
      camera_empty_streak_.emplace(kv.first,
                                   std::make_pair(frame.t_unix, frame.t_unix));
    } else {
      it->second.second = frame.t_unix;  // extend the observed-empty streak
    }
  }
}

void LiveOccupancyModel::observeVesselFix(const VesselFix& fix) {
  // Store in the fixed anchor frame (like a charted point / camera sensor pos).
  VesselFix f = fix;
  f.position_enu = toAnchorEnu(fix.position_enu);
  vessel_fixes_.push_back(f);
}

void LiveOccupancyModel::observeVesselFix(double t_unix,
                                          const Eigen::Vector2d& position_enu) {
  observeVesselFix(VesselFix{t_unix, position_enu});
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
  // Drop camera streaks for cells that no longer exist (bound memory).
  for (auto it = camera_empty_streak_.begin();
       it != camera_empty_streak_.end();) {
    if (persistence_.find(it->first) == persistence_.end())
      it = camera_empty_streak_.erase(it);
    else
      ++it;
  }

  // Prune stale vessel-fix vetoes (older than the recency window relative to this
  // scan) so an AIS/cooperative feed going quiet lets the veto lapse.
  if (!vessel_fixes_.empty() && !by_sensor.empty()) {
    const double now = by_sensor.front().time.seconds();
    vessel_fixes_.erase(
        std::remove_if(vessel_fixes_.begin(), vessel_fixes_.end(),
                       [&](const VesselFix& f) {
                         return now - f.t_unix > params_.veto_window_s;
                       }),
        vessel_fixes_.end());
  }

  // 4b) Camera eviction pre-pass (increment ii): spend (erase) the persistence
  //     of any structure cell the camera has refuted, so recompute below cannot
  //     re-emit it. now = this scan's time (recency test). Off/unwired ⇒ skipped,
  //     so behaviour is bit-identical to the label-only increment.
  if (params_.evict_camera_empty && !by_sensor.empty())
    evictCameraRefutedCells(by_sensor.front().time.seconds());

  // 5) Re-classify structure (persistent AND extended) → suppression + hazards.
  recomputeStructure();

  // Introspection peaks.
  peak_structure_count_ =
      std::max(peak_structure_count_, static_cast<int>(obstacles_.size()));
  for (const auto& kv : persistence_)
    peak_persistence_ = std::max(peak_persistence_, kv.second);
}

std::map<LiveOccupancyModel::Cell, double>
LiveOccupancyModel::persistentCells() const {
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
  const double exit_bar = bar * params_.membership_exit_factor;

  // Persistent cells (ordered → deterministic component labeling), with
  // membership hysteresis: a cell already in last scan's structure set holds
  // down to `exit_bar`; a new cell must clear the (higher) enter `bar`. With
  // membership_exit_factor == 1 the two coincide (no hysteresis, bit-identical).
  std::map<Cell, double> persistent;
  for (const auto& kv : persistence_) {
    const double thresh = persistent_prev_.count(kv.first) ? exit_bar : bar;
    if (kv.second >= thresh) persistent.emplace(kv);
  }
  return persistent;
}

std::vector<std::vector<LiveOccupancyModel::Cell>>
LiveOccupancyModel::structureComponents() const {
  const std::map<Cell, double> persistent = persistentCells();

  std::vector<std::vector<Cell>> components;
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
    if (static_cast<int>(component.size()) >= params_.extended_cells_min)
      components.push_back(std::move(component));  // extended → structure
    // compact → a boat / small feature → never suppressed (dropped here)
  }
  return components;
}

void LiveOccupancyModel::evictCameraRefutedCells(double now_s) {
  if (camera_empty_streak_.empty()) return;
  const double r2 = params_.chart_corroboration_radius_m *
                    params_.chart_corroboration_radius_m;
  std::vector<Cell> evict;
  for (const auto& component : structureComponents()) {
    // Evidence precedence: a chart-confirmed component is HELD regardless of the
    // camera (same centroid test as the emitted-hazard chart label).
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const Cell& c : component) centroid += cellCenter(c);
    centroid /= static_cast<double>(component.size());
    bool corroborated = false;
    for (const Eigen::Vector2d& cp : charted_enu_)
      if ((cp - centroid).squaredNorm() <= r2) {
        corroborated = true;
        break;
      }
    if (corroborated) continue;
    // Keyed by CELL: a cell whose observed-empty streak is matured (span ≥
    // sustain) AND recent (last frame within the window of now) is spent — a
    // departed vessel the camera has confirmed empty. Erased below, so the
    // frozen persistence cannot re-emit it (no blink) and it restarts from
    // fresh returns.
    for (const Cell& c : component) {
      const auto it = camera_empty_streak_.find(c);
      if (it == camera_empty_streak_.end()) continue;
      const double span = it->second.second - it->second.first;
      const double staleness = now_s - it->second.second;
      if (span >= params_.camera_empty_sustain_s &&
          staleness <= params_.camera_empty_recency_window_s)
        evict.push_back(c);
    }
  }
  for (const Cell& c : evict) {
    persistence_.erase(c);
    camera_empty_streak_.erase(c);
  }
}

void LiveOccupancyModel::recomputeStructure() {
  obstacles_.clear();
  obstacle_center_.clear();
  obstacle_conf_.clear();
  obstacle_corroborated_.clear();
  obstacle_camera_empty_.clear();

  for (const auto& component : structureComponents()) {
    // Structure component → ONE synthesized live (uncharted) hazard whose
    // footprint COVERS every cell in the component, so the birth suppression
    // derived from it (below) can never exceed the emitted hazard's keep-clear
    // ring (the conservation invariant). conf = the component's peak
    // persistence (capped), used as the suppression strength.
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    double conf = 0.0;
    for (const Cell& c : component) {
      centroid += cellCenter(c);
      conf = std::max(conf, std::min(1.0, persistence_.at(c)));  // ≥ bar
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

    // Chart corroboration (label only): this live hazard is CONFIRMED as
    // structure if a charted structure point lies within the corroboration
    // radius of its centroid. Empty chart set ⇒ never corroborated (inert).
    bool corroborated = false;
    const double r2 = params_.chart_corroboration_radius_m *
                      params_.chart_corroboration_radius_m;
    for (const Eigen::Vector2d& c : charted_enu_)
      if ((c - centroid).squaredNorm() <= r2) {
        corroborated = true;
        break;
      }
    obstacle_corroborated_.push_back(corroborated);

    // Camera-observed-empty (label): the centroid cell has been continuously
    // camera-observed-empty for at least the sustain window.
    bool cam_empty = false;
    const auto cit = camera_empty_streak_.find(cellOf(centroid));
    if (cit != camera_empty_streak_.end() &&
        (cit->second.second - cit->second.first) >= params_.camera_empty_sustain_s)
      cam_empty = true;
    obstacle_camera_empty_.push_back(cam_empty);
  }

  // Membership hysteresis: record this scan's persistent set for next scan's
  // enter/exit test. `persistent_prev_` was unchanged through this scan, so
  // persistentCells() here returns exactly the set structureComponents() used.
  const std::map<Cell, double> persistent = persistentCells();
  persistent_prev_.clear();
  for (const auto& kv : persistent) persistent_prev_.insert(kv.first);
}

double LiveOccupancyModel::birthSuppression(
    const Eigen::Vector2d& enu_xy) const {
  if (obstacles_.empty()) return 0.0;
  const Eigen::Vector2d q = toAnchorEnu(enu_xy);
  // Corroboration VETO (R9 item 1b): an AIS/cooperative-known vessel must remain
  // track-eligible — never suppress a birth within veto_radius of a recent fix.
  // The veto only lowers suppression to 0, so the conservation invariant holds.
  if (!vessel_fixes_.empty()) {
    const double vr2 = params_.veto_radius_m * params_.veto_radius_m;
    for (const VesselFix& f : vessel_fixes_)
      if ((f.position_enu - q).squaredNorm() <= vr2) return 0.0;
  }
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

std::vector<Eigen::Vector2d> LiveOccupancyModel::cameraObservedEmptyCells() const {
  std::vector<Eigen::Vector2d> out;
  for (const auto& kv : camera_empty_streak_)
    if ((kv.second.second - kv.second.first) >= params_.camera_empty_sustain_s)
      out.push_back(cellCenter(kv.first));  // anchor-frame cell centre
  return out;
}

std::vector<std::pair<Eigen::Vector2d, double>>
LiveOccupancyModel::persistenceCells() const {
  std::vector<std::pair<Eigen::Vector2d, double>> out;
  out.reserve(persistence_.size());
  for (const auto& kv : persistence_)
    out.emplace_back(cellCenter(kv.first), kv.second);  // anchor-frame centre + value
  return out;
}

std::vector<Eigen::Vector2d> LiveOccupancyModel::vesselFixPositions() const {
  std::vector<Eigen::Vector2d> out;
  out.reserve(vessel_fixes_.size());
  for (const auto& f : vessel_fixes_) out.push_back(f.position_enu);
  return out;
}

}  // namespace navtracker
