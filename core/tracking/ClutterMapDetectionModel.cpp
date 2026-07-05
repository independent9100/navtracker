#include "core/tracking/ClutterMapDetectionModel.hpp"

#include <algorithm>
#include <cmath>

namespace navtracker {

namespace {

constexpr double kTwoPi = 6.283185307179586476925287;

bool positionMapped(MeasurementModel m) {
  return m == MeasurementModel::Position2D ||
         m == MeasurementModel::PositionVelocity2D;
}

// Wrap an azimuth to [-π, π).
double wrapAzimuth(double a) {
  const double w = std::remainder(a, kTwoPi);
  return (w >= kTwoPi / 2.0) ? w - kTwoPi : w;
}

}  // namespace

ClutterMapSensorDetectionModel::ClutterMapSensorDetectionModel(
    std::shared_ptr<ISensorDetectionModel> inner, ClutterMapParams params)
    : inner_(std::move(inner)), p_(params) {
  bearing_cell_count_ = static_cast<std::size_t>(
      std::max(1.0, std::round(kTwoPi / p_.bearing_cell_rad)));
}

DetectionParams ClutterMapSensorDetectionModel::paramsFor(
    SensorKind sensor, MeasurementModel model) const {
  return inner_->paramsFor(sensor, model);
}

DetectionParams ClutterMapSensorDetectionModel::paramsFor(
    SensorKind sensor, MeasurementModel model,
    const std::string& source_id) const {
  return inner_->paramsFor(sensor, model, source_id);
}

double ClutterMapSensorDetectionModel::positionCellLambda(
    const PositionMap& m, std::pair<std::int64_t, std::int64_t> idx,
    double baseline) const {
  const auto it = m.cells.find(idx);
  if (it == m.cells.end()) return baseline;
  return it->second.rate / (p_.cell_size_m * p_.cell_size_m);
}

double ClutterMapSensorDetectionModel::bearingCellLambda(
    const BearingMap& m, std::size_t idx, double baseline) const {
  if (idx >= m.cells.size() || !m.touched[idx]) return baseline;
  return m.cells[idx].rate / (kTwoPi / static_cast<double>(m.cells.size()));
}

std::vector<std::pair<Eigen::Vector2d, double>>
ClutterMapSensorDetectionModel::positionClutterCells() const {
  const double h = p_.cell_size_m;
  std::vector<std::pair<Eigen::Vector2d, double>> out;
  for (const auto& [key, map] : position_maps_)
    for (const auto& [idx, cell] : map.cells)
      out.emplace_back(Eigen::Vector2d((idx.first + 0.5) * h, (idx.second + 0.5) * h),
                       cell.rate / (h * h));   // λ_c = rate / area; centre of cell
  return out;
}

std::vector<std::pair<double, double>>
ClutterMapSensorDetectionModel::bearingClutterCells() const {
  std::vector<std::pair<double, double>> out;
  for (const auto& [key, map] : bearing_maps_) {
    const std::size_t n = map.cells.size();
    if (n == 0) continue;
    const double cell_w = kTwoPi / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (i >= map.touched.size() || !map.touched[i]) continue;
      const double az = (static_cast<double>(i) + 0.5) * cell_w - kTwoPi / 2.0;
      out.emplace_back(az, map.cells[i].rate / cell_w);
    }
  }
  return out;
}

DetectionParams ClutterMapSensorDetectionModel::paramsFor(
    const Measurement& z) const {
  DetectionParams dp = inner_->paramsFor(z);
  const double base = dp.clutter_intensity;
  double lambda = base;

  if (positionMapped(z.model) && z.value.size() >= 2) {
    const auto map_it = position_maps_.find(MapKey{z.sensor, z.model});
    if (map_it == position_maps_.end()) return dp;
    // Bilinear interpolation over the 4 cell centers surrounding the
    // query point. Cell (i, j) is [i·h, (i+1)·h) × [j·h, (j+1)·h) with
    // center at ((i+0.5)·h, (j+0.5)·h).
    const double h = p_.cell_size_m;
    const double u = z.value(0) / h - 0.5;
    const double v = z.value(1) / h - 0.5;
    const auto i0 = static_cast<std::int64_t>(std::floor(u));
    const auto j0 = static_cast<std::int64_t>(std::floor(v));
    const double fx = u - static_cast<double>(i0);
    const double fy = v - static_cast<double>(j0);
    const PositionMap& m = map_it->second;
    lambda = (1.0 - fx) * (1.0 - fy) * positionCellLambda(m, {i0, j0}, base) +
             fx * (1.0 - fy) * positionCellLambda(m, {i0 + 1, j0}, base) +
             (1.0 - fx) * fy * positionCellLambda(m, {i0, j0 + 1}, base) +
             fx * fy * positionCellLambda(m, {i0 + 1, j0 + 1}, base);
  } else if (z.model == MeasurementModel::Bearing2D && z.value.size() >= 1) {
    const auto map_it = bearing_maps_.find(MapKey{z.sensor, z.model});
    if (map_it == bearing_maps_.end()) return dp;
    const BearingMap& m = map_it->second;
    const auto n = static_cast<std::int64_t>(m.cells.size());
    const double cell_w = kTwoPi / static_cast<double>(n);
    // Circular linear interpolation between the two adjacent centers.
    const double u = (wrapAzimuth(z.value(0)) + kTwoPi / 2.0) / cell_w - 0.5;
    const auto i_raw = static_cast<std::int64_t>(std::floor(u));
    const double f = u - static_cast<double>(i_raw);
    const std::size_t i0 = static_cast<std::size_t>(((i_raw % n) + n) % n);
    const std::size_t i1 = (i0 + 1) % static_cast<std::size_t>(n);
    lambda = (1.0 - f) * bearingCellLambda(m, i0, base) +
             f * bearingCellLambda(m, i1, base);
  } else {
    // No map for this measurement space (RangeBearing2D, …): table λ.
    return dp;
  }

  dp.clutter_intensity =
      std::clamp(lambda, base * p_.min_ratio, base * p_.max_ratio);
  return dp;
}

void ClutterMapSensorDetectionModel::observe(
    const std::vector<ScanObservation>& by_sensor) {
  for (const ScanObservation& s : by_sensor) {
    // Kind-wide baseline seeds new cells so an untouched map reads back
    // exactly the table value.
    const double base = inner_->paramsFor(s.sensor, s.model).clutter_intensity;

    // touch(): one cell, one scan. n = weighted clutter-evidence sum in
    // the cell; cells holding only claimed returns get n = 0 (decay).
    const auto touch = [&](Cell& cell, bool is_new, double n) {
      double dt = p_.prior_dt_s;
      if (!is_new) dt = std::max(s.time.secondsSince(cell.last), 0.0);
      const double w = 1.0 - std::exp(-dt / p_.time_constant_s);
      cell.rate += w * (n - cell.rate);
      cell.last = s.time;
    };
    // Per-return clutter weight: aligned weights vector, or 1.0 each
    // when the producer supplies binary labels only.
    const auto weightAt = [](const std::vector<double>& ws, std::size_t i) {
      return (i < ws.size()) ? ws[i] : 1.0;
    };

    if (positionMapped(s.model)) {
      PositionMap& m = position_maps_[MapKey{s.sensor, s.model}];
      const double h = p_.cell_size_m;
      const auto cellOf = [&](const Eigen::Vector2d& q) {
        return std::pair<std::int64_t, std::int64_t>{
            static_cast<std::int64_t>(std::floor(q.x() / h)),
            static_cast<std::int64_t>(std::floor(q.y() / h))};
      };
      std::map<std::pair<std::int64_t, std::int64_t>, double> counts;
      for (const auto& q : s.positions) counts[cellOf(q)];  // touch, n = 0
      for (std::size_t i = 0; i < s.clutter_positions.size(); ++i)
        counts[cellOf(s.clutter_positions[i])] +=
            weightAt(s.clutter_position_weights, i);
      for (const auto& [idx, n] : counts) {
        const auto [it, is_new] = m.cells.try_emplace(
            idx, Cell{base * h * h, s.time});
        touch(it->second, is_new, n);
      }
    } else if (s.model == MeasurementModel::Bearing2D &&
               p_.enable_bearing_map) {
      BearingMap& m = bearing_maps_[MapKey{s.sensor, s.model}];
      if (m.cells.empty()) {
        m.cells.resize(bearing_cell_count_);
        m.touched.assign(bearing_cell_count_, false);
      }
      const auto n_cells = static_cast<std::int64_t>(m.cells.size());
      const double cell_w = kTwoPi / static_cast<double>(n_cells);
      const auto cellOf = [&](double az) {
        const auto i = static_cast<std::int64_t>(
            std::floor((wrapAzimuth(az) + kTwoPi / 2.0) / cell_w));
        return static_cast<std::size_t>(((i % n_cells) + n_cells) % n_cells);
      };
      std::map<std::size_t, double> counts;
      for (double az : s.bearings) counts[cellOf(az)];  // touch, n = 0
      for (std::size_t i = 0; i < s.clutter_bearings.size(); ++i)
        counts[cellOf(s.clutter_bearings[i])] +=
            weightAt(s.clutter_bearing_weights, i);
      for (const auto& [idx, n] : counts) {
        const bool is_new = !m.touched[idx];
        if (is_new) m.cells[idx] = Cell{base * cell_w, s.time};
        touch(m.cells[idx], is_new, n);
        m.touched[idx] = true;
      }
    }
  }
  inner_->observe(by_sensor);
}

}  // namespace navtracker
