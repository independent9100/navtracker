#include "adapters/replay/CameraBearingCsvReader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "core/estimation/MeasurementModels.hpp"  // wrapAngle

namespace navtracker::replay {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// Split a comma-separated record into `fields` (cleared first). No quote /
// escape handling — the fixture CSVs we own do not use them. Mirrors
// PlotCsvReplayAdapter's splitLine.
bool splitLine(const std::string& line, std::vector<std::string>& fields) {
  fields.clear();
  if (line.empty()) return false;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(std::move(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  fields.push_back(std::move(cur));
  return true;
}

}  // namespace

std::vector<Measurement> loadCameraBearingsCsv(
    const std::string& path,
    const OwnShipProvider& provider,
    std::string source_prefix,
    CameraBearingLoadStats* stats) {
  std::vector<Measurement> out;
  CameraBearingLoadStats local;

  std::ifstream f(path);
  if (!f) {
    if (stats) *stats = local;
    return out;
  }
  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) {  // header
    if (stats) *stats = local;
    return out;
  }

  // Column order: unix_time,camera,bearing_rel_deg,sigma_deg,confidence,...
  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 4) continue;
    ++local.rows_read;

    const double t_s = std::strtod(fields[0].c_str(), nullptr);
    const std::string& camera = fields[1];
    const double bearing_rel_deg = std::strtod(fields[2].c_str(), nullptr);
    const double sigma_deg = std::strtod(fields[3].c_str(), nullptr);

    if (!std::isfinite(bearing_rel_deg) || !std::isfinite(sigma_deg) ||
        sigma_deg <= 0.0) {
      ++local.dropped_invalid;
      continue;
    }

    const Timestamp t = Timestamp::fromSeconds(t_s);
    const auto pose_opt = provider.poseAtOrBefore(t);
    if (!pose_opt) {
      ++local.dropped_no_pose;
      continue;
    }
    const auto& pose = *pose_opt;

    // Compose heading -> absolute ENU math bearing (atan2(dN, dE)).
    const double marine_true_deg = pose.heading_true_deg + bearing_rel_deg;
    const double math_rad = wrapAngle(kPi / 2.0 - marine_true_deg * kDeg2Rad);

    const Eigen::Vector3d own_enu =
        provider.datum().toEnu({pose.lat_deg, pose.lon_deg, 0.0});

    Measurement m;
    m.time = t;
    m.sensor = SensorKind::EoIr;
    m.source_id = source_prefix + "_" + camera;
    m.model = MeasurementModel::Bearing2D;
    Eigen::VectorXd v(1);
    v(0) = math_rad;
    m.value = std::move(v);
    Eigen::MatrixXd R(1, 1);
    const double sigma_rad = sigma_deg * kDeg2Rad;
    R(0, 0) = sigma_rad * sigma_rad;
    m.covariance = std::move(R);
    m.sensor_position_enu = Eigen::Vector2d(own_enu.x(), own_enu.y());
    m.sensor_position_std_m = pose.position_std_m;
    out.push_back(std::move(m));
    ++local.emitted;
  }

  std::sort(out.begin(), out.end(), [](const Measurement& a, const Measurement& b) {
    return a.time < b.time;
  });
  if (stats) *stats = local;
  return out;
}

}  // namespace navtracker::replay
