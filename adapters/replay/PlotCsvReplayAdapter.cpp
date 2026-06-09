#include "adapters/replay/PlotCsvReplayAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "core/projection/Projection.hpp"

namespace navtracker::replay {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

double wrap(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

// Read a single comma-separated record into `fields` (cleared first).
// Empty input returns false. No quote / escape handling — the fixture
// CSVs we own do not use them.
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

StationMap loadStations(const std::string& path) {
  StationMap out;
  std::ifstream f(path);
  if (!f) return out;
  std::string line;
  std::vector<std::string> fields;
  // Header: station,x (meters),y (meters)
  if (!std::getline(f, line)) return out;
  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 3) continue;
    out[fields[0]] = Eigen::Vector2d(std::strtod(fields[1].c_str(), nullptr),
                                     std::strtod(fields[2].c_str(), nullptr));
  }
  return out;
}

std::vector<Measurement> loadPlotCsv(const std::string& plots_csv_path,
                                    const StationMap& stations,
                                    SensorKind sensor,
                                    std::string source_id) {
  std::vector<Measurement> out;
  std::ifstream f(plots_csv_path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;  // header

  constexpr double kDefaultSigmaR = 25.0;
  constexpr double kDefaultSigmaAzDeg = 1.0;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 8) continue;
    const double tod = std::strtod(fields[0].c_str(), nullptr);
    const double range_m = std::strtod(fields[1].c_str(), nullptr);
    const double az_marine_deg = std::strtod(fields[2].c_str(), nullptr);
    double sigma_r = std::strtod(fields[3].c_str(), nullptr);
    double sigma_az_deg = std::strtod(fields[4].c_str(), nullptr);
    const std::string& station = fields[7];

    auto it = stations.find(station);
    if (it == stations.end()) continue;
    if (range_m <= 0.0) continue;
    if (sigma_r <= 0.0) sigma_r = kDefaultSigmaR;
    if (sigma_az_deg <= 0.0) sigma_az_deg = kDefaultSigmaAzDeg;
    const double sigma_az = sigma_az_deg * kDeg2Rad;

    // Project range/azimuth into the station's ENU frame as Position2D.
    // The existing EkfEstimator::initiate path only knows how to
    // initialise Cartesian state from value(0..1) = (x, y), so each
    // adapter projects to Position2D at the edge (the ArpaAdapter does
    // the same — see adapters/arpa/ArpaAdapter.cpp).
    //
    // `bearing_true_rad` for projectRangeBearingToEnu is the marine
    // convention (north = 0, clockwise). The plot CSV's azimuth_deg is
    // already in that convention, so no remap is needed beyond the
    // deg→rad scale. Heading and GPS uncertainty are zero because
    // shore stations have a known fixed position.
    const double az_marine_rad = az_marine_deg * kDeg2Rad;
    const PointAndCov2D proj = projectRangeBearingToEnu(
        range_m, az_marine_rad, sigma_r, sigma_az,
        /*sigma_heading=*/0.0, /*sigma_gps=*/0.0, it->second);

    Measurement m;
    m.time = Timestamp::fromSeconds(tod);
    m.sensor = sensor;
    m.source_id = source_id;
    m.model = MeasurementModel::Position2D;
    m.value = proj.pos_enu;
    m.covariance = proj.cov;
    m.sensor_position_enu = it->second;
    out.push_back(std::move(m));
  }

  std::sort(out.begin(), out.end(), [](const Measurement& a, const Measurement& b) {
    return a.time < b.time;
  });
  return out;
}

std::vector<Measurement> loadPlotCsvBodyFrame(const std::string& plots_csv_path,
                                              const OwnShipProvider& provider,
                                              SensorKind sensor,
                                              std::string source_id) {
  std::vector<Measurement> out;
  std::ifstream f(plots_csv_path);
  if (!f) return out;
  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;  // header

  constexpr double kDefaultSigmaR = 25.0;
  constexpr double kDefaultSigmaAzDeg = 1.0;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 8) continue;
    const double tod = std::strtod(fields[0].c_str(), nullptr);
    const double range_m = std::strtod(fields[1].c_str(), nullptr);
    const double az_body_deg = std::strtod(fields[2].c_str(), nullptr);
    double sigma_r = std::strtod(fields[3].c_str(), nullptr);
    double sigma_az_deg = std::strtod(fields[4].c_str(), nullptr);
    if (range_m <= 0.0) continue;
    if (sigma_r <= 0.0) sigma_r = kDefaultSigmaR;
    if (sigma_az_deg <= 0.0) sigma_az_deg = kDefaultSigmaAzDeg;
    const double sigma_az = sigma_az_deg * kDeg2Rad;

    const Timestamp t = Timestamp::fromSeconds(tod);
    const auto pose_opt = provider.poseAtOrBefore(t);
    if (!pose_opt) continue;
    const auto& pose = *pose_opt;

    // Rotate body bearing into world bearing using ownship heading,
    // then project from ownship's ENU position.
    const double world_bearing_rad =
        (pose.heading_true_deg + az_body_deg) * kDeg2Rad;
    const Eigen::Vector3d own_enu =
        provider.datum().toEnu({pose.lat_deg, pose.lon_deg, 0.0});
    const Eigen::Vector2d own_xy(own_enu.x(), own_enu.y());

    const PointAndCov2D proj = projectRangeBearingToEnu(
        range_m, world_bearing_rad, sigma_r, sigma_az,
        /*sigma_heading=*/0.0, /*sigma_gps=*/0.0, own_xy);

    Measurement m;
    m.time = t;
    m.sensor = sensor;
    m.source_id = source_id;
    m.model = MeasurementModel::Position2D;
    m.value = proj.pos_enu;
    m.covariance = proj.cov;
    m.sensor_position_enu = own_xy;
    out.push_back(std::move(m));
  }
  std::sort(out.begin(), out.end(), [](const Measurement& a, const Measurement& b) {
    return a.time < b.time;
  });
  return out;
}

}  // namespace navtracker::replay
