#include "adapters/replay/RadarTruthCsvReader.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "core/projection/Projection.hpp"

namespace navtracker::replay {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// Split a comma-separated line into `fields` (cleared first).
// No quote/escape handling — fixture CSVs don't use them.
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

std::vector<TruthSample> loadRadarTruthCsv(const std::string& path,
                                            const OwnShipProvider& provider) {
  std::vector<TruthSample> out;
  std::ifstream f(path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  // Skip header row: tod,uid,range_m,azimuth_deg
  if (!std::getline(f, line)) return out;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 4) continue;

    const double tod = std::strtod(fields[0].c_str(), nullptr);
    const std::uint64_t uid =
        static_cast<std::uint64_t>(std::strtoull(fields[1].c_str(), nullptr, 10));
    const double range_m = std::strtod(fields[2].c_str(), nullptr);
    const double az_body_deg = std::strtod(fields[3].c_str(), nullptr);

    if (range_m <= 0.0) continue;
    if (uid == 0) continue;

    const Timestamp t = Timestamp::fromSeconds(tod);
    const auto pose_opt = provider.poseAtOrBefore(t);
    if (!pose_opt) continue;
    const auto& pose = *pose_opt;

    // Rotate body-frame azimuth (marine convention: north=0, clockwise) by
    // ownship heading (also true north, marine convention) to get world bearing
    // in marine convention. Then project from ownship ENU position.
    // This is the identical convention used by loadPlotCsvBodyFrame for
    // radar_plots.csv — same producer, same file format.
    const double world_bearing_rad =
        (pose.heading_true_deg + az_body_deg) * kDeg2Rad;
    const Eigen::Vector3d own_enu_3d =
        provider.datum().toEnu({pose.lat_deg, pose.lon_deg, 0.0});
    const Eigen::Vector2d own_xy(own_enu_3d.x(), own_enu_3d.y());

    // projectRangeBearingToEnu uses the marine bearing convention
    // (north=0, clockwise → east component = range*sin(bearing),
    //  north component = range*cos(bearing)). Sigma values are
    // not needed for truth samples, so we pass nominal small values.
    constexpr double kNominalSigmaR = 1.0;
    constexpr double kNominalSigmaAz = 1.0 * kDeg2Rad;
    const PointAndCov2D proj = projectRangeBearingToEnu(
        range_m, world_bearing_rad, kNominalSigmaR, kNominalSigmaAz,
        /*sigma_heading=*/0.0, /*sigma_gps=*/0.0, own_xy);

    TruthSample s;
    s.time = t;
    s.truth_id = uid;
    s.position = proj.pos_enu;
    // velocity stays zero — not available in a static range/azimuth CSV
    out.push_back(std::move(s));
  }

  std::sort(out.begin(), out.end(), [](const TruthSample& a, const TruthSample& b) {
    return a.time < b.time;
  });
  return out;
}

}  // namespace navtracker::replay
