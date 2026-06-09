#include "adapters/replay/HaxrTruthLoader.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace navtracker::replay {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

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

std::uint64_t fnv1a(const std::string& s) {
  std::uint64_t h = 1469598103934665603ull;
  for (char c : s) {
    h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
    h *= 1099511628211ull;
  }
  // Reserve 0 for "unassigned".
  return h == 0 ? 1 : h;
}

}  // namespace

std::vector<TruthSample> loadHaxrTruth(const std::string& path,
                                       const std::string& station,
                                       const StationMap& stations) {
  std::vector<TruthSample> out;
  auto it = stations.find(station);
  if (it == stations.end()) return out;
  const Eigen::Vector2d station_xy = it->second;

  std::ifstream f(path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;  // header

  while (std::getline(f, line)) {
    if (!splitLine(line, fields) || fields.size() < 4) continue;
    const double tod = std::strtod(fields[0].c_str(), nullptr);
    const std::string& uid = fields[1];
    const double range_m = std::strtod(fields[2].c_str(), nullptr);
    const double az_marine_deg = std::strtod(fields[3].c_str(), nullptr);
    if (range_m <= 0.0) continue;

    const double az_rad = az_marine_deg * kDeg2Rad;
    const Eigen::Vector2d offset(range_m * std::sin(az_rad),
                                 range_m * std::cos(az_rad));
    TruthSample s;
    s.time = Timestamp::fromSeconds(tod);
    s.truth_id = fnv1a(uid);
    s.position = station_xy + offset;
    s.velocity = Eigen::Vector2d::Zero();
    out.push_back(std::move(s));
  }
  return out;
}

}  // namespace navtracker::replay
