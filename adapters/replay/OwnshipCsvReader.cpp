#include "adapters/replay/OwnshipCsvReader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace navtracker::replay {
namespace {

bool splitLine(const std::string& line, std::vector<std::string>& fields) {
  fields.clear();
  if (line.empty()) return false;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(std::move(cur));
      cur.clear();
    } else if (c != '\r') {
      cur.push_back(c);
    }
  }
  fields.push_back(std::move(cur));
  return true;
}

std::string lowerTrim(std::string s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

}  // namespace

std::vector<OwnShipPose> loadOwnshipCsv(const std::string& path) {
  std::vector<OwnShipPose> out;
  std::ifstream f(path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;
  splitLine(line, fields);
  int c_time = -1, c_lat = -1, c_lon = -1, c_hdg = -1;
  for (int i = 0; i < static_cast<int>(fields.size()); ++i) {
    const std::string h = lowerTrim(fields[i]);
    if (h == "unix_time" || h == "time" || h == "timestamp") c_time = i;
    else if (h == "lat" || h == "latitude") c_lat = i;
    else if (h == "lon" || h == "longitude") c_lon = i;
    else if (h == "heading_deg" || h == "heading") c_hdg = i;
  }
  if (c_time < 0 || c_lat < 0 || c_lon < 0) return out;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields)) continue;
    const int max_col = std::max({c_time, c_lat, c_lon, c_hdg});
    if (max_col >= static_cast<int>(fields.size())) continue;
    // #26 M22: validate each row at the edge. A blank lat/lon field parses via
    // strtod to 0.0, so an unvalidated row became a (0,0) Null-Island pose that
    // poisons every body-frame projection in its window. Skip non-finite times
    // and implausible / (0,0) positions — the sibling AIS loader already does.
    const double t_s = std::strtod(fields[c_time].c_str(), nullptr);
    const double lat = std::strtod(fields[c_lat].c_str(), nullptr);
    const double lon = std::strtod(fields[c_lon].c_str(), nullptr);
    // Inline the plausibility check (same as the sibling AIS loader) rather
    // than depend on adapters/util/EdgeValidation — navtracker_replay is linked
    // into targets (e.g. navtracker_bench_baseline) that do not pull in the
    // edge-validation TU.
    if (!std::isfinite(t_s) || !std::isfinite(lat) || !std::isfinite(lon))
      continue;
    if (lat == 0.0 && lon == 0.0) continue;
    if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) continue;
    OwnShipPose p;
    p.time = Timestamp::fromSeconds(t_s);
    p.lat_deg = lat;
    p.lon_deg = lon;
    p.heading_true_deg = (c_hdg >= 0) ? std::strtod(fields[c_hdg].c_str(), nullptr) : 0.0;
    out.push_back(std::move(p));
  }
  std::sort(out.begin(), out.end(),
            [](const OwnShipPose& a, const OwnShipPose& b) { return a.time < b.time; });
  return out;
}

void feedOwnshipHistory(OwnShipProvider& provider,
                        const std::vector<OwnShipPose>& poses) {
  for (const auto& p : poses) provider.update(p);
}

}  // namespace navtracker::replay
