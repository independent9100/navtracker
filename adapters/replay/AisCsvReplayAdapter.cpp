#include "adapters/replay/AisCsvReplayAdapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::replay {
namespace {

constexpr double kAisDefaultSigma = 30.0;

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

struct ColumnMap {
  int time{-1};
  int mmsi{-1};
  int lat{-1};
  int lon{-1};
};

ColumnMap detectColumns(const std::vector<std::string>& header) {
  ColumnMap m;
  for (int i = 0; i < static_cast<int>(header.size()); ++i) {
    const std::string h = lowerTrim(header[i]);
    if (m.time < 0 && (h == "unix_time" || h == "time" || h == "basedatetime" ||
                       h == "# timestamp" || h == "timestamp"))
      m.time = i;
    else if (m.mmsi < 0 && h == "mmsi")
      m.mmsi = i;
    else if (m.lat < 0 && (h == "lat" || h == "latitude" || h == "latitude_degrees"))
      m.lat = i;
    else if (m.lon < 0 && (h == "lon" || h == "longitude" || h == "longitude_degrees"))
      m.lon = i;
  }
  return m;
}

// Parse a timestamp string. Returns false on failure.
// Supports: unix-epoch floats (with optional fractional part),
// ISO 8601 with T or space separator and optional fractional seconds.
bool parseTimeString(const std::string& s, Timestamp& out) {
  if (s.empty()) return false;
  // Unix epoch float: leading digit (no T/dash beyond a single optional minus).
  if (std::all_of(s.begin(), s.end(), [](char c) {
        return std::isdigit(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '+';
      })) {
    const double d = std::strtod(s.c_str(), nullptr);
    out = Timestamp::fromSeconds(d);
    return true;
  }
  // ISO 8601: YYYY-MM-DD[T| ]HH:MM:SS[.ffff][Z]
  std::tm tm{};
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  double frac = 0.0;
  if (std::sscanf(s.c_str(), "%d-%d-%d%*c%d:%d:%d", &year, &month, &day,
                  &hour, &minute, &second) < 6) {
    return false;
  }
  // Capture fractional seconds, if present.
  const auto dot = s.find('.');
  if (dot != std::string::npos) {
    frac = std::strtod(s.c_str() + dot, nullptr);  // e.g. ".218" -> 0.218
  }
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = second;
  const time_t t = timegm(&tm);
  if (t == -1) return false;
  out = Timestamp::fromSeconds(static_cast<double>(t) + frac);
  return true;
}

}  // namespace

std::vector<Measurement> loadAisCsv(const std::string& path,
                                    const geo::Datum& datum,
                                    std::string source_id) {
  std::vector<Measurement> out;
  std::ifstream f(path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;
  splitLine(line, fields);
  const ColumnMap cols = detectColumns(fields);
  if (cols.time < 0 || cols.lat < 0 || cols.lon < 0) return out;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields)) continue;
    const int max_col = std::max({cols.time, cols.mmsi, cols.lat, cols.lon});
    if (max_col >= static_cast<int>(fields.size())) continue;
    Timestamp t;
    if (!parseTimeString(fields[cols.time], t)) continue;
    const double lat = std::strtod(fields[cols.lat].c_str(), nullptr);
    const double lon = std::strtod(fields[cols.lon].c_str(), nullptr);
    if (lat == 0.0 && lon == 0.0) continue;
    if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) continue;
    std::uint32_t mmsi = 0;
    if (cols.mmsi >= 0) {
      mmsi = static_cast<std::uint32_t>(
          std::strtoul(fields[cols.mmsi].c_str(), nullptr, 10));
    }

    const Eigen::Vector3d enu = datum.toEnu({lat, lon, 0.0});
    Measurement m;
    m.time = t;
    m.sensor = SensorKind::Ais;
    m.source_id = source_id;
    m.model = MeasurementModel::Position2D;
    m.value = Eigen::Vector2d(enu.x(), enu.y());
    m.covariance = Eigen::Matrix2d::Identity() * (kAisDefaultSigma * kAisDefaultSigma);
    if (mmsi != 0) m.hints.mmsi = mmsi;
    out.push_back(std::move(m));
  }
  std::sort(out.begin(), out.end(),
            [](const Measurement& a, const Measurement& b) { return a.time < b.time; });
  return out;
}

}  // namespace navtracker::replay
