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

#include "core/estimation/PolarVelocity.hpp"
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
  int sog{-1};         // m/s (backlog #20; only read when emit_velocity)
  int cog{-1};         // deg, marine/true (N=0 CW)
  int nav_status{-1};  // AIS navigational status code 0..15
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
    else if (m.sog < 0 && (h == "sog" || h == "sog_mps" || h == "sog_ms"))
      m.sog = i;
    else if (m.cog < 0 && (h == "cog" || h == "cog_deg"))
      m.cog = i;
    else if (m.nav_status < 0 && (h == "nav_status" || h == "navstatus" ||
                                  h == "status"))
      m.nav_status = i;
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
                                    std::string source_id,
                                    bool emit_velocity) {
  std::vector<Measurement> out;
  std::ifstream f(path);
  if (!f) return out;

  std::string line;
  std::vector<std::string> fields;
  if (!std::getline(f, line)) return out;
  splitLine(line, fields);
  const ColumnMap cols = detectColumns(fields);
  if (cols.time < 0 || cols.lat < 0 || cols.lon < 0) return out;
  // Velocity content only when explicitly requested AND the CSV carries both
  // SOG and COG. Off (or missing columns) reproduces the historical Position2D
  // output bit-for-bit — the default-off byte-identical contract (#20).
  const bool want_velocity = emit_velocity && cols.sog >= 0 && cols.cog >= 0;

  while (std::getline(f, line)) {
    if (!splitLine(line, fields)) continue;
    int max_col = std::max({cols.time, cols.mmsi, cols.lat, cols.lon});
    if (want_velocity) max_col = std::max({max_col, cols.sog, cols.cog});
    if (cols.nav_status >= 0) max_col = std::max(max_col, cols.nav_status);
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

    // #20 velocity content: same rules as AisAdapter increment 2 — SOG/COG from
    // the target's own GPS become PositionVelocity2D above the SOG threshold
    // (below it COG is meaningless → Position2D). CSV SOG is already m/s, so no
    // knots conversion; the polar-Jacobian + isotropic-floor math is the SHARED
    // helper so replay and NMEA paths cannot diverge.
    const double sog_mps =
        (want_velocity) ? std::strtod(fields[cols.sog].c_str(), nullptr) : 0.0;
    const double cog_deg =
        (want_velocity) ? std::strtod(fields[cols.cog].c_str(), nullptr) : 0.0;
    const bool use_velocity = want_velocity && sog_mps >= kAisSogVelocityMinMps &&
                              cog_deg >= 0.0 && cog_deg < 360.0;
    if (use_velocity) {
      const double cog_rad = cog_deg * (M_PI / 180.0);
      const EnuVelocity2D vel = sogCogToEnuVelocity(
          sog_mps, cog_rad, kAisSogStdMps, kAisCogStdDeg * (M_PI / 180.0),
          kAisVelocityIsoFloorMps);
      m.model = MeasurementModel::PositionVelocity2D;
      Eigen::VectorXd v(4);
      v << enu.x(), enu.y(), vel.velocity.x(), vel.velocity.y();
      m.value = v;
      Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
      R.topLeftCorner<2, 2>() =
          Eigen::Matrix2d::Identity() * (kAisDefaultSigma * kAisDefaultSigma);
      R.bottomRightCorner<2, 2>() = vel.covariance;
      m.covariance = R;
    } else {
      m.model = MeasurementModel::Position2D;
      m.value = Eigen::Vector2d(enu.x(), enu.y());
      m.covariance =
          Eigen::Matrix2d::Identity() * (kAisDefaultSigma * kAisDefaultSigma);
    }
    if (mmsi != 0) m.hints.mmsi = mmsi;
    // nav_status corroboration cue (ADR 0002 / R3): surface it when the column
    // is present and requested. Gated with the velocity toggle (same flag) so
    // default-off stays byte-identical; 15 = undefined is dropped at the edge.
    if (emit_velocity && cols.nav_status >= 0) {
      char* end = nullptr;
      const long ns = std::strtol(fields[cols.nav_status].c_str(), &end, 10);
      if (end != fields[cols.nav_status].c_str() && ns >= 0 && ns <= 14) {
        m.hints.nav_status = static_cast<std::uint8_t>(ns);
      }
    }
    out.push_back(std::move(m));
  }
  std::sort(out.begin(), out.end(),
            [](const Measurement& a, const Measurement& b) { return a.time < b.time; });
  return out;
}

}  // namespace navtracker::replay
