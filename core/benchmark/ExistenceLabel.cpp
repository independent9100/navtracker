#include "core/benchmark/ExistenceLabel.hpp"

#include <istream>
#include <string>
#include <vector>

namespace navtracker {
namespace benchmark {

namespace {

// Split `line` into at most `max_fields` comma-separated fields; the final
// field keeps any remaining commas (so a free-text notes column survives).
std::vector<std::string> splitN(const std::string& line, std::size_t max_fields) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (out.size() + 1 < max_fields) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) break;
    out.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  out.push_back(line.substr(start));
  return out;
}

double parseDoubleOr(const std::string& s, double fallback) {
  if (s.empty()) return fallback;
  try {
    return std::stod(s);
  } catch (...) {
    return fallback;
  }
}

ExistenceLabelClass parseClass(const std::string& s) {
  if (s == "KEEP_VESSEL") return ExistenceLabelClass::KeepVessel;
  if (s == "SUPPRESS_STRUCTURE") return ExistenceLabelClass::SuppressStructure;
  if (s == "KEEP_ANCHORAGE") return ExistenceLabelClass::KeepAnchorage;
  if (s == "KEEP_MIXED") return ExistenceLabelClass::KeepMixed;
  return ExistenceLabelClass::Unknown;
}

}  // namespace

std::vector<ExistenceLabel> parseExistenceLabels(std::istream& is) {
  std::vector<ExistenceLabel> out;
  std::string line;
  bool header_seen = false;
  while (std::getline(is, line)) {
    // Strip a trailing CR (CRLF files).
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    if (!header_seen) {  // first non-comment, non-blank line is the header
      header_seen = true;
      continue;
    }
    const auto f = splitN(line, 11);
    if (f.size() < 11) continue;  // malformed row — skip

    ExistenceLabel lab;
    lab.region_id = f[0];
    lab.source_rank = f[1];
    lab.lat_deg = parseDoubleOr(f[2], 0.0);
    lab.lon_deg = parseDoubleOr(f[3], 0.0);
    lab.radius_m = parseDoubleOr(f[4], 0.0);
    const bool blank_start = f[5].empty();
    const bool blank_end = f[6].empty();
    lab.covers_whole_clip = blank_start && blank_end;
    lab.t_start_s = parseDoubleOr(f[5], 0.0);
    lab.t_end_s = parseDoubleOr(f[6], 0.0);
    lab.label = parseClass(f[7]);
    lab.evidence = f[8];
    lab.confidence = f[9];
    lab.notes = f[10];
    out.push_back(std::move(lab));
  }
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
