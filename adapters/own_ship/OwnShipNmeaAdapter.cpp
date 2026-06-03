#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"

#include <cstdlib>

#include "adapters/util/Nmea.hpp"

namespace navtracker {
namespace {

double parseDdmm(const std::string& s) {
  if (s.empty()) return 0.0;
  const auto dot = s.find('.');
  const std::size_t mm_digits = 2;
  const std::size_t deg_end = (dot == std::string::npos ? s.size() : dot) - mm_digits;
  if (deg_end > s.size()) return 0.0;
  const double deg = std::strtod(s.substr(0, deg_end).c_str(), nullptr);
  const double min = std::strtod(s.substr(deg_end).c_str(), nullptr);
  return deg + min / 60.0;
}

}  // namespace

OwnShipNmeaAdapter::OwnShipNmeaAdapter(OwnShipProvider& provider)
    : provider_(provider) {}

void OwnShipNmeaAdapter::setPositionStd(double sigma_m) {
  position_std_m_ = sigma_m;
}

bool OwnShipNmeaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;
  OwnShipPose pose = provider_.latest().value_or(OwnShipPose{});
  pose.time = t;
  pose.position_std_m = position_std_m_;

  if (parsed->formatter == "GGA") {
    if (parsed->fields.size() < 5) return false;
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    pose.lat_deg = lat;
    pose.lon_deg = lon;
    provider_.update(pose);
    return true;
  }
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    pose.heading_true_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    provider_.update(pose);
    return true;
  }
  return false;
}

}  // namespace navtracker
