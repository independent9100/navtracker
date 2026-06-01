#include "sim/NmeaEncode.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace navtracker::sim {
namespace {

std::string formatDdmm(double abs_deg, int deg_width) {
  const int deg = static_cast<int>(abs_deg);
  const double minutes = (abs_deg - static_cast<double>(deg)) * 60.0;
  char buf[32];
  // %0*d for zero-padded degrees; %08.5f keeps width 8 (e.g. "30.00000").
  std::snprintf(buf, sizeof(buf), "%0*d%08.5f", deg_width, deg, minutes);
  return std::string(buf);
}

}  // namespace

std::string formatLatDdmm(double deg) {
  return formatDdmm(std::fabs(deg), 2);
}

std::string formatLonDdmm(double deg) {
  return formatDdmm(std::fabs(deg), 3);
}

char latHemisphere(double deg) { return deg >= 0.0 ? 'N' : 'S'; }
char lonHemisphere(double deg) { return deg >= 0.0 ? 'E' : 'W'; }

std::string wrapWithChecksum(std::string_view body) {
  std::uint8_t cs = 0;
  for (char c : body) cs ^= static_cast<std::uint8_t>(c);
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02X", cs);
  std::string out;
  out.reserve(body.size() + 4);
  out.push_back('$');
  out.append(body.data(), body.size());
  out.push_back('*');
  out.append(hex);
  return out;
}

}  // namespace navtracker::sim
