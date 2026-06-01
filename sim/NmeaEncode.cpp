#include "sim/NmeaEncode.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace navtracker::sim {
namespace {

std::string formatDdmm(double abs_deg, int deg_width) {
  int deg = static_cast<int>(abs_deg);
  double minutes = (abs_deg - static_cast<double>(deg)) * 60.0;
  // Round minutes to 5 fractional digits via integer math, carrying if it
  // rounds up to 60.00000.
  long long frac_units = static_cast<long long>(std::llround(minutes * 100000.0));
  if (frac_units >= 6000000LL) {  // 60.00000 minutes
    frac_units -= 6000000LL;
    deg += 1;
  }
  const int int_minutes  = static_cast<int>(frac_units / 100000LL);
  const int frac_minutes = static_cast<int>(frac_units % 100000LL);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%0*d%02d.%05d",
                deg_width, deg, int_minutes, frac_minutes);
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
