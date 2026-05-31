#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace navtracker_test {

// Build a full NMEA 0183 sentence: prepends '$', appends '*' and the XOR
// checksum (two uppercase hex digits) computed over the body.
inline std::string makeNmea(std::string_view body) {
  std::uint8_t cs = 0;
  for (char c : body) cs ^= static_cast<std::uint8_t>(c);
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02X", cs);
  std::string out = "$";
  out.append(body.data(), body.size());
  out.push_back('*');
  out.append(hex);
  return out;
}

}  // namespace navtracker_test
