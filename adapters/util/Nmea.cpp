#include "adapters/util/Nmea.hpp"

#include <cstdint>

namespace navtracker {
namespace {

bool fromHex(char c, std::uint8_t& out) {
  if (c >= '0' && c <= '9') { out = static_cast<std::uint8_t>(c - '0'); return true; }
  if (c >= 'A' && c <= 'F') { out = static_cast<std::uint8_t>(10 + c - 'A'); return true; }
  if (c >= 'a' && c <= 'f') { out = static_cast<std::uint8_t>(10 + c - 'a'); return true; }
  return false;
}

}  // namespace

std::optional<NmeaSentence> parseNmea(std::string_view line) {
  if (line.size() < 8) return std::nullopt;
  if (line.front() != '$' && line.front() != '!') return std::nullopt;
  const auto star = line.find('*');
  if (star == std::string_view::npos) return std::nullopt;
  if (line.size() < star + 3) return std::nullopt;

  std::uint8_t hi = 0, lo = 0;
  if (!fromHex(line[star + 1], hi) || !fromHex(line[star + 2], lo)) return std::nullopt;
  const std::uint8_t expected = static_cast<std::uint8_t>((hi << 4) | lo);

  std::uint8_t cs = 0;
  for (std::size_t i = 1; i < star; ++i) cs ^= static_cast<std::uint8_t>(line[i]);
  if (cs != expected) return std::nullopt;

  const auto comma = line.find(',', 1);
  if (comma == std::string_view::npos || comma >= star) return std::nullopt;
  const auto header = line.substr(1, comma - 1);
  if (header.size() < 3) return std::nullopt;

  NmeaSentence s;
  s.talker = std::string(header.substr(0, 2));
  s.formatter = std::string(header.substr(2));

  std::size_t i = comma + 1;
  while (i <= star) {
    const auto next = line.find_first_of(",*", i);
    const auto end = (next == std::string_view::npos) ? star : next;
    s.fields.emplace_back(line.substr(i, end - i));
    if (end == star) break;
    i = end + 1;
  }
  return s;
}

}  // namespace navtracker
