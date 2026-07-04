#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace navtracker {

/**
 * A parsed NMEA 0183 sentence, split into its address header
 * (`talker` = 2-char station code, `formatter` = sentence type) and the
 * comma-separated data `fields` (checksum stripped). The shared parse
 * result the sensor adapters (AIS/ARPA/own-ship) build their readings from.
 */
struct NmeaSentence {
  std::string talker;
  std::string formatter;
  std::vector<std::string> fields;
};

/**
 * Parse and checksum-validate one NMEA 0183 line. Returns the split sentence,
 * or `std::nullopt` if the line is too short, lacks the leading `$`/`!`, has
 * no `*` checksum field, or the XOR checksum does not match — validate-at-the-
 * edge (invariant #6) so downstream adapters trust their input.
 */
std::optional<NmeaSentence> parseNmea(std::string_view line);

}  // namespace navtracker
