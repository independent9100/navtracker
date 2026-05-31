#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace navtracker {

struct NmeaSentence {
  std::string talker;
  std::string formatter;
  std::vector<std::string> fields;
};

std::optional<NmeaSentence> parseNmea(std::string_view line);

}  // namespace navtracker
