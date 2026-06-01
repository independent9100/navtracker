#pragma once

#include <string>
#include <string_view>

namespace navtracker::sim {

// "DDMM.mmmmm" with 5 fractional minute digits, no sign (caller pairs with
// hemisphere). Absolute value of `deg` is used.
std::string formatLatDdmm(double deg);
std::string formatLonDdmm(double deg);  // "DDDMM.mmmmm" (three deg digits)

char latHemisphere(double deg);  // 'N' if deg >= 0, else 'S'
char lonHemisphere(double deg);  // 'E' if deg >= 0, else 'W'

// Returns "$" + body + "*" + two-hex-digit XOR checksum (uppercase).
std::string wrapWithChecksum(std::string_view body);

}  // namespace navtracker::sim
