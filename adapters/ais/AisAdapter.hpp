#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

/**
 * One decoded AIS dynamic position report — the input a consumer feeds to
 * `AisAdapter`. Carries an absolute WGS84 fix plus the reporting vessel's
 * MMSI and an accuracy flag; the adapter turns it into an ENU `Position2D`
 * `Measurement`.
 */
struct AisDynamicReport {
  Timestamp time;
  std::uint32_t mmsi{0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  bool high_accuracy{false};
  std::string source_id{"ais"};

  // Target-reported kinematics (backlog #20). All optional: leave unset for a
  // report that does not carry the field, or set the AIS "not available"
  // sentinel value and the adapter will drop it at the edge (invariant #6):
  //   - sog_knots: speed over ground, knots ([0, 102.2]; 1023 sentinel dropped).
  //   - cog_deg:   course over ground, deg true ([0, 360); 3600 sentinel).
  //   - heading_deg: true heading, deg ([0, 360); 511 sentinel dropped).
  //   - nav_status: AIS navigational status code (0..15; 15 = undefined/default,
  //                 dropped). 1 = at anchor, 5 = moored — the "never suppress a
  //                 self-declared vessel" cue (ADR 0002 / R3).
  // SOG/COG become PositionVelocity2D measurement content; heading/nav_status
  // are attribute/corroboration hints. AIS is an independent witness, so all of
  // these are legitimate (contrast ARPA TTM speed/course — guide §3).
  std::optional<double> sog_knots;
  std::optional<double> cog_deg;
  std::optional<double> heading_deg;
  std::optional<std::uint8_t> nav_status;
};

/**
 * Sensor adapter that converts AIS dynamic reports into ENU `Position2D`
 * measurements in the supplied `Datum` frame. Validates at the edge
 * (invariant #6): implausible / sentinel / NaN fixes — e.g. AIS lat 91° /
 * lon 181° "position not available" — are rejected before they can become
 * phantom tracks. High-accuracy fixes get a tighter position σ than
 * standard ones; the MMSI is attached as a non-fusion identity hint.
 */
class AisAdapter : public ISensorAdapter {
 public:
  explicit AisAdapter(geo::Datum datum);

  /** Validate report `r` and, if plausible, buffer an ENU `Position2D` measurement. */
  void ingest(const AisDynamicReport& r);
  /** Drain and return all measurements buffered since the last poll. */
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
