#pragma once

#include <cstdint>
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
