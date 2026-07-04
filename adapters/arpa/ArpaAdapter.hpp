#pragma once

#include <string_view>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

/**
 * Tuning for `ArpaAdapter`. The defaults preserve the historically
 * hardcoded radar noise so existing consumers see no behavior change; each
 * field is overridable per deployment without a recompile.
 */
struct ArpaAdapterConfig {
  double heading_std_deg{0.0};
  // Position 1-σ (m) applied to TLL (absolute lat/lon) targets and as the
  // range/cross-range measurement σ for TTM range/bearing projection.
  // Defaults preserve the historically hardcoded radar noise; override per
  // deployment without a recompile.
  double position_std_m{50.0};
  // Bearing 1-σ (deg) for TTM range/bearing projection.
  double bearing_std_deg{1.0};
};

/**
 * Parses NMEA 0183 TTM (range/bearing) and TLL (target lat/lon) into
 * Position2D Measurements in the supplied Datum's ENU frame. TTM needs
 * the latest own-ship pose to project relative measurements.
 *
 * PREFER TTM over TLL when the radar offers both. Only the TTM path
 * composes the range-dependent error ellipse (cross-range σ grows with
 * range), applies the heading-bias correction, and separates own-pose
 * error; TLL gets a flat position_std_m and no bias correction because
 * the radar already baked its own heading/GPS into the fix. Full
 * comparison table: docs/sensors/sensor-reference.md §2 "TTM vs TLL".
 * Also set heading_std_deg — the 0.0 default means "perfect gyro".
 *
 * Validates at the edge (invariant #6): implausible ranges/bearings and
 * out-of-range TLL fixes are rejected at parse time before reaching the core.
 */
class ArpaAdapter : public ISensorAdapter {
 public:
  ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              ArpaAdapterConfig cfg = {},
              const IHeadingBiasProvider* bias_provider = nullptr);

  /**
   * Parse one NMEA line (TTM or TLL) stamped at `t` and, if valid, buffer a
   * measurement. Returns true if the line was a recognized, plausible
   * sentence that produced a measurement.
   */
  bool ingest(std::string_view line, Timestamp t);
  /** Drain and return all measurements buffered since the last poll. */
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  ArpaAdapterConfig cfg_;
  const IHeadingBiasProvider* bias_provider_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
