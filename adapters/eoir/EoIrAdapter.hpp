#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

/**
 * One EO/IR camera detection — the input a consumer feeds to `EoIrAdapter`.
 * A relative bearing (deg off own-ship heading) plus a range, each with its
 * own 1-σ. `sensor_track_id`, when present, is the camera's own track number
 * (carried through as a hint, not the fusion key).
 */
struct CameraDetection {
  Timestamp time;
  double bearing_relative_deg{0.0};
  double range_m{0.0};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  std::optional<std::int32_t> sensor_track_id;
  std::string source_id{"eo_ir"};
};

/**
 * Tuning for `EoIrAdapter`. `heading_std_deg` is the own-ship heading 1-σ
 * folded into each projection; the 0.0 default means "perfect gyro".
 */
struct EoIrAdapterConfig {
  double heading_std_deg{0.0};
};

/**
 * Sensor adapter that projects EO/IR range/bearing detections into ENU
 * `Position2D` measurements in the supplied `Datum` frame. Composes the
 * detection's relative bearing with the latest own-ship pose/heading, folds
 * in own-pose and heading uncertainty, and applies the heading-bias
 * correction from the optional `IHeadingBiasProvider`. Validates at the edge
 * (invariant #6): non-positive / non-finite ranges and non-finite bearings
 * are rejected before projection.
 */
class EoIrAdapter : public ISensorAdapter {
 public:
  EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              EoIrAdapterConfig cfg = {},
              const IHeadingBiasProvider* bias_provider = nullptr);

  /**
   * Validate detection `d` and, if plausible and an own-ship pose is
   * available at its time, project it to ENU and buffer a measurement.
   */
  void ingest(const CameraDetection& d);
  /** Drain and return all measurements buffered since the last poll. */
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  EoIrAdapterConfig cfg_;
  const IHeadingBiasProvider* bias_provider_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
