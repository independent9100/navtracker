#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Opportunistic identity cues from a sensor; never the fusion key.
struct AssociationHints {
  std::optional<std::uint32_t> mmsi;
  // The emitting sensor's own track/target number (e.g. ARPA TTM/TLL target
  // number, EO-IR detector track id). Scoped to that ONE sensor (source_id):
  // unique only within it, never across sensors, and may be reused by the
  // sensor after a track drop or target swap. Currently carried for
  // diagnostics/attribute purposes only — association does not consume it.
  std::optional<std::int32_t> sensor_track_id;
  // Cooperative-channel native id (numeric, settled 2026-06-29). Always
  // set by the Cooperative adapter; assumed unique per fleet member. A
  // strong association prior but still a hint, never the fusion key.
  std::optional<std::uint64_t> platform_id;
};

// Normalized sensor output consumed by the tracker. `value` and `covariance`
// (R) are laid out according to `model`; e.g. Position2D -> [east, north] in
// the working ENU frame with a 2x2 R.
struct Measurement {
  Timestamp time;
  SensorKind sensor{SensorKind::Unknown};
  std::string source_id;
  MeasurementModel model{MeasurementModel::Position2D};
  Eigen::VectorXd value;
  Eigen::MatrixXd covariance;
  AssociationHints hints;

  // Where the sensor was at measurement time, in the ENU frame. Default
  // is the origin (stationary sensor at the datum). For a sensor mounted
  // on a moving platform this is the platform's ENU position at the
  // measurement timestamp.
  Eigen::Vector2d sensor_position_enu{Eigen::Vector2d::Zero()};

  // Own-ship GPS position 1-sigma (m) in effect when this measurement was
  // produced. Carried so downstream consumers (e.g. heading-bias estimator)
  // can budget the GPS noise floor. Default 0 means "no floor known".
  double sensor_position_std_m{0.0};

  // True when the covariance was populated from SensorDefaults rather
  // than from a real sensor uncertainty. Diagnostic only — the tracker
  // behaves identically regardless of this flag.
  bool covariance_is_default{false};

  int dim() const { return static_cast<int>(value.size()); }
};

}  // namespace navtracker
