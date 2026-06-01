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
  std::optional<std::int32_t> sensor_track_id;
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

  int dim() const { return static_cast<int>(value.size()); }
};

}  // namespace navtracker
