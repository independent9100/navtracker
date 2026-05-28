#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Fused, non-kinematic attributes. Every field is optional: a non-cooperative
// target without AIS still has a valid Track keyed by TrackId.
struct TrackAttributes {
  std::optional<std::uint32_t> mmsi;
  std::optional<std::string> name;
  std::optional<std::string> vessel_type;
  std::optional<double> length_m;
  std::optional<double> beam_m;
};

// The authoritative fused track. Kinematic state/covariance are populated by
// the estimator (later plan); this defines the carrier type.
struct Track {
  TrackId id;
  Timestamp last_update;
  TrackStatus status{TrackStatus::Tentative};
  Eigen::VectorXd state;       // e.g. [px, py, vx, vy] in ENU
  Eigen::MatrixXd covariance;  // P
  TrackAttributes attributes;
  std::vector<std::string> contributing_sources;  // provenance
};

}  // namespace navtracker
