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

  // Optional ensemble carrier used by ensemble-based estimators (particle
  // filter today; IMM later). The Gaussian (state, covariance) above remains
  // the canonical kinematic snapshot consumed by gating / association / sinks.
  Eigen::MatrixXd particles;        // n_state x N_particles, empty if unused
  Eigen::VectorXd particle_weights; // N_particles, sums to 1, empty if unused

  // Optional ensemble carrier used by IMM. `imm_means` is n_state × K
  // (one column per model), `imm_covariances` is a vector of K n_state × n_state
  // matrices, `imm_mode_probabilities` is the K-vector of mode probabilities
  // (sums to 1). The Gaussian (state, covariance) above is the mixture's
  // moment-matched projection consumed by gating / association / sinks.
  Eigen::MatrixXd imm_means;
  std::vector<Eigen::MatrixXd> imm_covariances;
  Eigen::VectorXd imm_mode_probabilities;

  // Per-cycle provenance for downstream components (e.g. bias estimator).
  // Populated by Tracker when a measurement updates this track. Consumers
  // read after a cycle completes; they are responsible for clearing.
  struct SourceTouch {
    SensorKind sensor{SensorKind::Unknown};
    std::string source_id;
    Timestamp time;
    Eigen::Vector2d value_enu{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
    Eigen::Vector2d sensor_position_enu{Eigen::Vector2d::Zero()};
  };
  std::vector<SourceTouch> recent_contributions;
};

}  // namespace navtracker
