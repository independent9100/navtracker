#pragma once

#include <cstddef>
#include <functional>
#include <string>

#include <Eigen/Core>

#include "core/types/Ids.hpp"

namespace navtracker {

// Primary key of a per-sensor bias estimator: (sensor class, source_id).
// EO and IR cameras share SensorKind::EoIr but have distinct source_ids,
// so two cameras of the same kind get two independent estimators.
struct SensorBiasKey {
  SensorKind sensor{SensorKind::Unknown};
  std::string source_id;

  friend bool operator==(const SensorBiasKey& a, const SensorBiasKey& b) {
    return a.sensor == b.sensor && a.source_id == b.source_id;
  }
  friend bool operator<(const SensorBiasKey& a, const SensorBiasKey& b) {
    if (a.sensor != b.sensor) return a.sensor < b.sensor;
    return a.source_id < b.source_id;
  }
};

// Position-bias snapshot (in ENU metres, applied to Position2D /
// RangeBearing2D measurements). `is_published` is false when the
// estimator has not converged below its publish threshold; consumers
// treat that as "use b = 0" (pre-estimator behaviour).
struct PositionBiasEstimate {
  Eigen::Vector2d bias_enu_m{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance_m2{Eigen::Matrix2d::Zero()};
  bool is_published{false};
};

// Bearing-bias snapshot (added to a camera's reported bearing, rad).
struct BearingBiasEstimate {
  double bias_rad{0.0};
  double variance_rad2{0.0};
  bool is_published{false};
};

// Read-only port. Adapters / Tracker query the current published
// estimate; null providers return zero / not-published.
class ISensorBiasProvider {
 public:
  virtual ~ISensorBiasProvider() = default;
  virtual PositionBiasEstimate positionBias(const SensorBiasKey& key) const = 0;
  virtual BearingBiasEstimate bearingBias(const SensorBiasKey& key) const = 0;
};

// Sentinel implementation: always returns zero, not-published. The
// default for callers that do not wire an estimator.
class NullBiasProvider : public ISensorBiasProvider {
 public:
  PositionBiasEstimate positionBias(const SensorBiasKey&) const override {
    return {};
  }
  BearingBiasEstimate bearingBias(const SensorBiasKey&) const override {
    return {};
  }
};

}  // namespace navtracker

namespace std {
template <>
struct hash<navtracker::SensorBiasKey> {
  std::size_t operator()(const navtracker::SensorBiasKey& k) const noexcept {
    const std::size_t h1 =
        std::hash<int>{}(static_cast<int>(k.sensor));
    const std::size_t h2 = std::hash<std::string>{}(k.source_id);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};
}  // namespace std
