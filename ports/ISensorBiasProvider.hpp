#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>

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

namespace navtracker {

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

// Static lookup of known biases. For deployments where the
// per-sensor mounting offsets are known from calibration
// documentation (e.g., factory survey, plan drawings) and the user
// wants them applied immediately without waiting for the online
// SensorBiasEstimator to converge — or instead of running it. Each
// configured entry is published immediately; unknown keys return
// not-published (zero correction, matches NullBiasProvider).
//
// Use directly via Tracker::setSensorBiasProvider; or combine with a
// SensorBiasEstimator by seeding the estimator's prior, if the
// intent is "known starting point, refine with observations".
class FixedSensorBiasProvider : public ISensorBiasProvider {
 public:
  // Set / overwrite a per-key position bias. Covariance defaults to
  // 1 cm² isotropic (very tight) — the user is asserting they know
  // the offset.
  void setPositionBias(const SensorBiasKey& key,
                       const Eigen::Vector2d& bias_enu_m,
                       const Eigen::Matrix2d& covariance_m2 =
                           Eigen::Matrix2d::Identity() * 1e-4) {
    pos_[key] = PositionBiasEstimate{bias_enu_m, covariance_m2, true};
  }
  void setBearingBias(const SensorBiasKey& key, double bias_rad,
                      double variance_rad2 = 1e-8) {
    brg_[key] = BearingBiasEstimate{bias_rad, variance_rad2, true};
  }

  PositionBiasEstimate positionBias(const SensorBiasKey& key) const override {
    auto it = pos_.find(key);
    return it != pos_.end() ? it->second : PositionBiasEstimate{};
  }
  BearingBiasEstimate bearingBias(const SensorBiasKey& key) const override {
    auto it = brg_.find(key);
    return it != brg_.end() ? it->second : BearingBiasEstimate{};
  }

 private:
  std::unordered_map<SensorBiasKey, PositionBiasEstimate> pos_;
  std::unordered_map<SensorBiasKey, BearingBiasEstimate> brg_;
};

}  // namespace navtracker
