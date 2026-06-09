#pragma once

#include <algorithm>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {

// Fixed per-sensor detection model. A table of (P_D, λ_C) entries
// keyed by (SensorKind, MeasurementModel); anything not in the table
// falls back to `defaults`.
//
// Use the default-only constructor (or omit overrides) to reproduce the
// legacy single-sensor behaviour — same constant for every measurement,
// bit-identical to a single fixed λ_C / P_D.
class FixedSensorDetectionModel : public ISensorDetectionModel {
 public:
  using Key = std::tuple<SensorKind, MeasurementModel>;

  explicit FixedSensorDetectionModel(DetectionParams defaults)
      : defaults_(defaults) {}

  FixedSensorDetectionModel(DetectionParams defaults,
                            std::map<Key, DetectionParams> table)
      : defaults_(defaults), table_(std::move(table)) {}

  // Set or replace one per-sensor entry. Useful for builders that
  // construct the model incrementally.
  void set(SensorKind sensor, MeasurementModel model, DetectionParams p) {
    table_[Key{sensor, model}] = p;
  }

  DetectionParams paramsFor(const Measurement& z) const override {
    const auto it = table_.find(Key{z.sensor, z.model});
    return (it == table_.end()) ? defaults_ : it->second;
  }

  void observe(const std::vector<ScanObservation>&) override {}

  const DetectionParams& defaults() const { return defaults_; }

 private:
  DetectionParams defaults_;
  std::map<Key, DetectionParams> table_;
};

// Adaptive per-sensor detection model. Per (sensor, model) bucket:
//
//   rate^(s)  ← (1−α)·rate^(s) + α · num_unassociated^(s)        (EWMA)
//   area^(s)  = bounding-box area of positions reported by sensor s
//   λ_C^(s)   = clamp(rate^(s) / area^(s), [min_density, max_density])
//
// Querying paramsFor returns the bucket's current (P_D, λ_C^(s)).
// P_D is fixed per sensor (carried in the bucket's init); online P_D
// estimation is out of scope here — it requires either per-track
// existence (JIPDA recursion) or per-sensor truth, neither of which
// we have yet.
//
// Bearing-only measurements: 2-d bounding-box area is meaningless for
// pure bearings (rad^-1 ≠ m^-2). For Bearing2D buckets we therefore
// keep λ_C frozen at the bucket's init. A proper bearing-only rate
// estimator needs an angular FOV — slot for later.
//
// Falls back to `defaults_` for any sensor that has not been seen yet
// (no bucket exists). After the first observe() for a sensor, the
// bucket holds the per-sensor adaptive estimate.
class AdaptiveSensorDetectionModel : public ISensorDetectionModel {
 public:
  AdaptiveSensorDetectionModel(DetectionParams defaults,
                               double alpha = 0.1,
                               double min_density = 1e-6,
                               double max_density = 1.0)
      : defaults_(defaults),
        alpha_(alpha),
        min_density_(min_density),
        max_density_(max_density) {}

  // Pre-seed a per-sensor init (and P_D) explicitly. Otherwise the
  // bucket inherits `defaults_` on first observation.
  void setSensorInit(SensorKind sensor, MeasurementModel model,
                     DetectionParams init) {
    auto& b = bucket(sensor, model);
    b.params = init;
  }

  DetectionParams paramsFor(const Measurement& z) const override {
    const auto it = buckets_.find(Key{z.sensor, z.model});
    return (it == buckets_.end()) ? defaults_ : it->second.params;
  }

  void observe(const std::vector<ScanObservation>& by_sensor) override {
    for (const auto& s : by_sensor) {
      Bucket& b = bucket(s.sensor, s.model);
      // EWMA on the unassociated count regardless of sensor type.
      b.rate = (1.0 - alpha_) * b.rate +
               alpha_ * static_cast<double>(std::max(0, s.num_unassociated));
      // For position-bearing measurements, also grow the bounding box
      // and refresh λ_C. Pure-bearing sensors leave λ_C at its init.
      if (s.model == MeasurementModel::Bearing2D) continue;
      for (const auto& p : s.positions) {
        b.xmin = std::min(b.xmin, p.x());
        b.xmax = std::max(b.xmax, p.x());
        b.ymin = std::min(b.ymin, p.y());
        b.ymax = std::max(b.ymax, p.y());
      }
      const double w = b.xmax - b.xmin;
      const double h = b.ymax - b.ymin;
      const double area = w * h;
      if (area > 1.0) {
        double d = b.rate / area;
        d = std::clamp(d, min_density_, max_density_);
        b.params.clutter_intensity = d;
      }
    }
  }

  // Diagnostics — current λ_C for one (sensor, model) bucket, or the
  // default if no bucket exists yet.
  double densityFor(SensorKind sensor, MeasurementModel model) const {
    const auto it = buckets_.find(Key{sensor, model});
    return (it == buckets_.end()) ? defaults_.clutter_intensity
                                  : it->second.params.clutter_intensity;
  }

 private:
  using Key = std::tuple<SensorKind, MeasurementModel>;
  struct Bucket {
    DetectionParams params;
    double rate = 0.0;
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
  };

  Bucket& bucket(SensorKind s, MeasurementModel m) {
    const Key k{s, m};
    auto it = buckets_.find(k);
    if (it == buckets_.end()) {
      it = buckets_.emplace(k, Bucket{defaults_, 0.0,
                                       std::numeric_limits<double>::infinity(),
                                      -std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::infinity(),
                                      -std::numeric_limits<double>::infinity()})
               .first;
    }
    return it->second;
  }

  DetectionParams defaults_;
  double alpha_;
  double min_density_;
  double max_density_;
  std::map<Key, Bucket> buckets_;
};

}  // namespace navtracker
