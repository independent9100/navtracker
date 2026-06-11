#pragma once

#include <limits>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

// Detection-model parameters for one sensor: probability of detection
// and spatial clutter intensity. λ_C is expressed in the *measurement-
// space* units appropriate for the sensor's MeasurementModel:
//
//   Position2D       → m^-2     (2-d ENU position)
//   PositionVelocity → m^-2     (PV measurements; clutter still over the
//                                 2-d position fold)
//   RangeBearing2D   → (m·rad)^-1
//   Bearing2D        → rad^-1
//
// This is the textbook formulation: the MHT/JIPDA branch score
//   s = log P_D + log p(z | x) − log λ_C
// is only dimensionally consistent if λ_C and p(z|x) share units; and
// p(z|x) lives in the measurement's natural space. A single global
// scalar λ_C therefore cannot be correct across mixed sensors.
struct DetectionParams {
  double probability_of_detection;
  double clutter_intensity;
  // Coverage radius (metres) about the sensor position. Beyond it the
  // sensor cannot have detected anything, so the miss branch charges no
  // penalty (effective P_D → 0). Infinite by default — position-
  // independent P_D, the legacy behaviour for sensors without coverage
  // information.
  double max_range_m{std::numeric_limits<double>::infinity()};
};

// Per-sensor detection model. Strategy: at every per-measurement score
// step the tracker asks `paramsFor(z)` to get the (P_D, λ_C) pair for
// *this* sensor in *this* sensor's units. Online variants update from
// the per-scan outcome via `observe(...)`.
//
// Rationale: this is the multi-sensor JIPDA / PMBM port. It subsumes the
// earlier single-sensor IClutterModel:
//  - FixedSensorDetectionModel(defaults) reproduces a single-sensor
//    constant λ_C.
//  - FixedSensorDetectionModel with a per-(sensor,model) table is the
//    correct multi-sensor formulation.
//  - AdaptiveSensorDetectionModel runs a per-bucket EWMA — no more
//    cross-sensor pollution (a noisy camera doesn't lift the radar
//    estimate).
class ISensorDetectionModel {
 public:
  virtual ~ISensorDetectionModel() = default;

  // Lookup (P_D, λ_C) for one (sensor, model) key. MUST be O(1) —
  // called once per (leaf, gated-measurement) pair inside
  // TrackTree::branch, and once per distinct scan sensor for the miss
  // branch.
  virtual DetectionParams paramsFor(SensorKind sensor,
                                    MeasurementModel model) const = 0;

  // Convenience: lookup keyed by a measurement's (sensor, model).
  DetectionParams paramsFor(const Measurement& z) const {
    return paramsFor(z.sensor, z.model);
  }

  // Detection probability for the MISS branch: "could this sensor have
  // detected a target at track_pos_enu at all?". Coverage-conditioned:
  // 0 beyond the entry's max_range_m about the sensor position, the
  // table P_D inside. log(1 − 0) = 0 — an out-of-coverage scan charges
  // no miss penalty and leaves IPDA existence untouched.
  double missDetectionProbability(SensorKind sensor, MeasurementModel model,
                                  const Eigen::Vector2d& track_pos_enu,
                                  const Eigen::Vector2d& sensor_pos_enu) const {
    const DetectionParams p = paramsFor(sensor, model);
    if ((track_pos_enu - sensor_pos_enu).norm() > p.max_range_m) return 0.0;
    return p.probability_of_detection;
  }

  // One bucket of post-scan evidence, partitioned by (sensor, model).
  struct ScanObservation {
    SensorKind sensor;
    MeasurementModel model;
    int num_unassociated;                 // clutter proxy this scan
    std::vector<Eigen::Vector2d> positions; // ENU; empty for pure bearings
  };

  // Feed the scan outcome for adaptation. Fixed models ignore.
  virtual void observe(const std::vector<ScanObservation>& by_sensor) = 0;
};

}  // namespace navtracker
