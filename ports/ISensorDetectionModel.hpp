#pragma once

#include <cmath>
#include <limits>
#include <string>
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

  // Azimuth-sector coverage about the sensor position (cameras, sector
  // radars). Angles use the ENU math convention — atan2(dy, dx), CCW
  // from east — matching Bearing2D. A track whose azimuth from the
  // sensor falls outside center ± width/2 cannot have been detected
  // (miss P_D → 0), same contract as max_range_m. Default width = full
  // circle = legacy omnidirectional behaviour.
  //
  // The sector is fixed in the ENU frame. For a sensor on a rotating
  // platform the entries must be expressed in absolute azimuth (or
  // refreshed with platform heading) — per-measurement sensor attitude
  // is a future extension.
  static constexpr double kFullCircleRad = 6.283185307179586476925287;
  double sector_center_rad{0.0};
  double sector_width_rad{kFullCircleRad};

  // Per-sensor association/birth gate (χ², same convention as the
  // tracker-wide gate_threshold). 0 (default) = use the tracker-wide
  // gate. Rationale (AutoFerry sc5 conveyor diagnosis, 2026-06-12): a
  // track carried by 16 Hz bearings drifts and turns overconfident in
  // range; the sparse radar return then misses the shared gate, births
  // a duplicate, and identity hands off every few seconds. Sparse,
  // informative position sensors can declare a wider recapture gate
  // here without widening the gate for clutter-prone high-rate
  // bearings.
  double gate_threshold{0.0};
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

  // Source-aware lookup. Two physical sensors can share a SensorKind
  // (EO and IR cameras are both SensorKind::EoIr) yet have very
  // different (P_D, λ_C); Measurement::source_id distinguishes them.
  // Default: ignore the source and fall back to the kind-wide entry —
  // models without per-source calibration behave exactly as before.
  virtual DetectionParams paramsFor(SensorKind sensor, MeasurementModel model,
                                    const std::string& /*source_id*/) const {
    return paramsFor(sensor, model);
  }

  // Measurement-resolved lookup — the hot-path entry point used by the
  // per-(leaf, measurement) score step. Virtual so spatially-varying
  // models can resolve λ_C *at the measurement's position* (clutter
  // map, backlog item 5); the default ignores position and reproduces
  // the (sensor, model, source) table lookup exactly.
  virtual DetectionParams paramsFor(const Measurement& z) const {
    return paramsFor(z.sensor, z.model, z.source_id);
  }

  // Detection probability for the MISS branch: "could this sensor have
  // detected a target at track_pos_enu at all?". Coverage-conditioned:
  // 0 beyond the entry's max_range_m about the sensor position or
  // outside its azimuth sector, the table P_D inside. log(1 − 0) = 0 —
  // an out-of-coverage scan charges no miss penalty and leaves IPDA
  // existence untouched.
  double missDetectionProbability(SensorKind sensor, MeasurementModel model,
                                  const Eigen::Vector2d& track_pos_enu,
                                  const Eigen::Vector2d& sensor_pos_enu,
                                  const std::string& source_id = {}) const {
    const DetectionParams p = paramsFor(sensor, model, source_id);
    const Eigen::Vector2d d = track_pos_enu - sensor_pos_enu;
    if (d.norm() > p.max_range_m) return 0.0;
    if (p.sector_width_rad < DetectionParams::kFullCircleRad) {
      const double az = std::atan2(d.y(), d.x());
      // std::remainder wraps the difference to [-π, π].
      const double off = std::remainder(az - p.sector_center_rad,
                                        DetectionParams::kFullCircleRad);
      if (std::abs(off) > 0.5 * p.sector_width_rad) return 0.0;
    }
    return p.probability_of_detection;
  }

  // Per-scan coverage footprint of the sensor that produced a bundle, for
  // COVERAGE-AWARE occupancy decay (R8.4 / Stage 1b-ii): a live-occupancy cell
  // forgets only when it was OBSERVABLE (inside this footprint) and returned
  // empty — absence of returns where the sensor did not look is not evidence of
  // vacancy. `valid == false` (the default) ⇒ full coverage assumed, i.e.
  // universal decay, i.e. the legacy behaviour — bit-identical for synthetic /
  // unwired feeds that supply no footprint. A disc is the degenerate full-circle
  // sector (`sector_width_rad == kFullCircleRad`). Same angle/range convention as
  // DetectionParams: ENU math azimuth (atan2(dy,dx), CCW from east); the
  // footprint is expressed in the feed's ENU frame (the consumer re-expresses it
  // in its own frame; angles are datum-relative — exact for a fixed datum).
  // Under-estimated coverage is the SAFE error direction (no decay ⇒ hazards
  // persist longer, never the reverse).
  struct CoverageSector {
    bool valid{false};
    Eigen::Vector2d sensor_enu{Eigen::Vector2d::Zero()};
    double max_range_m{std::numeric_limits<double>::infinity()};
    double sector_center_rad{0.0};
    double sector_width_rad{DetectionParams::kFullCircleRad};

    // Is ENU point `p` (SAME frame as `sensor_enu`) inside the footprint?
    bool covers(const Eigen::Vector2d& p) const {
      const Eigen::Vector2d d = p - sensor_enu;
      if (d.norm() > max_range_m) return false;
      if (sector_width_rad < DetectionParams::kFullCircleRad) {
        const double az = std::atan2(d.y(), d.x());
        const double off = std::remainder(az - sector_center_rad,
                                          DetectionParams::kFullCircleRad);
        if (std::abs(off) > 0.5 * sector_width_rad) return false;
      }
      return true;
    }
  };

  // One bucket of post-scan evidence, partitioned by (sensor, model).
  // The trailing fields (time, clutter_*, bearings, coverage) feed spatial
  // clutter / occupancy estimators; they are additive so existing aggregate
  // initialisers `{sensor, model, n, positions}` stay valid.
  struct ScanObservation {
    SensorKind sensor;
    MeasurementModel model;
    // Count of returns claimed by NO hypothesis at all (post-solve in
    // MhtTracker). Coarse clutter proxy for non-spatial estimators.
    int num_unassociated;
    std::vector<Eigen::Vector2d> positions; // ENU; empty for pure bearings
    Timestamp time;                       // scan timestamp
    // Spatial clutter evidence, labeled from the tracker's chosen
    // global hypothesis: each entry is a return with clutter weight
    // 1 − r, where r is the existence probability of the hypothesis
    // (track or this-scan birth) that claims it — 1.0 when unclaimed.
    // Weight vectors align with their return lists; an empty weight
    // vector means weight 1.0 per return (binary labeling).
    std::vector<Eigen::Vector2d> clutter_positions;
    std::vector<double> clutter_position_weights;
    // Bearing2D returns: absolute ENU azimuths (rad, atan2 convention),
    // all returns and the clutter-evidence subset.
    std::vector<double> bearings;
    std::vector<double> clutter_bearings;
    std::vector<double> clutter_bearing_weights;
    // Coverage footprint of this bundle's sensor this scan (see CoverageSector).
    // Default (valid=false) ⇒ full coverage ⇒ universal occupancy decay.
    CoverageSector coverage;
  };

  // Feed the scan outcome for adaptation. Fixed models ignore.
  virtual void observe(const std::vector<ScanObservation>& by_sensor) = 0;
};

}  // namespace navtracker
