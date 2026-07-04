#pragma once

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

/**
 * Per-sensor 1-sigma noise magnitudes used to synthesize a fallback
 * measurement covariance when a sensor reports none. Only the fields
 * relevant to a given MeasurementModel are consulted (position σ for
 * absolute-position models, range/bearing σ for range-bearing models).
 */
struct PerSensorCov {
  double sigma_pos_m{0.0};
  double sigma_range_m{0.0};
  double sigma_bearing_rad{0.0};
};

/**
 * Table of fallback measurement noise, one PerSensorCov per (sensor,
 * measurement-model) combination the library knows how to synthesize.
 * Used at the edges to fill missing covariance so the tracker always has
 * an R; construct via pessimisticSensorDefaults() and override fields for
 * which the operator has real sensor specs.
 */
struct SensorDefaults {
  PerSensorCov ais_position;
  PerSensorCov arpa_tll_position;
  PerSensorCov arpa_ttm_range_bearing;
  PerSensorCov eoir_range_bearing;
  PerSensorCov eoir_bearing_only;
  PerSensorCov cooperative_position;  // fleet-partner GNSS fix

  /**
   * Build the fallback R matrix for the given (sensor, model) pair, laid
   * out to match the model's value vector. Returns an empty matrix when no
   * default is defined for the pair.
   */
  Eigen::MatrixXd covarianceFor(SensorKind sensor,
                                MeasurementModel model) const;
};

/**
 * Pessimistic, literature-based defaults. Operators with real specs
 * override the relevant fields after constructing this.
 */
SensorDefaults pessimisticSensorDefaults();

/**
 * If m.covariance is empty (size==0), fill it from defaults and set
 * m.covariance_is_default = true. No-op when covariance already set
 * or when defaults don't have a value for the (sensor, model) pair.
 */
void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d);

}  // namespace navtracker
