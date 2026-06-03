#pragma once

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

struct PerSensorCov {
  double sigma_pos_m{0.0};
  double sigma_range_m{0.0};
  double sigma_bearing_rad{0.0};
};

struct SensorDefaults {
  PerSensorCov ais_position;
  PerSensorCov arpa_tll_position;
  PerSensorCov arpa_ttm_range_bearing;
  PerSensorCov eoir_range_bearing;
  PerSensorCov eoir_bearing_only;

  Eigen::MatrixXd covarianceFor(SensorKind sensor,
                                MeasurementModel model) const;
};

// Pessimistic, literature-based defaults. Operators with real specs
// override the relevant fields after constructing this.
SensorDefaults pessimisticSensorDefaults();

// If m.covariance is empty (size==0), fill it from defaults and set
// m.covariance_is_default = true. No-op when covariance already set
// or when defaults don't have a value for the (sensor, model) pair.
void applyDefaultsIfEmpty(Measurement& m, const SensorDefaults& d);

}  // namespace navtracker
