#pragma once

#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * === GpsHeadingObservation (gold-standard, no offset) ===
 * r = wrap(gyro_rad - gps_true_heading_rad)
 * R = gps_true_heading_std_rad^2
 * Gate: outlier only. Used with multi-antenna GPS receivers that produce
 * true heading from baseline phase difference.
 */
struct GyroVsGpsHeadingObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double gps_true_heading_rad{0.0};
  double gps_true_heading_std_rad{0.0};
};

/**
 * === GpsCogObservation (needs gating against crab) ===
 * r = wrap(gyro_rad - gps_cog_rad)
 * R = gps_cog_std_rad^2 + cog_crab_budget_rad^2
 * Gates:
 *   C1 sog_mps >= cog_min_sog_mps
 *   C2 |gyro_rate_rad_per_s| <= cog_max_gyro_rate_rad_per_s
 *   C3 outlier on r vs sqrt(R + P_b)
 */
struct GyroVsGpsCogObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double gps_cog_rad{0.0};
  double gps_cog_std_rad{0.0};
  double sog_mps{0.0};
  double gyro_rate_rad_per_s{0.0};
};

/**
 * === MagneticObservation (variation must be supplied) ===
 * r = wrap(gyro_rad - (magnetic_heading_rad + variation_rad))
 * R = magnetic_heading_std_rad^2 + mag_deviation_budget_rad^2
 * Gates:
 *   M1 magnetic_variation_rad must be present (adapter responsibility)
 *   M2 outlier
 */
struct GyroVsMagneticObservation {
  Timestamp time;
  double gyro_rad{0.0};
  double magnetic_heading_rad{0.0};
  double magnetic_heading_std_rad{0.0};
  std::optional<double> magnetic_variation_rad;
};

}  // namespace navtracker
