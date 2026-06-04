#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/own_ship/OwnShipVelocityEstimator.hpp"
#include "core/own_ship/UereEstimator.hpp"

namespace navtracker {

class HeadingBiasEstimator;

// Config for OwnShipNmeaAdapter. UERE (User Equivalent Range Error) is the
// per-satellite ranging error used to derive a horizontal position sigma from
// the GGA HDOP: sigma_pos = HDOP * uere_m. Default 5 m matches the
// commonly-cited consumer-GPS value.
//
// When enable_adaptive_uere is true, the adapter additionally runs a
// UereEstimator over GGA-derived local-meter offsets (equirectangular
// projection about the first fix). When the estimator publishes, its sigma
// overrides the HDOP * uere_m static path; otherwise the static path applies.
//
// Velocity fields:
//   sigma_sog_m_per_s, sigma_cog_deg     — assumed RMC noise floors used to
//                                          derive sigma_v from a parsed RMC.
//   prefer_rmc_velocity                  — when an RMC is fresh, use it over
//                                          the GGA-derived estimator.
//   rmc_stale_seconds                    — RMC older than this falls back to
//                                          the estimator on the next GGA.
//   enable_velocity_estimator            — feed GGAs into OwnShipVelocityEstimator
//                                          as a fallback when RMC is absent/stale.
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
  bool enable_adaptive_uere{false};
  UereEstimatorConfig uere_estimator_cfg{};
  double sigma_sog_m_per_s{0.5};
  double sigma_cog_deg{1.0};
  bool prefer_rmc_velocity{true};
  double rmc_stale_seconds{5.0};
  bool enable_velocity_estimator{true};
  OwnShipVelocityEstimatorConfig velocity_estimator_cfg{};

  // Multi-heading-source wiring (v3 NMEA). Empty default for talkers
  // preserves backward compat with $GPHDT-as-gyro consumers.
  std::unordered_set<std::string> gps_heading_talkers{};
  double gps_heading_sigma_deg{0.5};
  double magnetic_heading_sigma_deg{0.5};
  double gps_cog_sigma_deg{1.0};
  double magnetic_variation_fallback_deg{std::nan("")};
  double gyro_max_age_s{2.0};
};

// Parses NMEA 0183 GGA (position) and HDT (true heading) into OwnShipPose
// updates on the supplied OwnShipProvider. The caller supplies a full
// Timestamp per ingest.
class OwnShipNmeaAdapter {
 public:
  explicit OwnShipNmeaAdapter(OwnShipProvider& provider,
                              OwnShipNmeaAdapterConfig cfg = {});

  bool ingest(std::string_view line, Timestamp t);

  // Sim hook: sets a sticky position uncertainty applied to subsequently
  // published poses. Used by sim paths that don't emit GGA HDOP. For GGA
  // messages that carry a positive HDOP, the HDOP-derived sigma
  // (HDOP * uere_m) takes precedence for that message; the sticky value
  // is only used as a fallback (e.g. when HDOP is empty or non-positive,
  // and for non-GGA messages).
  void setPositionStd(double sigma_m);

  // Optional. When non-null, the adapter dispatches the appropriate
  // HeadingBiasEstimator::observe(...) overload for each parsed
  // HDG / GPS-talker HDT / RMC.
  void setHeadingBiasEstimator(HeadingBiasEstimator* estimator) {
    bias_estimator_ = estimator;
  }

  // Diagnostics for multi-heading-source dispatch (v3 NMEA).
  std::size_t dispatchedGpsHeading()   const { return d_gps_hdg_; }
  std::size_t dispatchedGpsCog()       const { return d_cog_; }
  std::size_t dispatchedMagnetic()     const { return d_mag_; }
  std::size_t skippedMagNoVariation()  const { return skip_mag_var_; }
  std::size_t skippedGyroStale()       const { return skip_stale_; }

 private:
  void pushGyroSample(Timestamp t, double heading_deg);
  std::optional<double> latestGyroRad(Timestamp t,
                                      double max_age_s) const;
  double gyroRateRadPerSec(Timestamp t, double max_dt_s) const;

  OwnShipProvider& provider_;
  OwnShipNmeaAdapterConfig cfg_;
  double position_std_m_{0.0};
  UereEstimator uere_estimator_;
  OwnShipVelocityEstimator velocity_estimator_;
  // Equirectangular reference, captured on the first GGA fix. Used only
  // to feed the UereEstimator meter-scale inputs; not exposed externally.
  bool enu_ref_set_{false};
  double enu_ref_lat_deg_{0.0};
  double enu_ref_lon_deg_{0.0};

  // Most recent RMC-derived velocity (with timestamp for staleness). Used
  // by the precedence rule applied at each GGA pose composition.
  struct RmcBuffer {
    Timestamp time;
    Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
    double sigma_v_m_per_s{0.0};
    bool has_value{false};
  };
  RmcBuffer rmc_buffer_;

  HeadingBiasEstimator* bias_estimator_{nullptr};

  // Gyro HDT ring (last 4 samples) for rate computation.
  struct GyroSample { Timestamp t; double heading_rad{0.0}; };
  std::array<GyroSample, 4> gyro_history_{};
  std::size_t gyro_history_count_{0};
  std::size_t gyro_history_head_{0};

  // Variation cache (last value seen from HDG or RMC; deg, signed).
  double cached_variation_deg_{std::nan("")};

  // Diagnostic counters.
  std::size_t d_gps_hdg_{0};
  std::size_t d_cog_{0};
  std::size_t d_mag_{0};
  std::size_t skip_mag_var_{0};
  std::size_t skip_stale_{0};
};

}  // namespace navtracker
