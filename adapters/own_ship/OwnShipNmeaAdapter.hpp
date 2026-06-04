#pragma once

#include <string_view>

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/own_ship/OwnShipVelocityEstimator.hpp"
#include "core/own_ship/UereEstimator.hpp"

namespace navtracker {

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

 private:
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
};

}  // namespace navtracker
