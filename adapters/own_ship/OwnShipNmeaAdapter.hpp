#pragma once

#include <string_view>

#include "adapters/own_ship/OwnShipProvider.hpp"
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
struct OwnShipNmeaAdapterConfig {
  double uere_m{5.0};
  bool enable_adaptive_uere{false};
  UereEstimatorConfig uere_estimator_cfg{};
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
  // Equirectangular reference, captured on the first GGA fix. Used only
  // to feed the UereEstimator meter-scale inputs; not exposed externally.
  bool enu_ref_set_{false};
  double enu_ref_lat_deg_{0.0};
  double enu_ref_lon_deg_{0.0};
};

}  // namespace navtracker
