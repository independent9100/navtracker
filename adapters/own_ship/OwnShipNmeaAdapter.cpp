#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"

#include <cmath>
#include <cstdlib>

#include "adapters/util/Nmea.hpp"

namespace navtracker {
namespace {

double parseDdmm(const std::string& s) {
  if (s.empty()) return 0.0;
  const auto dot = s.find('.');
  const std::size_t mm_digits = 2;
  const std::size_t deg_end = (dot == std::string::npos ? s.size() : dot) - mm_digits;
  if (deg_end > s.size()) return 0.0;
  const double deg = std::strtod(s.substr(0, deg_end).c_str(), nullptr);
  const double min = std::strtod(s.substr(deg_end).c_str(), nullptr);
  return deg + min / 60.0;
}

// WGS-84 semi-major axis. Used only as the meter-per-radian scale in the
// local equirectangular projection that feeds the UereEstimator. We do not
// need a true ENU frame here — the estimator is invariant to translation
// and to a coordinate rotation, and an axis-wise linear scaling preserves
// residual variance in meters. (We avoid pulling in geo::Datum to keep
// this adapter dependency-free.)
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

OwnShipNmeaAdapter::OwnShipNmeaAdapter(OwnShipProvider& provider,
                                       OwnShipNmeaAdapterConfig cfg)
    : provider_(provider),
      cfg_(cfg),
      uere_estimator_(cfg.uere_estimator_cfg) {}

void OwnShipNmeaAdapter::setPositionStd(double sigma_m) {
  position_std_m_ = sigma_m;
}

bool OwnShipNmeaAdapter::ingest(std::string_view line, Timestamp t) {
  const auto parsed = parseNmea(line);
  if (!parsed) return false;
  OwnShipPose pose = provider_.latest().value_or(OwnShipPose{});
  pose.time = t;
  pose.position_std_m = position_std_m_;

  if (parsed->formatter == "GGA") {
    if (parsed->fields.size() < 5) return false;
    double lat = parseDdmm(parsed->fields[1]);
    if (parsed->fields[2] == "S") lat = -lat;
    double lon = parseDdmm(parsed->fields[3]);
    if (parsed->fields[4] == "W") lon = -lon;
    pose.lat_deg = lat;
    pose.lon_deg = lon;

    // Adaptive UERE: feed an equirectangular meter offset from the first
    // fix into the sliding-window estimator. The reference latitude is
    // captured on the first GGA and never updated — over the timescale of
    // the estimator's window (8 samples * ~1 s) the cos(lat) scale change
    // is negligible.
    if (cfg_.enable_adaptive_uere) {
      if (!enu_ref_set_) {
        enu_ref_lat_deg_ = lat;
        enu_ref_lon_deg_ = lon;
        enu_ref_set_ = true;
      }
      const double cos_ref = std::cos(enu_ref_lat_deg_ * kDegToRad);
      const double x_m = (lon - enu_ref_lon_deg_) * kDegToRad *
                         kEarthRadiusM * cos_ref;
      const double y_m = (lat - enu_ref_lat_deg_) * kDegToRad * kEarthRadiusM;
      uere_estimator_.observe(t, x_m, y_m);
    }

    // sigma precedence: adaptive (when published) > HDOP * UERE_static
    // (when HDOP > 0) > sticky setter (already in pose.position_std_m).
    double hdop = 0.0;
    if (parsed->fields.size() > 7 && !parsed->fields[7].empty()) {
      hdop = std::strtod(parsed->fields[7].c_str(), nullptr);
    }
    const UereEstimate adaptive = uere_estimator_.current();
    if (cfg_.enable_adaptive_uere && adaptive.is_published) {
      pose.position_std_m = adaptive.sigma_m;
    } else if (hdop > 0.0) {
      pose.position_std_m = hdop * cfg_.uere_m;
    }
    // else: keep the sticky-setter value loaded into pose.position_std_m
    // at the top of this function.
    provider_.update(pose);
    return true;
  }
  if (parsed->formatter == "HDT") {
    if (parsed->fields.empty()) return false;
    pose.heading_true_deg = std::strtod(parsed->fields[0].c_str(), nullptr);
    provider_.update(pose);
    return true;
  }
  return false;
}

}  // namespace navtracker
