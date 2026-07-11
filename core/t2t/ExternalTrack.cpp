#include "core/t2t/ExternalTrack.hpp"

#include <cmath>
#include <utility>

#include "core/estimation/MeasurementModels.hpp"  // isMeasurementCovariancePsd
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/own_ship/OwnShipProvider.hpp"

namespace navtracker::t2t {

void applyExternalDefaultsIfEmpty(ExternalTrack& t, const ExternalTrackDefaults& d) {
  if (t.position_cov.isZero()) {
    const double var = d.position_std_m * d.position_std_m;
    t.position_cov = Eigen::Matrix2d::Identity() * var;
    t.covariance_is_pessimistic_default = true;
  }
  if (t.velocity_valid && t.velocity_cov.isZero()) {
    const double var = d.velocity_std_mps * d.velocity_std_mps;
    t.velocity_cov = Eigen::Matrix2d::Identity() * var;
    // Velocity default deliberately does NOT set the position-covariance
    // diagnostic flag — that flag is about the position estimate.
  }
}

bool validateExternalTrack(const ExternalTrack& t, std::string* reason) {
  const auto fail = [&](const char* r) {
    if (reason) *reason = r;
    return false;
  };
  if (t.source_tracker_id.empty()) return fail("empty source_tracker_id");
  if (t.source_track_id.empty()) return fail("empty source_track_id");
  if (!t.position_enu.allFinite()) return fail("non-finite position");
  if (!t.position_cov.allFinite()) return fail("non-finite position covariance");
  // A non-zero covariance must be positive-definite; all-zero is the accepted
  // "unset" sentinel (defaults fill it before it is relied on).
  if (!t.position_cov.isZero() &&
      !isMeasurementCovariancePsd(Eigen::MatrixXd(t.position_cov)))
    return fail("non-PSD position covariance");
  if (t.velocity_valid) {
    if (!t.velocity_enu.allFinite()) return fail("non-finite velocity");
    if (!t.velocity_cov.allFinite()) return fail("non-finite velocity covariance");
    if (!t.velocity_cov.isZero() &&
        !isMeasurementCovariancePsd(Eigen::MatrixXd(t.velocity_cov)))
      return fail("non-PSD velocity covariance");
  }
  return true;
}

bool PerSourceStaleGuard::accept(const ExternalTrack& t) {
  auto it = high_water_.find(t.source_tracker_id);
  if (it == high_water_.end()) {
    high_water_.emplace(t.source_tracker_id, t.time);
    return true;
  }
  if (t.time < it->second) {
    if (reject_stale_) {
      ++stale_dropped_;
      return false;
    }
    return true;  // rejection disabled: accept but never rewind the high-water
  }
  it->second = t.time;  // forward-or-equal motion advances the mark
  return true;
}

ExternalTrack makeExternalTrackFromEnu(std::string source_tracker_id,
                                       std::string source_track_id, Timestamp t,
                                       Eigen::Vector2d position_enu,
                                       Eigen::Matrix2d position_cov,
                                       TrackAttributes attributes,
                                       std::optional<SourcePedigree> pedigree) {
  ExternalTrack e;
  e.source_tracker_id = std::move(source_tracker_id);
  e.source_track_id = std::move(source_track_id);
  e.time = t;
  e.position_enu = position_enu;
  e.position_cov = position_cov;
  e.attributes = std::move(attributes);
  e.pedigree = std::move(pedigree);
  return e;
}

std::optional<ExternalTrack> makeExternalTrackFromGeodetic(
    std::string source_tracker_id, std::string source_track_id, Timestamp t,
    double lat_deg, double lon_deg, Eigen::Matrix2d position_cov,
    const OwnShipProvider& provider, TrackAttributes attributes,
    std::optional<SourcePedigree> pedigree) {
  if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg)) return std::nullopt;
  if (lat_deg < -90.0 || lat_deg > 90.0 || lon_deg < -180.0 || lon_deg > 180.0)
    return std::nullopt;
  if (!provider.hasDatum()) return std::nullopt;  // no fix yet: caller buffers
  const geo::Datum& d = provider.datum();
  const Eigen::Vector3d enu = d.toEnu(geo::Geodetic{lat_deg, lon_deg, 0.0});
  return makeExternalTrackFromEnu(
      std::move(source_tracker_id), std::move(source_track_id), t,
      Eigen::Vector2d(enu.x(), enu.y()), position_cov, std::move(attributes),
      std::move(pedigree));
}

}  // namespace navtracker::t2t
