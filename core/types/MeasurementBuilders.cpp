#include "core/types/MeasurementBuilders.hpp"

#include <algorithm>
#include <utility>

#include "core/projection/Projection.hpp"

namespace navtracker {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Shared implementation for the two bearing builders. `heading_combo`
// controls whether the input bearing is RELATIVE (then own-ship heading
// is added to obtain the true bearing) or already TRUE.
enum class BearingKind { Relative, True };

Measurement buildBearingMeasurement(SensorKind sensor,
                                    std::string source_id,
                                    Timestamp t,
                                    double range_m,
                                    double bearing_in_rad,
                                    double range_std_m,
                                    double bearing_std_rad,
                                    BearingKind kind,
                                    const OwnShipProvider& provider,
                                    AssociationHints hints,
                                    double heading_std_floor_deg) {
  Measurement m;
  m.time = t;
  m.sensor = sensor;
  m.source_id = std::move(source_id);
  m.model = MeasurementModel::Position2D;
  m.hints = std::move(hints);
  m.covariance_is_default = false;

  const auto pose_opt = provider.poseAtOrBefore(t);
  if (!pose_opt) {
    // No pose available — return Measurement with empty value/covariance.
    // Caller should drop or buffer.
    return m;
  }
  const OwnShipPose& pose = *pose_opt;

  // Check if the provider has a datum. If not, return empty measurement.
  if (!provider.hasDatum()) {
    return m;
  }

  const double bearing_true_rad =
      (kind == BearingKind::Relative)
          ? bearing_in_rad + pose.heading_true_deg * kDeg2Rad
          : bearing_in_rad;

  const Eigen::Vector3d own_enu_3 =
      provider.datum().toEnu(geo::Geodetic{pose.lat_deg, pose.lon_deg, 0.0});
  const Eigen::Vector2d own_xy(own_enu_3.x(), own_enu_3.y());

  // #16: heading σ composition. The effective heading uncertainty folded into
  // the angular covariance is max(per-pose σ, config floor): the floor is the
  // consumer's static trust bound (a pose claiming implausibly tight heading σ
  // must NOT make bearing measurements overconfident — the floor can only
  // widen, never tighten). When the pose carries no per-fix σ AND the floor is
  // 0 (both defaults), this is 0.0 — exactly the prior behaviour, bit-identical
  // (heading σ then comes only from bearing_std_rad, which the adapters
  // pre-inflate, or the caller supplies). projectRangeBearingToEnu adds it in
  // quadrature with bearing_std_rad on the angular component.
  const double sigma_heading_deg =
      std::max(pose.heading_std_deg.value_or(0.0), heading_std_floor_deg);
  const PointAndCov2D proj = projectRangeBearingToEnu(
      range_m,
      bearing_true_rad,
      range_std_m,
      bearing_std_rad,
      /*sigma_heading_rad=*/sigma_heading_deg * kDeg2Rad,
      pose.position_std_m,
      own_xy);

  m.value = proj.pos_enu;
  m.covariance = proj.cov;
  m.sensor_position_enu = own_xy;
  m.sensor_position_std_m = pose.position_std_m;
  return m;
}

}  // namespace

Measurement makeMeasurementFromRelativeBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double relative_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints,
    double heading_std_floor_deg) {
  return buildBearingMeasurement(sensor,
                                 std::move(source_id),
                                 t,
                                 range_m,
                                 relative_bearing_rad,
                                 range_std_m,
                                 bearing_std_rad,
                                 BearingKind::Relative,
                                 provider,
                                 std::move(hints),
                                 heading_std_floor_deg);
}

Measurement makeMeasurementFromTrueBearing(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    double range_m,
    double true_bearing_rad,
    double range_std_m,
    double bearing_std_rad,
    const OwnShipProvider& provider,
    AssociationHints hints,
    double heading_std_floor_deg) {
  return buildBearingMeasurement(sensor,
                                 std::move(source_id),
                                 t,
                                 range_m,
                                 true_bearing_rad,
                                 range_std_m,
                                 bearing_std_rad,
                                 BearingKind::True,
                                 provider,
                                 std::move(hints),
                                 heading_std_floor_deg);
}

Measurement makeMeasurementFromEnuPosition(
    SensorKind sensor,
    std::string source_id,
    Timestamp t,
    Eigen::Vector2d enu_xy,
    Eigen::Matrix2d covariance,
    AssociationHints hints) {
  Measurement m;
  m.time = t;
  m.sensor = sensor;
  m.source_id = std::move(source_id);
  m.model = MeasurementModel::Position2D;
  m.value = enu_xy;
  // Treat an all-zero 2x2 as the "empty / unknown" sentinel so that the
  // result composes with applyDefaultsIfEmpty(...). A literal zero
  // covariance is unphysical anyway (would imply a perfect measurement),
  // so this overlap is acceptable in practice.
  if (!covariance.isZero()) {
    m.covariance = covariance;
  }
  m.hints = std::move(hints);
  m.covariance_is_default = false;
  return m;
}

}  // namespace navtracker
