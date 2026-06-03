#include "core/collision/CpaOwnShip.hpp"

namespace navtracker {

Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             const Eigen::Vector2d& velocity_enu,
                             double sigma_pos_m,
                             Timestamp t,
                             const geo::Datum& datum) {
  Track tr;
  tr.id = TrackId{0};
  tr.last_update = t;
  tr.status = TrackStatus::Confirmed;

  const Eigen::Vector3d enu = datum.toEnu({pose.lat_deg, pose.lon_deg, 0.0});
  tr.state.resize(4);
  tr.state << enu.x(), enu.y(), velocity_enu.x(), velocity_enu.y();

  tr.covariance = Eigen::Matrix4d::Zero();
  const double pp = sigma_pos_m * sigma_pos_m;
  tr.covariance(0, 0) = pp;
  tr.covariance(1, 1) = pp;
  // velocity covariance zero per v1 decision (caller knows velocity).

  return tr;
}

}  // namespace navtracker
