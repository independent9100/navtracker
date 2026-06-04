#include "core/collision/CpaOwnShip.hpp"

namespace navtracker {

Track synthesizeOwnShipTrack(const OwnShipPose& pose,
                             Timestamp t,
                             const OwnShipProvider& provider) {
  Track tr;
  tr.id = TrackId{0};
  tr.last_update = t;
  tr.status = TrackStatus::Confirmed;

  const Eigen::Vector3d enu = provider.datum().toEnu({pose.lat_deg, pose.lon_deg, 0.0});
  tr.state.resize(4);
  tr.state << enu.x(), enu.y(), pose.velocity_enu.x(), pose.velocity_enu.y();

  tr.covariance = Eigen::Matrix4d::Zero();
  const double sigma_pos = pose.position_std_m;
  const double pp = sigma_pos * sigma_pos;
  tr.covariance(0, 0) = pp;
  tr.covariance(1, 1) = pp;
  const double sigma_v = pose.velocity_is_valid ? pose.velocity_std_m_per_s : 0.0;
  const double vv = sigma_v * sigma_v;
  tr.covariance(2, 2) = vv;
  tr.covariance(3, 3) = vv;

  return tr;
}

}  // namespace navtracker
