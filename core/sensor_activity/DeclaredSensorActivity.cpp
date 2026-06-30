#include "core/sensor_activity/DeclaredSensorActivity.hpp"

#include <cmath>

namespace navtracker {

namespace {
bool inCoverage(const DeclaredSensorActivity::ChannelProfile& p,
                const Eigen::Vector2d& track_pos_enu) {
  // Sensor assumed at the ENU origin for the declared profile (own-ship
  // datum). Range gate then optional azimuth-sector gate.
  const double range = track_pos_enu.norm();
  if (range > p.max_range_m) return false;
  if (p.sector_width_rad < 6.283185307179586) {
    const double az = std::atan2(track_pos_enu.y(), track_pos_enu.x());
    const double off = std::remainder(az - p.sector_center_rad,
                                      6.283185307179586);
    if (std::abs(off) > 0.5 * p.sector_width_rad) return false;
  }
  return true;
}
}  // namespace

std::optional<ChannelKind> DeclaredSensorActivity::channelKindFor(
    SensorKind sensor) const {
  for (const auto& p : profiles_) {
    if (p.sensor == sensor) return p.kind;
  }
  return std::nullopt;
}

MissOpportunity DeclaredSensorActivity::evaluate(
    const Eigen::Vector2d& track_pos_enu,
    std::optional<std::uint32_t> /*mmsi*/,
    std::optional<std::uint64_t> /*platform_id*/,
    Timestamp last_checked, Timestamp now) const {
  MissOpportunity out;
  const double dt = now.seconds() - last_checked.seconds();
  if (dt <= 0.0) return out;
  for (const auto& p : profiles_) {
    if (p.kind == ChannelKind::Surveillance) {
      if (p.duty_cycle_sec <= 0.0) continue;
      if (dt < p.duty_cycle_sec) continue;          // mid-sweep: no chance
      if (!inCoverage(p, track_pos_enu)) continue;  // not covered: no chance
      // One completed sweep that covered the track and returned nothing.
      // Aggregate: keep the strongest p_D among surveillance channels.
      out.surveillance_miss = true;
      if (p.p_D > out.p_D) out.p_D = p.p_D;
    } else {  // Cooperative
      if (p.expected_report_interval_sec <= 0.0) continue;
      if (dt > p.expected_report_interval_sec) out.cooperative_overdue = true;
    }
  }
  return out;
}

}  // namespace navtracker
