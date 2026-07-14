#include "core/sensor_activity/DeclaredSensorActivity.hpp"

#include <cmath>

namespace navtracker {

namespace {
bool inCoverage(const DeclaredSensorActivity::ChannelProfile& p,
                const Eigen::Vector2d& track_pos_enu,
                const Eigen::Vector2d& own_ship_enu) {
  // W2.4a: the declared surveillance profile is mounted on own-ship, so range
  // and azimuth are measured from own-ship's ENU position, NOT the datum origin
  // (own-ship drifts up to the recenter threshold away from the datum). Pass
  // (0,0) for a sensor fixed at the datum. Range gate then optional sector gate.
  const Eigen::Vector2d rel = track_pos_enu - own_ship_enu;
  const double range = rel.norm();
  if (range > p.max_range_m) return false;
  if (p.sector_width_rad < 6.283185307179586) {
    const double az = std::atan2(rel.y(), rel.x());
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
    const Eigen::Vector2d& own_ship_enu,
    std::optional<std::uint32_t> mmsi,
    std::optional<std::uint64_t> platform_id,
    Timestamp last_checked, Timestamp now) const {
  MissOpportunity out;
  const double dt = now.seconds() - last_checked.seconds();
  if (dt <= 0.0) return out;
  // W2.4b: a cooperative channel can only be "overdue" for a track that
  // actually carries a cooperative identity. A radar-only track (no MMSI /
  // platform id) never announces on AIS, so its silence there is not evidence —
  // marking it overdue is what let the identity-blind path hard-delete
  // radar-only tracks. Silence is asymmetric, keyed on identity (spec §2).
  const bool has_cooperative_identity =
      mmsi.has_value() || platform_id.has_value();
  for (const auto& p : profiles_) {
    if (p.kind == ChannelKind::Surveillance) {
      if (p.duty_cycle_sec <= 0.0) continue;
      if (dt < p.duty_cycle_sec) continue;          // mid-sweep: no chance
      if (!inCoverage(p, track_pos_enu, own_ship_enu))
        continue;  // not covered: no chance
      // One completed sweep that covered the track and returned nothing.
      // Aggregate: keep the strongest p_D among surveillance channels.
      out.surveillance_miss = true;
      if (p.p_D > out.p_D) out.p_D = p.p_D;
    } else {  // Cooperative
      if (p.expected_report_interval_sec <= 0.0) continue;
      if (!has_cooperative_identity) continue;  // W2.4b: no identity, no signal
      if (dt > p.expected_report_interval_sec) out.cooperative_overdue = true;
    }
  }
  return out;
}

}  // namespace navtracker
