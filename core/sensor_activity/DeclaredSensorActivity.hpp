#pragma once

#include <vector>

#include "ports/ISensorActivity.hpp"

namespace navtracker {

// Static, deterministic implementation of ISensorActivity (decision §9a:
// declared profile behind the exchangeable port). An adaptive learned
// provider is a future implementation of the same ISensorActivity
// interface (spec roadmap §13.1).
class DeclaredSensorActivity : public ISensorActivity {
 public:
  struct ChannelProfile {
    ChannelKind kind{ChannelKind::Surveillance};
    SensorKind sensor{SensorKind::Unknown};
    // Surveillance:
    double duty_cycle_sec{0.0};
    double max_range_m{0.0};
    double sector_center_rad{0.0};
    double sector_width_rad{6.283185307179586};  // 2*pi = full circle
    double p_D{0.0};
    // Cooperative:
    double expected_report_interval_sec{0.0};
  };

  explicit DeclaredSensorActivity(std::vector<ChannelProfile> profiles)
      : profiles_(std::move(profiles)) {}

  MissOpportunity evaluate(const Eigen::Vector2d& track_pos_enu,
                           std::optional<std::uint32_t> mmsi,
                           std::optional<std::uint64_t> platform_id,
                           Timestamp last_checked,
                           Timestamp now) const override;

  // Return the ChannelKind of the first profile whose sensor field matches,
  // or std::nullopt when no profile covers that sensor.
  std::optional<ChannelKind> channelKindFor(SensorKind sensor) const override;

 private:
  std::vector<ChannelProfile> profiles_;
};

}  // namespace navtracker
