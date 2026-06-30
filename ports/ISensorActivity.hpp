#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Two sensor archetypes (see docs spec §2):
//  - Surveillance (radar/EO/lidar): searches a place on a duty cycle;
//    silence over covered ground is STRONG, symmetric evidence.
//  - Cooperative-announce (AIS, Cooperative channel): the target announces
//    itself; silence is WEAK, asymmetric evidence keyed on identity.
enum class ChannelKind { Surveillance, Cooperative };

// What the misdetection step needs to know for one (channel, track) pair
// at one query time. A pure value; no I/O.
struct MissOpportunity {
  // A surveillance sensor completed a sweep that COVERED this track's
  // predicted position and returned nothing for it -> charge exactly one
  // miss with `p_D`. False for cooperative channels and for surveillance
  // channels that were off / out of coverage / mid-sweep.
  bool surveillance_miss{false};
  double p_D{0.0};  // only meaningful when surveillance_miss == true

  // A cooperative source's own-identity report was overdue at the query
  // time. Raises a comms-loss/stale signal; NEVER changes existence
  // (decision spec §9c).
  bool cooperative_overdue{false};
};

// Nullable port. If unwired, PMBM behaves exactly as before.
class ISensorActivity {
 public:
  virtual ~ISensorActivity() = default;

  // Aggregate over every channel this provider knows. `track_pos_enu` is
  // the track's predicted position; `mmsi`/`platform_id` are its identity
  // hints (either may be empty); `now` and `last_checked` bound the
  // interval being evaluated. Implementations MUST be a pure function of
  // declared profiles + the arguments (no wall-clock, no RNG).
  virtual MissOpportunity evaluate(const Eigen::Vector2d& track_pos_enu,
                                   std::optional<std::uint32_t> mmsi,
                                   std::optional<std::uint64_t> platform_id,
                                   Timestamp last_checked,
                                   Timestamp now) const = 0;

  // Return the declared ChannelKind for the given sensor, or std::nullopt
  // if no profile covers that sensor. Used by the tracker to generalise
  // "is this measurement from a cooperative-announce source?" beyond the
  // hardcoded SensorKind::Cooperative check — any sensor declared as
  // ChannelKind::Cooperative in the profile (e.g. SensorKind::Ais) will
  // update the cooperative-touch timer and trigger stale / retirement logic.
  virtual std::optional<ChannelKind> channelKindFor(SensorKind sensor) const = 0;
};

}  // namespace navtracker
