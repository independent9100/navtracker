#pragma once

#include <cstdint>

namespace navtracker {

// Which physical sensor class produced a measurement.
enum class SensorKind { Unknown, Ais, ArpaTtm, ArpaTll, EoIr, OwnShip, Lidar };

// Track lifecycle states. Default-constructs to Tentative.
enum class TrackStatus { Tentative, Confirmed, Coasting, Deleted };

// How a Measurement's value/covariance vectors are laid out.
enum class MeasurementModel { Position2D, PositionVelocity2D, RangeBearing2D };

// Stable internal track identity (primary key, never reused).
struct TrackId {
  std::uint64_t value{0};
  friend bool operator==(TrackId a, TrackId b) { return a.value == b.value; }
  friend bool operator!=(TrackId a, TrackId b) { return a.value != b.value; }
  friend bool operator<(TrackId a, TrackId b) { return a.value < b.value; }
};

}  // namespace navtracker
