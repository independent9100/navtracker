#pragma once

#include <cstdint>

namespace navtracker {

/**
 * Which physical sensor class produced a measurement.
 * Cooperative: a fleet partner that shares its own platform GNSS fix
 * (e.g. consort vessel, harbor support boat, fleet escort).
 * Acts as a positional anchor alongside AIS — see
 * SensorBiasPairExtractor::isAnchorKind and
 * AisArpaPairExtractor::isAisKind. External identity (call sign, fleet
 * id) belongs in Track::attributes, not the fusion key (invariant 5).
 */
enum class SensorKind {
  Unknown, Ais, ArpaTtm, ArpaTll, EoIr, OwnShip, Lidar, Cooperative
};

/**
 * A non-scanning source reports vessel POSITIONS without sweeping a footprint:
 * AIS and Cooperative (fleet-partner GNSS) fixes. Their returns are real objects,
 * not a swept arc, so they must be excluded from occupancy coverage-sector
 * estimation (self-estimating a "wedge" from them over-claims coverage — the
 * unsafe decay direction, R9 item 1) and are the strongest vessel-evidence for
 * the corroboration suppression veto ("a cooperative/AIS-known platform must
 * track, never be suppressed"). RemoteTrack (shore/VTS pseudo-tracks, R10) joins
 * this set when added. Scanning sources (ArpaTtm/ArpaTll radar, Lidar, and the
 * EoIr camera FOV) are NOT non-scanning.
 */
inline bool isNonScanningSource(SensorKind k) {
  return k == SensorKind::Ais || k == SensorKind::Cooperative;
}

/** Track lifecycle states. Default-constructs to Tentative. */
enum class TrackStatus { Tentative, Confirmed, Coasting, Deleted };

/** How a Measurement's value/covariance vectors are laid out. */
enum class MeasurementModel { Position2D, PositionVelocity2D, RangeBearing2D, Bearing2D };

/** Stable internal track identity (primary key, never reused). */
struct TrackId {
  std::uint64_t value{0};
  friend bool operator==(TrackId a, TrackId b) { return a.value == b.value; }
  friend bool operator!=(TrackId a, TrackId b) { return a.value != b.value; }
  friend bool operator<(TrackId a, TrackId b) { return a.value < b.value; }
};

}  // namespace navtracker
