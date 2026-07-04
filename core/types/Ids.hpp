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
// RemoteTrack (last enumerator) is a shore/VTS station's filtered track output
// ingested as a pseudo-measurement (R10). ALWAYS append new kinds at the end:
// serialized integer values of existing kinds must not shift, and
// sim/SkewInjector.hpp sizes a per-kind array from the enumerator count.
enum class SensorKind {
  Unknown, Ais, ArpaTtm, ArpaTll, EoIr, OwnShip, Lidar, Cooperative, RemoteTrack
};

/**
 * A non-scanning source reports vessel POSITIONS without sweeping a footprint:
 * AIS and Cooperative (fleet-partner GNSS) fixes, and RemoteTrack (a shore/VTS
 * station's filtered pseudo-tracks, R10). Their returns are real objects — or
 * another tracker's estimate of one — not a swept arc, so they must be excluded
 * from occupancy coverage-sector self-estimation (inferring a "wedge" from them
 * over-claims coverage — the unsafe decay direction, R9 item 1) and they are the
 * strongest vessel-evidence for the corroboration suppression veto ("a
 * cooperative/AIS/remote-known platform must track, never be suppressed").
 * Scanning sources (ArpaTtm/ArpaTll radar, Lidar, and the EoIr camera FOV) are
 * NOT non-scanning. NOTE: a remote station may still carry a DECLARED
 * surveillance coverage area (DeclaredSensorActivity) — that is the operator
 * declaring where the feed sees, which is distinct from us self-estimating a
 * swept wedge from point reports (the thing excluded here).
 */
inline bool isNonScanningSource(SensorKind k) {
  return k == SensorKind::Ais || k == SensorKind::Cooperative ||
         k == SensorKind::RemoteTrack;
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
