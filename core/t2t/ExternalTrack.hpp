#pragma once

// ExternalTrack — one report from another tracker's track, at one report time.
// This is the T2T input contract, the analog of core/types/Measurement.hpp:
// a consumer constructs ExternalTracks from whatever another tracker emits and
// feeds them to the T2tFuser. Like Measurement, the NMEA/geodetic builders are
// one optional convenience; the canonical path is to fill the struct directly.
//
// (source_tracker_id, source_track_id) is the SOURCE-track key. It is NEVER the
// fused key (architecture invariant 5): the fuser mints its own fused id.
// Identity attributes (MMSI/name/platform) are evidence for association, never
// the key.
//
// Frame: position/velocity are ENU meters / m/s against the SAME shared datum
// as Measurement (built via the OwnShipProvider/datum machinery). Covariances
// are ENU m^2 / (m/s)^2. All internal units SI; geodetic degrees only at the
// makeExternalTrackFromGeodetic boundary.

#include <Eigen/Core>
#include <cstddef>
#include <map>
#include <optional>
#include <string>

#include "core/t2t/Pedigree.hpp"
#include "core/types/Ids.hpp"        // TrackStatus, TrackId
#include "core/types/Timestamp.hpp"  // Timestamp
#include "core/types/Track.hpp"      // TrackAttributes

namespace navtracker {
class OwnShipProvider;  // fwd; only the geodetic builder (in the .cpp) needs it
}

namespace navtracker::t2t {

struct ExternalTrack {
  // --- Required identity + time ---
  std::string source_tracker_id;  // reporting tracker instance; REQUIRED, non-empty
  std::string source_track_id;    // that tracker's local track id; REQUIRED, non-empty
  Timestamp time;                 // report time; REQUIRED (engine advances on this)

  // --- Position (ENU meters vs the shared datum) ---
  Eigen::Vector2d position_enu{Eigen::Vector2d::Zero()};
  // Position covariance, m^2, ENU. All-zero == "unset"; applyExternalDefaults-
  // IfEmpty fills it from pessimisticExternalDefaults() at the edge.
  Eigen::Matrix2d position_cov{Eigen::Matrix2d::Zero()};

  // --- Velocity (optional) ---
  bool velocity_valid{false};
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};  // ENU m/s
  Eigen::Matrix2d velocity_cov{Eigen::Matrix2d::Zero()};  // (m/s)^2, ENU; all-zero == unset

  // --- Hints (never the fusion key) ---
  // Optional lifecycle hint from the source. nullopt == Unknown (the existing
  // TrackStatus enum has no Unknown value; optional models "not stated").
  std::optional<TrackStatus> source_status;
  // MMSI / name / platform_id hints, same semantics as everywhere: evidence.
  TrackAttributes attributes;
  // Per-track pedigree OVERRIDE. nullopt == "use the fuser-registered pedigree
  // for this source, else all-Unknown". An absent pedigree therefore behaves
  // IDENTICALLY to an explicit all-Unknown pedigree (see effectivePedigree).
  std::optional<SourcePedigree> pedigree;

  // Diagnostic: set true by applyExternalDefaultsIfEmpty when the POSITION
  // covariance came from the pessimistic default rather than a real sensor.
  bool covariance_is_pessimistic_default{false};
};

// The pedigree to use for THIS report, considered in isolation: the per-track
// override if present, else all-Unknown. (The fuser layers its per-source
// registered pedigree in between; this pure helper encodes only the
// "absent == all-Unknown" invariant the contract must guarantee.)
inline SourcePedigree effectivePedigree(const ExternalTrack& t) {
  return t.pedigree.value_or(SourcePedigree{});
}

// ---------------------------------------------------------------------------
// Edge defaults (mirror core/types/SensorDefaults.hpp)
// ---------------------------------------------------------------------------

// Pessimistic 1-sigma fallbacks used when a source states no covariance. These
// are deliberately loose: an unknown-quality external estimate should widen the
// fused covariance, never shrink it. Operators with real specs override.
struct ExternalTrackDefaults {
  double position_std_m{50.0};    // 1-sigma position, meters
  double velocity_std_mps{3.0};   // 1-sigma velocity, m/s
};

inline ExternalTrackDefaults pessimisticExternalDefaults() { return {}; }

// If the position covariance is unset (all-zero), fill it with
// diag(std^2) from `d` and set covariance_is_pessimistic_default = true. Same
// for the velocity covariance when velocity_valid and its covariance is unset
// (this does NOT set the position flag). No-op on already-set covariances.
void applyExternalDefaultsIfEmpty(
    ExternalTrack& t, const ExternalTrackDefaults& d = pessimisticExternalDefaults());

// ---------------------------------------------------------------------------
// Edge validation (architecture invariant 6: validate at the edges)
// ---------------------------------------------------------------------------

// Stateless field validation: non-empty required ids; finite position/velocity;
// finite and (if set) positive-definite covariances. An all-zero covariance is
// the accepted "unset" sentinel (apply defaults before relying on it). Returns
// false and, if `reason` is non-null, writes a short explanation. Does NOT
// check timestamp ordering — that is per-source and stateful (see
// PerSourceStaleGuard).
bool validateExternalTrack(const ExternalTrack& t, std::string* reason = nullptr);

// Per-source high-water stale guard. Each source tracker has its own clock and
// latency, so out-of-order detection is tracked PER source_tracker_id — an old
// report from a slow source must not be judged against a fast source's clock.
// Mirrors the Tracker high-water pattern and its staleDropped() counter.
// Per-instance state; never global.
class PerSourceStaleGuard {
 public:
  explicit PerSourceStaleGuard(bool reject_stale = true) : reject_stale_(reject_stale) {}

  // Accept `t` if its time is >= that source's high-water mark (advancing the
  // mark) and return true; if it is older and rejection is enabled, count a
  // drop and return false. With rejection disabled, always accepts (and still
  // advances the mark on forward motion).
  bool accept(const ExternalTrack& t);

  std::size_t staleDropped() const { return stale_dropped_; }

 private:
  bool reject_stale_;
  std::size_t stale_dropped_{0};
  std::map<std::string, Timestamp> high_water_;  // per source_tracker_id
};

// ---------------------------------------------------------------------------
// Builders
// ---------------------------------------------------------------------------

// Absolute-ENU builder (the direct path when you already have ENU meters).
// Pass Eigen::Matrix2d::Zero() for position_cov to request the pessimistic
// default (apply it with applyExternalDefaultsIfEmpty).
ExternalTrack makeExternalTrackFromEnu(
    std::string source_tracker_id, std::string source_track_id, Timestamp t,
    Eigen::Vector2d position_enu, Eigen::Matrix2d position_cov,
    TrackAttributes attributes = {},
    std::optional<SourcePedigree> pedigree = std::nullopt);

// Geodetic builder: projects (lat_deg, lon_deg) to ENU via the provider's
// current datum, so a consumer with lat/lon never touches the datum by hand.
// Requires provider.hasDatum() (push >= 1 OwnShipPose first); returns nullopt
// if no datum is established yet (caller drops or buffers, like the sensor
// builders do). position_cov is ENU m^2 (already in the local tangent plane);
// pass Zero() to request the pessimistic default.
std::optional<ExternalTrack> makeExternalTrackFromGeodetic(
    std::string source_tracker_id, std::string source_track_id, Timestamp t,
    double lat_deg, double lon_deg, Eigen::Matrix2d position_cov,
    const OwnShipProvider& provider, TrackAttributes attributes = {},
    std::optional<SourcePedigree> pedigree = std::nullopt);

}  // namespace navtracker::t2t
