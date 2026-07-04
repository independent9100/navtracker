#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

/**
 * One track update from a remote surveillance station (shore radar / VTS) that
 * ships its OWN tracker's output. This is a **pseudo-measurement** (design spec
 * §13): another tracker's filtered, correlated, lifecycle-managed estimate — not
 * an independent observation. The adapter prices that with R-inflation + rate
 * thinning; it never treats a remote track as a raw detection.
 */
struct RemoteTrackReport {
  Timestamp time;
  // The remote system's own track number. Scoped to ONE station (`source_id`):
  // unique only within it, may be reused after the remote drops/swaps a track.
  // Carried as a hint (never the fusion key) and used to key rate thinning.
  std::int32_t remote_track_id{0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  // Stated position covariance (m²) in the local ENU frame. Left all-zero when
  // the remote system states none — the adapter then falls back to a
  // pessimistic absolute default (never inflation AND default, only one).
  Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Zero()};
  // Opt-in velocity (ENU m/s). Consumed only when the adapter config sets
  // accept_velocity; otherwise ignored and the measurement is position-only.
  bool has_velocity{false};
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d velocity_covariance{Eigen::Matrix2d::Zero()};
  // The remote system's identity for this track, when it carries one. Passed
  // through as a hint; also feeds the circular-AIS guard.
  std::optional<std::uint32_t> mmsi;
  // One `source_id` per remote STATION so multi-station feeds stay disjoint.
  std::string source_id{"remote"};
};

/**
 * Tuning for RemoteTrackAdapter. All fields are config-overridable; the defaults
 * are chosen BLIND (no deployment numbers yet) and biased to the safe direction
 * — see the integration guide. Replace with the shore feed's real numbers when
 * known; nothing here touches code.
 */
struct RemoteTrackAdapterConfig {
  // Scalar multiplier on the remote system's STATED covariance MATRIX (variance,
  // not σ): R_used = r_inflation_factor · R_stated. A pseudo-measurement from
  // another filter is correlated and often overconfident; inflating R prices
  // that. Default ×3 is pessimistic on purpose — over-inflation only wastes a
  // little of the feed's precision, under-inflation is the dangerous direction.
  double r_inflation_factor{3.0};
  // Pessimistic 1-σ position (m) used when a report states NO covariance.
  // Applied INSTEAD of inflation (never both).
  double default_position_std_m{50.0};
  // Pessimistic 1-σ velocity (m/s) for opt-in velocity with none stated.
  double default_velocity_std_mps{3.0};
  // Rate thinning: minimum spacing (s) between accepted updates for a single
  // remote track id. Consecutive filtered outputs are correlated, not
  // independent; one update per interval keeps the independence assumption the
  // tracker relies on honest. Default 2 s (below typical VTS refresh).
  double min_update_interval_s{2.0};
  // Velocity opt-in (extra suspicion). When false (default) velocity fields are
  // ignored and every measurement is Position2D.
  bool accept_velocity{false};
};

/**
 * Converts RemoteTrackReports into ENU measurements in the supplied Datum frame.
 * Validates at the edge (invariant #6): implausible / NaN lat-lon is rejected
 * before it can become a phantom track. Emits `SensorKind::RemoteTrack` (a
 * non-scanning source: excluded from occupancy coverage self-estimation, strong
 * evidence for the suppression veto). Rate-thins per remote track id and inflates
 * R per the config. `sensor_track_id` = remote track id; `mmsi` passed through.
 */
class RemoteTrackAdapter : public ISensorAdapter {
 public:
  explicit RemoteTrackAdapter(geo::Datum datum,
                              RemoteTrackAdapterConfig config = {});

  /** Validate `r`, apply rate thinning, and buffer a pseudo-measurement. */
  void ingest(const RemoteTrackReport& r);
  /** Drain and return all measurements buffered since the last poll. */
  std::vector<Measurement> poll() override;

  // --- Diagnostics (invariant #6 + the circular-AIS deployment guard) ---

  /** Reports dropped at the edge for implausible / NaN lat-lon. */
  std::size_t rejectedCount() const { return rejected_; }
  /** Updates dropped by rate thinning (too soon after the last accepted one). */
  std::size_t thinnedCount() const { return thinned_; }
  /** MMSIs this remote feed has relayed (for the circular-AIS guard). */
  const std::set<std::uint32_t>& seenMmsis() const { return seen_mmsis_; }
  /**
   * Circular-AIS guard: the MMSIs this feed carries that ALSO arrive on a
   * directly-wired raw-AIS channel. A non-empty result means the same
   * transmission is being counted twice — the consumer must pick one path per
   * vessel or inflate for the correlation. The adapter surfaces the overlap; it
   * cannot make that deployment decision silently. See the integration guide.
   */
  std::vector<std::uint32_t> circularAisMmsis(
      const std::set<std::uint32_t>& raw_ais_mmsis) const;

 private:
  geo::Datum datum_;
  RemoteTrackAdapterConfig config_;
  std::vector<Measurement> buffer_;
  // Rate-thinning state: last accepted time per (source_id, remote_track_id).
  std::map<std::pair<std::string, std::int32_t>, Timestamp> last_emit_;
  std::set<std::uint32_t> seen_mmsis_;
  std::size_t rejected_{0};
  std::size_t thinned_{0};
};

}  // namespace navtracker
