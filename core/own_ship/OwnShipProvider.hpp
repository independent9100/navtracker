#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/own_ship/IDatumChangeSink.hpp"
#include "core/own_ship/NavInputGuard.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

class INavHealthSink;  // ports/INavHealthSink.hpp

/**
 * One own-ship navigation fix: geodetic position, true heading, and (when
 * available) ENU velocity, each with its 1-σ uncertainty. This is the
 * per-GPS-fix input a consumer pushes via `OwnShipProvider::update`; it also
 * carries the multi-heading-source fields (v3 NMEA wiring) that feed the
 * heading-bias estimator. NaN in a heading field means "not present".
 *
 * ANGLE CONVENTION: every *_heading_* and *_variation_* field below is a
 * MARINE COMPASS angle — degrees, 0 = true north, clockwise-positive, range
 * [0,360). This is the opposite turn direction from the ENU-math frame
 * (β = atan2(dN,dE); 0 = east, counter-clockwise) used inside the tracker;
 * the boundary conversion is φ_enu = π/2 − θ_compass.
 */
struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};              // WGS84 latitude, degrees
  double lon_deg{0.0};              // WGS84 longitude, degrees
  double alt_m{0.0};
  double heading_true_deg{0.0};     // true heading, compass deg (0=N, CW+)
  double position_std_m{0.0};
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
  double velocity_std_m_per_s{0.0};
  bool velocity_is_valid{false};

  // Per-fix 1-σ uncertainty of `heading_true_deg`, in DEGREES (matching the
  // heading field convention above). Optional (#16): when the nav source
  // reports per-fix heading quality that varies (gyro settling, sensor
  // switchover), set it here and every relative-bearing measurement composed
  // through this pose will widen its bearing covariance accordingly — floored
  // by the consumer's config σ so an implausibly tight value cannot make
  // measurements overconfident. Absent ⇒ today's behaviour (heading σ comes
  // only from the consumer's static config), bit-identical.
  std::optional<double> heading_std_deg;

  // Multi-heading-source fields (v3 NMEA wiring). NaN = not present. All are
  // marine compass angles (deg, 0 = true north, clockwise-positive), same
  // convention as heading_true_deg. `magnetic_variation_deg` is added to the
  // magnetic heading to obtain true (east variation positive).
  double gps_true_heading_deg{std::nan("")};
  double gps_true_heading_std_deg{0.0};
  double magnetic_heading_deg{std::nan("")};
  double magnetic_heading_std_deg{0.0};
  double magnetic_variation_deg{std::nan("")};
};

// IDatumChangeSink is defined in core/own_ship/IDatumChangeSink.hpp (included
// above) so sensor adapters can depend on the tiny sink interface without
// pulling in the whole provider.

/**
 * Policy controlling when OwnShipProvider auto-recenters its working
 * datum onto the current own-ship position.
 */
struct DatumRecenterPolicy {
  bool enable_auto_recenter{true};
  double recenter_threshold_km{30.0};
};

/**
 * Owns and manages the working datum (local tangent plane origin) and a short
 * ring of recent own-ship poses. This is the consumer entry point for
 * navigation data: construct with no datum and it auto-initializes from the
 * first `update(pose)`, auto-recenters when own-ship moves past the policy
 * threshold, and fires registered `IDatumChangeSink`s on each recenter so
 * ENU-cached state stays consistent. The pose history serves time-aligned
 * lookups (`poseAtOrBefore`) for measurement projection.
 */
class OwnShipProvider {
 public:
  /** Library-friendly: no datum. Lazy-init from the first update(). */
  explicit OwnShipProvider(std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  /** Backward-compat: pin the datum explicitly. */
  explicit OwnShipProvider(geo::Datum initial_datum,
                           std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  /**
   * Push one own-ship pose. Initializes the datum on first call and may
   * trigger an auto-recenter (firing registered datum sinks).
   */
  void update(const OwnShipPose& pose);

  /** Most recently pushed pose. */
  std::optional<OwnShipPose> latest() const;

  /**
   * Most recent pose with pose.time <= t. Returns nullopt when the
   * history is empty or every stored pose is strictly newer than t.
   */
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;

  /** Diagnostic: how many poses are currently stored. */
  std::size_t historySize() const { return history_.size(); }

  /**
   * Current working datum. Throws std::runtime_error if no datum has
   * been established yet (no pose pushed and no explicit datum passed
   * to the constructor).
   */
  const geo::Datum& datum() const;
  /** True once a working datum has been established. */
  bool hasDatum() const noexcept;

  /**
   * Register/unregister a sink notified on datum recenter events.
   * Lifetime of the sink is the caller's responsibility; the provider
   * stores only the raw pointer.
   */
  void registerDatumSink(IDatumChangeSink* sink);
  void unregisterDatumSink(IDatumChangeSink* sink);

  /**
   * Wire the fact-free nav-input guard (backlog #18). With a sink set, each
   * `update(pose)` is checked against the previous pose and, if it trips a
   * sanity flag (low-SOG heading, stale gap, position/heading jump), the sink is
   * fired — degrade VISIBLY. The guard never rewrites the pose; the tracker
   * keeps trusting its input (validate at the edge, invariant #6). Nullable sink
   * ⇒ the guard does not run, bit-identical to today.
   */
  void setNavHealthSink(INavHealthSink* sink, NavInputGuardConfig cfg = {});

 private:
  std::deque<OwnShipPose> history_;
  std::size_t history_size_limit_;
  std::optional<geo::Datum> current_datum_;
  DatumRecenterPolicy policy_;
  std::vector<IDatumChangeSink*> sinks_;
  INavHealthSink* nav_sink_{nullptr};       // #18 nav-input guard (nullable)
  NavInputGuardConfig nav_cfg_{};
};

}  // namespace navtracker
