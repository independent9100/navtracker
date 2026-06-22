#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
  Eigen::Vector2d velocity_enu{Eigen::Vector2d::Zero()};
  double velocity_std_m_per_s{0.0};
  bool velocity_is_valid{false};

  // Multi-heading-source fields (v3 NMEA wiring). NaN = not present.
  double gps_true_heading_deg{std::nan("")};
  double gps_true_heading_std_deg{0.0};
  double magnetic_heading_deg{std::nan("")};
  double magnetic_heading_std_deg{0.0};
  double magnetic_variation_deg{std::nan("")};
};

// Notified when OwnShipProvider replaces its working datum (e.g. when
// the ship has moved far enough that the local tangent plane needs to
// be re-anchored). Consumers that hold state in ENU coordinates should
// react to this event to keep their state consistent with the new frame.
class IDatumChangeSink {
 public:
  virtual ~IDatumChangeSink() = default;
  virtual void onDatumRecentered(const geo::Datum& old_datum,
                                 const geo::Datum& new_datum) = 0;
};

// Policy controlling when OwnShipProvider auto-recenters its working
// datum onto the current own-ship position.
struct DatumRecenterPolicy {
  bool enable_auto_recenter{true};
  double recenter_threshold_km{30.0};
};

class OwnShipProvider {
 public:
  // Library-friendly: no datum. Lazy-init from the first update().
  explicit OwnShipProvider(std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  // Backward-compat: pin the datum explicitly.
  explicit OwnShipProvider(geo::Datum initial_datum,
                           std::size_t history_size = 16,
                           DatumRecenterPolicy policy = {});

  void update(const OwnShipPose& pose);

  // Most recently pushed pose.
  std::optional<OwnShipPose> latest() const;

  // Most recent pose with pose.time <= t. Returns nullopt when the
  // history is empty or every stored pose is strictly newer than t.
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;

  // Diagnostic: how many poses are currently stored.
  std::size_t historySize() const { return history_.size(); }

  // Current working datum. Throws std::runtime_error if no datum has
  // been established yet (no pose pushed and no explicit datum passed
  // to the constructor).
  const geo::Datum& datum() const;
  bool hasDatum() const noexcept;

  // Register/unregister a sink notified on datum recenter events.
  // Lifetime of the sink is the caller's responsibility; the provider
  // stores only the raw pointer.
  void registerDatumSink(IDatumChangeSink* sink);
  void unregisterDatumSink(IDatumChangeSink* sink);

 private:
  std::deque<OwnShipPose> history_;
  std::size_t history_size_limit_;
  std::optional<geo::Datum> current_datum_;
  DatumRecenterPolicy policy_;
  std::vector<IDatumChangeSink*> sinks_;
};

}  // namespace navtracker
