#pragma once

#include <cstddef>
#include <deque>
#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
};

class OwnShipProvider {
 public:
  explicit OwnShipProvider(std::size_t history_size = 16);

  void update(const OwnShipPose& pose);

  // Most recently pushed pose.
  std::optional<OwnShipPose> latest() const;

  // Most recent pose with pose.time <= t. Returns nullopt when the
  // history is empty or every stored pose is strictly newer than t.
  std::optional<OwnShipPose> poseAtOrBefore(Timestamp t) const;

  // Diagnostic: how many poses are currently stored.
  std::size_t historySize() const { return history_.size(); }

 private:
  std::deque<OwnShipPose> history_;
  std::size_t history_size_limit_;
};

}  // namespace navtracker
