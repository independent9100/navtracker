#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

OwnShipProvider::OwnShipProvider(std::size_t history_size)
    : history_size_limit_(history_size > 0 ? history_size : 1) {}

void OwnShipProvider::update(const OwnShipPose& pose) {
  history_.push_back(pose);
  while (history_.size() > history_size_limit_) history_.pop_front();
}

std::optional<OwnShipPose> OwnShipProvider::latest() const {
  if (history_.empty()) return std::nullopt;
  return history_.back();
}

std::optional<OwnShipPose> OwnShipProvider::poseAtOrBefore(Timestamp t) const {
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (!(t < it->time)) return *it;  // it->time <= t
  }
  return std::nullopt;
}

}  // namespace navtracker
