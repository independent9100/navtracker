#include "core/own_ship/OwnShipProvider.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "ports/INavHealthSink.hpp"

namespace navtracker {

OwnShipProvider::OwnShipProvider(std::size_t history_size,
                                 DatumRecenterPolicy policy)
    : history_size_limit_(history_size > 0 ? history_size : 1),
      policy_(policy) {}

OwnShipProvider::OwnShipProvider(geo::Datum initial_datum,
                                 std::size_t history_size,
                                 DatumRecenterPolicy policy)
    : history_size_limit_(history_size > 0 ? history_size : 1),
      current_datum_(std::move(initial_datum)),
      policy_(policy) {}

void OwnShipProvider::update(const OwnShipPose& pose) {
  // #18: fact-free nav-input guard at the edge. Evaluate the incoming pose
  // against the previous latest BEFORE inserting; degrade visibly if it trips a
  // sanity flag. Never rewrites the pose — the tracker keeps trusting its input.
  if (nav_sink_) {
    const NavHealth health = evaluateNavInput(latest(), pose, nav_cfg_);
    if (health.any()) nav_sink_->onNavHealth(health);
  }

  if (!current_datum_) {
    current_datum_ = geo::Datum(geo::Geodetic{pose.lat_deg, pose.lon_deg, pose.alt_m});
  } else if (policy_.enable_auto_recenter) {
    const Eigen::Vector3d enu = current_datum_->toEnu(
        geo::Geodetic{pose.lat_deg, pose.lon_deg, pose.alt_m});
    const double d_m = std::sqrt(enu.x() * enu.x() + enu.y() * enu.y());
    if (d_m > policy_.recenter_threshold_km * 1000.0) {
      const geo::Datum old_datum = *current_datum_;
      current_datum_ = geo::Datum(geo::Geodetic{pose.lat_deg, pose.lon_deg, pose.alt_m});
      for (IDatumChangeSink* sink : sinks_) {
        if (sink) sink->onDatumRecentered(old_datum, *current_datum_);
      }
    }
  }
  // Insert in timestamp order: multi-source nav or a late-arriving sentence
  // can deliver fixes out of time order, and poseAtOrBefore's reverse walk
  // requires a sorted history. In-order pushes (the common case) hit the
  // upper_bound at end() immediately. Equal timestamps insert after existing
  // entries, so the most recently pushed pose wins the lookup.
  const auto it = std::upper_bound(
      history_.begin(), history_.end(), pose.time,
      [](const Timestamp& t, const OwnShipPose& h) { return t < h.time; });
  history_.insert(it, pose);
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

const geo::Datum& OwnShipProvider::datum() const {
  if (!current_datum_) {
    throw std::runtime_error(
        "OwnShipProvider::datum(): no datum yet; call update(pose) first "
        "or construct with an explicit datum.");
  }
  return *current_datum_;
}

bool OwnShipProvider::hasDatum() const noexcept {
  return current_datum_.has_value();
}

void OwnShipProvider::registerDatumSink(IDatumChangeSink* sink) {
  if (sink) sinks_.push_back(sink);
}

void OwnShipProvider::unregisterDatumSink(IDatumChangeSink* sink) {
  sinks_.erase(std::remove(sinks_.begin(), sinks_.end(), sink), sinks_.end());
}

void OwnShipProvider::setNavHealthSink(INavHealthSink* sink,
                                       NavInputGuardConfig cfg) {
  nav_sink_ = sink;
  nav_cfg_ = cfg;
}

}  // namespace navtracker
