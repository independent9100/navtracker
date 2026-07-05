#include "core/static/BearingWedgeModel.hpp"

#include <algorithm>
#include <cmath>

namespace navtracker {
namespace {
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// Smallest absolute difference between two angles (rad), wrapped to [0, π].
double angleDelta(double a, double b) {
  double d = std::fmod(a - b, kTwoPi);
  if (d < -3.14159265358979323846) d += kTwoPi;
  if (d > 3.14159265358979323846) d -= kTwoPi;
  return std::abs(d);
}
}  // namespace

Eigen::Vector2d BearingWedgeModel::toAnchorEnu(
    const Eigen::Vector2d& enu_current) const {
  // Fast path: no recenter yet (the incoming ENU is already the anchor frame).
  const geo::Geodetic& a = anchor_.origin();
  const geo::Geodetic& c = current_.origin();
  if (a.lat_deg == c.lat_deg && a.lon_deg == c.lon_deg && a.alt_m == c.alt_m)
    return enu_current;
  const geo::Geodetic g =
      current_.toGeodetic(Eigen::Vector3d(enu_current.x(), enu_current.y(), 0.0));
  const Eigen::Vector3d e = anchor_.toEnu(g);
  return Eigen::Vector2d(e.x(), e.y());
}

void BearingWedgeModel::observeBearing(double t_s,
                                       const Eigen::Vector2d& own_enu,
                                       double bearing_math_rad,
                                       double bearing_sigma_rad,
                                       const std::string& source_id,
                                       std::int64_t contact_id,
                                       std::optional<double> max_range,
                                       bool suspect) {
  const Key key{source_id, contact_id};
  const double half_width =
      std::max(params_.half_width_sigma_mult * bearing_sigma_rad,
               params_.min_half_width_rad);
  const std::optional<double> range =
      max_range.has_value() ? max_range : params_.default_max_range_m;

  auto it = wedges_.find(key);
  // A suspected number-reuse (#20 sensor_track_id_suspect) must not resurrect the
  // previous contact's identity — mint a fresh wedge_id even without a stale gap.
  const bool mint_new = (it == wedges_.end()) || suspect;

  BearingWedge& w = wedges_[key];
  if (mint_new) w.wedge_id = next_wedge_id_++;
  w.apex_enu = toAnchorEnu(own_enu);
  w.bearing_math_rad = bearing_math_rad;
  w.half_width_rad = half_width;
  w.max_range_m = range;
  w.last_update_s = t_s;
  w.source_id = source_id;
}

void BearingWedgeModel::observeConfirmedTracks(
    const std::vector<Eigen::Vector2d>& track_enu) {
  confirmed_tracks_.clear();
  confirmed_tracks_.reserve(track_enu.size());
  for (const auto& t : track_enu) confirmed_tracks_.push_back(toAnchorEnu(t));
}

void BearingWedgeModel::pruneStale(double now_s) {
  for (auto it = wedges_.begin(); it != wedges_.end();) {
    if (now_s - it->second.last_update_s > params_.stale_window_s)
      it = wedges_.erase(it);  // key removed → a reappearance mints a new id
    else
      ++it;
  }
}

bool BearingWedgeModel::isClaimed(const BearingWedge& w) const {
  for (const auto& t : confirmed_tracks_) {
    const Eigen::Vector2d d = t - w.apex_enu;
    const double range = d.norm();
    if (range < params_.claim_min_range_m) continue;  // degenerate near apex
    if (w.max_range_m.has_value() && range > *w.max_range_m) continue;
    const double track_bearing = std::atan2(d.y(), d.x());  // CCW from east
    if (angleDelta(track_bearing, w.bearing_math_rad) <=
        w.half_width_rad + params_.handover_margin_rad)
      return true;  // a confirmed track claims this direction
  }
  return false;
}

std::vector<BearingWedge> BearingWedgeModel::liveWedges() const {
  std::vector<BearingWedge> out;
  out.reserve(wedges_.size());
  for (const auto& [key, w] : wedges_) out.push_back(w);
  return out;
}

std::vector<BearingWedge> BearingWedgeModel::activeWedges() const {
  std::vector<BearingWedge> out;
  for (const auto& [key, w] : wedges_)
    if (!isClaimed(w)) out.push_back(w);  // suppression recomputed, never latched
  return out;
}

std::vector<BearingWedgeOutput> BearingWedgeModel::hazardOutputs() const {
  std::vector<BearingWedgeOutput> out;
  for (const auto& w : activeWedges())
    out.push_back(toBearingWedgeOutput(w, anchor_));
  return out;
}

}  // namespace navtracker
