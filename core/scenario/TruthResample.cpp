#include "core/scenario/TruthResample.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>

namespace navtracker {

namespace {

// Segment finite-difference velocity between two samples; zero when the
// segment is degenerate.
Eigen::Vector2d fdVelocity(const TruthSample& a, const TruthSample& b) {
  const double dt = b.time.secondsSince(a.time);
  if (!(dt > 0.0)) return Eigen::Vector2d::Zero();
  return (b.position - a.position) / dt;
}

// Velocity to report when a tick is clamped to sample `i` of `track`:
// the adjacent segment's finite difference (toward the interior), or
// the raw sample's own velocity for single-fix targets.
Eigen::Vector2d clampVelocity(const std::vector<TruthSample>& track,
                              std::size_t i) {
  if (track.size() < 2) return track[i].velocity;
  if (i + 1 < track.size()) return fdVelocity(track[i], track[i + 1]);
  return fdVelocity(track[i - 1], track[i]);
}

}  // namespace

std::vector<TruthSample> resampleTruthToClock(
    const std::vector<TruthSample>& samples,
    double period_s,
    double max_gap_s) {
  if (samples.empty() || !(period_s > 0.0)) return samples;

  // Per-target time-sorted tracks (std::map keeps truth_id order for
  // deterministic same-tick output).
  std::map<std::uint64_t, std::vector<TruthSample>> tracks;
  for (const TruthSample& s : samples) tracks[s.truth_id].push_back(s);
  Timestamp t_min = samples.front().time;
  Timestamp t_max = samples.front().time;
  for (auto& [id, track] : tracks) {
    std::sort(track.begin(), track.end(),
              [](const TruthSample& a, const TruthSample& b) {
                return a.time < b.time;
              });
    t_min = std::min(t_min, track.front().time);
    t_max = std::max(t_max, track.back().time);
  }

  const double half = 0.5 * period_s;
  std::vector<TruthSample> out;
  const std::int64_t ticks =
      static_cast<std::int64_t>(
          std::floor(t_max.secondsSince(t_min) / period_s + 1e-9)) + 1;
  for (std::int64_t k = 0; k < ticks; ++k) {
    const Timestamp tick =
        Timestamp::fromSeconds(t_min.seconds() + k * period_s);
    for (const auto& [id, track] : tracks) {
      // Nearest-tick snap window is half-open [t − half, t + half) so a
      // sample equidistant from two ticks lands on exactly one.
      const auto inSnapWindow = [&](const TruthSample& s) {
        const double d = tick.secondsSince(s.time);
        return d >= -half && d < half;
      };

      TruthSample r;
      r.time = tick;
      r.truth_id = id;

      if (tick < track.front().time) {
        if (!inSnapWindow(track.front())) continue;
        r.position = track.front().position;
        r.velocity = clampVelocity(track, 0);
      } else if (track.back().time < tick) {
        if (!inSnapWindow(track.back())) continue;
        r.position = track.back().position;
        r.velocity = clampVelocity(track, track.size() - 1);
      } else {
        // Bracketed: first sample b with tick ≤ b.time.
        std::size_t hi = 0;
        while (track[hi].time < tick) ++hi;
        if (hi == 0) {
          r.position = track[0].position;
          r.velocity = clampVelocity(track, 0);
        } else {
          const TruthSample& a = track[hi - 1];
          const TruthSample& b = track[hi];
          const double gap = b.time.secondsSince(a.time);
          if (gap > max_gap_s) {
            // Do not bridge long dropouts; keep presence only right at
            // the gap's endpoints.
            if (inSnapWindow(a)) {
              r.position = a.position;
              r.velocity = clampVelocity(track, hi - 1);
            } else if (inSnapWindow(b)) {
              r.position = b.position;
              r.velocity = clampVelocity(track, hi);
            } else {
              continue;
            }
          } else {
            const double w = tick.secondsSince(a.time) / gap;
            r.position = a.position + w * (b.position - a.position);
            r.velocity = fdVelocity(a, b);
          }
        }
      }
      out.push_back(r);
    }
  }
  return out;
}

}  // namespace navtracker
