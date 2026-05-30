#include "core/association/GnnAssociator.hpp"

#include <limits>

#include "core/association/Gating.hpp"

namespace navtracker {

GnnAssociator::GnnAssociator(double gate_threshold)
    : gate_threshold_(gate_threshold) {}

AssociationResult GnnAssociator::associate(
    const std::vector<Track>& tracks,
    const std::vector<Measurement>& measurements) const {
  const std::size_t nt = tracks.size();
  const std::size_t nm = measurements.size();
  std::vector<bool> track_used(nt, false);
  std::vector<bool> meas_used(nm, false);
  AssociationResult result;

  while (true) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t best_t = 0;
    std::size_t best_m = 0;
    bool found = false;
    for (std::size_t ti = 0; ti < nt; ++ti) {
      if (track_used[ti]) continue;
      for (std::size_t mi = 0; mi < nm; ++mi) {
        if (meas_used[mi]) continue;
        const double d2 = mahalanobisDistance(tracks[ti], measurements[mi]);
        if (d2 <= gate_threshold_ && d2 < best) {
          best = d2;
          best_t = ti;
          best_m = mi;
          found = true;
        }
      }
    }
    if (!found) break;
    result.matches.emplace_back(best_t, best_m);
    track_used[best_t] = true;
    meas_used[best_m] = true;
  }

  for (std::size_t ti = 0; ti < nt; ++ti) {
    if (!track_used[ti]) result.unmatched_tracks.push_back(ti);
  }
  for (std::size_t mi = 0; mi < nm; ++mi) {
    if (!meas_used[mi]) result.unmatched_measurements.push_back(mi);
  }
  return result;
}

}  // namespace navtracker
