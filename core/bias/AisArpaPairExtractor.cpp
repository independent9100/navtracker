#include "core/bias/AisArpaPairExtractor.hpp"

#include <algorithm>
#include <cmath>

#include "core/types/Ids.hpp"  // SensorKind

namespace navtracker {
namespace {

bool isAisKind(SensorKind k) {
  return k == SensorKind::Ais || k == SensorKind::Cooperative;
}

bool isArpaKind(SensorKind k) {
  return k == SensorKind::ArpaTtm || k == SensorKind::ArpaTll;
}

double sigmaFromCov2D(const Eigen::Matrix2d& c, double fallback) {
  // Use sqrt of mean eigenvalue (≈ isotropic std). Fall back if degenerate.
  const double tr = c.trace();
  if (!(tr > 0.0)) return fallback;
  return std::sqrt(tr * 0.5);
}

}  // namespace

std::vector<AisArpaPairObservation> extractPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    AisArpaPairExtractorConfig cfg) {
  std::vector<AisArpaPairObservation> out;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(cfg.cycle_window_seconds * 1e9);
  for (const Track& tr : tracks) {
    const Track::SourceTouch* ais = nullptr;
    const Track::SourceTouch* arpa = nullptr;
    for (const auto& t : tr.recent_contributions) {
      const std::int64_t age = cycle_time.nanos() - t.time.nanos();
      if (age < 0 || age > window_ns) continue;
      if (!ais && isAisKind(t.sensor)) ais = &t;
      if (!arpa && isArpaKind(t.sensor)) arpa = &t;
      if (ais && arpa) break;
    }
    if (!ais || !arpa) continue;
    AisArpaPairObservation obs;
    obs.time = cycle_time;
    obs.own_position_enu = arpa->sensor_position_enu;
    obs.ais_target_position_enu = ais->value_enu;
    obs.arpa_target_position_enu = arpa->value_enu;
    obs.ais_position_std_m =
        sigmaFromCov2D(ais->covariance, cfg.ais_position_std_fallback_m);
    obs.arpa_bearing_std_rad = cfg.arpa_bearing_std_fallback_rad;
    obs.own_position_std_m = arpa->own_position_std_m;
    out.push_back(std::move(obs));
  }
  return out;
}

}  // namespace navtracker
