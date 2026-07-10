#include "core/t2t/FusedTrackOutput.hpp"

#include <utility>

namespace navtracker::t2t {

FusedTrackOutput toFusedTrackOutput(
    const Track& fused, const geo::Datum& datum,
    std::vector<ContributingTracker> contributing_trackers,
    IndependenceClass independence_class,
    bool covariance_is_pessimistic_default) {
  FusedTrackOutput out;
  out.track = toTrackOutput(fused, datum);  // reuse the canonical drain
  out.contributing_trackers = std::move(contributing_trackers);
  out.independence_class = independence_class;
  out.fusion_rule = "CI";
  out.covariance_is_pessimistic_default = covariance_is_pessimistic_default;
  return out;
}

}  // namespace navtracker::t2t
