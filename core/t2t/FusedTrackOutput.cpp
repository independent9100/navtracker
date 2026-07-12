#include "core/t2t/FusedTrackOutput.hpp"

#include <utility>

namespace navtracker::t2t {

FusedTrackOutput toFusedTrackOutput(
    const Track& fused, const geo::Datum& datum,
    std::vector<ContributingTracker> contributing_trackers,
    IndependenceClass independence_class,
    bool covariance_is_pessimistic_default) {
  FusedTrackOutput out;
  // ENU ordering: the T2T fuser works entirely in the shared datum-ENU frame
  // (NavtrackerSource feeds raw ENU covariance straight through), so the fused
  // drain stays ENU. Consumers wanting north-first can re-drain via
  // toTrackOutputNED. (F3 dual-API, 2026-07-12.)
  out.track = toTrackOutputENU(fused, datum);
  out.contributing_trackers = std::move(contributing_trackers);
  out.independence_class = independence_class;
  out.fusion_rule = "CI";
  out.covariance_is_pessimistic_default = covariance_is_pessimistic_default;
  return out;
}

}  // namespace navtracker::t2t
