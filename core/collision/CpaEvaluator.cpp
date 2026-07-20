#include "core/collision/CpaEvaluator.hpp"

#include <vector>

#include "core/own_ship/OwnShipProvider.hpp"
#include "core/collision/CpaOwnShip.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

CpaEvaluator::CpaEvaluator(const TrackManager& manager,
                           const OwnShipProvider& provider,
                           CpaEvaluatorConfig cfg)
    : manager_(manager), provider_(provider), cfg_(cfg) {}

// L24: computeCpaWithUncertainty reads state(0..3) and the 4×4 kinematic block
// of the covariance and inverts a relative-position covariance. A malformed
// track (too-short state, wrong-size or non-finite covariance) would abort in
// Eigen (debug) / read OOB (NDEBUG) or produce NaN risk. Gate the uncertainty
// path on a cheap sanity check and skip the offending pair.
namespace {
bool cpaStateSane(const Track& tr) {
  return tr.state.size() >= 4 && tr.covariance.rows() >= 4 &&
         tr.covariance.cols() >= 4 && tr.state.allFinite() &&
         tr.covariance.allFinite();
}
}  // namespace

void CpaEvaluator::evaluate(Timestamp t) {
  const auto pose = provider_.latest();
  if (!pose.has_value()) return;
  const Track own = synthesizeOwnShipTrack(*pose, provider_);
  if (!cpaStateSane(own)) return;  // L24: no CPA on a malformed own-ship state

  std::unordered_set<std::uint64_t> seen_this_cycle;

  for (const Track& tr : manager_.tracks()) {
    if (tr.id.value == 0) continue;
    const bool status_ok =
        cfg_.evaluate_tentative
        || tr.status == TrackStatus::Confirmed
        || tr.status == TrackStatus::Coasting;
    if (!status_ok) continue;
    if (!cpaStateSane(tr)) continue;  // L24: skip a malformed target track

    const CpaPrediction pred =
        computeCpaWithUncertainty(own, tr, t, cfg_.d_threshold_m);
    const double p = pred.probability_below_threshold;
    const bool was_risky = state_.count(tr.id.value) > 0;
    const bool now_risky = was_risky
                               ? (p >= cfg_.exit_probability)
                               : (p >= cfg_.enter_probability);

    if (!was_risky && now_risky) {
      state_.insert(tr.id.value);
      ++n_entered_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Entered, tr.id, t, pred});
      }
    } else if (was_risky && !now_risky) {
      state_.erase(tr.id.value);
      ++n_exited_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Exited, tr.id, t, pred});
      }
    } else if (was_risky && now_risky && cfg_.emit_updates) {
      ++n_updated_;
      if (sink_ != nullptr) {
        sink_->onCollisionRisk(
            {CollisionRiskTransition::Updated, tr.id, t, pred});
      }
    }
    seen_this_cycle.insert(tr.id.value);
  }

  // Pairs in state_ not seen this cycle - track was deleted or no longer
  // matches the status gate. Fire Exited.
  std::vector<std::uint64_t> dropped;
  for (auto id : state_) {
    if (seen_this_cycle.count(id) == 0) dropped.push_back(id);
  }
  for (auto id : dropped) {
    state_.erase(id);
    ++n_exited_;
    if (sink_ != nullptr) {
      CpaPrediction empty{};
      empty.d_threshold_m = cfg_.d_threshold_m;
      sink_->onCollisionRisk(
          {CollisionRiskTransition::Exited, TrackId{id}, t, empty});
    }
  }
}

}  // namespace navtracker
