#include "core/collision/StaticHazardEvaluator.hpp"

#include "core/output/StaticHazardOutput.hpp"  // staticHazardId

namespace navtracker {

void StaticHazardEvaluator::evaluate(const Eigen::Vector2d& own_ship_enu,
                                     const geo::Datum& datum, Timestamp t) {
  if (model_ == nullptr) return;
  const auto& obstacles = model_->obstacles();
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obs = obstacles[i];
    const Eigen::Vector3d e = datum.toEnu(obs.position);
    const double d = (Eigen::Vector2d(e.x(), e.y()) - own_ship_enu).norm();
    const std::uint64_t id = staticHazardId(obs);  // for the event payload
    const double enter_r = obs.keep_clear_radius_m;
    const double exit_r = obs.keep_clear_radius_m * cfg_.exit_hysteresis;

    // Hysteresis state is keyed by obstacle index so co-located obstacles
    // (colliding hazard_id) keep independent enter/exit state (R7.3). Finding #8:
    // this relies on the model's obstacles() being STABLE-INDEXED for the
    // evaluator's lifetime — index i must always refer to the same physical
    // obstacle. StaticObstacleModel satisfies this (its obstacle list is fixed at
    // construction; a datum recenter rebuilds ENU positions, never the list). A
    // model that inserts/removes/reorders obstacles at runtime would transfer one
    // obstacle's inside-state to another via index reuse and must not be paired
    // with this evaluator without clearing inside_.
    auto it = inside_.find(i);
    const bool was_inside = (it != inside_.end()) ? it->second : false;
    bool now_inside = was_inside;
    if (!was_inside && d < enter_r) {
      now_inside = true;
    } else if (was_inside && d > exit_r) {
      now_inside = false;
    }

    if (sink_ != nullptr) {
      if (now_inside && !was_inside) {
        sink_->onStaticHazard({StaticHazardTransition::Entered, id, t, d,
                               obs.keep_clear_radius_m});
      } else if (!now_inside && was_inside) {
        sink_->onStaticHazard({StaticHazardTransition::Exited, id, t, d,
                               obs.keep_clear_radius_m});
      } else if (now_inside && cfg_.emit_updates) {
        sink_->onStaticHazard({StaticHazardTransition::Updated, id, t, d,
                               obs.keep_clear_radius_m});
      }
    }
    inside_[i] = now_inside;
  }
}

}  // namespace navtracker
