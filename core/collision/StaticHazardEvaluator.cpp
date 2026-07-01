#include "core/collision/StaticHazardEvaluator.hpp"

#include "core/output/StaticHazardOutput.hpp"  // staticHazardId

namespace navtracker {

void StaticHazardEvaluator::evaluate(const Eigen::Vector2d& own_ship_enu,
                                     const geo::Datum& datum, Timestamp t) {
  if (model_ == nullptr) return;
  for (const auto& obs : model_->obstacles()) {
    const Eigen::Vector3d e = datum.toEnu(obs.position);
    const double d = (Eigen::Vector2d(e.x(), e.y()) - own_ship_enu).norm();
    const std::uint64_t id = staticHazardId(obs);
    const double enter_r = obs.keep_clear_radius_m;
    const double exit_r = obs.keep_clear_radius_m * cfg_.exit_hysteresis;

    auto it = inside_.find(id);
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
    inside_[id] = now_inside;
  }
}

}  // namespace navtracker
