#include "core/pmbm/PmbmTracker.hpp"

#include <utility>

namespace navtracker::pmbm {

PmbmTracker::PmbmTracker(const IEstimator& estimator, Config cfg,
                         BirthModelFn birth_model)
    : estimator_(estimator),
      cfg_(cfg),
      birth_model_(std::move(birth_model)) {}

void PmbmTracker::predict(Timestamp to) {
  if (!has_current_time_) {
    current_time_ = to;
    has_current_time_ = true;
    if (birth_model_) {
      auto births = birth_model_(to, 0.0);
      density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
    }
    return;
  }

  const double dt = to.secondsSince(current_time_);
  if (dt <= 0.0) {
    current_time_ = to;
    return;
  }

  // PPP predict: weight decay by p_S, motion propagation per component.
  for (auto& c : density_.ppp) {
    c.weight *= cfg_.survival_probability;
    Track t = toTrack(c, current_time_);
    estimator_.predict(t, to);
    fromTrack(c, t);
  }

  // MBM predict: per-Bernoulli existence decay by p_S, motion
  // propagation; mixture weights w^j unchanged (the per-target predicts
  // do not redistribute mass across global hypotheses).
  for (auto& h : density_.mbm) {
    for (auto& b : h.bernoullis) {
      b.existence_probability *= cfg_.survival_probability;
      Track t = toTrack(b);
      estimator_.predict(t, to);
      fromTrack(b, t);
    }
  }

  // Birth intensity: appended after survival decay so new births start
  // at full weight.
  if (birth_model_) {
    auto births = birth_model_(to, dt);
    density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
  }

  current_time_ = to;
}

}  // namespace navtracker::pmbm
