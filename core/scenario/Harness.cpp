#include "core/scenario/Harness.hpp"

#include "core/scenario/Ospa.hpp"

namespace navtracker {

ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff) {
  ScenarioResult r;
  for (const Measurement& z : scenario.measurements) {
    tracker.process(z);

    std::vector<Eigen::Vector2d> truth_xy;
    for (const TruthSample& ts : scenario.truth) {
      if (ts.time == z.time) truth_xy.push_back(ts.position);
    }
    std::vector<Eigen::Vector2d> est_xy;
    for (const Track& tr : manager.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
      }
    }
    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));
  }
  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}

}  // namespace navtracker
