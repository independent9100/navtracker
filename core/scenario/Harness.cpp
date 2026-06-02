#include "core/scenario/Harness.hpp"

#include "core/scenario/Ospa.hpp"

namespace navtracker {

ScenarioResult runScenario(const Scenario& scenario,
                           Tracker& tracker,
                           const TrackManager& manager,
                           double ospa_cutoff) {
  ScenarioResult r;
  if (scenario.truth.empty()) return r;

  std::size_t mi = 0;  // next unprocessed measurement
  std::size_t ti = 0;  // first truth sample of the current tick group

  while (ti < scenario.truth.size()) {
    const Timestamp tick = scenario.truth[ti].time;

    // Process every measurement whose timestamp is <= the current truth tick.
    while (mi < scenario.measurements.size() &&
           !(tick < scenario.measurements[mi].time)) {
      tracker.process(scenario.measurements[mi]);
      ++mi;
    }

    // Collect all truth samples for this tick.
    std::vector<Eigen::Vector2d> truth_xy;
    std::size_t tj = ti;
    while (tj < scenario.truth.size() && scenario.truth[tj].time == tick) {
      truth_xy.push_back(scenario.truth[tj].position);
      ++tj;
    }

    // Snapshot current tracks.
    std::vector<Eigen::Vector2d> est_xy;
    std::vector<TrackSnapshot> snaps;
    for (const Track& tr : manager.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
        snaps.push_back(TrackSnapshot{tr.id, Eigen::Vector2d(tr.state(0), tr.state(1))});
      }
    }

    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

    ScenarioStep step;
    step.time = tick;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));

    ti = tj;
  }

  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}

}  // namespace navtracker
