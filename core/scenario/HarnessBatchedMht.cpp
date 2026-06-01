#include "core/scenario/HarnessBatchedMht.hpp"

#include "core/scenario/Ospa.hpp"

namespace navtracker {

ScenarioResult runScenarioBatchedMht(const Scenario& scenario,
                                     MhtTracker& tracker,
                                     double ospa_cutoff) {
  ScenarioResult r;
  std::size_t i = 0;
  while (i < scenario.measurements.size()) {
    const Timestamp t = scenario.measurements[i].time;
    std::vector<Measurement> scan;
    while (i < scenario.measurements.size() &&
           scenario.measurements[i].time == t) {
      scan.push_back(scenario.measurements[i]);
      ++i;
    }
    tracker.processBatch(scan);

    std::vector<Eigen::Vector2d> truth_xy;
    for (const TruthSample& ts : scenario.truth) {
      if (ts.time == t) truth_xy.push_back(ts.position);
    }
    std::vector<Eigen::Vector2d> est_xy;
    std::vector<TrackSnapshot> snaps;
    for (const Track& tr : tracker.tracks()) {
      if (tr.state.size() >= 2) {
        est_xy.emplace_back(tr.state(0), tr.state(1));
        snaps.push_back(TrackSnapshot{tr.id, Eigen::Vector2d(tr.state(0), tr.state(1))});
      }
    }
    r.ospa_per_step.push_back(ospaGreedy(truth_xy, est_xy, ospa_cutoff));

    ScenarioStep step;
    step.time = t;
    step.truth = std::move(truth_xy);
    step.tracks = std::move(snaps);
    r.steps.push_back(std::move(step));
  }
  if (!r.ospa_per_step.empty()) {
    double sum = 0.0;
    for (double v : r.ospa_per_step) sum += v;
    r.mean_ospa = sum / static_cast<double>(r.ospa_per_step.size());
  }
  return r;
}

}  // namespace navtracker
