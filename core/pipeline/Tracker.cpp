#include "core/pipeline/Tracker.hpp"

#include <cstdint>
#include <vector>

#include "core/tracking/TrackManager.hpp"

namespace navtracker {

Tracker::Tracker(const IEstimator& estimator,
                 const IDataAssociator& associator,
                 TrackManager& manager,
                 double miss_timeout_seconds)
    : estimator_(estimator),
      associator_(associator),
      manager_(manager),
      miss_timeout_seconds_(miss_timeout_seconds) {}

void Tracker::process(const Measurement& z) {
  manager_.predictAll(estimator_, z.time);

  const std::vector<Measurement> batch{z};
  const AssociationResult result =
      associator_.associate(manager_.tracks(), batch);

  if (!result.matches.empty()) {
    const std::size_t ti = result.matches.front().first;
    Track& tr = manager_.mutableTracks()[ti];
    estimator_.update(tr, z);
    bool has_src = false;
    for (const auto& s : tr.contributing_sources) {
      if (s == z.source_id) {
        has_src = true;
        break;
      }
    }
    if (!has_src) tr.contributing_sources.push_back(z.source_id);
    const TrackId id = tr.id;
    manager_.recordHit(id);
    manager_.noteObservation(id, z.time);
  } else {
    Track seed = estimator_.initiate(z);
    manager_.add(seed, z.time);
  }

  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(miss_timeout_seconds_ * 1e9);
  std::vector<TrackId> stale;
  for (const auto& tr : manager_.tracks()) {
    const std::int64_t age =
        z.time.nanos() - manager_.lastObservation(tr.id).nanos();
    if (age > timeout_ns) stale.push_back(tr.id);
  }
  for (const TrackId id : stale) manager_.recordMiss(id);
}

}  // namespace navtracker
