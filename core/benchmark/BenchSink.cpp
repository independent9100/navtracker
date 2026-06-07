#include "core/benchmark/BenchSink.hpp"

namespace navtracker {
namespace benchmark {

void BenchSink::onTrackInitiated(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Initiated, e.id, e.time, e.status});
}
void BenchSink::onTrackConfirmed(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Confirmed, e.id, e.time, e.status});
}
void BenchSink::onTrackUpdated(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Updated, e.id, e.time, e.status});
}
void BenchSink::onTrackDeleted(const TrackLifecycleEvent& e) {
  events_.push_back({Kind::Deleted, e.id, e.time, e.status});
}

}  // namespace benchmark
}  // namespace navtracker
