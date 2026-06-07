#pragma once

#include <cstdint>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ITrackSink.hpp"

namespace navtracker {
namespace benchmark {

class BenchSink : public ITrackSink {
 public:
  enum class Kind { Initiated, Confirmed, Updated, Deleted };
  struct Event {
    Kind kind;
    TrackId id;
    Timestamp time;
    TrackStatus status;
  };

  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  void onTrackDeleted(const TrackLifecycleEvent& e) override;

  const std::vector<Event>& events() const { return events_; }
  void clear() { events_.clear(); }

 private:
  std::vector<Event> events_;
};

}  // namespace benchmark
}  // namespace navtracker
