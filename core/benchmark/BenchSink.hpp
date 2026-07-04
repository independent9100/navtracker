#pragma once

#include <cstdint>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ITrackSink.hpp"

namespace navtracker {
namespace benchmark {

/**
 * Recording track sink for benches: captures the full lifecycle stream
 * (initiate/confirm/update/delete) as a flat, time-ordered event list so a
 * bench run can assert continuity, ID stability, and lifecycle behaviour
 * after the fact.
 */
class BenchSink : public ITrackSink {
 public:
  enum class Kind { Initiated, Confirmed, Updated, Deleted };
  /** One recorded lifecycle event: what happened, to which track, when. */
  struct Event {
    Kind kind;
    TrackId id;
    Timestamp time;
    TrackStatus status;
  };

  /** Record a track-initiation event. */
  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  /** Record a track-confirmation event. */
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  /** Record a track-update event. */
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  /** Record a track-deletion event. */
  void onTrackDeleted(const TrackLifecycleEvent& e) override;

  /** The recorded events in the order they fired. */
  const std::vector<Event>& events() const { return events_; }
  /** Drop all recorded events (reuse the sink across runs). */
  void clear() { events_.clear(); }

 private:
  std::vector<Event> events_;
};

}  // namespace benchmark
}  // namespace navtracker
