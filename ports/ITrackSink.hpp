#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Snapshot fired on a track-lifecycle transition. `status` reflects the
// status AT the moment the event fires. See spec
// 2026-06-04-track-and-collision-risk-sinks-design.md §3.
struct TrackLifecycleEvent {
  TrackId id;
  Timestamp time;
  TrackStatus status;
};

// Push-based lifecycle observer. Consumers (UI, logger, downstream
// pipelines) implement this and register via TrackManager::setTrackSink.
//
// Event semantics:
//   onTrackInitiated - fires from TrackManager::add (status = Tentative)
//   onTrackConfirmed - fires once when Tentative -> Confirmed
//   onTrackUpdated   - fires after a successful estimator.update via
//                      TrackManager::recordUpdated (any status)
//   onTrackDeleted   - fires BEFORE erasure when the miss threshold is hit
//                      (status reflects pre-erasure value)
//
// All methods are required; provide empty implementations for events you
// don't care about, or use a multiplexing wrapper to fan out.
class ITrackSink {
 public:
  virtual ~ITrackSink() = default;
  virtual void onTrackInitiated(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackConfirmed(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackUpdated(const TrackLifecycleEvent& e) = 0;
  virtual void onTrackDeleted(const TrackLifecycleEvent& e) = 0;
};

}  // namespace navtracker
