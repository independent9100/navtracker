#pragma once

#include <vector>

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Driven-side edge port: receives the authoritative track set after each
 * processing step (for display, logging, downstream consumers, etc).
 */
class ITrackSnapshotSink {
 public:
  virtual ~ITrackSnapshotSink() = default;
  /** Deliver the full track set as it stands at `now`, after a process step. */
  virtual void onTracks(const std::vector<Track>& tracks, Timestamp now) = 0;
};

}  // namespace navtracker
