#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * Raised when a cooperative source's own-identity report is overdue
 * (spec §9c): "we lost comms", NOT "the vessel sank". Pure notification;
 * MUST NOT be wired to anything that lowers existence.
 */
class IStaleSignalSink {
 public:
  virtual ~IStaleSignalSink() = default;
  /** Notify that track `id` has an overdue cooperative report as of `now`. */
  virtual void onTrackStale(TrackId id, Timestamp now) = 0;
};

}  // namespace navtracker
