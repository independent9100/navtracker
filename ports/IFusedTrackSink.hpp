#pragma once

// IFusedTrackSink — push-based lifecycle events for FUSED tracks, mirroring
// ports/ITrackSink.hpp exactly so consumers get the identical push/pull dual
// they already know. Four events, one lightweight payload each. The sink is
// nullable and non-owning: null = today's behavior, no overhead (guard every
// fire site with `if (sink != nullptr)`). Pull access via
// T2tFuser::fusedTracks() stays fully supported alongside.

#include "core/types/Ids.hpp"        // TrackId, TrackStatus
#include "core/types/Timestamp.hpp"  // Timestamp

namespace navtracker::t2t {

// Deliberately thin, like TrackLifecycleEvent: carries only identity, time,
// and status. Consumers that need kinematics pull the full FusedTrackOutput.
struct FusedTrackLifecycleEvent {
  TrackId id;          // fused track id (minted by the fuser, never reused)
  Timestamp time;      // event time (message-timestamp driven, never wall clock)
  TrackStatus status;  // status at the moment the event fires
};

class IFusedTrackSink {
 public:
  virtual ~IFusedTrackSink() = default;

  // A new fused track was created (status Tentative).
  virtual void onFusedTrackInitiated(const FusedTrackLifecycleEvent& e) = 0;
  // A fused track transitioned to Confirmed (fires once).
  virtual void onFusedTrackConfirmed(const FusedTrackLifecycleEvent& e) = 0;
  // A fused track's estimate was updated by a fresh source contribution.
  virtual void onFusedTrackUpdated(const FusedTrackLifecycleEvent& e) = 0;
  // A fused track is being deleted (fires BEFORE removal; status is the
  // pre-deletion value).
  virtual void onFusedTrackDeleted(const FusedTrackLifecycleEvent& e) = 0;
};

}  // namespace navtracker::t2t
