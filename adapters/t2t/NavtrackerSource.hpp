#pragma once

// NavtrackerSource — the zero-effort first T2T input: an ITrackSink that turns
// navtracker's own track events into ExternalTracks and feeds them to a fuser.
// navtracker is the one source whose pedigree we know EXACTLY, per track, from
// its contributing_sources (everything else NotUsed). This gives every consumer
// a working first input and gives our tests a realistic tracker-in-the-loop.
//
// Frame: navtracker's tracks are already ENU in the shared datum, so the state
// is copied straight across — no datum conversion (the fuser works in the same
// frame). See docs/algorithms/t2t-fusion.md §5-analog and the ticket §5.
//
// Wiring:
//   NavtrackerSource src("navtracker", mgr, [&](ExternalTrack e){ fuser.process(std::move(e)); });
//   mgr.setTrackSink(&src);
// The lifecycle events resolve the full Track by scanning mgr.tracks() (there
// is no by-id accessor) and emit its current estimate. onTrackDeleted emits
// nothing — the fuser ages the contribution out via max_report_age_s.

#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "core/t2t/ExternalTrack.hpp"
#include "core/t2t/Pedigree.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Track.hpp"
#include "ports/ITrackSink.hpp"

namespace navtracker::t2t {

class NavtrackerSource : public ITrackSink {
 public:
  NavtrackerSource(std::string source_tracker_id, const TrackManager& manager,
                   std::function<void(ExternalTrack)> emit)
      : source_tracker_id_(std::move(source_tracker_id)),
        manager_(&manager),
        emit_(std::move(emit)) {}

  // Pure conversion: a navtracker Track -> an ExternalTrack tagged with this
  // source's id, the track's own id as the source track id, ENU state/cov,
  // velocity (only when observed), status hint, MMSI/name attributes, and a
  // per-track pedigree filled EXACTLY from contributing_sources (default
  // NotUsed). Returns nullopt if the track has no usable position yet.
  static std::optional<ExternalTrack> toExternalTrack(
      const std::string& source_tracker_id, const Track& track, Timestamp time) {
    if (track.state.size() < 2) return std::nullopt;
    ExternalTrack e;
    e.source_tracker_id = source_tracker_id;
    e.source_track_id = std::to_string(track.id.value);
    e.time = time;
    e.position_enu = track.state.head<2>();
    if (track.covariance.rows() >= 2 && track.covariance.cols() >= 2)
      e.position_cov = track.covariance.topLeftCorner<2, 2>();
    if (track.velocity_observed && track.state.size() >= 4 &&
        track.covariance.rows() >= 4 && track.covariance.cols() >= 4) {
      e.velocity_valid = true;
      e.velocity_enu = track.state.segment<2>(2);
      e.velocity_cov = track.covariance.block<2, 2>(2, 2);
    }
    e.source_status = track.status;
    e.attributes = track.attributes;  // mmsi/name/platform_id — evidence only

    // navtracker's pedigree is known per track: it used exactly the streams in
    // contributing_sources and nothing else.
    SourcePedigree ped;
    ped.default_usage = SensorUsage::NotUsed;
    for (const auto& s : track.contributing_sources)
      ped.sensors[s] = SensorUsage::Used;
    e.pedigree = std::move(ped);
    return e;
  }

  void onTrackInitiated(const TrackLifecycleEvent& e) override { emit(e); }
  void onTrackConfirmed(const TrackLifecycleEvent& e) override { emit(e); }
  void onTrackUpdated(const TrackLifecycleEvent& e) override { emit(e); }
  void onTrackDeleted(const TrackLifecycleEvent&) override {
    // Nothing to emit: the fuser ages the contribution out via max_report_age_s.
  }

 private:
  const Track* resolve(TrackId id) const {
    for (const Track& t : manager_->tracks())
      if (t.id == id) return &t;
    return nullptr;  // still present during onTrackDeleted, but we do not emit there
  }

  void emit(const TrackLifecycleEvent& e) {
    const Track* t = resolve(e.id);
    if (t == nullptr) return;
    auto ext = toExternalTrack(source_tracker_id_, *t, e.time);
    if (ext.has_value() && emit_) emit_(std::move(*ext));
  }

  std::string source_tracker_id_;
  const TrackManager* manager_;
  std::function<void(ExternalTrack)> emit_;
};

}  // namespace navtracker::t2t
