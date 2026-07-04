#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ITrackSink.hpp"

namespace navtracker {

class IEstimator;  // fwd

/**
 * Owns the active track set, allocates stable TrackIds, and runs the M-of-N
 * lifecycle state machine. IDs are monotonic and never reused. The cycle that
 * drives hits/misses (predict/associate/update) lives in the pipeline.
 */
class TrackManager {
 public:
  /**
   * Construct with the M-of-N lifecycle thresholds: `confirm_hits` hits
   * promote a Tentative track to Confirmed; `delete_misses` consecutive
   * misses delete it.
   */
  TrackManager(int confirm_hits, int delete_misses);

  /** Register a new Tentative track; assigns and returns a fresh stable id. */
  TrackId add(const Track& track,
              Timestamp first_observation = Timestamp{});

  /** Record a detection hit for track `id`, advancing its confirmation counter. */
  void recordHit(TrackId id);
  /** Record a missed detection for track `id`, advancing its deletion counter. */
  void recordMiss(TrackId id);

  /** Stamp the most recent observation time for track `id`. */
  void noteObservation(TrackId id, Timestamp t);
  /** Time of the last recorded observation for track `id`. */
  Timestamp lastObservation(TrackId id) const;

  /** Advance every active track to `to` via the estimator. */
  void predictAll(const IEstimator& estimator, Timestamp to);

  /** Optional lifecycle event sink. Null = no-op (today's behavior). */
  void setTrackSink(ITrackSink* sink) { sink_ = sink; }

  /**
   * Notify the sink (if any) that a track's kinematic state has changed.
   * Called by Tracker after a successful estimator.update. Pure event
   * fire — no state mutation here.
   */
  void recordUpdated(TrackId id, Timestamp t);

  /** Read-only view of the active track set (iteration order is stable). */
  const std::vector<Track>& tracks() const { return tracks_; }
  /** Mutable view of the active track set (pipeline predict/update path). */
  std::vector<Track>& mutableTracks() { return tracks_; }
  /** Number of active tracks. */
  std::size_t size() const { return tracks_.size(); }

 private:
  struct Counters {
    int hits;
    int misses;
  };
  int index(TrackId id) const;

  int confirm_hits_;
  int delete_misses_;
  std::uint64_t next_id_{1};
  std::vector<Track> tracks_;
  std::vector<Counters> counters_;
  std::vector<Timestamp> last_observation_;
  // TrackId.value -> position in tracks_/counters_/last_observation_. Keeps
  // per-track lookup O(1) so the per-cycle hit/miss/observation bookkeeping
  // is O(n) rather than O(n²) (review #9). Iteration order is still driven by
  // tracks_, so this is purely a lookup accelerator — replay stays
  // deterministic (invariant #4).
  std::unordered_map<std::uint64_t, std::size_t> id_to_index_;
  ITrackSink* sink_{nullptr};
};

}  // namespace navtracker
