#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Owns the active track set, allocates stable TrackIds, and runs the M-of-N
// lifecycle state machine. IDs are monotonic and never reused. The cycle that
// drives hits/misses (predict/associate/update) lives in the pipeline.
class TrackManager {
 public:
  TrackManager(int confirm_hits, int delete_misses);

  // Register a new Tentative track; assigns and returns a fresh stable id.
  // `first_observation` seeds the lifecycle's "last observed" clock.
  TrackId add(const Track& track,
              Timestamp first_observation = Timestamp{});

  void recordHit(TrackId id);
  void recordMiss(TrackId id);

  // Record that this track absorbed a measurement at time `t`.
  void noteObservation(TrackId id, Timestamp t);
  Timestamp lastObservation(TrackId id) const;

  const std::vector<Track>& tracks() const { return tracks_; }
  std::vector<Track>& mutableTracks() { return tracks_; }
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
  std::vector<Counters> counters_;     // parallel to tracks_
  std::vector<Timestamp> last_observation_;  // parallel to tracks_
};

}  // namespace navtracker
