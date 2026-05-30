#include "core/tracking/TrackManager.hpp"

namespace navtracker {

TrackManager::TrackManager(int confirm_hits, int delete_misses)
    : confirm_hits_(confirm_hits), delete_misses_(delete_misses) {}

TrackId TrackManager::add(const Track& track) {
  Track t = track;
  t.id = TrackId{next_id_++};
  t.status = TrackStatus::Tentative;
  tracks_.push_back(t);
  counters_.push_back(Counters{1, 0});
  return t.id;
}

int TrackManager::index(TrackId id) const {
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

void TrackManager::recordHit(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].hits += 1;
  counters_[i].misses = 0;
  if (counters_[i].hits >= confirm_hits_) {
    tracks_[i].status = TrackStatus::Confirmed;
  }
}

void TrackManager::recordMiss(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].misses += 1;
  counters_[i].hits = 0;
  if (counters_[i].misses >= delete_misses_) {
    tracks_.erase(tracks_.begin() + i);
    counters_.erase(counters_.begin() + i);
    return;
  }
  tracks_[i].status = TrackStatus::Coasting;
}

}  // namespace navtracker
