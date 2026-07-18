#include "core/tracking/TrackManager.hpp"

#include "ports/IEstimator.hpp"

namespace navtracker {

TrackManager::TrackManager(int confirm_hits, int delete_misses)
    : confirm_hits_(confirm_hits), delete_misses_(delete_misses) {}

TrackId TrackManager::add(const Track& track, Timestamp first_observation) {
  Track t = track;
  t.id = TrackId{next_id_++};
  t.status = TrackStatus::Tentative;
  tracks_.push_back(t);
  counters_.push_back(Counters{1, 0});
  last_observation_.push_back(first_observation);
  id_to_index_[t.id.value] = tracks_.size() - 1;
  if (sink_ != nullptr) {
    sink_->onTrackInitiated({t.id, first_observation, t.status});
  }
  return t.id;
}

int TrackManager::index(TrackId id) const {
  const auto it = id_to_index_.find(id.value);
  if (it == id_to_index_.end()) return -1;
  return static_cast<int>(it->second);
}

void TrackManager::recordHit(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].hits += 1;
  counters_[i].misses = 0;
  const bool was_unconfirmed = tracks_[i].status != TrackStatus::Confirmed;
  if (counters_[i].hits >= confirm_hits_) {
    tracks_[i].status = TrackStatus::Confirmed;
    if (was_unconfirmed && sink_ != nullptr) {
      sink_->onTrackConfirmed({id, last_observation_[i], tracks_[i].status});
    }
  }
}

void TrackManager::recordMiss(TrackId id) {
  const int i = index(id);
  if (i < 0) return;
  counters_[i].misses += 1;
  counters_[i].hits = 0;
  if (counters_[i].misses >= delete_misses_) {
    if (sink_ != nullptr) {
      sink_->onTrackDeleted({id, last_observation_[i], tracks_[i].status});
    }
    tracks_.erase(tracks_.begin() + i);
    counters_.erase(counters_.begin() + i);
    last_observation_.erase(last_observation_.begin() + i);
    // The erase shifted every element after i down by one slot. Drop the
    // deleted id and reindex the shifted tail so id_to_index_ stays exact.
    id_to_index_.erase(id.value);
    for (std::size_t k = static_cast<std::size_t>(i); k < tracks_.size(); ++k) {
      id_to_index_[tracks_[k].id.value] = k;
    }
    return;
  }
  // W5.4: only an already-Confirmed (or still-Coasting) track demotes to
  // Coasting on a miss. A never-confirmed Tentative track keeps its status and
  // ages its miss counter toward deletion — it must NOT become Coasting, which
  // is CPA-eligible (CpaEvaluator gates on Confirmed||Coasting) and would emit
  // false collision-risk events for a one-hit clutter blip; Coasting is also
  // defined as "was Confirmed" (docs/output-contract.md). Setting Coasting on an
  // already-Coasting track is a no-op, so the || Coasting arm just documents
  // intent.
  if (tracks_[i].status == TrackStatus::Confirmed ||
      tracks_[i].status == TrackStatus::Coasting) {
    tracks_[i].status = TrackStatus::Coasting;
  }
}

void TrackManager::noteObservation(TrackId id, Timestamp t) {
  const int i = index(id);
  if (i < 0) return;
  last_observation_[i] = t;
}

Timestamp TrackManager::lastObservation(TrackId id) const {
  const int i = index(id);
  if (i < 0) return Timestamp{};
  return last_observation_[i];
}

void TrackManager::predictAll(const IEstimator& estimator, Timestamp to) {
  for (auto& t : tracks_) {
    estimator.predict(t, to);
  }
}

void TrackManager::recordUpdated(TrackId id, Timestamp t) {
  if (sink_ == nullptr) return;
  const int i = index(id);
  if (i < 0) return;
  sink_->onTrackUpdated({id, t, tracks_[i].status});
}

}  // namespace navtracker
