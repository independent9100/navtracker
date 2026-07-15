#include <gtest/gtest.h>
#include <Eigen/Core>
#include "core/tracking/TrackManager.hpp"

using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackManager;
using navtracker::TrackStatus;

TEST(TrackManager, AssignsMonotonicStableIds) {
  TrackManager mgr(3, 3);
  const TrackId id1 = mgr.add(Track{});
  const TrackId id2 = mgr.add(Track{});
  EXPECT_EQ(id1.value, 1u);
  EXPECT_EQ(id2.value, 2u);
  EXPECT_EQ(mgr.size(), 2u);
  EXPECT_EQ(mgr.tracks()[0].id, id1);
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
}

TEST(TrackManager, ConfirmsAfterConsecutiveHits) {
  TrackManager mgr(3, 3);  // confirm after 3 detections
  const TrackId id = mgr.add(Track{});  // detection #1 (hits=1)
  mgr.recordHit(id);                    // detection #2
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
  mgr.recordHit(id);                    // detection #3 -> Confirmed
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Confirmed);
}

TEST(TrackManager, CoastsThenDeletesAfterMisses) {
  TrackManager mgr(2, 2);  // confirm after 2 hits, delete after 2 misses
  const TrackId id = mgr.add(Track{});  // hits=1, Tentative
  mgr.recordHit(id);                    // hits=2 -> Confirmed
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Confirmed);
  mgr.recordMiss(id);                   // misses=1 -> Coasting
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Coasting);
  mgr.recordMiss(id);                   // misses=2 -> Deleted (removed)
  EXPECT_EQ(mgr.size(), 0u);
}

// W5.4: a never-confirmed Tentative track that misses must NOT become Coasting.
// Coasting is CPA-eligible (CpaEvaluator gates on Confirmed||Coasting) while
// Tentative is not, so a one-hit clutter blip that then misses would otherwise
// emit false collision-risk events; it also violates the documented Coasting
// definition ("was Confirmed"). A missed Tentative stays Tentative and dies per
// M-of-N — it never Coasts. (The legitimate Confirmed->Coasting path is still
// guarded by CoastsThenDeletesAfterMisses above.)
TEST(TrackManager, TentativeMissStaysTentativeNeverCoasting) {
  TrackManager mgr(3, 3);  // confirm after 3 hits, delete after 3 misses
  const TrackId id = mgr.add(Track{});  // hits=1, Tentative, never confirmed
  ASSERT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
  mgr.recordMiss(id);                   // misses=1 < 3
  ASSERT_EQ(mgr.size(), 1u);            // not deleted yet
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative)  // TEETH: was Coasting
      << "a missed Tentative must not be promoted to CPA-eligible Coasting";
  mgr.recordMiss(id);                   // misses=2 < 3, still Tentative
  EXPECT_EQ(mgr.tracks()[0].status, TrackStatus::Tentative);
  mgr.recordMiss(id);                   // misses=3 -> deleted, never having Coasted
  EXPECT_EQ(mgr.size(), 0u);
}

// Guards the TrackId->index map (review #9): deleting a middle track shifts
// the tail of tracks_ down by one, so every surviving track's cached index
// must still resolve to the correct element afterward.
TEST(TrackManager, ResolvesIdsCorrectlyAfterMiddleErase) {
  TrackManager mgr(2, 1);  // confirm after 2 hits, delete after 1 miss
  const TrackId id1 = mgr.add(Track{}, navtracker::Timestamp::fromSeconds(1.0));
  const TrackId id2 = mgr.add(Track{}, navtracker::Timestamp::fromSeconds(2.0));
  const TrackId id3 = mgr.add(Track{}, navtracker::Timestamp::fromSeconds(3.0));
  ASSERT_EQ(mgr.size(), 3u);

  // Delete the middle track; id1/id3 must still be addressable.
  mgr.recordMiss(id2);
  ASSERT_EQ(mgr.size(), 2u);

  // lastObservation must still map to the right surviving tracks.
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id1).seconds(), 1.0);
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id3).seconds(), 3.0);
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id2).seconds(), 0.0);  // gone -> default

  // A hit on the post-shift track (id3, now at index 1) must confirm id3,
  // not the track that previously occupied that slot.
  mgr.recordHit(id3);  // hits=2 -> Confirmed
  for (const auto& t : mgr.tracks()) {
    if (t.id == id3) EXPECT_EQ(t.status, TrackStatus::Confirmed);
    if (t.id == id1) EXPECT_EQ(t.status, TrackStatus::Tentative);
  }
}

TEST(TrackManager, TracksLastObservationAndPredictAll) {
  TrackManager mgr(3, 3);
  const TrackId id =
      mgr.add(Track{}, navtracker::Timestamp::fromSeconds(10.0));
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id).seconds(), 10.0);

  mgr.noteObservation(id, navtracker::Timestamp::fromSeconds(15.5));
  EXPECT_DOUBLE_EQ(mgr.lastObservation(id).seconds(), 15.5);

  // mutableTracks gives writable access (the EKF will write through it).
  mgr.mutableTracks()[0].state = Eigen::VectorXd::Zero(4);
  EXPECT_EQ(mgr.tracks()[0].state.size(), 4);
}
