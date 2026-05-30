#include <gtest/gtest.h>
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
