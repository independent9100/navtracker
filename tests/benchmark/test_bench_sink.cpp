#include <gtest/gtest.h>

#include "core/benchmark/BenchSink.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

using namespace navtracker;
using benchmark::BenchSink;

TEST(BenchSink, RecordsAllLifecycleEvents) {
  BenchSink sink;
  TrackLifecycleEvent e{TrackId{1}, Timestamp::fromSeconds(0.0), TrackStatus::Tentative};
  sink.onTrackInitiated(e);
  e.status = TrackStatus::Confirmed;
  e.time = Timestamp::fromSeconds(1.0);
  sink.onTrackConfirmed(e);
  e.time = Timestamp::fromSeconds(2.0);
  sink.onTrackUpdated(e);
  e.time = Timestamp::fromSeconds(3.0);
  sink.onTrackDeleted(e);

  const auto& events = sink.events();
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[0].kind, BenchSink::Kind::Initiated);
  EXPECT_EQ(events[1].kind, BenchSink::Kind::Confirmed);
  EXPECT_EQ(events[2].kind, BenchSink::Kind::Updated);
  EXPECT_EQ(events[3].kind, BenchSink::Kind::Deleted);
  EXPECT_EQ(events[0].id.value, 1u);
}
