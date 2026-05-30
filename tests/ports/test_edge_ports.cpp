#include <vector>

#include <gtest/gtest.h>
#include "ports/ISensorAdapter.hpp"
#include "ports/ITrackSink.hpp"

namespace {
class FakeAdapter : public navtracker::ISensorAdapter {
 public:
  std::vector<navtracker::Measurement> poll() override { return {}; }
};
class FakeSink : public navtracker::ITrackSink {
 public:
  void onTracks(const std::vector<navtracker::Track>&,
                navtracker::Timestamp) override {
    ++calls;
  }
  int calls = 0;
};
}  // namespace

TEST(EdgePorts, FakesImplementAndDispatch) {
  FakeAdapter a;
  navtracker::ISensorAdapter& ar = a;
  EXPECT_TRUE(ar.poll().empty());

  FakeSink s;
  navtracker::ITrackSink& sr = s;
  sr.onTracks({}, navtracker::Timestamp::fromSeconds(1.0));
  EXPECT_EQ(s.calls, 1);
}
