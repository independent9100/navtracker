#include <gtest/gtest.h>

#include <map>
#include <set>
#include <vector>

#include "core/scenario/TruthResample.hpp"

using navtracker::resampleTruthToClock;
using navtracker::Timestamp;
using navtracker::TruthSample;

namespace {

TruthSample sample(double t, std::uint64_t id, double x, double y) {
  TruthSample s;
  s.time = Timestamp::fromSeconds(t);
  s.truth_id = id;
  s.position = Eigen::Vector2d(x, y);
  return s;
}

// Bucket resampled output by tick time (the BenchRunner grouping
// invariant: same-tick samples must carry the identical Timestamp and
// be contiguous in time order).
std::map<double, std::vector<TruthSample>> byTick(
    const std::vector<TruthSample>& out) {
  std::map<double, std::vector<TruthSample>> m;
  for (const auto& s : out) m[s.time.seconds()].push_back(s);
  return m;
}

}  // namespace

TEST(TruthResample, TwoAsyncTargetsShareTickTimes) {
  // Target 1 reports at 0.0 / 10.0 s, target 2 at 0.3 / 9.7 s — the
  // philos AIS pattern: no two raw samples share a timestamp, so the
  // raw stream fragments into 1-truth evaluation steps. Resampled at
  // 1 Hz, every interior tick must carry BOTH targets at the same
  // Timestamp.
  std::vector<TruthSample> raw = {
      sample(0.0, 1, 0.0, 0.0), sample(10.0, 1, 100.0, 0.0),
      sample(0.3, 2, 0.0, 50.0), sample(9.7, 2, 94.0, 50.0)};
  const auto out = resampleTruthToClock(raw, 1.0, 30.0);
  ASSERT_FALSE(out.empty());
  const auto ticks = byTick(out);
  int both = 0;
  for (const auto& [t, samples] : ticks) {
    std::set<std::uint64_t> ids;
    for (const auto& s : samples) ids.insert(s.truth_id);
    if (ids.size() == 2u) ++both;
  }
  EXPECT_GE(both, 9);  // ticks 1..9 lie inside both spans
}

TEST(TruthResample, LinearInterpolationOfPositionAndFdVelocity) {
  // One target moving east at 10 m/s, fixes 10 s apart. At the tick
  // 4 s past the first fix the interpolated position is 40 m east and
  // the velocity is the segment finite difference.
  std::vector<TruthSample> raw = {sample(0.0, 7, 0.0, 0.0),
                                  sample(10.0, 7, 100.0, 0.0)};
  const auto out = resampleTruthToClock(raw, 1.0, 30.0);
  const auto ticks = byTick(out);
  const auto it = ticks.find(4.0);
  ASSERT_NE(it, ticks.end());
  ASSERT_EQ(it->second.size(), 1u);
  EXPECT_NEAR(it->second[0].position.x(), 40.0, 1e-9);
  EXPECT_NEAR(it->second[0].position.y(), 0.0, 1e-9);
  EXPECT_NEAR(it->second[0].velocity.x(), 10.0, 1e-9);
  EXPECT_NEAR(it->second[0].velocity.y(), 0.0, 1e-9);
}

TEST(TruthResample, GapLongerThanMaxGapBreaksPresence) {
  // Fixes at 0 and 100 s with max_gap 30 s: the target must be absent
  // at interior ticks (no 100 s straight-line bridging), but present
  // near both endpoints.
  std::vector<TruthSample> raw = {sample(0.0, 3, 0.0, 0.0),
                                  sample(100.0, 3, 1000.0, 0.0)};
  const auto out = resampleTruthToClock(raw, 1.0, 30.0);
  const auto ticks = byTick(out);
  EXPECT_TRUE(ticks.count(0.0));
  EXPECT_TRUE(ticks.count(100.0));
  EXPECT_FALSE(ticks.count(50.0));
}

TEST(TruthResample, SingleFixTargetAppearsAtExactlyOneTick) {
  // A target with one AIS message (common in the philos fixture) must
  // appear at its nearest clock tick — and only there — so the tracker
  // is not charged a phantom cardinality error for its whole lifetime.
  std::vector<TruthSample> raw = {sample(0.0, 1, 0.0, 0.0),
                                  sample(20.0, 1, 200.0, 0.0),
                                  sample(7.4, 9, 5.0, 5.0)};
  const auto out = resampleTruthToClock(raw, 1.0, 30.0);
  int target9 = 0;
  double t9 = -1.0;
  for (const auto& s : out) {
    if (s.truth_id == 9u) {
      ++target9;
      t9 = s.time.seconds();
    }
  }
  EXPECT_EQ(target9, 1);
  EXPECT_NEAR(t9, 7.0, 1e-9);
}

TEST(TruthResample, OutputIsSortedAndTickContiguous) {
  std::vector<TruthSample> raw = {
      sample(0.2, 2, 0.0, 50.0), sample(0.0, 1, 0.0, 0.0),
      sample(9.7, 2, 94.0, 50.0), sample(10.0, 1, 100.0, 0.0)};
  const auto out = resampleTruthToClock(raw, 1.0, 30.0);
  for (std::size_t i = 1; i < out.size(); ++i) {
    const bool time_ok = !(out[i].time < out[i - 1].time);
    EXPECT_TRUE(time_ok) << "out of order at " << i;
    if (out[i].time == out[i - 1].time) {
      EXPECT_GT(out[i].truth_id, out[i - 1].truth_id)
          << "same-tick samples must be id-ordered for determinism";
    }
  }
}

TEST(TruthResample, EmptyInputAndDisabledPeriod) {
  EXPECT_TRUE(resampleTruthToClock({}, 1.0, 30.0).empty());
  // Non-positive period = passthrough (caller keeps raw truth).
  std::vector<TruthSample> raw = {sample(0.0, 1, 0.0, 0.0)};
  const auto out = resampleTruthToClock(raw, 0.0, 30.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].truth_id, 1u);
}
