#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/ReorderBuffer.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Harness.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Ids.hpp"
#include "sim/SkewInjector.hpp"

using namespace navtracker;

namespace {

struct BufferedRun {
  ScenarioResult result;
  std::size_t dropped{0};
  std::size_t buffer_drains{0};
  bool all_drains_monotonic{true};
};

BufferedRun runBuffered(const Scenario& scenario_in_arrival_order,
                        double window_s,
                        double ospa_cutoff,
                        Tracker& tracker,
                        TrackManager& mgr) {
  BufferedRun out;
  ReorderBuffer buf(window_s);

  std::vector<Measurement> flushed_in_order;
  flushed_in_order.reserve(scenario_in_arrival_order.measurements.size());

  Timestamp last_drained{};
  bool last_drained_set = false;
  for (const auto& m : scenario_in_arrival_order.measurements) {
    buf.push(m);
    const auto drained = buf.drain();
    ++out.buffer_drains;
    for (const auto& dm : drained) {
      if (last_drained_set && dm.time < last_drained) {
        out.all_drains_monotonic = false;
      }
      last_drained = dm.time;
      last_drained_set = true;
      flushed_in_order.push_back(dm);
    }
  }

  // Flush tail: push a synthetic future measurement so the cutoff advances
  // past every remaining real measurement. The marker itself stays in the
  // buffer (its time equals `latest`, so cutoff = latest - W < marker.time).
  if (!scenario_in_arrival_order.measurements.empty()) {
    Measurement flush_marker;
    flush_marker.time = Timestamp{
        scenario_in_arrival_order.measurements.back().time.nanos() +
        static_cast<std::int64_t>(window_s * 1e9) * 4};
    flush_marker.sensor = SensorKind::Unknown;
    flush_marker.model = MeasurementModel::Position2D;
    buf.push(flush_marker);
    const auto tail = buf.drain();
    for (const auto& dm : tail) {
      if (last_drained_set && dm.time < last_drained) {
        out.all_drains_monotonic = false;
      }
      last_drained = dm.time;
      last_drained_set = true;
      flushed_in_order.push_back(dm);
    }
  }
  out.dropped = buf.dropped();

  Scenario s;
  s.measurements = std::move(flushed_in_order);
  s.truth = scenario_in_arrival_order.truth;
  out.result = runScenario(s, tracker, mgr, ospa_cutoff);
  return out;
}

// Mirror ReorderBuffer's drop policy: an item arriving in arrival order
// is dropped iff its truth time is strictly less than (latest_seen - W),
// where latest_seen is the rolling max of truth times across NON-DROPPED
// items pushed so far. Replays applySkew to get the same arrival sequence.
std::size_t expectedLateCount(const std::vector<Measurement>& truth_ordered,
                              const SkewProfile& profile,
                              std::uint64_t seed,
                              double window_s) {
  const auto arrival = applySkew(truth_ordered, profile, seed);
  const std::int64_t window_ns =
      static_cast<std::int64_t>(window_s * 1e9);
  std::size_t late = 0;
  std::int64_t latest_ns = 0;
  bool seen = false;
  for (const auto& m : arrival) {
    if (!seen) {
      latest_ns = m.time.nanos();
      seen = true;
      continue;
    }
    if (m.time.nanos() < latest_ns - window_ns) {
      ++late;
      continue;  // dropped, does not update latest
    }
    if (m.time.nanos() > latest_ns) latest_ns = m.time.nanos();
  }
  return late;
}

bool stepsEqual(const ScenarioResult& a, const ScenarioResult& b) {
  if (a.steps.size() != b.steps.size()) return false;
  for (std::size_t i = 0; i < a.steps.size(); ++i) {
    const auto& sa = a.steps[i];
    const auto& sb = b.steps[i];
    if (sa.time.nanos() != sb.time.nanos()) return false;
    if (sa.tracks.size() != sb.tracks.size()) return false;
    for (std::size_t j = 0; j < sa.tracks.size(); ++j) {
      if (sa.tracks[j].id != sb.tracks[j].id) return false;
      if (sa.tracks[j].position != sb.tracks[j].position) return false;
    }
  }
  return true;
}

Scenario buildCrossing() {
  std::vector<double> times;
  for (int i = 1; i <= 40; ++i) times.push_back(static_cast<double>(i));
  return buildCrossingTargetsScenario(
      Eigen::Vector2d(-500.0, 10.0), Eigen::Vector2d(25.0, 0.0),
      Eigen::Vector2d(500.0, -10.0), Eigen::Vector2d(-25.0, 0.0),
      times, 8.0, 11);
}

Scenario buildAisDropout() {
  std::vector<double> times;
  for (int i = 1; i <= 5; ++i) times.push_back(static_cast<double>(i));
  for (int i = 12; i <= 20; ++i) times.push_back(static_cast<double>(i));
  return buildStraightLineScenario(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0),
      times, 5.0, 3);
}

struct Pipeline {
  std::shared_ptr<ConstantVelocity2D> motion;
  EkfEstimator est;
  GnnAssociator assoc;
  TrackManager mgr;
  Tracker tracker;
  Pipeline(double assoc_gate, std::size_t min_hits, std::size_t miss_to_delete,
           double prune_age)
      : motion(std::make_shared<ConstantVelocity2D>(0.1)),
        est(motion, 5.0),
        assoc(assoc_gate),
        mgr(min_hits, miss_to_delete),
        tracker(est, assoc, mgr, prune_age) {}
};

constexpr double kComfortableWindow = 5.0;  // see spec §I-2
constexpr double kTightWindow = 0.25;
constexpr std::uint64_t kSeed = 42;

}  // namespace

// ===== Crossing =====

TEST(ReorderBufferE2E, CrossingDeterminismUnderSkew) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p1(50.0, 2, 4, 30.0);
  const auto r1 = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                              p1.tracker, p1.mgr);
  Pipeline p2(50.0, 2, 4, 30.0);
  const auto r2 = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                              p2.tracker, p2.mgr);

  EXPECT_TRUE(stepsEqual(r1.result, r2.result));  // I-1
  EXPECT_TRUE(r1.all_drains_monotonic);           // I-4
  EXPECT_TRUE(r2.all_drains_monotonic);
}

TEST(ReorderBufferE2E, CrossingAccuracyParityVsBaseline) {
  const Scenario truth_ordered = buildCrossing();
  Pipeline pb(50.0, 2, 4, 30.0);
  const auto baseline =
      runScenario(truth_ordered, pb.tracker, pb.mgr, 50.0);

  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);
  Pipeline ps(50.0, 2, 4, 30.0);
  const auto skewed = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                                  ps.tracker, ps.mgr);

  const double rel_tol = 0.05;
  const double abs_tol = 0.5;
  const double diff = std::abs(skewed.result.mean_ospa - baseline.mean_ospa);
  const double allowed =
      std::max(abs_tol, rel_tol * std::abs(baseline.mean_ospa));
  EXPECT_LE(diff, allowed) << "mean_ospa baseline=" << baseline.mean_ospa
                           << " skewed=" << skewed.result.mean_ospa;

  EXPECT_EQ(ps.mgr.size(), pb.mgr.size());
}

TEST(ReorderBufferE2E, CrossingComfortableWindowDropsZero) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(50.0, 2, 4, 30.0);
  const auto r = runBuffered(arrival_ordered, kComfortableWindow, 50.0,
                             p.tracker, p.mgr);

  EXPECT_EQ(r.dropped, 0u);  // I-2 (comfortable)
}

TEST(ReorderBufferE2E, CrossingTightWindowDropsMatchGroundTruth) {
  const Scenario truth_ordered = buildCrossing();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(80.0, 2, 4, 30.0);
  const auto r = runBuffered(arrival_ordered, kTightWindow, 80.0,
                             p.tracker, p.mgr);

  const std::size_t expected =
      expectedLateCount(truth_ordered.measurements, profile, kSeed,
                        kTightWindow);

  EXPECT_EQ(r.dropped, expected);  // I-2 (tight)
  EXPECT_GT(expected, 0u);
}

// ===== AIS dropout =====

TEST(ReorderBufferE2E, AisDropoutDeterminismUnderSkew) {
  const Scenario truth_ordered = buildAisDropout();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p1(80.0, 2, 5, 15.0);
  const auto r1 = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                              p1.tracker, p1.mgr);
  Pipeline p2(80.0, 2, 5, 15.0);
  const auto r2 = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                              p2.tracker, p2.mgr);

  EXPECT_TRUE(stepsEqual(r1.result, r2.result));
  EXPECT_TRUE(r1.all_drains_monotonic);
  EXPECT_TRUE(r2.all_drains_monotonic);
}

TEST(ReorderBufferE2E, AisDropoutTrackSurvivesSkew) {
  const Scenario truth_ordered = buildAisDropout();
  const auto profile = defaultMaritimeSkewProfile();
  Scenario arrival_ordered = truth_ordered;
  arrival_ordered.measurements =
      applySkew(truth_ordered.measurements, profile, kSeed);

  Pipeline p(80.0, 2, 5, 15.0);
  const auto r = runBuffered(arrival_ordered, kComfortableWindow, 80.0,
                             p.tracker, p.mgr);

  EXPECT_EQ(p.mgr.size(), 1u);
}
