// Backlog #15: processBatch must not require a pre-sorted scan. The canonical
// fixed-rate consumer (collect every measurement since the last tick, hand the
// lot to the tracker) produces a batch that is NOT timestamp-sorted. The tracker
// treats a batch as one scan at its earliest instant; feeding it unsorted must
// yield the SAME result as feeding it pre-sorted, and must not silently drop the
// out-of-order tail. Both trackers sort the batch internally (stable, so
// deterministic; a no-op for already-sorted input).
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;
using navtracker::pmbm::PmbmTracker;

namespace {

Measurement pos(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.sensor = SensorKind::Ais;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 4.0;
  return m;
}

struct Summary {
  double x, y, t;
  bool operator==(const Summary& o) const {
    return std::abs(x - o.x) < 1e-6 && std::abs(y - o.y) < 1e-6 &&
           std::abs(t - o.t) < 1e-9;
  }
};

template <typename Tracker>
std::vector<Summary> summarize(const Tracker& trk) {
  std::vector<Summary> v;
  for (const Track& t : trk.tracks())
    v.push_back({t.state(0), t.state(1), t.last_update.seconds()});
  std::sort(v.begin(), v.end(),
            [](const Summary& a, const Summary& b) { return a.x < b.x; });
  return v;
}

// Two targets, each emitting once per tick at DIFFERENT sub-times within the
// tick (T1 at k.0, T2 at k.3). Feed 10 ticks two ways: one tracker gets each
// batch time-sorted, the other gets it reversed. The two must end identical.
template <typename Tracker>
std::pair<std::vector<Summary>, std::vector<Summary>> runBothOrders(
    Tracker& sorted, Tracker& reversed) {
  for (int k = 1; k <= 10; ++k) {
    const double x = 5.0 * k;
    Measurement t1 = pos(x, 0.0, k);          // earlier sub-time (k.0)
    Measurement t2 = pos(x, 120.0, k + 0.3);  // later sub-time  (k.3)
    sorted.processBatch({t1, t2});    // ascending by time
    reversed.processBatch({t2, t1});  // descending by time
  }
  return {summarize(sorted), summarize(reversed)};
}

}  // namespace

// MHT: the scan instant is scan.front().time, so an unsorted batch picks the
// wrong instant (and with reject_stale on, a front older than the high-water
// mark drops the whole batch). Sorting internally makes front() the earliest.
TEST(BatchOrdering, MhtUnsortedBatchMatchesSorted) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker sorted(est, cfg), reversed(est, cfg);
  auto [a, b] = runBothOrders(sorted, reversed);
  EXPECT_FALSE(a.empty());
  EXPECT_EQ(a, b);
  EXPECT_EQ(sorted.staleDropped(), 0u);
  EXPECT_EQ(reversed.staleDropped(), 0u);  // no tail silently dropped
}

// PMBM already predicts to t_max and associates set-wise, so it is order-robust
// for default configs; this guard locks that contract in (and the internal sort
// keeps it uniform with MHT + order-independent for the optional idle-decay knob).
TEST(BatchOrdering, PmbmUnsortedBatchMatchesSorted) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  PmbmTracker::Config cfg;
  cfg.adaptive_birth = true;
  cfg.birth_existence_target = 0.3;
  PmbmTracker sorted(est, cfg), reversed(est, cfg);
  auto [a, b] = runBothOrders(sorted, reversed);
  EXPECT_FALSE(a.empty());
  EXPECT_EQ(a, b);
}

// Determinism / no-op guard: an already-sorted batch is unaffected by the
// internal sort — same result fed once vs twice with identical inputs.
TEST(BatchOrdering, MhtSortedInputIsDeterministic) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  MhtTracker::Config cfg;
  auto run = [&]() {
    MhtTracker m(est, cfg);
    for (int k = 1; k <= 8; ++k)
      m.processBatch({pos(5.0 * k, 0.0, k), pos(5.0 * k, 120.0, k + 0.3)});
    return summarize(m);
  };
  EXPECT_EQ(run(), run());
}
