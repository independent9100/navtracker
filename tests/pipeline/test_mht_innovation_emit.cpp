#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IInnovationSink.hpp"

using namespace navtracker;

namespace {

class RecordingSink : public IInnovationSink {
 public:
  std::vector<InnovationEvent> events;
  void onInnovation(const InnovationEvent& e) override { events.push_back(e); }
};

Measurement pos(double x, double y, double t, const std::string& src = "p") {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.sensor = SensorKind::Ais;
  m.source_id = src;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity();
  return m;
}

}  // namespace

// Null sink → no emission, no overhead, behaviour unchanged.
TEST(MhtInnovationEmit, NullSinkIsSafe) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(est, cfg);
  for (int i = 1; i <= 8; ++i) {
    mht.processBatch({pos(static_cast<double>(i) * 5.0, 0.0,
                          static_cast<double>(i))});
  }
  EXPECT_EQ(mht.tracks().size(), 1u);
}

// Single target tracked cleanly: exactly one InnovationEvent per scan
// after the birth scan. Emission is once per measurement on the
// surviving (chosen) leaf — not on pruned alternatives.
TEST(MhtInnovationEmit, OneEventPerScanOnSurvivingLeaf) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  MhtTracker::Config cfg;
  cfg.k_best = 3;  // protect alternatives — emission must still be 1/scan
  MhtTracker mht(est, cfg);
  RecordingSink sink;
  mht.setInnovationSink(&sink);

  const int kScans = 10;
  for (int i = 1; i <= kScans; ++i) {
    mht.processBatch({pos(static_cast<double>(i) * 5.0, 0.0,
                          static_cast<double>(i))});
  }
  // Birth scan emits no innovation (no prior). Subsequent scans emit
  // exactly one per surviving tree (= 1).
  EXPECT_EQ(sink.events.size(), static_cast<std::size_t>(kScans - 1));
  for (const auto& e : sink.events) {
    EXPECT_EQ(e.dim, 2u);
    EXPECT_EQ(e.model, MeasurementModel::Position2D);
    EXPECT_EQ(e.sensor, SensorKind::Ais);
    EXPECT_EQ(e.residual.size(), 2);
    EXPECT_TRUE(e.S.isApprox(e.S.transpose(), 1e-9));
  }
}

// Stale-scan guard ON by default → emission skipped along with the
// rest of processBatch.
TEST(MhtInnovationEmit, StaleScanSilent) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator est(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(est, cfg);
  RecordingSink sink;
  mht.setInnovationSink(&sink);
  mht.processBatch({pos(0.0, 0.0, 5.0)});
  mht.processBatch({pos(5.0, 0.0, 6.0)});
  const std::size_t before = sink.events.size();
  mht.processBatch({pos(-5.0, 0.0, 1.0)});  // stale
  EXPECT_EQ(sink.events.size(), before);
  EXPECT_GT(mht.staleDropped(), 0u);
}
