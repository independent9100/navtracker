#include <gtest/gtest.h>
#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::MhtTracker;
using navtracker::Timestamp;

namespace {

Measurement positionMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "t";
  return m;
}

}  // namespace

TEST(MhtTracker, SingleTargetCleanlyTracked) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  for (int i = 1; i <= 10; ++i) {
    mht.processBatch({positionMeas(static_cast<double>(i) * 5.0, 0.0,
                                   static_cast<double>(i))});
  }
  EXPECT_EQ(mht.treeCount(), 1u);
  EXPECT_EQ(mht.tracks().size(), 1u);
}

TEST(MhtTracker, NewMeasurementsGetTheirOwnTrees) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  mht.processBatch({positionMeas(   0.0,   0.0, 1.0),
                    positionMeas(1000.0, 1000.0, 1.0)});
  EXPECT_EQ(mht.treeCount(), 2u);
}
