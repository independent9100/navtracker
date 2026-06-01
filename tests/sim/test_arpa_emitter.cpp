#include "sim/ArpaEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext makeCtx(double t_seconds,
                         const Eigen::Vector2d& own_pos,
                         std::uint64_t target_id,
                         const Eigen::Vector2d& target_pos) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{own_pos, Eigen::Vector2d::Zero()};
  ctx.targets.push_back(sim::TargetTruth{target_id, sim::TruthState{target_pos, Eigen::Vector2d::Zero()}});
  return ctx;
}

}  // namespace

TEST(ArpaEmitter, EmitsAtRotationCadence) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/1, /*arpa_track_num=*/3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  cfg.rotation_dt_s = 3.0;

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  const Eigen::Vector2d own_pos(0.0, 0.0);
  const Eigen::Vector2d tgt_pos(1000.0, 0.0);  // 1 km east, 1 km range

  emitter.emit(makeCtx(0.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);

  emitter.emit(makeCtx(2.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 0u);

  emitter.emit(makeCtx(3.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(ArpaEmitter, ProducesMeasurementNearTruthInEnu) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;  // bow pointing north
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({1, 3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/2);

  // Target 1 km east of own-ship. ENU east is +x.
  emitter.emit(makeCtx(0.0,
                       Eigen::Vector2d(0.0, 0.0),
                       /*truth_id=*/1,
                       Eigen::Vector2d(1000.0, 0.0)));

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, MeasurementModel::Position2D);
  EXPECT_NEAR(out[0].value.x(), 1000.0, 2.0);
  EXPECT_NEAR(out[0].value.y(),    0.0, 2.0);
}

TEST(ArpaEmitter, SkipsTargetsOutsideRangeGate) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  ArpaAdapter adapter(datum, own);

  sim::ArpaEmitterConfig cfg;
  cfg.targets.push_back({1, 3});
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  cfg.max_range_m = 500.0;  // 500 m gate

  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/3);

  emitter.emit(makeCtx(0.0,
                       Eigen::Vector2d(0.0, 0.0),
                       /*truth_id=*/1,
                       Eigen::Vector2d(1000.0, 0.0)));   // out of range
  EXPECT_EQ(adapter.poll().size(), 0u);
}
