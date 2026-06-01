#include "sim/EoIrEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/eoir/EoIrAdapter.hpp"
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

OwnShipProvider makeProviderAtOrigin() {
  OwnShipProvider own;
  OwnShipPose pose;
  pose.time = Timestamp::fromSeconds(0.0);
  pose.lat_deg = 53.5;
  pose.lon_deg = 8.0;
  pose.heading_true_deg = 0.0;
  own.update(pose);
  return own;
}

}  // namespace

TEST(EoIrEmitter, EmitsAtConfiguredCadence) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;  // 10 Hz

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/1);

  const Eigen::Vector2d own_pos(0.0, 0.0);
  const Eigen::Vector2d tgt_pos(0.0, 1000.0);  // 1 km north (ENU +y) — in beam

  emitter.emit(makeCtx(0.0, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(0.05, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(0.10, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(0.31, own_pos, 1, tgt_pos));
  EXPECT_EQ(adapter.poll().size(), 2u);  // ticks at 0.20 and 0.30
}

TEST(EoIrEmitter, FovGateSkipsOutOfBeamTargets) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.fov_deg = 60.0;  // ±30° around boresight (0° relative = north)
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/2);

  // Target due NORTH (relative bearing 0°) — in beam.
  emitter.emit(makeCtx(0.0, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(0.0, 1000.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);

  // Target due EAST (relative bearing 90°) — outside ±30° FOV.
  emitter.emit(makeCtx(0.1, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);
}

TEST(EoIrEmitter, RangeGateSkipsBeyondMaxRange) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  EoIrAdapter adapter(datum, own);

  sim::EoIrEmitterConfig cfg;
  cfg.targets.push_back({1, 5});
  cfg.fov_deg = 360.0;       // disable FOV gate
  cfg.bearing_std_deg = 0.0;
  cfg.range_std_m = 0.0;
  cfg.dt_s = 0.1;
  cfg.max_range_m = 500.0;

  sim::EoIrEmitter emitter(adapter, cfg, /*seed=*/3);

  emitter.emit(makeCtx(0.0, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(0.1, Eigen::Vector2d::Zero(), 1, Eigen::Vector2d(300.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);
}
