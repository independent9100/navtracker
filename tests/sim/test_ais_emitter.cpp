#include "sim/AisEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext makeCtx(double t_seconds,
                         std::uint64_t target_id,
                         const Eigen::Vector2d& target_pos,
                         const Eigen::Vector2d& target_vel) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{};
  ctx.targets.push_back(sim::TargetTruth{target_id, sim::TruthState{target_pos, target_vel}});
  return ctx;
}

}  // namespace

TEST(AisEmitter, EmitsAtSpeedBasedCadenceSlow) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/1, /*mmsi=*/200000001u, /*high_accuracy=*/true});
  cfg.pos_std_m = 0.0;

  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  // Slow target (3 m/s ~ 5.8 kn -> 10 s cadence). Emit at t=0, then nothing
  // until t=10.
  emitter.emit(makeCtx(0.0, 1, Eigen::Vector2d::Zero(), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);

  emitter.emit(makeCtx(5.0, 1, Eigen::Vector2d(15.0, 0.0), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 0u);

  emitter.emit(makeCtx(10.0, 1, Eigen::Vector2d(30.0, 0.0), Eigen::Vector2d(3.0, 0.0)));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(AisEmitter, EmitsAtSpeedBasedCadenceFast) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({/*truth_id=*/2, /*mmsi=*/200000002u, /*high_accuracy=*/true});
  cfg.pos_std_m = 0.0;
  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/2);

  // Fast target (15 m/s ~ 29 kn -> 2 s cadence).
  const Eigen::Vector2d v(15.0, 0.0);
  emitter.emit(makeCtx(0.0, 2, Eigen::Vector2d::Zero(), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(1.5, 2, Eigen::Vector2d(22.5, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 0u);
  emitter.emit(makeCtx(2.0, 2, Eigen::Vector2d(30.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(4.0, 2, Eigen::Vector2d(60.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(AisEmitter, DropoutWindowSuppressesEmission) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({1u, 200000001u, true});
  cfg.dropout_windows_s.emplace_back(5.0, 25.0);
  cfg.pos_std_m = 0.0;

  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/3);

  const Eigen::Vector2d v(3.0, 0.0);  // 10 s cadence
  emitter.emit(makeCtx(0.0,  1, Eigen::Vector2d::Zero(),       v));
  EXPECT_EQ(adapter.poll().size(), 1u);
  emitter.emit(makeCtx(10.0, 1, Eigen::Vector2d(30.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // dropped
  emitter.emit(makeCtx(20.0, 1, Eigen::Vector2d(60.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // dropped
  emitter.emit(makeCtx(30.0, 1, Eigen::Vector2d(90.0, 0.0),     v));
  EXPECT_EQ(adapter.poll().size(), 1u);
}

TEST(AisEmitter, DropoutPreservesCadencePhase) {
  Datum datum({53.5, 8.0, 0.0});
  AisAdapter adapter(datum);

  sim::AisEmitterConfig cfg;
  cfg.targets.push_back({1u, 200000001u, true});
  // Window NOT aligned to the 10 s cadence — start mid-period, end mid-period.
  cfg.dropout_windows_s.emplace_back(5.0, 12.0);
  cfg.pos_std_m = 0.0;

  sim::AisEmitter emitter(adapter, datum, cfg, /*seed=*/4);

  const Eigen::Vector2d v(3.0, 0.0);  // 10 s cadence

  emitter.emit(makeCtx(0.0,  1, Eigen::Vector2d::Zero(),    v));
  EXPECT_EQ(adapter.poll().size(), 1u);  // natural slot at t=0
  emitter.emit(makeCtx(10.0, 1, Eigen::Vector2d(30.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // dropped (10 in [5,12))
  // The next natural slot is t=20, NOT t=12. A buggy impl that resets
  // next_emit_ on dropout exit would emit at t=12 here.
  emitter.emit(makeCtx(15.0, 1, Eigen::Vector2d(45.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 0u);  // still nothing — out of dropout but not at a slot
  emitter.emit(makeCtx(20.0, 1, Eigen::Vector2d(60.0, 0.0), v));
  EXPECT_EQ(adapter.poll().size(), 1u);  // natural slot at t=20
}
