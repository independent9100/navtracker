#include "sim/ArpaEmitter.hpp"

#include <gtest/gtest.h>

#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

namespace {

sim::EmitContext emptyCtx(double t_seconds, const Eigen::Vector2d& own_pos) {
  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(t_seconds);
  ctx.ownship_truth = sim::TruthState{own_pos, Eigen::Vector2d::Zero()};
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

TEST(ArpaClutter, NoClutterByDefault) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;  // default clutter_per_rotation = 0
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/1);

  for (double t : {0.0, 3.0, 6.0, 9.0}) {
    emitter.emit(emptyCtx(t, Eigen::Vector2d::Zero()));
  }
  EXPECT_EQ(adapter.poll().size(), 0u);
}

TEST(ArpaClutter, FiresApproximatelyPoissonCountPerRotation) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;
  cfg.clutter_per_rotation = 5;
  cfg.rotation_dt_s = 3.0;
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/7);

  // 30 s @ 3 s rotation -> 11 rotation slots (t=0,3,...,30) -> mean ~= 55.
  for (int i = 0; i <= 30; ++i) {
    emitter.emit(emptyCtx(static_cast<double>(i),
                          Eigen::Vector2d::Zero()));
  }
  const auto out = adapter.poll();
  // Bracket: with mean 55 and stddev sqrt(55) ~ 7.4, +/-20 covers ~2.7 sigma.
  EXPECT_GE(out.size(), 35u);
  EXPECT_LE(out.size(), 75u);
}

TEST(ArpaClutter, EveryClutterMeasurementWithinConfiguredRangeBox) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider own = makeProviderAtOrigin();
  ArpaAdapter adapter(datum, own);
  sim::ArpaEmitterConfig cfg;
  cfg.clutter_per_rotation = 10;
  cfg.clutter_min_range_m = 100.0;
  cfg.max_range_m = 1000.0;
  cfg.rotation_dt_s = 3.0;
  cfg.range_std_m = 0.0;
  cfg.bearing_std_deg = 0.0;
  sim::ArpaEmitter emitter(adapter, datum, cfg, /*seed=*/9);

  for (int i = 0; i <= 9; ++i) {
    emitter.emit(emptyCtx(static_cast<double>(i),
                          Eigen::Vector2d::Zero()));
  }
  const auto out = adapter.poll();
  ASSERT_GT(out.size(), 0u);
  for (const auto& m : out) {
    const double r = m.value.norm();  // own-ship at origin
    EXPECT_GE(r,  99.0);   // 1 m slack for numeric noise
    EXPECT_LE(r, 1001.0);
  }
}
