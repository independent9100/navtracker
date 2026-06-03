#include "sim/OwnShipEmitter.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "sim/TruthTrajectory.hpp"

using namespace navtracker;
using navtracker::geo::Datum;

TEST(OwnShipEmitter, EmitsGgaAndHdtAtOneHz) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),       // starts at datum origin
      Eigen::Vector2d(2.0, 0.0),     // 2 m/s east
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 0.0;   // no noise for this test
  cfg.heading_true_deg = 90.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/42);

  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  emitter.emit(ctx);
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->lat_deg, 53.5, 1e-6);
  EXPECT_NEAR(provider.latest()->heading_true_deg, 90.0, 1e-6);

  // 0.5 s later: cadence is 1 Hz, nothing should emit.
  const auto previous = *provider.latest();
  ctx.now = Timestamp::fromSeconds(0.5);
  emitter.emit(ctx);
  EXPECT_DOUBLE_EQ(provider.latest()->lon_deg, previous.lon_deg);

  // 1.0 s: next emission. Ownship has moved 2 m east => lon_deg slightly larger.
  ctx.now = Timestamp::fromSeconds(1.0);
  emitter.emit(ctx);
  EXPECT_GT(provider.latest()->lon_deg, previous.lon_deg);
}

TEST(OwnShipEmitter, AppliesGpsPositionNoise) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 5.0;
  cfg.heading_true_deg = 0.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/123);

  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  emitter.emit(ctx);

  // With 5 m noise, lat should deviate from the truth (53.5).
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NE(provider.latest()->lat_deg, 53.5);
  // But not by more than ~5*sigma worth of metres (1 deg lat ~= 111 km).
  EXPECT_NEAR(provider.latest()->lat_deg, 53.5, 30.0 / 111000.0);
}

TEST(OwnShipEmitter, HeadingWrapsToZero360Range) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 0.0;
  cfg.heading_true_deg = 370.0;  // > 360, should wrap to 10
  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/1);

  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  emitter.emit(ctx);
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->heading_true_deg, 10.0, 1e-6);

  cfg.heading_true_deg = -45.0;  // negative, should wrap to 315
  sim::OwnShipEmitter emitter2(adapter, datum, *traj, cfg, /*seed=*/2);
  ctx.now = Timestamp::fromSeconds(0.0);
  emitter2.emit(ctx);
  EXPECT_NEAR(provider.latest()->heading_true_deg, 315.0, 1e-6);
}

TEST(OwnShipEmitter, HeadingNoiseShowsUpAsExpectedStddev) {
  using namespace navtracker;
  geo::Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 0.0;
  cfg.heading_true_deg = 0.0;
  cfg.heading_noise_std_deg = 2.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/42);

  // Pull a few hundred 1-Hz HDT ticks and accumulate the parsed heading.
  std::vector<double> samples;
  for (int k = 0; k < 400; ++k) {
    sim::EmitContext ctx;
    ctx.now = Timestamp::fromSeconds(static_cast<double>(k));
    emitter.emit(ctx);
    ASSERT_TRUE(provider.latest().has_value());
    double h = provider.latest()->heading_true_deg;
    // Heading wraps to [0, 360). Re-centre near 0 by mapping (180, 360) to
    // (-180, 0) so stats around the nominal 0° aren't broken by the wrap.
    if (h > 180.0) h -= 360.0;
    samples.push_back(h);
  }

  double mean = 0.0;
  for (double s : samples) mean += s;
  mean /= static_cast<double>(samples.size());
  double sse = 0.0;
  for (double s : samples) sse += (s - mean) * (s - mean);
  const double stddev =
      std::sqrt(sse / static_cast<double>(samples.size() - 1));

  EXPECT_NEAR(mean, 0.0, 0.3);          // sample mean near 0 with N=400
  EXPECT_NEAR(stddev, 2.0, 0.3);        // empirical stddev near σ
}

TEST(OwnShipEmitter, EmittedPoseCarriesGpsStd) {
  Datum datum({53.5, 8.0, 0.0});
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);

  auto traj = std::make_shared<sim::ConstantVelocityTrajectory>(
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Timestamp::fromSeconds(0.0));

  sim::OwnShipEmitterConfig cfg;
  cfg.dt_s = 1.0;
  cfg.gps_pos_std_m = 5.0;
  cfg.heading_true_deg = 0.0;

  sim::OwnShipEmitter emitter(adapter, datum, *traj, cfg, /*seed=*/7);

  sim::EmitContext ctx;
  ctx.now = Timestamp::fromSeconds(0.0);
  emitter.emit(ctx);

  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_DOUBLE_EQ(provider.latest()->position_std_m, 5.0);
}
