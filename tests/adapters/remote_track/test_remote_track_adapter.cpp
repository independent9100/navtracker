#include "adapters/remote_track/RemoteTrackAdapter.hpp"

#include <limits>
#include <set>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/geo/Datum.hpp"

using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::RemoteTrackAdapter;
using navtracker::RemoteTrackAdapterConfig;
using navtracker::RemoteTrackReport;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::geo::Datum;

namespace {
// Hamburg-ish origin (the R10 target deployment is a shore/VTS station).
Datum hh() { return Datum({53.54, 9.97, 0.0}); }

RemoteTrackReport at(double t_s, std::int32_t id, double lat, double lon) {
  RemoteTrackReport r;
  r.time = Timestamp::fromSeconds(t_s);
  r.remote_track_id = id;
  r.lat_deg = lat;
  r.lon_deg = lon;
  r.source_id = "vts_a";
  return r;
}
}  // namespace

TEST(RemoteTrackAdapter, IngestProducesRemoteTrackPosition2D) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);
  RemoteTrackReport r = at(100.0, 7, 53.54, 9.97);  // at the datum origin
  r.mmsi = 211000123u;
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  const Measurement& m = out[0];
  EXPECT_EQ(m.sensor, SensorKind::RemoteTrack);
  EXPECT_EQ(m.model, MeasurementModel::Position2D);
  EXPECT_EQ(m.source_id, "vts_a");
  EXPECT_NEAR(m.value(0), 0.0, 1e-6);
  EXPECT_NEAR(m.value(1), 0.0, 1e-6);
  ASSERT_TRUE(m.hints.sensor_track_id.has_value());
  EXPECT_EQ(*m.hints.sensor_track_id, 7);
  ASSERT_TRUE(m.hints.mmsi.has_value());
  EXPECT_EQ(*m.hints.mmsi, 211000123u);
  EXPECT_TRUE(adapter.poll().empty());  // drained
}

TEST(RemoteTrackAdapter, InflatesStatedCovarianceByFactor) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.r_inflation_factor = 3.0;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.position_covariance = Eigen::Matrix2d::Identity() * 100.0;  // σ=10 m
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  // Inflation multiplies the covariance MATRIX (variance) by the factor.
  EXPECT_DOUBLE_EQ(out[0].covariance(0, 0), 300.0);
  EXPECT_DOUBLE_EQ(out[0].covariance(1, 1), 300.0);
  EXPECT_FALSE(out[0].covariance_is_default);
}

TEST(RemoteTrackAdapter, PessimisticDefaultWhenNoStatedCovariance) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;  // default_position_std_m = 50
  RemoteTrackAdapter adapter(d, cfg);
  adapter.ingest(at(1.0, 1, 53.54, 9.97));  // covariance left zero

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].covariance(0, 0), 2500.0);  // 50²
  EXPECT_DOUBLE_EQ(out[0].covariance(1, 1), 2500.0);
  EXPECT_TRUE(out[0].covariance_is_default);
}

TEST(RemoteTrackAdapter, RejectsImplausibleLatLonAtEdge) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);
  adapter.ingest(at(1.0, 1, 91.0, 9.97));  // lat 91° = "not available" sentinel
  RemoteTrackReport nan = at(2.0, 2, 53.54, 9.97);
  nan.lon_deg = std::numeric_limits<double>::quiet_NaN();
  adapter.ingest(nan);

  EXPECT_TRUE(adapter.poll().empty());
  EXPECT_EQ(adapter.rejectedCount(), 2u);
}

// #26 M18: the feed gated only lat/lon. A NaN/Inf or non-PSD stated covariance
// and a non-finite velocity flowed straight into the Measurement, poisoning the
// track's uncertainty (and, downstream, aborting the estimator). Validate the
// stated covariances and (accepted) velocity at the edge.
TEST(RemoteTrackAdapter, RejectsNonFiniteStatedCovariance) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.position_covariance = Eigen::Matrix2d::Identity() * 100.0;
  r.position_covariance(0, 1) = std::numeric_limits<double>::quiet_NaN();
  r.position_covariance(1, 0) = std::numeric_limits<double>::quiet_NaN();
  adapter.ingest(r);
  EXPECT_TRUE(adapter.poll().empty());
  EXPECT_EQ(adapter.rejectedCount(), 1u);
}

TEST(RemoteTrackAdapter, RejectsNonPsdStatedCovariance) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  // Negative variance is finite but not a valid covariance.
  r.position_covariance = Eigen::Matrix2d::Identity() * -4.0;
  adapter.ingest(r);
  EXPECT_TRUE(adapter.poll().empty());
  EXPECT_EQ(adapter.rejectedCount(), 1u);
}

TEST(RemoteTrackAdapter, RejectsNonFiniteVelocityWhenAccepted) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.accept_velocity = true;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.has_velocity = true;
  r.velocity_enu = Eigen::Vector2d(std::numeric_limits<double>::infinity(), 0.0);
  adapter.ingest(r);
  EXPECT_TRUE(adapter.poll().empty());
  EXPECT_EQ(adapter.rejectedCount(), 1u);
}

TEST(RemoteTrackAdapter, ValidCovarianceAndVelocityStillAccepted) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.accept_velocity = true;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.position_covariance = Eigen::Matrix2d::Identity() * 25.0;
  r.has_velocity = true;
  r.velocity_enu = Eigen::Vector2d(1.0, -2.0);
  r.velocity_covariance = Eigen::Matrix2d::Identity() * 0.5;
  adapter.ingest(r);
  EXPECT_EQ(adapter.poll().size(), 1u);
  EXPECT_EQ(adapter.rejectedCount(), 0u);
}

TEST(RemoteTrackAdapter, RateThinningDropsTooSoonSameTrack) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.min_update_interval_s = 2.0;
  RemoteTrackAdapter adapter(d, cfg);
  adapter.ingest(at(10.0, 1, 53.54, 9.97));  // accepted (first)
  adapter.ingest(at(11.0, 1, 53.54, 9.97));  // +1 s < 2 s → thinned

  EXPECT_EQ(adapter.poll().size(), 1u);
  EXPECT_EQ(adapter.thinnedCount(), 1u);
}

TEST(RemoteTrackAdapter, RateThinningKeepsSpacedUpdates) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.min_update_interval_s = 2.0;
  RemoteTrackAdapter adapter(d, cfg);
  adapter.ingest(at(10.0, 1, 53.54, 9.97));
  adapter.ingest(at(12.5, 1, 53.54, 9.97));  // +2.5 s ≥ 2 s → accepted

  EXPECT_EQ(adapter.poll().size(), 2u);
  EXPECT_EQ(adapter.thinnedCount(), 0u);
}

TEST(RemoteTrackAdapter, RateThinningIsPerTrackId) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.min_update_interval_s = 2.0;
  RemoteTrackAdapter adapter(d, cfg);
  adapter.ingest(at(10.0, 1, 53.54, 9.97));
  adapter.ingest(at(10.0, 2, 53.54, 9.97));  // different id, same time → kept

  EXPECT_EQ(adapter.poll().size(), 2u);
  EXPECT_EQ(adapter.thinnedCount(), 0u);
}

TEST(RemoteTrackAdapter, RateThinningIsPerStation) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.min_update_interval_s = 2.0;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport a = at(10.0, 1, 53.54, 9.97);
  a.source_id = "vts_a";
  RemoteTrackReport b = at(10.0, 1, 53.54, 9.97);  // same track id...
  b.source_id = "vts_b";                            // ...different station
  adapter.ingest(a);
  adapter.ingest(b);

  EXPECT_EQ(adapter.poll().size(), 2u);  // disjoint per source_id
}

TEST(RemoteTrackAdapter, VelocityIgnoredWhenOptInOff) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);  // accept_velocity defaults false
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.has_velocity = true;
  r.velocity_enu = Eigen::Vector2d(2.0, -1.0);
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].model, MeasurementModel::Position2D);
  EXPECT_EQ(out[0].value.size(), 2);
}

TEST(RemoteTrackAdapter, VelocityProducesPositionVelocityWhenEnabled) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.accept_velocity = true;
  cfg.r_inflation_factor = 3.0;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.position_covariance = Eigen::Matrix2d::Identity() * 100.0;  // σ=10 m
  r.has_velocity = true;
  r.velocity_enu = Eigen::Vector2d(2.0, -1.0);
  r.velocity_covariance = Eigen::Matrix2d::Identity() * 4.0;  // σ=2 m/s
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  const Measurement& m = out[0];
  EXPECT_EQ(m.model, MeasurementModel::PositionVelocity2D);
  ASSERT_EQ(m.value.size(), 4);
  EXPECT_NEAR(m.value(0), 0.0, 1e-6);
  EXPECT_NEAR(m.value(1), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(m.value(2), 2.0);
  EXPECT_DOUBLE_EQ(m.value(3), -1.0);
  ASSERT_EQ(m.covariance.rows(), 4);
  ASSERT_EQ(m.covariance.cols(), 4);
  EXPECT_DOUBLE_EQ(m.covariance(0, 0), 300.0);  // 3×100 position
  EXPECT_DOUBLE_EQ(m.covariance(2, 2), 12.0);   // 3×4 velocity
  EXPECT_DOUBLE_EQ(m.covariance(3, 3), 12.0);
  // Off-diagonal position/velocity cross terms are zero (block-diagonal).
  EXPECT_DOUBLE_EQ(m.covariance(0, 2), 0.0);
}

TEST(RemoteTrackAdapter, VelocityDefaultWhenNoneStatedButOptInOn) {
  Datum d = hh();
  RemoteTrackAdapterConfig cfg;
  cfg.accept_velocity = true;
  cfg.default_velocity_std_mps = 3.0;
  RemoteTrackAdapter adapter(d, cfg);
  RemoteTrackReport r = at(1.0, 1, 53.54, 9.97);
  r.has_velocity = true;
  r.velocity_enu = Eigen::Vector2d(0.0, 0.0);  // no stated velocity_covariance
  adapter.ingest(r);

  const auto out = adapter.poll();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].covariance(2, 2), 9.0);  // 3²
  EXPECT_DOUBLE_EQ(out[0].covariance(3, 3), 9.0);
}

TEST(RemoteTrackAdapter, CircularAisGuardDetectsSharedMmsi) {
  Datum d = hh();
  RemoteTrackAdapter adapter(d);
  RemoteTrackReport a = at(1.0, 1, 53.54, 9.97);
  a.mmsi = 111u;
  RemoteTrackReport b = at(1.0, 2, 53.55, 9.98);
  b.mmsi = 222u;
  adapter.ingest(a);
  adapter.ingest(b);

  EXPECT_EQ(adapter.seenMmsis(), (std::set<std::uint32_t>{111u, 222u}));
  // Raw AIS also carries 222 and 333 → the overlap on 222 is the double-count.
  const auto overlap = adapter.circularAisMmsis({222u, 333u});
  ASSERT_EQ(overlap.size(), 1u);
  EXPECT_EQ(overlap[0], 222u);
  // No overlap → empty (the safe, no-warning case).
  EXPECT_TRUE(adapter.circularAisMmsis({999u}).empty());
}
