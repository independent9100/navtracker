// W2.1 — Section-D backfill for the whole datum/own-ship cluster.
//
// Every sensor adapter used to cache a private `Datum` copy fixed at
// construction and NEVER update it on an OwnShipProvider auto-recenter, so
// after a >30 km own-ship move every non-cooperative measurement projected in
// the OLD ENU frame (silently corrupt positions). The fix makes each adapter an
// IDatumChangeSink: on recenter it swaps in the new datum AND re-expresses any
// already-buffered (old-frame) measurements into the new frame. These tests
// wire the four adapters as datum sinks, cross the recenter threshold, and
// assert that adapter-built measurements land in the NEW frame.
#include <gtest/gtest.h>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/remote_track/RemoteTrackAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

namespace navtracker {
namespace {

using navtracker_test::makeNmea;

// Convert an ENU measurement position back through `d` and assert it matches
// the expected geodetic. If the adapter kept the STALE datum, the ENU value is
// in the old frame and this round-trip lands far from (lat, lon).
void expectGeodetic(const geo::Datum& d, const Eigen::Vector2d& enu, double lat,
                    double lon, double tol = 1e-4) {
  const auto g = d.toGeodetic(Eigen::Vector3d(enu.x(), enu.y(), 0.0));
  EXPECT_NEAR(g.lat_deg, lat, tol);
  EXPECT_NEAR(g.lon_deg, lon, tol);
}

// OwnShipProvider seeded at (53.5, 8.0); recenter() pushes a pose ~66 km east
// (past the 30 km threshold) so the working datum jumps to (53.5, 9.0).
struct Fixture {
  OwnShipProvider provider;
  Fixture() {
    OwnShipPose p;
    p.time = Timestamp::fromSeconds(0.0);
    p.lat_deg = 53.5;
    p.lon_deg = 8.0;
    p.heading_true_deg = 0.0;
    p.position_std_m = 5.0;
    provider.update(p);
  }
  void recenter() {
    OwnShipPose far;
    far.time = Timestamp::fromSeconds(60.0);
    far.lat_deg = 53.5;
    far.lon_deg = 9.0;  // ~66 km east
    far.heading_true_deg = 0.0;
    far.position_std_m = 5.0;
    provider.update(far);
  }
};

TEST(AdapterDatumRecenter, AisLandsInNewFrameAfterRecenter) {
  Fixture fx;
  AisAdapter ais(fx.provider.datum());
  fx.provider.registerDatumSink(&ais);

  fx.recenter();

  AisDynamicReport r;
  r.time = Timestamp::fromSeconds(61.0);
  r.mmsi = 111;
  r.lat_deg = 53.6;
  r.lon_deg = 9.1;
  ais.ingest(r);

  auto m = ais.poll();
  ASSERT_EQ(m.size(), 1u);
  expectGeodetic(fx.provider.datum(), m[0].value.head<2>(), 53.6, 9.1);
}

TEST(AdapterDatumRecenter, RemoteTrackLandsInNewFrameAfterRecenter) {
  Fixture fx;
  RemoteTrackAdapter remote(fx.provider.datum());
  fx.provider.registerDatumSink(&remote);

  fx.recenter();

  RemoteTrackReport r;
  r.time = Timestamp::fromSeconds(61.0);
  r.remote_track_id = 7;
  r.lat_deg = 53.6;
  r.lon_deg = 9.1;
  remote.ingest(r);

  auto m = remote.poll();
  ASSERT_EQ(m.size(), 1u);
  expectGeodetic(fx.provider.datum(), m[0].value.head<2>(), 53.6, 9.1);
}

TEST(AdapterDatumRecenter, ArpaTllLandsInNewFrameAfterRecenter) {
  Fixture fx;
  ArpaAdapter arpa(fx.provider.datum(), fx.provider);
  fx.provider.registerDatumSink(&arpa);

  fx.recenter();

  // TLL target at 53.6 N, 9.1 E → ddmm "5336.000" / "00906.000".
  ASSERT_TRUE(arpa.ingest(makeNmea("RATLL,01,5336.000,N,00906.000,E,TGT,123456,T,R"),
                          Timestamp::fromSeconds(61.0)));
  auto m = arpa.poll();
  ASSERT_EQ(m.size(), 1u);
  expectGeodetic(fx.provider.datum(), m[0].value.head<2>(), 53.6, 9.1, 1e-3);
}

TEST(AdapterDatumRecenter, EoIrLandsInNewFrameAfterRecenter) {
  Fixture fx;
  EoIrAdapter eoir(fx.provider.datum(), fx.provider);
  fx.provider.registerDatumSink(&eoir);

  fx.recenter();

  // Detection due north (bearing 0, heading 0), 500 m out. After recenter own-
  // ship sits at the new datum origin, so the target lands near ENU (0, 500).
  // With the STALE datum own-ship would be ~66 km east of origin and the
  // measurement would be near (66000, 500) — the discriminator below.
  CameraDetection d;
  d.time = Timestamp::fromSeconds(61.0);
  d.bearing_relative_deg = 0.0;
  d.range_m = 500.0;
  eoir.ingest(d);

  auto m = eoir.poll();
  ASSERT_EQ(m.size(), 1u);
  EXPECT_LT(std::abs(m[0].value(0)), 1000.0);       // near new origin, not 66 km
  EXPECT_NEAR(m[0].value(1), 500.0, 50.0);          // ~500 m north
}

// The subtle case: a measurement buffered BEFORE the recenter (not yet polled)
// must be re-expressed into the new frame when the recenter fires, otherwise a
// single poll() would mix old-frame and new-frame measurements.
TEST(AdapterDatumRecenter, BufferedMeasurementReprojectedOnRecenter) {
  Fixture fx;
  AisAdapter ais(fx.provider.datum());
  fx.provider.registerDatumSink(&ais);

  // Ingest BEFORE recenter → buffered in the OLD frame.
  AisDynamicReport before;
  before.time = Timestamp::fromSeconds(10.0);
  before.mmsi = 222;
  before.lat_deg = 53.55;
  before.lon_deg = 8.05;
  ais.ingest(before);

  fx.recenter();  // fires the sink; buffered measurement must be reprojected

  // Ingest AFTER recenter → new frame.
  AisDynamicReport after;
  after.time = Timestamp::fromSeconds(61.0);
  after.mmsi = 333;
  after.lat_deg = 53.6;
  after.lon_deg = 9.1;
  ais.ingest(after);

  auto m = ais.poll();
  ASSERT_EQ(m.size(), 2u);
  // Both interpreted through the NEW datum must recover their true geodetics.
  expectGeodetic(fx.provider.datum(), m[0].value.head<2>(), 53.55, 8.05);
  expectGeodetic(fx.provider.datum(), m[1].value.head<2>(), 53.6, 9.1);
}

}  // namespace
}  // namespace navtracker
