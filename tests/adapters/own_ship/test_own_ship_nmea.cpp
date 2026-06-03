#include <cmath>
#include <random>
#include <string>

#include <gtest/gtest.h>
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "sim/NmeaEncode.hpp"
#include "tests/adapters/util/NmeaTestHelpers.hpp"

using navtracker::OwnShipNmeaAdapter;
using navtracker::OwnShipNmeaAdapterConfig;
using navtracker::OwnShipProvider;
using navtracker::Timestamp;
using navtracker_test::makeNmea;

TEST(OwnShipNmeaAdapter, GgaUpdatesPositionInProvider) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // 4807.038 N -> 48 + 7.038/60 = 48.1173 deg
  // 01131.000 E -> 11 + 31.000/60 = 11.5166666... deg
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_NEAR(provider.latest()->lat_deg, 48.1173, 1e-4);
  EXPECT_NEAR(provider.latest()->lon_deg, 11.5166666, 1e-4);
}

TEST(OwnShipNmeaAdapter, HdtUpdatesHeading) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0));
  EXPECT_TRUE(adapter.ingest(makeNmea("GPHDT,123.5,T"),
                              Timestamp::fromSeconds(1001.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_DOUBLE_EQ(provider.latest()->heading_true_deg, 123.5);
}

TEST(OwnShipNmeaAdapter, RejectsMalformedLines) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  EXPECT_FALSE(adapter.ingest("garbage", Timestamp::fromSeconds(0.0)));
}

TEST(OwnShipNmeaAdapterTest, ParsesHdopFromGga) {
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.uere_m = 5.0;
  OwnShipNmeaAdapter adapter(provider, cfg);

  // GGA with HDOP = 1.2 in field 7 (zero-based). uere_m = 5.0 -> sigma = 6.0.
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(0.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position_std_m, 6.0, 1e-9);
}

TEST(OwnShipNmeaAdapterTest, AbsentHdopLeavesStdAtZero) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});

  // GGA with empty HDOP field. No sticky setter call -> sigma stays at 0.
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(0.0)));
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->position_std_m, 0.0);
}

namespace {

// Build a GGA sentence with the given lat/lon (degrees, signed) and HDOP.
// HDOP <= 0 -> empty HDOP field.
std::string makeGga(double lat_deg, double lon_deg, double hdop) {
  std::string body = "GPGGA,000000.00,";
  body += navtracker::sim::formatLatDdmm(lat_deg);
  body += ',';
  body += navtracker::sim::latHemisphere(lat_deg);
  body += ',';
  body += navtracker::sim::formatLonDdmm(lon_deg);
  body += ',';
  body += navtracker::sim::lonHemisphere(lon_deg);
  body += ",1,08,";
  if (hdop > 0.0) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f", hdop);
    body += buf;
  }
  body += ",0.0,M,0.0,M,,";
  return navtracker::sim::wrapWithChecksum(body);
}

// Convert a local-meter offset (east, north) at reference latitude lat_ref
// (deg) into a lat/lon pair. Inverse of the adapter's equirectangular
// projection.
struct LatLon { double lat_deg; double lon_deg; };
LatLon offsetToLatLon(double lat_ref_deg, double lon_ref_deg,
                      double east_m, double north_m) {
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double dlat_deg = north_m / (kEarthRadiusM * kDegToRad);
  const double dlon_deg =
      east_m / (kEarthRadiusM * kDegToRad * std::cos(lat_ref_deg * kDegToRad));
  return {lat_ref_deg + dlat_deg, lon_ref_deg + dlon_deg};
}

}  // namespace

TEST(OwnShipNmeaAdapterTest, AdaptiveDisabledMatchesStaticPath) {
  // Default cfg (enable_adaptive_uere = false): feed a series of GGA fixes
  // with HDOP=1.2 and assert pose.position_std_m == 1.2 * 5.0 on every
  // message. This is the regression guard against G3's static path.
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});  // adaptive off, uere_m=5.0

  const double lat0 = 48.1173;
  const double lon0 = 11.5166666;
  for (int i = 0; i < 10; ++i) {
    // Walk gently east so successive fixes differ. With adaptive off the
    // motion is irrelevant — sigma must stay at HDOP*UERE.
    const auto pos = offsetToLatLon(lat0, lon0, 5.0 * i, 0.0);
    const std::string sentence = makeGga(pos.lat_deg, pos.lon_deg, 1.2);
    ASSERT_TRUE(adapter.ingest(sentence, Timestamp::fromSeconds(i * 1.0)));
    const auto pose = provider.latest();
    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(pose->position_std_m, 1.2 * 5.0, 1e-9);
  }
}

TEST(OwnShipNmeaAdapterTest, AdaptivePublishesAfterWindowAndDominatesStatic) {
  // Opt in to adaptive. Stream 10 GGA fixes with HDOP=2.0 (so the static
  // path would publish sigma = 2.0 * 5.0 = 10 m) but with actual ENU
  // residual sigma ~1 m. After the window fills (8 samples), the adaptive
  // estimator should publish ~1 m and dominate the static value.
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.enable_adaptive_uere = true;
  // Default window_size = 8.
  OwnShipNmeaAdapter adapter(provider, cfg);

  std::mt19937 rng{42};
  const double noise_sigma_m = 1.0;
  std::normal_distribution<double> n(0.0, noise_sigma_m);

  const double lat0 = 48.1173;
  const double lon0 = 11.5166666;
  const double v_east_mps = 5.0;
  const double dt = 1.0;

  double last_sigma = -1.0;
  for (int i = 0; i < 10; ++i) {
    const double east_m = v_east_mps * i * dt + n(rng);
    const double north_m = n(rng);
    const auto pos = offsetToLatLon(lat0, lon0, east_m, north_m);
    const std::string sentence = makeGga(pos.lat_deg, pos.lon_deg, 2.0);
    ASSERT_TRUE(adapter.ingest(sentence, Timestamp::fromSeconds(i * dt)));
    const auto pose = provider.latest();
    ASSERT_TRUE(pose.has_value());
    if (i < 7) {
      // Window not yet full; static path applies: sigma = 2.0 * 5.0 = 10.
      EXPECT_NEAR(pose->position_std_m, 10.0, 1e-9);
    } else {
      // Window full from i >= 7 (8 samples ingested). Adaptive publishes.
      last_sigma = pose->position_std_m;
    }
  }
  // Adaptive sigma should track the injected noise (~1 m), not the static
  // 10 m. Loose factor-of-3 bounds to absorb LS-fit variance on 8 points.
  EXPECT_GT(last_sigma, 0.3);
  EXPECT_LT(last_sigma, 3.0);
  EXPECT_LT(last_sigma, 5.0);  // strictly below static 10 m.
}

TEST(OwnShipNmeaAdapterTest, AdaptiveFallsBackOnManeuver) {
  // Stream 4 steady samples moving east at 5 m/s, then 4 samples moving
  // north at 5 m/s — a clear maneuver. The estimator suppresses
  // publication for windows containing the change, so pose.position_std_m
  // must fall back to HDOP * UERE_static (2.0 * 5.0 = 10 m).
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.enable_adaptive_uere = true;
  OwnShipNmeaAdapter adapter(provider, cfg);

  const double lat0 = 48.1173;
  const double lon0 = 11.5166666;
  const double dt = 1.0;
  double east_m = 0.0;
  double north_m = 0.0;

  // First half: 4 samples moving east, no noise.
  for (int i = 0; i < 4; ++i) {
    const auto pos = offsetToLatLon(lat0, lon0, east_m, north_m);
    const std::string sentence = makeGga(pos.lat_deg, pos.lon_deg, 2.0);
    ASSERT_TRUE(adapter.ingest(sentence, Timestamp::fromSeconds(i * dt)));
    east_m += 5.0;
  }
  // Second half: 4 samples moving north, no noise.
  for (int i = 4; i < 8; ++i) {
    const auto pos = offsetToLatLon(lat0, lon0, east_m, north_m);
    const std::string sentence = makeGga(pos.lat_deg, pos.lon_deg, 2.0);
    ASSERT_TRUE(adapter.ingest(sentence, Timestamp::fromSeconds(i * dt)));
    north_m += 5.0;
  }
  // The window is now full and straddles the maneuver. The estimator's
  // two-halves Δv test triggers, so sigma falls back to HDOP * UERE.
  const auto pose = provider.latest();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position_std_m, 10.0, 1e-9);
}
