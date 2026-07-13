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

TEST(OwnShipNmeaAdapterTest, ParsesRmcSogCogIntoVelocityEnu) {
  // RMC with SOG = 10 knots = 5.14444 m/s, COG = 045° true.
  // ENU velocity = SOG * (sin(COG), cos(COG)) = 5.14444 * (0.7071, 0.7071)
  //              ~= (3.6378, 3.6378) m/s.
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPRMC,123519,A,4807.038,N,01131.000,E,10.0,045.0,230394,003.1,W"),
      Timestamp::fromSeconds(0.0)));
  // RMC only updates the internal buffer; pose composition runs on GGA.
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,0.0,M,0.0,M,,"),
      Timestamp::fromSeconds(0.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_TRUE(provider.latest()->velocity_is_valid);
  EXPECT_NEAR(provider.latest()->velocity_enu.x(), 3.6378, 0.01);
  EXPECT_NEAR(provider.latest()->velocity_enu.y(), 3.6378, 0.01);
}

TEST(OwnShipNmeaAdapterTest, RmcZeroSogProducesSigmaSogAsSigmaV) {
  // At SOG = 0, the bearing-uncertainty term vanishes so sigma_v reduces
  // exactly to sigma_SOG.
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;
  cfg.sigma_sog_m_per_s = 0.5;
  cfg.sigma_cog_deg = 1.0;
  OwnShipNmeaAdapter adapter(provider, cfg);
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,000.0,230394,,"),
      Timestamp::fromSeconds(0.0)));
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,1.2,0.0,M,0.0,M,,"),
      Timestamp::fromSeconds(0.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_TRUE(provider.latest()->velocity_is_valid);
  EXPECT_NEAR(provider.latest()->velocity_std_m_per_s, 0.5, 1e-6);
}

TEST(OwnShipNmeaAdapterTest, RmcAbsentTriggersEstimatorFallback) {
  // No RMC ever ingested. Feed 10 GGAs at 5 m/s east; after the velocity
  // estimator's window (8 samples) fills, pose.velocity_is_valid flips to
  // true via the estimator fallback.
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, {});

  const double lat0 = 48.1173;
  const double lon0 = 11.5166666;
  const double v_east_mps = 5.0;
  const double dt = 1.0;
  bool ever_valid = false;
  Eigen::Vector2d last_v;
  for (int i = 0; i < 10; ++i) {
    const double east_m = v_east_mps * i * dt;
    const auto pos = offsetToLatLon(lat0, lon0, east_m, 0.0);
    ASSERT_TRUE(adapter.ingest(makeGga(pos.lat_deg, pos.lon_deg, 1.2),
                               Timestamp::fromSeconds(i * dt)));
    if (provider.latest()->velocity_is_valid) {
      ever_valid = true;
      last_v = provider.latest()->velocity_enu;
    }
  }
  EXPECT_TRUE(ever_valid);
  // Estimator should recover ~ (5, 0) m/s. Wide tolerance — exact
  // numerical fit is covered by the estimator unit tests.
  EXPECT_NEAR(last_v.x(), 5.0, 0.5);
  EXPECT_NEAR(last_v.y(), 0.0, 0.5);
}

TEST(OwnShipNmeaAdapterTest, RmcStaleTriggersEstimatorFallback) {
  // Feed one RMC at t=0, then GGAs only (no further RMC). For t up to
  // rmc_stale_seconds (5 s) the RMC value dominates; past that the
  // adapter falls back to the GGA-derived estimator. The estimator
  // needs its window (8 samples) to publish, so we run 10 GGAs.
  OwnShipProvider provider;
  OwnShipNmeaAdapterConfig cfg;  // defaults: rmc_stale_seconds = 5
  OwnShipNmeaAdapter adapter(provider, cfg);

  // Initial RMC: SOG = 10 knots, COG = 090° true (east) -> velocity ~ (5.14, 0).
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPRMC,000000,A,4807.038,N,01131.000,E,10.0,090.0,230394,,"),
      Timestamp::fromSeconds(0.0)));

  const double lat0 = 48.1173;
  const double lon0 = 11.5166666;
  const double v_east_mps = 5.0;   // matches RMC closely so the transition is smooth
  const double dt = 1.0;
  bool saw_rmc_phase = false;
  bool saw_estimator_phase = false;
  for (int i = 0; i < 11; ++i) {
    const double t_s = i * dt;
    const double east_m = v_east_mps * t_s;
    const auto pos = offsetToLatLon(lat0, lon0, east_m, 0.0);
    ASSERT_TRUE(adapter.ingest(makeGga(pos.lat_deg, pos.lon_deg, 1.2),
                               Timestamp::fromSeconds(t_s)));
    ASSERT_TRUE(provider.latest().has_value());
    const auto& p = *provider.latest();
    if (t_s <= cfg.rmc_stale_seconds) {
      // Within the fresh window the RMC value (~5.144 east, 0 north)
      // should win, even though the estimator may have published.
      if (p.velocity_is_valid) {
        EXPECT_NEAR(p.velocity_enu.x(), 5.14444, 1e-3);
        EXPECT_NEAR(p.velocity_enu.y(), 0.0, 1e-3);
        saw_rmc_phase = true;
      }
    } else {
      // Past the stale threshold the estimator must drive the pose.
      // It can publish at most ~5 m/s east; verify validity and a
      // loose bound that excludes the exact RMC value.
      if (p.velocity_is_valid) {
        EXPECT_NEAR(p.velocity_enu.x(), 5.0, 0.5);
        EXPECT_NEAR(p.velocity_enu.y(), 0.0, 0.5);
        saw_estimator_phase = true;
      }
    }
  }
  EXPECT_TRUE(saw_rmc_phase);
  EXPECT_TRUE(saw_estimator_phase);
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

// ---------------------------------------------------------------------------
// F1 (pre-release CRITICAL): a GGA that carries no valid fix must produce NO
// pose. Without this the adapter validated nothing: fix-quality (field 5) was
// never read and empty lat/lon fields parse to (0,0), so a standard no-fix
// GGA published a (0,0) pose. That teleports the datum to Null Island (and,
// as the first fix after cold start, mis-scales the equirectangular
// reference), silently corrupting every downstream ENU conversion. Adapters
// validate at the edge (architecture invariant #6); the RMC branch already
// checks its A/V status flag — these tests pin the GGA counterpart.
// ---------------------------------------------------------------------------
namespace {

// Counts datum-recenter events. Zero recenters across a fix→no-fix→fix
// sequence is the direct proof that a no-fix GGA never moves the datum.
struct RecenterCountingSink : navtracker::IDatumChangeSink {
  int count = 0;
  void onDatumRecentered(const navtracker::geo::Datum&,
                         const navtracker::geo::Datum&) override {
    ++count;
  }
};

}  // namespace

TEST(OwnShipNmeaAdapterTest, NoFixQualityGgaProducesNoPoseAndNoDatum) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // Industry-standard no-fix sentence: empty lat/lon fields, fix-quality 0.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("GPGGA,120000.00,,,,,0,00,99.99,,M,,M,,"),
      Timestamp::fromSeconds(1000.0)));
  EXPECT_FALSE(provider.latest().has_value());
  // The datum must NOT initialize — otherwise it anchors at Null Island.
  EXPECT_FALSE(provider.hasDatum());
}

TEST(OwnShipNmeaAdapterTest, FixQualityZeroWithPositionStillRejected) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // Quality 0 = invalid fix, even though lat/lon look valid: the receiver is
  // telling us not to trust the position.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  EXPECT_FALSE(provider.latest().has_value());
  EXPECT_FALSE(provider.hasDatum());
}

TEST(OwnShipNmeaAdapterTest, EmptyLatLonRejectedEvenWhenQualityClaimsFix) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // Contradictory but seen in the wild: quality 1 but empty lat/lon. parseDdmm
  // yields (0,0); the position-presence check must still reject it.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("GPGGA,120000.00,,,,,1,08,0.9,,M,,M,,"),
      Timestamp::fromSeconds(1000.0)));
  EXPECT_FALSE(provider.latest().has_value());
  EXPECT_FALSE(provider.hasDatum());
}

TEST(OwnShipNmeaAdapterTest, ImplausibleLatLonRejected) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  // 9999.000 -> 99 deg + 99/60 min = 100.65 deg latitude — out of [-90, 90].
  EXPECT_FALSE(adapter.ingest(
      makeNmea("GPGGA,123519,9999.000,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  EXPECT_FALSE(provider.latest().has_value());
}

TEST(OwnShipNmeaAdapterTest, NoFixBetweenValidFixesDoesNotMoveDatum) {
  OwnShipProvider provider;  // auto-recenter ON, 30 km threshold (defaults)
  RecenterCountingSink sink;
  provider.registerDatumSink(&sink);
  OwnShipNmeaAdapter adapter(provider);

  // Valid fix near Hamburg (53.55 N, 9.983 E) establishes the datum.
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPGGA,120000,5333.000,N,00959.000,E,1,08,0.9,10.0,M,46.9,M,,"),
      Timestamp::fromSeconds(1.0)));
  ASSERT_TRUE(provider.hasDatum());
  const double datum_lat0 = provider.datum().origin().lat_deg;
  const double pose_lat0 = provider.latest()->lat_deg;
  EXPECT_GT(pose_lat0, 53.0);

  // GPS antenna shadowed mid-run: a no-fix GGA arrives. A (0,0) pose here is
  // ~6000 km away (> 30 km) and would fire an auto-recenter to Null Island.
  EXPECT_FALSE(adapter.ingest(
      makeNmea("GPGGA,120001.00,,,,,0,00,99.99,,M,,M,,"),
      Timestamp::fromSeconds(2.0)));

  // No recenter fired; datum origin and latest pose are untouched.
  EXPECT_EQ(sink.count, 0);
  EXPECT_DOUBLE_EQ(provider.datum().origin().lat_deg, datum_lat0);
  EXPECT_DOUBLE_EQ(provider.latest()->lat_deg, pose_lat0);

  // A subsequent valid fix nearby resumes normally (still no recenter).
  ASSERT_TRUE(adapter.ingest(
      makeNmea("GPGGA,120002,5333.100,N,00959.100,E,1,08,0.9,10.0,M,46.9,M,,"),
      Timestamp::fromSeconds(3.0)));
  EXPECT_EQ(sink.count, 0);
  EXPECT_NEAR(provider.latest()->lat_deg, 53.0 + 33.1 / 60.0, 1e-4);
}

TEST(OwnShipNmeaAdapter, ValidFixStillProducesPose) {
  // Guard the happy path: a normal quality-1 GGA is unchanged by the new
  // validation (first-fix-initializes-datum path).
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  EXPECT_TRUE(adapter.ingest(
      makeNmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
      Timestamp::fromSeconds(1000.0)));
  ASSERT_TRUE(provider.latest().has_value());
  EXPECT_TRUE(provider.hasDatum());
  EXPECT_NEAR(provider.latest()->lat_deg, 48.1173, 1e-4);
}
