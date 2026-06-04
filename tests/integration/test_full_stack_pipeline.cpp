// End-to-end integration test of the assembled navtracker library.
//
// Scenario: own-ship transits east at 5 m/s for 20 s. A target inbound
// from the east at 5 m/s closes head-on, meets own-ship at t=10 s, then
// continues westward. NMEA stream from own-ship sensors is fed to the
// adapter; the target is reported as Position2D measurements (AIS-style).
//
// Sources injected:
//   - GGA (own-ship position at the datum, then drifting east)
//   - IIHDT (gyro heading) with +2° bias relative to truth
//   - GPHDT (multi-antenna GPS heading) at truth heading (no bias)
//   - IIHDG (magnetic heading) with known variation (no deviation)
//   - GPRMC (SOG/COG, variation)
//
// Asserts: track lifecycle events sequence, bias estimator converges to
// ~2° via the three multi-source paths, CPA evaluator fires Entered
// before the pass and Exited after, pull-side TrackOutput agrees with
// the push-side track state.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/collision/CpaEvaluator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/output/TrackOutput.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ICollisionRiskSink.hpp"
#include "ports/ITrackSink.hpp"

using namespace navtracker;

namespace {

constexpr double kBiasTrueRad = 2.0 * 3.14159265358979323846 / 180.0;
constexpr double kVariationDeg = 3.0;  // known geographic variation
constexpr double kPi = 3.14159265358979323846;

class LifecycleRecorder : public ITrackSink {
 public:
  std::vector<TrackLifecycleEvent> initiated, confirmed, updated, deleted;
  void onTrackInitiated(const TrackLifecycleEvent& e) override { initiated.push_back(e); }
  void onTrackConfirmed(const TrackLifecycleEvent& e) override { confirmed.push_back(e); }
  void onTrackUpdated(const TrackLifecycleEvent& e) override { updated.push_back(e); }
  void onTrackDeleted(const TrackLifecycleEvent& e) override { deleted.push_back(e); }
};

class RiskRecorder : public ICollisionRiskSink {
 public:
  std::vector<CollisionRiskEvent> events;
  void onCollisionRisk(const CollisionRiskEvent& e) override { events.push_back(e); }
  std::size_t countOf(CollisionRiskTransition tr) const {
    std::size_t n = 0;
    for (const auto& e : events) if (e.transition == tr) ++n;
    return n;
  }
};

std::string nmeaWithChecksum(const std::string& payload) {
  std::uint8_t cs = 0;
  for (char c : payload) cs ^= static_cast<std::uint8_t>(c);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + payload + buf;
}

// ddmm.mmmm latitude/longitude formatter. lat positive = N, lon positive = E.
std::string formatLatNmea(double deg) {
  const double abs_deg = std::abs(deg);
  const int d = static_cast<int>(abs_deg);
  const double m = (abs_deg - d) * 60.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d%07.4f", d, m);
  return buf;
}

std::string formatLonNmea(double deg) {
  const double abs_deg = std::abs(deg);
  const int d = static_cast<int>(abs_deg);
  const double m = (abs_deg - d) * 60.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%03d%07.4f", d, m);
  return buf;
}

// Build a GGA sentence with the given lat/lon. HDOP=1.0 so adapter
// produces a position sigma of uere_m*1.0.
std::string ggaSentence(double lat_deg, double lon_deg) {
  std::string s = "GPGGA,000000,";
  s += formatLatNmea(lat_deg);
  s += "," + std::string(lat_deg >= 0 ? "N" : "S") + ",";
  s += formatLonNmea(lon_deg);
  s += "," + std::string(lon_deg >= 0 ? "E" : "W") + ",1,12,1.0,0.0,M,0.0,M,,";
  return nmeaWithChecksum(s);
}

std::string hdtSentence(const std::string& talker, double heading_deg) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%sHDT,%.3f,T", talker.c_str(), heading_deg);
  return nmeaWithChecksum(buf);
}

std::string hdgSentence(double mag_heading_deg, double variation_deg) {
  char buf[64];
  const char var_dir = variation_deg >= 0 ? 'E' : 'W';
  std::snprintf(buf, sizeof(buf), "IIHDG,%.3f,0.0,E,%.3f,%c",
                mag_heading_deg, std::abs(variation_deg), var_dir);
  return nmeaWithChecksum(buf);
}

std::string rmcSentence(double sog_knots, double cog_deg,
                        double variation_deg) {
  char buf[128];
  const char var_dir = variation_deg >= 0 ? 'E' : 'W';
  std::snprintf(buf, sizeof(buf),
                "GPRMC,000000,A,0000.0000,N,00000.0000,E,%.3f,%.3f,010100,%.3f,%c,A",
                sog_knots, cog_deg, std::abs(variation_deg), var_dir);
  return nmeaWithChecksum(buf);
}

Measurement positionAt(double x, double y, double t_s, double sigma_m,
                       const std::string& src) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (sigma_m * sigma_m);
  return m;
}

}  // namespace

TEST(FullStackIntegration, NmeaTargetTrackingBiasAndCpaAllCompose) {
  // ===== Composition root =====
  const geo::Datum datum(geo::Geodetic{0.0, 0.0, 0.0});

  OwnShipProvider provider(datum);
  OwnShipNmeaAdapterConfig adapter_cfg;
  adapter_cfg.gps_heading_talkers = {"GP"};   // GPHDT routes as GPS heading
  adapter_cfg.uere_m = 5.0;
  OwnShipNmeaAdapter adapter(provider, adapter_cfg);

  HeadingBiasEstimator bias({});
  adapter.setHeadingBiasEstimator(&bias);

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator estimator(motion, 5.0);
  GnnAssociator associator(100.0);
  TrackManager manager(1, 4);          // confirm after 1 hit (already have 1 from add)
  LifecycleRecorder lifecycle;
  manager.setTrackSink(&lifecycle);

  Tracker tracker(estimator, associator, manager, 30.0);
  tracker.setBearingInnovationSink(&bias);

  CpaEvaluatorConfig cpa_cfg;
  cpa_cfg.d_threshold_m = 75.0;
  cpa_cfg.enter_probability = 0.5;
  cpa_cfg.exit_probability = 0.3;
  CpaEvaluator cpa(manager, provider, cpa_cfg);
  RiskRecorder risk;
  cpa.setSink(&risk);

  // ===== Scenario loop =====
  // Own-ship at origin heading east at 5 m/s. Target starts 100 m east
  // heading west at 5 m/s. They meet at t=10s, then separate.
  // Inject +2° bias into the gyro (IIHDT) only.
  const double sog_knots_ownship = 5.0 / 0.514444;   // 5 m/s -> knots
  const double cog_deg_ownship = 90.0;               // east
  const double truth_heading_deg = 90.0;
  const double bias_deg = kBiasTrueRad * 180.0 / kPi;

  // Equirectangular scale at the equator: 1 m east ~ 1/(R*cos(0)) rad lon.
  constexpr double kR = 6378137.0;
  auto lonForEastMeters = [&](double e_m) { return (e_m / kR) * 180.0 / kPi; };

  for (int sec = 1; sec <= 30; ++sec) {
    const double t_s = static_cast<double>(sec);
    const Timestamp t = Timestamp::fromSeconds(t_s);

    // Own-ship NMEA stream — order matters: HDT before RMC so the COG
    // dispatch finds a fresh gyro sample.
    adapter.ingest(hdtSentence("II", truth_heading_deg + bias_deg), t);
    adapter.ingest(hdtSentence("GP", truth_heading_deg), t);
    adapter.ingest(hdgSentence(truth_heading_deg - kVariationDeg,
                               kVariationDeg),
                   t);
    adapter.ingest(rmcSentence(sog_knots_ownship, cog_deg_ownship,
                               kVariationDeg),
                   t);
    // GGA last so all pose fields are populated and provider has fresh latest().
    const double own_east_m = 5.0 * t_s;
    adapter.ingest(ggaSentence(0.0, lonForEastMeters(own_east_m)), t);

    // Target measurement (AIS-style Position2D) at expected ENU position.
    // Target ENU = (100 - 5*t, 0). When 5*t > 100 the target has passed.
    const double target_east_m = 100.0 - 5.0 * t_s;
    const double target_north_m = 0.0;
    tracker.process(positionAt(target_east_m, target_north_m, t_s,
                               /*sigma_m=*/5.0, "ais_42"));

    // Evaluate CPA at this cycle.
    cpa.evaluate(t);
  }

  // ===== Assertions =====

  // 1. Track lifecycle: at least one Initiated + Confirmed; no Deleted in
  //    20s (track is fresh and miss timeout is 30s).
  ASSERT_GE(lifecycle.initiated.size(), 1u);
  ASSERT_GE(lifecycle.confirmed.size(), 1u);
  EXPECT_EQ(lifecycle.deleted.size(), 0u);
  // Updated fires on every successful match; we should have many.
  EXPECT_GE(lifecycle.updated.size(), 20u);

  // 2. Bias estimator converged within 0.5° of truth.
  const double err_deg = std::abs(bias.biasRad() - kBiasTrueRad)
                       * 180.0 / kPi;
  EXPECT_LT(err_deg, 0.5)
      << "biasRad=" << bias.biasRad()
      << " accepted_hdg=" << bias.acceptedGpsHeading()
      << " accepted_cog=" << bias.acceptedGpsCog()
      << " accepted_mag=" << bias.acceptedMagnetic();
  EXPECT_TRUE(bias.current().is_published);
  // All three NMEA paths contributed.
  EXPECT_GT(bias.acceptedGpsHeading(), 0u);
  EXPECT_GT(bias.acceptedGpsCog(),     0u);
  EXPECT_GT(bias.acceptedMagnetic(),   0u);

  // 3. Adapter dispatch counters reconcile with estimator accepts.
  EXPECT_GE(adapter.dispatchedGpsHeading(), bias.acceptedGpsHeading());
  EXPECT_GE(adapter.dispatchedGpsCog(),     bias.acceptedGpsCog());
  EXPECT_GE(adapter.dispatchedMagnetic(),   bias.acceptedMagnetic());

  // 4. CPA: Entered fired before the pass; Exited fired after.
  EXPECT_GE(risk.countOf(CollisionRiskTransition::Entered), 1u);
  EXPECT_GE(risk.countOf(CollisionRiskTransition::Exited),  1u);
  // The Entered event preceded the Exited event in time.
  Timestamp t_entered = Timestamp::fromSeconds(-1.0);
  Timestamp t_exited  = Timestamp::fromSeconds(-1.0);
  for (const auto& e : risk.events) {
    if (e.transition == CollisionRiskTransition::Entered
        && t_entered.seconds() < 0.0) {
      t_entered = e.time;
    }
    if (e.transition == CollisionRiskTransition::Exited
        && t_exited.seconds() < 0.0) {
      t_exited = e.time;
    }
  }
  EXPECT_LT(t_entered.seconds(), t_exited.seconds())
      << "entered=" << t_entered.seconds()
      << " exited=" << t_exited.seconds();

  // 5. Pull-side TrackOutput is consistent with push-side state.
  ASSERT_EQ(manager.tracks().size(), 1u);
  const Track& tr = manager.tracks().front();
  const TrackOutput out = toTrackOutput(tr, provider.datum());
  EXPECT_EQ(out.id, lifecycle.initiated.front().id);
  EXPECT_EQ(out.status, tr.status);
}
