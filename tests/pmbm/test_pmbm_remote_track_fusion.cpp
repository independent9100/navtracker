// R10 fusion scenario: a shore/VTS remote-track feed fused with radar + AIS +
// cooperative on the SAME vessel. Asserts the R10 acceptance contract:
//   (a) all four sources fuse to ONE confirmed track — the remote feed does NOT
//       spawn a second (dual) track;
//   (b) the remote pseudo-measurement actually contributes (RemoteTrack touch on
//       the fused track's provenance);
//   (c) the fused track ID is STABLE when the remote feed SWAPS its own track id
//       (external id is a hint, never the fusion key — invariant 5);
//   (d) the fused track ID is STABLE when the remote feed DROPS while radar
//       corroborates (R9-style no-retirement under surveillance);
//   (e) NEES consistency sanity: R-inflation must not leave the fused estimate
//       overconfident (the tripwire for "inflation stopped being enough").
//
// The remote channel is driven through the real RemoteTrackAdapter (dogfooded),
// so this also proves the adapter's output gates and fuses. Miss model is
// use_sensor_activity ALONE (the guard-compliant deployment config).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>  // Matrix::inverse() for the NEES quadratic form

#include "adapters/remote_track/RemoteTrackAdapter.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/sensor_activity/DeclaredSensorActivity.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace {

using navtracker::ConstantVelocity2D;
using navtracker::DeclaredSensorActivity;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::RemoteTrackAdapter;
using navtracker::RemoteTrackAdapterConfig;
using navtracker::RemoteTrackReport;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::geo::Datum;
using navtracker::pmbm::PmbmTracker;

Datum gDatum() { return Datum({53.54, 9.97, 0.0}); }

// Constant-velocity truth along +east at 2 m/s from the datum origin.
Eigen::Vector2d gTruth(double t) { return Eigen::Vector2d(2.0 * t, 0.0); }

double gauss(std::mt19937_64& rng, double sigma) {
  std::normal_distribution<double> d(0.0, sigma);
  return d(rng);
}

Measurement gPos(double t, SensorKind kind, const std::string& src,
                 const Eigen::Vector2d& xy, double sigma) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = kind;
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = xy;
  z.covariance = Eigen::Matrix2d::Identity() * (sigma * sigma);
  return z;
}

DeclaredSensorActivity gActivity() {
  DeclaredSensorActivity::ChannelProfile radar;
  radar.kind = navtracker::ChannelKind::Surveillance;
  radar.sensor = SensorKind::ArpaTtm;
  radar.duty_cycle_sec = 1.0;
  radar.max_range_m = 20000.0;
  radar.p_D = 0.9;
  // The remote STATION declares a surveillance coverage area (distinct from us
  // self-estimating a wedge from its point reports — RemoteTrack is excluded
  // from that self-estimation as a non-scanning source).
  DeclaredSensorActivity::ChannelProfile remote;
  remote.kind = navtracker::ChannelKind::Surveillance;
  remote.sensor = SensorKind::RemoteTrack;
  remote.duty_cycle_sec = 1.0;
  remote.max_range_m = 20000.0;
  remote.p_D = 0.85;
  DeclaredSensorActivity::ChannelProfile ais;
  ais.kind = navtracker::ChannelKind::Cooperative;
  ais.sensor = SensorKind::Ais;
  ais.expected_report_interval_sec = 5.0;
  DeclaredSensorActivity::ChannelProfile coop;
  coop.kind = navtracker::ChannelKind::Cooperative;
  coop.sensor = SensorKind::Cooperative;
  coop.expected_report_interval_sec = 5.0;
  return DeclaredSensorActivity{{radar, remote, ais, coop}};
}

long confirmedCount(const PmbmTracker& t) {
  long n = 0;
  for (const auto& tr : t.tracks())
    if (tr.status == navtracker::TrackStatus::Confirmed) ++n;
  return n;
}
const Track* confirmed(const PmbmTracker& t) {
  for (const auto& tr : t.tracks())
    if (tr.status == navtracker::TrackStatus::Confirmed) return &tr;
  return nullptr;
}

// Build a remote report at truth+noise, converting ENU truth back to geodetic.
RemoteTrackReport remoteReport(const Datum& d, double t, std::int32_t id,
                               const Eigen::Vector2d& xy_noisy,
                               std::uint32_t mmsi) {
  const auto g = d.toGeodetic(Eigen::Vector3d(xy_noisy.x(), xy_noisy.y(), 0.0));
  RemoteTrackReport r;
  r.time = Timestamp::fromSeconds(t);
  r.remote_track_id = id;
  r.lat_deg = g.lat_deg;
  r.lon_deg = g.lon_deg;
  r.position_covariance = Eigen::Matrix2d::Identity() * 100.0;  // σ=10 stated
  r.mmsi = mmsi;
  r.source_id = "vts_hamburg";
  return r;
}

}  // namespace

TEST(PmbmRemoteTrackFusion, FourSourcesOneTrackStableThroughRemoteSwapAndDrop) {
  const Datum d = gDatum();
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gActivity();

  PmbmTracker::Config c;
  c.gate_threshold = 20.0;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-4;
  // Production birth recipe (mirrors benchmark::makePmbmConfig): measurement-
  // driven birth WITH the Reuter-2014 smart-birth skip — never birth at a
  // measurement already explained by an existing high-r Bernoulli. This is the
  // mechanism that stops a moving multi-sensor target from spawning duplicate
  // (cross-hypothesis) tracks; a bare adaptive_birth path lacks the skip and
  // over-generates. Using the real recipe here is production-representative, not
  // a gate tuned to pass.
  c.measurement_driven_birth = true;
  c.birth_weight_per_measurement = 0.3;
  c.smart_birth_skip_existing = true;
  c.smart_birth_skip_r_min = 0.5;
  c.smart_birth_skip_gate = 20.0;
  c.min_new_bernoulli_existence = 0.05;  // phantom-birth gate (deployed value)
  c.k_best_per_hypothesis = 1;
  c.max_global_hypotheses = 10;
  c.confirm_threshold = 0.5;
  c.output_existence_floor = 0.1;
  c.r_min = 1e-5;
  c.idle_halflife_sec = 10.0;
  // Miss model: use_sensor_activity ALONE (the guard-compliant deployment
  // config). NOT source_aware_misdetection (makePmbmConfig uses that, but the
  // two are mutually exclusive — the R9 constructor guard refuses the pair).
  c.use_sensor_activity = true;
  c.cooperative_stale_timeout_sec = 20.0;
  PmbmTracker tracker(ekf, c);
  tracker.setSensorActivity(&activity);

  RemoteTrackAdapterConfig remote_cfg;
  remote_cfg.r_inflation_factor = 3.0;
  remote_cfg.min_update_interval_s = 0.5;  // let the per-second remote feed pass
  RemoteTrackAdapter remote(d, remote_cfg);

  std::mt19937_64 rng(20260704ULL);
  const std::uint32_t kMmsi = 211999888u;

  double nees_sum = 0.0;
  int nees_n = 0;
  auto sampleNees = [&](double t) {
    const Track* tr = confirmed(tracker);
    if (!tr || tr->state.size() < 2) return;
    const Eigen::Vector2d e = tr->state.head<2>() - gTruth(t);
    const Eigen::Matrix2d P = tr->covariance.topLeftCorner<2, 2>();
    nees_sum += e.transpose() * P.inverse() * e;
    ++nees_n;
  };

  // Feed one second: radar @k.0, ais @k.25, coop @k.5, remote @k.75. Staggered
  // so no two-in-one-scan competition; every source reports the same vessel.
  auto feedSecond = [&](int k, std::int32_t remote_id, bool include_remote) {
    const double base = k;
    tracker.predict(Timestamp::fromSeconds(base));
    tracker.processBatch({gPos(base, SensorKind::ArpaTtm, "radar",
                               gTruth(base) + Eigen::Vector2d(gauss(rng, 3.0),
                                                              gauss(rng, 3.0)),
                               3.0)});
    sampleNees(base);  // sample right after the truthful radar update

    tracker.predict(Timestamp::fromSeconds(base + 0.25));
    Measurement ais = gPos(base + 0.25, SensorKind::Ais, "ais",
                           gTruth(base + 0.25) + Eigen::Vector2d(
                               gauss(rng, 6.0), gauss(rng, 6.0)),
                           6.0);
    ais.hints.mmsi = kMmsi;
    tracker.processBatch({ais});

    tracker.predict(Timestamp::fromSeconds(base + 0.5));
    Measurement coop = gPos(base + 0.5, SensorKind::Cooperative, "fleet",
                            gTruth(base + 0.5) + Eigen::Vector2d(
                                gauss(rng, 4.0), gauss(rng, 4.0)),
                            4.0);
    coop.hints.platform_id = std::optional<std::uint64_t>{42ULL};
    tracker.processBatch({coop});

    if (include_remote) {
      tracker.predict(Timestamp::fromSeconds(base + 0.75));
      const Eigen::Vector2d xy =
          gTruth(base + 0.75) +
          Eigen::Vector2d(gauss(rng, 6.0), gauss(rng, 6.0));
      remote.ingest(remoteReport(d, base + 0.75, remote_id, xy, kMmsi));
      tracker.processBatch(remote.poll());
    }
  };

  // ---- Phase 1: all four sources, remote id = 100 (t = 0..15). ----
  for (int k = 0; k <= 15; ++k) feedSecond(k, /*remote_id=*/100, true);
  ASSERT_EQ(confirmedCount(tracker), 1)
      << "(a) remote+radar+ais+coop on one vessel must fuse to ONE track";
  const Track* tr = confirmed(tracker);
  ASSERT_NE(tr, nullptr);
  const std::uint64_t vessel_id = tr->id.value;
  // (b) the remote pseudo-measurement actually fused in (provenance carries a
  // RemoteTrack touch), not a separate track.
  const bool remote_fused = std::any_of(
      tr->recent_contributions.begin(), tr->recent_contributions.end(),
      [](const Track::SourceTouch& s) {
        return s.sensor == SensorKind::RemoteTrack;
      });
  EXPECT_TRUE(remote_fused) << "(b) the remote feed must fuse into the one track";

  // ---- Phase 2: remote SWAPS its own track id 100 -> 200 (t = 16..25). ----
  for (int k = 16; k <= 25; ++k) feedSecond(k, /*remote_id=*/200, true);
  ASSERT_EQ(confirmedCount(tracker), 1)
      << "(c) a remote id-swap must NOT spawn a second track";
  const Track* tr2 = confirmed(tracker);
  ASSERT_NE(tr2, nullptr);
  EXPECT_EQ(tr2->id.value, vessel_id)
      << "(c) fused track ID must be STABLE across a remote id-swap (id is a hint)";

  // ---- Phase 3: remote DROPS; radar+ais+coop continue (t = 26..45). ----
  for (int k = 26; k <= 45; ++k) feedSecond(k, /*remote_id=*/0, false);
  ASSERT_EQ(confirmedCount(tracker), 1)
      << "(d) remote dropout must not retire the track — radar corroborates";
  const Track* tr3 = confirmed(tracker);
  ASSERT_NE(tr3, nullptr);
  EXPECT_EQ(tr3->id.value, vessel_id)
      << "(d) fused track ID must be STABLE across the remote dropout";

  // (e) NEES consistency: with truthful radar R and a x3-inflated remote R, the
  // fused position estimate must not be OVERCONFIDENT. 2 DOF -> E[NEES]=2; a
  // generous ceiling of 8 catches gross overconfidence (the R-inflation tripwire)
  // without flaking on a single deterministic run. Recorded in the eval log.
  ASSERT_GT(nees_n, 10);
  const double mean_nees = nees_sum / nees_n;
  std::cout << "\n=== R10 fusion NEES (2 DOF, E=2) === mean_nees=" << mean_nees
            << " over " << nees_n << " samples\n"
            << std::flush;
  EXPECT_LT(mean_nees, 8.0)
      << "(e) fused estimate is overconfident — R-inflation is not enough";

  // The circular-AIS guard fires: the remote feed and the raw-AIS channel both
  // carry kMmsi, so a deployment wiring both must dedupe (adapter surfaces it).
  const auto circular = remote.circularAisMmsis({kMmsi});
  ASSERT_EQ(circular.size(), 1u);
  EXPECT_EQ(circular[0], kMmsi);
}
