// app/example.cpp — Library use example for navtracker.
//
// Demonstrates the canonical path for plugging navtracker into your
// stack: build the tracker, build the own-ship provider, feed parsed
// Measurements through tracker.process(), and drain track snapshots
// from the TrackManager whenever your sink wants them.
//
// This file is for documentation; it builds but the main() does not
// run any external I/O.

#include <iostream>
#include <memory>
#include <vector>

#ifdef NAVTRACKER_WITH_FOXGLOVE
#include <cstdlib>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#endif

#include "core/types/Measurement.hpp"
#include "core/types/MeasurementBuilders.hpp"
#include "core/types/SensorDefaults.hpp"
#include "core/types/Track.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/tracking/DatumShift.hpp"
#include "core/geo/Datum.hpp"
#include "core/output/TrackOutput.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"

int main() {
  using namespace navtracker;

  // ---- Composition root (build once at startup) -----------------------

  OwnShipProvider provider;  // library handles datum automatically
  auto motion = std::make_shared<ConstantVelocity2D>(/*q=*/0.1);
  EkfEstimator ekf(motion, /*init_pos_std_m=*/5.0);
  GnnAssociator gnn(/*chi2_gate=*/20.0);
  TrackManager mgr(/*confirm_hits=*/2, /*delete_misses=*/3);
  Tracker tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/30.0);

  // Wire datum-recenter event so tracks stay in the current ENU frame.
  struct TrackShifterSink : IDatumChangeSink {
    TrackManager* mgr;
    explicit TrackShifterSink(TrackManager* m) : mgr(m) {}
    void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
      shiftTracksOnDatumChange(*mgr, o, n);
    }
  };
  TrackShifterSink mgr_sink{&mgr};
  provider.registerDatumSink(&mgr_sink);

  const SensorDefaults defaults = pessimisticSensorDefaults();

  // ---- Initialize the datum with an own-ship pose FIRST ---------------
  //
  // The provider auto-initializes its working datum from the first pose.
  // Push a pose before constructing any measurements so the datum is set.

  {
    OwnShipPose pose;
    pose.time = Timestamp::fromSeconds(123.0);
    pose.lat_deg = 53.500;
    pose.lon_deg = 8.000;
    pose.heading_true_deg = 45.0;
    pose.position_std_m = 5.0;    // from your GPS receiver, or 0 + defaults
    provider.update(pose);
  }

#ifdef NAVTRACKER_WITH_FOXGLOVE
  std::unique_ptr<foxglove::FoxgloveDebugRecorder> recorder;
  if (const char* mcap_path = std::getenv("NAVTRACKER_MCAP")) {
    foxglove::RecorderConfig rc; rc.gate_gamma = 20.0;  // match GnnAssociator chi2_gate
    recorder = std::make_unique<foxglove::FoxgloveDebugRecorder>(
        mcap_path, provider.datum(), /*bias=*/nullptr, rc);
    mgr.setTrackSink(recorder.get());
    tracker.setInnovationSink(recorder.get());
  }
#endif

  // ---- Each time YOUR pipeline emits a parsed AIS report --------------
  //
  // AIS gives the target's absolute lat/lon directly. Convert to ENU
  // once at the boundary, then construct the Measurement.

  {
    const double lat = 53.55, lon = 8.05;
    const auto enu = provider.datum().toEnu({lat, lon, 0.0});

    Measurement m = makeMeasurementFromEnuPosition(
        SensorKind::Ais, "my_ais_feed",
        Timestamp::fromSeconds(123.0),
        Eigen::Vector2d(enu.x(), enu.y()),
        Eigen::Matrix2d::Zero(),  // empty -> defaults will fill in
        AssociationHints{/*mmsi=*/200000001u, std::nullopt});
    applyDefaultsIfEmpty(m, defaults);
#ifdef NAVTRACKER_WITH_FOXGLOVE
    if (recorder) recorder->recordMeasurement(m);
#endif
    tracker.process(m);
  }

  // ---- Each time YOUR pipeline emits a parsed radar return ------------
  //
  // Radar reports (range, relative_bearing). The library adds own-ship
  // heading and projects to ENU, including GPS and heading covariance.

  {
    const double range_m = 1500.0;
    const double rel_bearing_rad = 0.5;   // 28.6° to port of bow
    const double range_std_m = 50.0;
    const double bearing_std_rad = 1.0 * 3.14159265358979 / 180.0;

    Measurement m = makeMeasurementFromRelativeBearing(
        SensorKind::ArpaTtm, "my_radar",
        Timestamp::fromSeconds(123.0),
        range_m, rel_bearing_rad,
        range_std_m, bearing_std_rad,
        provider);
    // If your radar didn't report std values, leave them 0 and call:
    // applyDefaultsIfEmpty(m, defaults);
    if (m.value.size() > 0) {
#ifdef NAVTRACKER_WITH_FOXGLOVE
      if (recorder) recorder->recordMeasurement(m);
#endif
      tracker.process(m);
    }
  }

  // ---- Drain the current track snapshot in operator-friendly form ----

#ifdef NAVTRACKER_WITH_FOXGLOVE
  if (recorder) recorder->onTracks(mgr.tracks(), Timestamp::fromSeconds(123.0));
#endif

  for (const Track& t : mgr.tracks()) {
    if (t.status != TrackStatus::Confirmed) continue;
    const TrackOutput out = toTrackOutput(t, provider.datum());
    // out.position.lat_deg / lon_deg in WGS84 degrees
    // out.position.position_covariance_m2 in m^2 (north, east)
    // out.velocity.sog_m_per_s, .cog_deg, .sigma_*, .is_valid
    // out.id, out.status, out.attributes, out.contributing_sources
    std::cout << "Track id=" << out.id.value
              << "  lat=" << out.position.lat_deg
              << "  lon=" << out.position.lon_deg;
    if (out.velocity.is_valid) {
      std::cout << "  sog=" << out.velocity.sog_m_per_s
                << "  cog=" << out.velocity.cog_deg;
    }
    std::cout << "\n";
  }

  return 0;
}
