// tools/foxglove_autoferry.cpp — run an AutoFerry scenario through the tracker
// with the Foxglove debug recorder enabled, and write a Lichtblick-openable
// .mcap.
//
// Usage:
//   foxglove_autoferry [scenario_dir] [label] [out.mcap] [mht|gnn]
// Defaults: data/autoferry/scenario2  scenario2  autoferry_scenario2.mcap  mht
//
// The default tracker is the MHT (multiple-hypothesis) tracker with the
// IPDA existence + VIMM visibility lifecycle — the config that actually suits
// this heterogeneous multi-sensor data (it suppresses the tentative-track
// explosion the GNN baseline produces). Pass "gnn" to record the GNN/EKF
// baseline instead, for comparison.
//
// The AutoFerry dataset is framed in the Piren local tangent plane; the loader
// returns ENU metres about that origin, so the datum is the Piren LLA and the
// /map output lands in Trondheim harbour. IR/EO bearings are off (loader
// default), so detections are the lidar + radar position returns.

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/replay/AutoferryJsonReplay.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"

int main(int argc, char** argv) {
  using namespace navtracker;

  const std::string dir   = argc > 1 ? argv[1] : "data/autoferry/scenario2";
  const std::string label = argc > 2 ? argv[2] : "scenario2";
  const std::string out   = argc > 3 ? argv[3] : "autoferry_scenario2.mcap";
  const std::string which = argc > 4 ? argv[4] : "mht";
  const bool use_mht = (which != "gnn");

  const Scenario scn = replay::loadAutoferryScenario(dir, label);
  if (scn.measurements.empty()) {
    std::cerr << "No measurements loaded from " << dir << " (label " << label
              << "). Is the fixture present?\n";
    return 1;
  }

  // Dataset origin: Piren local tangent plane (see AutoferryJsonReplay.hpp).
  const geo::Datum datum(geo::Geodetic{63.4389029083, 10.39908278, 39.923});

  auto motion = std::make_shared<ConstantVelocity2D>(/*q=*/0.1);
  EkfEstimator ekf(motion, /*init_pos_std_m=*/5.0);

  // GNN baseline (only built when requested).
  GnnAssociator gnn(/*chi2_gate=*/20.0);
  TrackManager mgr(/*confirm_hits=*/2, /*delete_misses=*/3);
  Tracker gnn_tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/30.0);

  // MHT (default): IPDA + VIMM lifecycle are on by default in Config.
  MhtTracker::Config cfg;
  MhtTracker mht(ekf, cfg);

  foxglove::RecorderConfig rc;
  // Match the recorder's gate ellipse to the active tracker's gate threshold.
  rc.gate_gamma = use_mht ? cfg.gate_threshold : 20.0;
  rc.entity_lifetime_sec = 2.0;  // expire stale 3D entities -> clean "now" view
  foxglove::FoxgloveDebugRecorder recorder(out, datum, /*bias=*/nullptr, rc);
  if (use_mht) {
    mht.setInnovationSink(&recorder);
  } else {
    mgr.setTrackSink(&recorder);
    gnn_tracker.setInnovationSink(&recorder);
  }

  // Time-driven loop, grouped into same-timestamp scans (MHT is scan-batched;
  // mirrors core/scenario/HarnessBatchedMht.cpp). Per scan: show own-ship, tee
  // detections, advance the tracker, then snapshot the track set.
  std::size_t i = 0;
  std::size_t scans = 0;
  while (i < scn.measurements.size()) {
    const Timestamp t = scn.measurements[i].time;
    std::vector<Measurement> scan;
    while (i < scn.measurements.size() && scn.measurements[i].time == t) {
      scan.push_back(scn.measurements[i]);
      ++i;
    }
    ++scans;

    // Own-ship: loader carries the platform ENU position per detection;
    // convert back to lat/lon (heading unknown -> 0).
    const geo::Geodetic g = datum.toGeodetic(
        Eigen::Vector3d(scan.front().sensor_position_enu.x(),
                        scan.front().sensor_position_enu.y(), 0.0));
    OwnShipPose pose;
    pose.time = t;
    pose.lat_deg = g.lat_deg;
    pose.lon_deg = g.lon_deg;
    recorder.recordOwnShip(pose);

    for (const Measurement& m : scan) recorder.recordMeasurement(m);

    if (use_mht) {
      mht.processBatch(scan);
      recorder.onTracks(mht.tracks(), t);
    } else {
      for (const Measurement& m : scan) gnn_tracker.process(m);
      recorder.onTracks(mgr.tracks(), t);
    }
  }
  recorder.close();

  std::cout << "Wrote " << out << "  (" << (use_mht ? "MHT" : "GNN") << ", "
            << scn.measurements.size() << " detections over " << scans
            << " scans, " << scn.truth.size() << " truth samples)\n";
  return 0;
}
