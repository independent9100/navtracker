// tools/foxglove_autoferry.cpp — run an AutoFerry scenario through a
// standard EKF/GNN tracker with the Foxglove debug recorder enabled, and
// write a Lichtblick-openable .mcap.
//
// Usage:
//   foxglove_autoferry [scenario_dir] [label] [out.mcap]
// Defaults: data/autoferry/scenario2  scenario2  autoferry_scenario2.mcap
//
// The AutoFerry dataset is framed in the Piren local tangent plane; the
// loader returns ENU metres about that origin, so we construct the datum at
// the Piren LLA and every /map lat/lon lands in Trondheim harbour. IR/EO
// bearings are left off (loader default) — lone-bearing initiation is not
// yet supported — so /detections shows the lidar+radar position returns.

#include <iostream>
#include <memory>
#include <string>

#include "adapters/replay/AutoferryJsonReplay.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/tracking/TrackManager.hpp"
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"

int main(int argc, char** argv) {
  using namespace navtracker;

  const std::string dir   = argc > 1 ? argv[1] : "data/autoferry/scenario2";
  const std::string label = argc > 2 ? argv[2] : "scenario2";
  const std::string out   = argc > 3 ? argv[3] : "autoferry_scenario2.mcap";

  const Scenario scn = replay::loadAutoferryScenario(dir, label);
  if (scn.measurements.empty()) {
    std::cerr << "No measurements loaded from " << dir << " (label " << label
              << "). Is the fixture present?\n";
    return 1;
  }

  // Dataset origin: Piren local tangent plane (see AutoferryJsonReplay.hpp).
  const geo::Datum datum(geo::Geodetic{63.4389029083, 10.39908278, 39.923});

  // Standard composition (mirrors app/example.cpp).
  auto motion = std::make_shared<ConstantVelocity2D>(/*q=*/0.1);
  EkfEstimator ekf(motion, /*init_pos_std_m=*/5.0);
  GnnAssociator gnn(/*chi2_gate=*/20.0);
  TrackManager mgr(/*confirm_hits=*/2, /*delete_misses=*/3);
  Tracker tracker(ekf, gnn, mgr, /*miss_timeout_seconds=*/30.0);

  foxglove::RecorderConfig rc;
  rc.gate_gamma = 20.0;  // match the associator's chi2 gate so /gates is faithful
  foxglove::FoxgloveDebugRecorder recorder(out, datum, /*bias=*/nullptr, rc);
  mgr.setTrackSink(&recorder);
  tracker.setInnovationSink(&recorder);

  // Time-driven loop: tee each detection, process it, then snapshot the
  // track set at that measurement's timestamp so the scrub timeline shows
  // detections, gates, associations, and track evolution together.
  for (const Measurement& m : scn.measurements) {
    recorder.recordMeasurement(m);
    tracker.process(m);
    recorder.onTracks(mgr.tracks(), m.time);
  }
  recorder.close();

  std::cout << "Wrote " << out << "  (" << scn.measurements.size()
            << " detections, " << scn.truth.size() << " truth samples)\n";
  return 0;
}
