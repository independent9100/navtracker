// tools/foxglove_pmbm_scenario.cpp — run a scenario through the PMBM tracker
// with the full debug-drawing surface enabled, and write a Lichtblick-openable
// .mcap that exercises EVERY new layer: PMBM posterior (PPP / Bernoulli /
// trajectories), static obstacles, GeoJSON-style land, live occupancy, sensor
// coverage, estimator internals, and the master on/off flag.
//
// Usage:
//   foxglove_pmbm_scenario [scenario_dir] [label] [out.mcap]
// Defaults: data/autoferry/scenario2  scenario2  pmbm_scenario.mcap
//
// Master flag: set NAVTRACKER_DEBUG_DRAW=0 (or false) to disable ALL drawing
// via RecorderConfig.enabled — the run still completes but writes an empty file,
// demonstrating the single per-instance switch (no global state).
//
// The autoferry dataset is framed in the Piren local tangent plane; the loader
// returns ENU metres about that origin. We synthesize a couple of static
// obstacles and one land polygon near the datum so those layers are populated
// even though the dataset carries no chart — the point of this tool is to make
// every debug layer visible in one file.

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "adapters/replay/AutoferryJsonReplay.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/types/StaticObstacle.hpp"

int main(int argc, char** argv) {
  using namespace navtracker;

  const std::string dir   = argc > 1 ? argv[1] : "data/autoferry/scenario2";
  const std::string label = argc > 2 ? argv[2] : "scenario2";
  const std::string out   = argc > 3 ? argv[3] : "pmbm_scenario.mcap";

  // Master flag comes from the environment so it can be flipped without a
  // rebuild; it is threaded into the per-instance RecorderConfig (never global).
  bool draw = true;
  if (const char* e = std::getenv("NAVTRACKER_DEBUG_DRAW"))
    draw = !(std::string(e) == "0" || std::string(e) == "false");

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
  pmbm::PmbmTracker::Config cfg;
  cfg.feed_clutter_map = true;   // so the wrapped clutter map accumulates a heatmap
  pmbm::PmbmTracker tracker(ekf, cfg);

  // Spatial clutter map wrapping a fixed table, wired so the tracker feeds it
  // each scan (cfg.feed_clutter_map) → a real /clutter/position heatmap.
  auto clutter = std::make_shared<ClutterMapSensorDetectionModel>(
      std::make_shared<FixedSensorDetectionModel>(DetectionParams{/*p_D=*/0.9,
                                                                  /*lambda_C=*/1e-4}),
      ClutterMapParams{});
  tracker.setSensorDetectionModel(clutter);

  // Live occupancy wired as birth-suppression + per-scan feed (mirrors Sweep),
  // so the persistence grid accumulates from the scan clutter for a real heatmap.
  auto occupancy = std::make_shared<LiveOccupancyModel>(datum, LiveOccupancyParams{});
  tracker.setStaticObstacleModel(occupancy.get());
  tracker.setLiveOccupancyFeed(occupancy.get());

  foxglove::RecorderConfig rc;
  rc.enabled = draw;
  rc.entity_lifetime_sec = 2.0;   // expire stale 3D entities -> clean "now" view
  foxglove::FoxgloveDebugRecorder recorder(out, datum, /*bias=*/nullptr, rc);

  // Synthetic static world near the datum (the dataset has no chart), so the
  // /static_obstacles and /land layers are populated.
  std::vector<StaticObstacle> obstacles;
  {
    StaticObstacle rock;
    rock.position = geo::Geodetic{63.4400, 10.4010};
    rock.footprint_radius_m = 15.0; rock.keep_clear_radius_m = 120.0;
    rock.category = ObstacleCategory::Rock; rock.source_id = "demo-rock";
    StaticObstacle pile;
    pile.position = geo::Geodetic{63.4378, 10.3975};
    pile.footprint_radius_m = 8.0; pile.keep_clear_radius_m = 80.0;
    pile.category = ObstacleCategory::Pile; pile.source_id = "demo-pile";
    obstacles = {rock, pile};
  }
  LandPolygon land;
  land.outer = {{10.3950, 63.4370}, {10.3990, 63.4370},   // (lon, lat)
                {10.3990, 63.4382}, {10.3950, 63.4382}};
  occupancy->setChartedStructure(obstacles);   // charted-structure layer

  // Time-driven loop grouped into same-timestamp scans (mirrors Sweep / the
  // MHT harness). Per scan: own-ship, tee detections, advance PMBM, snapshot
  // tracks + PMBM posterior + occupancy.
  std::size_t i = 0, scans = 0;
  bool statics_drawn = false;
  while (i < scn.measurements.size()) {
    const Timestamp t = scn.measurements[i].time;
    std::vector<Measurement> scan;
    while (i < scn.measurements.size() && scn.measurements[i].time == t) {
      scan.push_back(scn.measurements[i]);
      ++i;
    }
    ++scans;

    // Own-ship from the platform ENU position carried per detection.
    const geo::Geodetic g = datum.toGeodetic(
        Eigen::Vector3d(scan.front().sensor_position_enu.x(),
                        scan.front().sensor_position_enu.y(), 0.0));
    OwnShipPose pose;
    pose.time = t; pose.lat_deg = g.lat_deg; pose.lon_deg = g.lon_deg;
    recorder.recordOwnShip(pose);

    // Draw the static world once (after the first pose sets last_time_), plus a
    // per-sensor coverage sector for each source seen in this first scan.
    if (!statics_drawn) {
      recorder.recordCoastline({land});
      recorder.recordStaticObstacles(obstacles);
      for (const Measurement& m : scan)
        recorder.recordSensorCoverage(m.source_id, m.sensor, m.sensor_position_enu,
                                      /*center_rad=*/0.0, /*half_width=*/M_PI,
                                      /*range=*/3000.0, t);
      statics_drawn = true;
    }

    for (const Measurement& m : scan) {
      recorder.recordMeasurement(m);
      // Feed position detections as occupancy vessel-fix vetoes (demo of the
      // /occupancy/veto layer).
      if (m.model == MeasurementModel::Position2D ||
          m.model == MeasurementModel::PositionVelocity2D)
        occupancy->observeVesselFix(static_cast<double>(t.nanos()) * 1e-9,
                                    m.value.head<2>());
    }

    tracker.processBatch(scan);
    recorder.onTracks(tracker.tracks(), t);
    recorder.recordPmbmDensity(tracker.density(), t);
    recorder.recordOccupancy(*occupancy, t);
    recorder.recordClutterMap(*clutter, scan.front().sensor_position_enu, t);
  }
  recorder.close();

  std::cout << "Wrote " << out << "  (PMBM, drawing "
            << (draw ? "ENABLED" : "DISABLED") << ", "
            << scn.measurements.size() << " detections over " << scans
            << " scans, " << scn.truth.size() << " truth samples)\n";
  return 0;
}
