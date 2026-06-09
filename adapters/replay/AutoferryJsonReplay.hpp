#pragma once

#include <string>

#include "core/scenario/Truth.hpp"

// Loader for the AutoFerry milliAmpere heterogeneous multi-sensor tracking
// benchmark (Helgesen et al., Ocean Engineering 252, 2022).
// Dataset: https://github.com/Autoferry/sensor_fusion_dataset
//
// Each scenario folder ships two JSON files:
//   <label>_detections.json  — flat array of detection objects
//   <label>_groundTruth.json — array (one entry per detection index) of
//                               arrays of per-target ground-truth objects
//
// Detection object:
//   { "sensorID": 1|2|3|4,          // 1=Lidar 2=Radar 3=IR 4=EO
//     "ownshipPosition": [N, E],    // Piren NED, metres
//     "time": <unix seconds>,
//     "measurement": ...            // see below
//   }
// For Lidar/Radar (1,2): `measurement` is 2×M `[[n0,n1,..],[e0,e1,..]]`
//   (vessel-fixed NED relative to ownship), or the flat `[n,e]` when M==1,
//   or `[]` for an empty scan. Absolute Piren-NED point = ownship + (n,e).
// For IR/EO (3,4): `measurement` is a 1×M list of bearings (or a bare
//   scalar when M==1), each a NED bearing atan2(e,n) measured from north.
//
// Ground-truth object: { "targetID", "position":[N,E,0], "time" }.
//
// Frame mapping. The whole dataset lives in the Piren local tangent plane
// (NED, origin LLA [63.4389029083, 10.39908278, 39.923]); navtracker's
// internal frame is ENU about a datum. We map NED→ENU by the planar axis
// swap (E,N) ← (N,E); no geodetic transform is needed because measurements
// and truth are already co-framed. The returned Scenario therefore uses
// ENU metres with an implicit datum at Piren — consistent across both
// measurements and truth, which is all OSPA/continuity scoring requires.

namespace navtracker::replay {

struct AutoferryLoadOptions {
  // Lidar/Radar (active, range-bearing → position) measurements are always
  // emitted as Position2D. IR/EO bearings are off by default: a lone
  // bearing that fails to gate would reach EkfEstimator::initiate, which
  // seeds position from value(0..1) and is undefined for a 1-D bearing.
  // Enabling bearings requires a bearing-safe initiation path (TODO).
  bool include_bearings = false;

  // Per-sensor measurement-noise std used to synthesize covariance (the
  // dataset ships detections without per-point R). Defaults follow the
  // sensor stack in the paper: Velodyne VLP-16 lidar is the most precise
  // active sensor, the Simrad 4G broadband radar the coarser one.
  double lidar_pos_std_m = 2.0;
  double radar_pos_std_m = 5.0;
  double bearing_std_rad = 0.0873;  // ~5°, matches observed EO/IR residual
};

// Load one AutoFerry scenario directory (e.g. data/autoferry/scenario2).
// `label` is the bare scenario name ("scenario2") used to form filenames.
// Returns an empty Scenario (no measurements) when the JSON files are
// absent or unparseable, so callers can skip gracefully.
Scenario loadAutoferryScenario(const std::string& dir,
                               const std::string& label,
                               const AutoferryLoadOptions& opts = {});

}  // namespace navtracker::replay
