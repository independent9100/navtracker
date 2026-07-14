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

/**
 * Knobs controlling how loadAutoferryScenario turns the AutoFerry dataset
 * into a Scenario: which sensor kinds to emit, the per-sensor measurement
 * noise used to synthesize covariance (the dataset ships no per-point R),
 * and whether to inject a synthetic truth-derived AIS anchor for bias
 * estimation. Defaults sit at the env-1 empirical σ values (see fields).
 */
struct AutoferryLoadOptions {
  // Lidar/Radar (active, range-bearing → position) measurements are always
  // emitted as Position2D. IR/EO bearings are off by default as a SCENARIO
  // choice (bearing-only refinement isn't part of the autoferry baseline), not
  // a safety gap: initiation is now bearing-safe. Bearing2D is not birth-eligible
  // (canInitiateTrack blocks it), so a lone bearing never reaches initiate() as a
  // birth; and RangeBearing2D births now convert polar→ENU about the sensor via
  // the shared initiationPosCov helper (W4.1) instead of planting value(0..1) as
  // ENU. Flip this flag to feed EO/IR bearings as refinement measurements.
  bool include_bearings = false;

  // Per-sensor measurement-noise std used to synthesize covariance (the
  // dataset ships detections without per-point R). Defaults updated
  // 2026-06-16 (item 12 (a)) from the empirical per-sensor residual
  // analysis in tools/autoferry_r_calibration.py — the original
  // sensor-spec values (lidar 2 m, radar 5 m) under-estimated the
  // *operational* noise the env-1 fixture actually exhibits, driving
  // NEES to 77 on sc5. Empirical per-env σ measured on the fixture:
  //   sensor   env-1 σ   env-2 σ   pooled σ
  //   lidar     8.1 m     2.8 m     7.1 m
  //   radar     6.5 m     4.8 m     5.9 m
  //   IR        0.05 rad  0.088 r   0.068 rad
  //   EO        0.06 rad  0.095 r   0.090 rad
  // Defaults below sit at the env-1 values — that is where the NEES
  // pathology was. For env 2 (sc 13/16/17/22) the operational σ is
  // smaller and the scenario factories in adapters/benchmark/
  // ReplayScenarioRun.cpp tighten these explicitly. Direct callers of
  // loadAutoferryScenario(... env-2 ...) should also override.
  double lidar_pos_std_m = 8.0;
  double radar_pos_std_m = 6.5;
  double bearing_std_rad = 0.0873;  // ~5°, both envs bracket this

  // Inject synthetic AIS-style position measurements derived from the
  // ground-truth file, one per (target, scan). Used as an unbiased
  // anchor for the SensorBiasEstimator (item 9). Off by default — the
  // AutoFerry dataset itself ships no AIS; turning this on adds a
  // Position2D measurement per truth sample with σ ≈
  // truth_anchor_std_m. Conceptually the role RTK-GNSS plays in
  // Helgesen 2022's per-sensor calibration. The injected sensor uses
  // SensorKind::Ais and source_id == "autoferry_truth_anchor".
  bool inject_truth_anchor = false;
  double truth_anchor_std_m = 5.0;
};

/**
 * Load one AutoFerry scenario directory (e.g. data/autoferry/scenario2).
 * `label` is the bare scenario name ("scenario2") used to form filenames.
 * Returns an empty Scenario (no measurements) when the JSON files are
 * absent or unparseable, so callers can skip gracefully.
 */
Scenario loadAutoferryScenario(const std::string& dir,
                               const std::string& label,
                               const AutoferryLoadOptions& opts = {});

}  // namespace navtracker::replay
