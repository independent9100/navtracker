#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker::replay {

/**
 * Per-station ENU position, looked up by the `station` column in a plot CSV.
 * HAXR ships `stations.csv` with columns `station,x_m,y_m`; loadStations
 * reads it.
 */
using StationMap = std::unordered_map<std::string, Eigen::Vector2d>;

/** Read a `stations.csv` (columns `station,x_m,y_m`) into a StationMap. */
StationMap loadStations(const std::string& stations_csv_path);

/**
 * Load a radar/lidar plot CSV (HAXR / Pohang radar / Pohang LiDAR share
 * schema) into a vector of Measurement(Position2D) sorted by time.
 * Range/azimuth are projected into the station's ENU frame at the edge
 * because the existing EkfEstimator::initiate path takes Cartesian state
 * from value(0..1) — mirroring the ArpaAdapter convention.
 *
 * Plot CSV columns:
 *   tod,range_m,azimuth_deg,sigma_r_m,sigma_az_deg,n_cells,amp_max,station
 *
 * - `tod` is seconds (seconds-of-day for HAXR; epoch seconds for Pohang
 *   - the adapter doesn't care, it just hands the value to
 *   Timestamp::fromSeconds).
 * - `azimuth_deg` is the marine convention (north = 0, clockwise);
 *   converted here to the math convention atan2(dy, dx) the tracker uses.
 * - `sigma_r_m` and `sigma_az_deg` become the diagonal of the 2x2 R
 *   (range variance in m², bearing variance in rad²). When a CSV row
 *   reports zero spread (single-cell plot) a per-sensor default is
 *   substituted: default_sigma_r = 25 m, default_sigma_az = 1.0 deg.
 * - `station` is looked up in `stations` to set sensor_position_enu;
 *   rows with an unknown station are dropped.
 *
 * `sensor` and `source_id` are written verbatim onto each measurement so
 * the test scaffolding can tag plots by source.
 */
std::vector<Measurement> loadPlotCsv(const std::string& plots_csv_path,
                                     const StationMap& stations,
                                     SensorKind sensor,
                                     std::string source_id);

/**
 * Body-frame variant for moving-platform radar (Philos, Pohang). Each
 * plot CSV row's (range_m, azimuth_deg) is interpreted in the ownship
 * body frame at the row's timestamp; the function looks up the
 * ownship pose via `OwnShipProvider::poseAtOrBefore(t)` to project
 * into the working ENU frame. Rows with no available pose are dropped.
 *
 * The `OwnShipProvider` must be pre-populated (e.g. via
 * `feedOwnshipHistory`) before this call; its working datum becomes
 * the ENU origin for the output Measurements.
 */
std::vector<Measurement> loadPlotCsvBodyFrame(const std::string& plots_csv_path,
                                              const OwnShipProvider& provider,
                                              SensorKind sensor,
                                              std::string source_id);

}  // namespace navtracker::replay
