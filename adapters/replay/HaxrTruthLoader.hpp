#pragma once

#include <string>
#include <vector>

#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/scenario/Truth.hpp"

namespace navtracker::replay {

/**
 * Load a HAXR per-station AIS CSV and project each row into the global
 * port frame using the supplied station map.
 *
 * HAXR AIS CSV columns:
 *   tod, uid, range (meters), azimuth (degrees)
 *
 * The hex `uid` is hashed into the `truth_id` (FNV-1a of the digit
 * string) so the same vessel reported through multiple stations
 * receives a stable id. Truth positions are computed as
 *   station_xy + range * [sin(az_marine), cos(az_marine)]
 * to match the same north-up port-frame convention used by the plot
 * adapter.
 */
std::vector<TruthSample> loadHaxrTruth(const std::string& ais_csv_path,
                                       const std::string& station,
                                       const StationMap& stations);

}  // namespace navtracker::replay
