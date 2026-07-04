#pragma once

#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/scenario/Truth.hpp"

namespace navtracker::replay {

/**
 * Load a radar-derived truth CSV (columns: tod,uid,range_m,azimuth_deg)
 * into a time-sorted vector of TruthSample.
 *
 * Schema:
 *   tod          — Unix epoch seconds (double)
 *   uid          — MMSI or other integer vessel identifier (uint64)
 *   range_m      — slant range from own-ship in metres
 *   azimuth_deg  — body-frame azimuth in marine convention (north=0,
 *                  clockwise), i.e. the same convention as radar_plots.csv
 *
 * Projection mirrors loadPlotCsvBodyFrame in PlotCsvReplayAdapter.cpp:
 * each row's body-frame (range, azimuth) is rotated into world bearing
 * using the ownship heading at or before `tod`, then projected from the
 * ownship's ENU position into the working ENU frame supplied by the
 * provider.
 *
 * The `provider` must be pre-populated (e.g. via `feedOwnshipHistory`)
 * before calling this function.  Rows with no available pose at or before
 * their timestamp are silently dropped.
 *
 * Output:
 *   TruthSample::time      = Timestamp::fromSeconds(tod)
 *   TruthSample::truth_id  = uid (MMSI)
 *   TruthSample::position  = projected ENU (x=East, y=North), metres
 *   TruthSample::velocity  = zero (unknown from static CSV)
 */
std::vector<TruthSample> loadRadarTruthCsv(const std::string& path,
                                            const OwnShipProvider& provider);

}  // namespace navtracker::replay
