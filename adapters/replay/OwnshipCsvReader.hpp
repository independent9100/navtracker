#pragma once

#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker::replay {

// Load a Philos / similar ownship CSV into a time-sorted vector of
// OwnShipPose. Schema (header row, case-insensitive):
//
//   unix_time, lat, lon, heading_deg[, ...]
//
// The C++ replay test then either replays these into an
// OwnShipProvider::update() loop in time order, or hands the whole
// vector to feedOwnshipHistory below for one-shot pre-fill.
std::vector<OwnShipPose> loadOwnshipCsv(const std::string& path);

// Convenience: push every pose into the provider in time order.
void feedOwnshipHistory(OwnShipProvider& provider,
                        const std::vector<OwnShipPose>& poses);

}  // namespace navtracker::replay
