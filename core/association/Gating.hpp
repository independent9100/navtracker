#pragma once

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

/**
 * Squared Mahalanobis distance between a measurement and a track's predicted
 * measurement: d2 = y^T S^-1 y, with S = H P H^T + R.
 */
double mahalanobisDistance(const Track& track, const Measurement& z);

}  // namespace navtracker
