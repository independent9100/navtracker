#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

namespace navtracker {

// Output of associating a batch of measurements to existing tracks. Indices
// refer into the input `tracks` and `measurements` vectors.
struct AssociationResult {
  std::vector<std::pair<std::size_t, std::size_t>> matches;  // (track_idx, meas_idx)
  std::vector<std::size_t> unmatched_tracks;
  std::vector<std::size_t> unmatched_measurements;
};

// Data-association strategy: assign measurements to tracks.
class IDataAssociator {
 public:
  virtual ~IDataAssociator() = default;
  virtual AssociationResult associate(
      const std::vector<Track>& tracks,
      const std::vector<Measurement>& measurements) const = 0;
};

}  // namespace navtracker
