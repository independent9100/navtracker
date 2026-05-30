#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// Time-ordered buffer that releases measurements once they are older than
// `window_seconds` behind the latest timestamp seen. Late arrivals (older
// than that cutoff) are dropped.
class ReorderBuffer {
 public:
  explicit ReorderBuffer(double window_seconds);

  // Returns true if the measurement was accepted; false if dropped as late.
  bool push(const Measurement& m);

  // Release all measurements with time <= (latest_seen - window), in
  // chronological order.
  std::vector<Measurement> drain();

  std::size_t pending() const { return queue_.size(); }
  std::size_t dropped() const { return dropped_; }

 private:
  std::int64_t window_nanos_;
  bool seen_{false};
  Timestamp latest_;
  std::multimap<Timestamp, Measurement> queue_;
  std::size_t dropped_{0};
};

}  // namespace navtracker
