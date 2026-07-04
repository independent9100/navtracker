#pragma once

#include <vector>

#include "core/types/Measurement.hpp"

namespace navtracker {

/**
 * Driving-side edge port: produces normalized measurements pulled from a
 * source (live sensor or a log replay). Concrete adapters live in `adapters/`.
 */
class ISensorAdapter {
 public:
  virtual ~ISensorAdapter() = default;
  /** Drain any measurements that have become available. Empty if none. */
  virtual std::vector<Measurement> poll() = 0;
};

}  // namespace navtracker
