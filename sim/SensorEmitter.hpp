#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "core/types/Timestamp.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct TargetTruth {
  std::uint64_t truth_id{0};
  TruthState state;
};

// All-emitters-shared context per bus tick. The bus pre-evaluates every truth
// trajectory once and passes everyone the same view, so all sensors at a
// given timestamp see a consistent world.
struct EmitContext {
  Timestamp now;
  TruthState ownship_truth;
  std::vector<TargetTruth> targets;
  // Some emitters do not need randomness (OwnShipEmitter when noise=0 still
  // pulls from its own member RNG). Keeping a pointer here so the bus can
  // pass a tick-scoped RNG if we ever need shared randomness; for now,
  // emitters own their own substream RNG and ignore this field.
  std::mt19937* rng_unused{nullptr};
};

class ISensorEmitter {
 public:
  virtual ~ISensorEmitter() = default;
  virtual void emit(const EmitContext& ctx) = 0;
};

}  // namespace navtracker::sim
