#pragma once

#include <cstdint>
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
};

class ISensorEmitter {
 public:
  virtual ~ISensorEmitter() = default;
  virtual void emit(const EmitContext& ctx) = 0;
};

}  // namespace navtracker::sim
