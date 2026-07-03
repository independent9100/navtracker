#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_set>

#include "core/collision/Cpa.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ICollisionRiskSink.hpp"

namespace navtracker {

class TrackManager;
class OwnShipProvider;

struct CpaEvaluatorConfig {
  double d_threshold_m{500.0};
  double enter_probability{0.5};
  double exit_probability{0.3};
  bool   evaluate_tentative{false};
  bool   emit_updates{false};
};

// Walks own-ship × each risk-eligible track (Confirmed or Coasting by
// default; Tentative too when evaluate_tentative) at every evaluate(t)
// call, computes CPA with uncertainty, and emits CollisionRiskEvents on
// per-pair Entered/Exited transitions (with hysteresis), plus optional
// Updated events when configured. See spec
// 2026-06-04-track-and-collision-risk-sinks-design.md §5.
//
// Assumptions:
//   1. provider.latest() returns the most recent own-ship pose.
//   2. Track ids never reused after deletion (matches TrackManager
//      invariant).
class CpaEvaluator {
 public:
  CpaEvaluator(const TrackManager& manager,
               const OwnShipProvider& provider,
               CpaEvaluatorConfig cfg = {});

  void setSink(ICollisionRiskSink* sink) { sink_ = sink; }

  // Run one evaluation pass. No-op if no own-ship pose is available.
  void evaluate(Timestamp t);

  // Diagnostics.
  std::size_t entered() const { return n_entered_; }
  std::size_t exited()  const { return n_exited_; }
  std::size_t updated() const { return n_updated_; }
  std::size_t riskyPairs() const { return state_.size(); }

 private:
  const TrackManager& manager_;
  const OwnShipProvider& provider_;
  CpaEvaluatorConfig cfg_;
  ICollisionRiskSink* sink_{nullptr};
  std::unordered_set<std::uint64_t> state_;
  std::size_t n_entered_{0};
  std::size_t n_exited_{0};
  std::size_t n_updated_{0};
};

}  // namespace navtracker
