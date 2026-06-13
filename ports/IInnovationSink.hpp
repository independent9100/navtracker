#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

// General per-update innovation, emitted from Tracker / MhtTracker after
// a successful hard-match estimator.update. The fields are computed from
// the PRE-update predicted state, so the residual r is the canonical KF
// innovation y = z − h(x̂⁻) the estimator's update saw.
//
// === Math ===
//   ν = z − h(x̂⁻)                        (residual; angle-wrapped where applicable)
//   S = H P⁻ Hᵀ + R                        (predicted innovation covariance)
//   εⁿⁱˢ = νᵀ S⁻¹ ν                       (NIS — consumer computes)
// where x̂⁻, P⁻ are the predicted state/covariance, H the linearised
// measurement Jacobian, R the measurement noise used by the update.
//
// === Assumptions ===
//   1. residual is already model-correct: bearing components are wrapped
//      to (−π, π].
//   2. S = HPHᵀ + R with the SAME R the estimator applied (the field `R`
//      below carries the matrix the emitter used). For JPDA soft updates
//      a per-β-weighted R variant is out of scope here — see footnote.
//   3. P⁻ on Tracker emission is the moment-matched single-Gaussian
//      covariance the EKF Jacobian path uses (mirrors
//      IBearingInnovationSink). For MhtTracker the parent leaf's stored
//      covariance is re-predicted to the scan time to reproduce the
//      pre-update predicted P bit-exactly.
//
// === Rationale ===
//   - Sink (not std::function): matches IBearingInnovationSink and
//     IDatumChangeSink — lifetime in the composition root, no per-event
//     allocations beyond the event payload.
//   - Pre-computed (ν, S, R) fields (not raw H/P/R): consumers don't
//     reimplement Jacobians; keeps the dependency direction inward.
//   - Generic across measurement kinds (Position2D / RangeBearing2D /
//     Bearing2D / PositionVelocity2D) so one aggregator handles the full
//     sensor mix.
//
// === Ways to improve / what to test next ===
//   - JPDA / soft-update emission: weighted innovation across betas, with
//     R/weight inflation matching the soft-update math (same direction
//     IBearingInnovationSink left open).
//   - Per-sensor sink registration if downstream wants to fan out by
//     SourceKey without a central multiplexer.
struct InnovationEvent {
  Timestamp time;
  TrackId track_id;
  SensorKind sensor{SensorKind::Unknown};
  std::string source_id;
  MeasurementModel model{MeasurementModel::Position2D};
  Eigen::VectorXd residual;       // ν, in measurement space
  Eigen::MatrixXd S;              // HPHᵀ + R (predicted innovation cov)
  Eigen::MatrixXd R;              // measurement noise the update used
  std::size_t dim{0};             // ν.size(), redundant but cheap and explicit
};

class IInnovationSink {
 public:
  virtual ~IInnovationSink() = default;
  virtual void onInnovation(const InnovationEvent& e) = 0;
};

}  // namespace navtracker
