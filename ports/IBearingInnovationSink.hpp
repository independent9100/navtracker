#pragma once

#include "core/types/Ids.hpp"
#include "core/types/Timestamp.hpp"

namespace navtracker {

/**
 * One bearing-domain innovation emitted by the Tracker after a successful
 * hard-match update on a Bearing2D or RangeBearing2D measurement. The
 * values are computed from the PRE-update predicted track state so the
 * innovation r is a measurement of the heading bias b (see spec §3).
 *
 * ANGLE CONVENTION (W3.4): innovation_rad is the RAW innovation in the
 * ENU-math bearing frame (β = atan2(dN, dE); 0 = east, counter-clockwise-
 * positive) — the same frame predictMeasurement uses. This is NOT the
 * compass heading-bias convention the HeadingBiasEstimator stores (0 = north,
 * clockwise-positive): a gyro reading high by +b (compass) yields β_obs =
 * β_true − b here, so this raw innovation is −b. The estimator negates it at
 * its observe(BearingInnovation) boundary; consumers must not assume the sign
 * already matches the compass bias.
 *
 * === Math ===
 * r = wrap(β_observed - β_predicted)    (ENU-math frame; = −b for a +b gyro)
 * variance_rad2 = H · P · Hᵀ + R         (predicted innovation variance)
 * predicted_state_var_rad2 = H · P · Hᵀ   (used for the state-dominance gate)
 * where H is the bearing-component row of the measurement Jacobian and
 * P is the predicted track covariance.
 *
 * === Assumptions ===
 *   1. Track state and covariance reflect the predicted state at z.time
 *      (Tracker calls manager_.predictAll before computing these fields).
 *   2. innovation_rad is already wrapped to [-π, π].
 *   3. The bearing component of the measurement noise R is populated by
 *      the adapter; absent (zero) R makes the state-dominance gate
 *      reject the observation harmlessly.
 *
 * === Rationale ===
 *   - Sink (not callback / std::function): matches IDatumChangeSink and
 *     keeps lifetime in the composition root.
 *   - Pre-computed variance and range fields (not raw state/cov): the
 *     estimator has no dependency on Tracker internals or on the Jacobian
 *     code, keeping the bias module decoupled.
 *
 * === Ways to improve / what to test next ===
 *   - JPDA soft-update emit (weighted innovation across betas).
 *   - Per-sensor variant of the sink so adapters can register independently.
 */
struct BearingInnovation {
  Timestamp time;
  TrackId track_id;             // diagnostic only
  double innovation_rad{0.0};
  double variance_rad2{0.0};
  double predicted_state_var_rad2{0.0};
  double range_m{0.0};
};

/**
 * Push-based observer for bearing-domain innovations. The heading-bias
 * estimator implements this and is registered with the Tracker; each
 * emitted `BearingInnovation` is one observation of the gyro/heading bias.
 */
class IBearingInnovationSink {
 public:
  virtual ~IBearingInnovationSink() = default;
  /** Receive one bearing-domain innovation observation. */
  virtual void onBearingInnovation(const BearingInnovation& obs) = 0;
};

}  // namespace navtracker
