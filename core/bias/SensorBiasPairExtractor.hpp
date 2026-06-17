#pragma once

#include <vector>

#include "core/bias/SensorBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"
#include "ports/ISensorBiasProvider.hpp"

namespace navtracker {

struct SensorBiasPairExtractorConfig {
  // Pairs from contributions older than this (relative to `cycle_time`)
  // are ignored.
  double cycle_window_seconds{0.5};
  // 1-sigma isotropic anchor (AIS) position uncertainty fallback when
  // the AIS measurement covariance is degenerate.
  double anchor_position_std_fallback_m{10.0};
  // 1-sigma isotropic non-anchor sensor position uncertainty fallback.
  double sensor_position_std_fallback_m{10.0};
};

// Eligibility gates for cross-sensor anchoring (item 13). A track may
// act as a cross-sensor anchor only when the tracker has it well
// converged AND the contributions in play come from different
// SensorBiasKeys (no self-anchoring, no double-anchoring through the
// same sensor twice in the same cycle).
struct CrossSensorEligibilityConfig {
  // Track-quality gate: existence probability ≥ this. Default 0.95
  // matches the spec; tracks below this are too tentative to anchor
  // anything. Tracks without an IPDA lifecycle keep r = 1.0 by
  // convention and pass trivially — that is intentional, the legacy
  // M-of-N path is "always confident" and the SPRT confirm already
  // gated it.
  double min_existence_probability{0.95};
  // Track-quality gate: 2x2 position covariance trace ≤ this (m²).
  // Default 25 m² means the track's position 1-σ envelope is below
  // ~3.5 m per axis — tight enough that the track-derived position
  // is essentially a "honest" anchor for the bias calibration. Loose
  // tracks would feed too much position uncertainty into R_anchor
  // and the bias updates would be near-no-ops anyway, so the gate
  // also keeps us out of the cost of running the math on them.
  double max_position_cov_trace_m2{25.0};
};

// Extract position-bias pair observations from a fused-track snapshot.
// For each track whose `recent_contributions` (within the cycle window
// of `cycle_time`) include both an AIS Position2D contribution and at
// least one *other* Position2D contribution (radar, lidar, ARPA), emit
// one PositionBiasPairObservation per (AIS, non-AIS) pair.
//
std::vector<PositionBiasPairObservation> extractPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg = {});

// Extract bearing-bias pair observations. For each track whose
// `recent_contributions` (within cycle_window_seconds of cycle_time)
// include both an AIS Position2D contribution and at least one
// Bearing2D contribution (EO / IR), emit one BearingBiasPairObservation
// per (AIS, bearing) pair.
//
// The bearing residual is computed as r = wrap(α_obs − α_pred), where
// α_pred is the bearing from the camera's sensor position to the
// AIS anchor's reported ENU position. The bias estimator's bearing
// update consumes this residual directly.
//
// Why this exists separately from the heading-bias estimator's bearing
// path: heading bias is shared across all sensors on the platform; the
// per-sensor mounting bias is sensor-specific. Both can coexist —
// heading bias is removed upstream by OwnShipProvider; the residual
// per-sensor bearing offset is what this estimator catches.
std::vector<BearingBiasPairObservation> extractBearingPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    SensorBiasPairExtractorConfig cfg = {});

// Cross-sensor anchored position-bias pair extraction (backlog item 13).
//
// For each track that (a) passes the eligibility gates in `gates`
// (existence + position-cov), and (b) has **no AIS contribution** in
// the cycle window, walk every ordered pair (Y, X) of positional
// contributions whose SensorBiasKeys differ. Each ordered pair emits
// one observation with X as the sensor under calibration and Y as the
// anchor. Schmidt-KF cov fold: Y's contribution is debiased by the
// current `bias_provider` and Y's published bias covariance is added
// to R_anchor so that the estimator update sees an honest noise floor.
//
// The function intentionally emits BOTH (Y, X) and (X, Y) per unordered
// pair — the joint coordinate descent across the two unknown biases
// is what unsticks the underdetermined "one equation, two unknowns"
// problem. The estimator's bias prior (zero-mean, σ = initial_pos_std)
// breaks the symmetry by pulling each bias to zero so the descent
// converges to the unique solution that minimises both the pair
// residuals and the prior.
//
// `bias_provider` may be null — in that case no debiasing happens
// (b_anchor_hat is treated as zero) and R_anchor is just Y's
// contribution covariance. This is the cold-start path on the first
// few cycles before either bias has any observations.
//
// Tracks with an AIS contribution in the window are skipped: the
// existing extractPositionPairs already handles them with a truth
// anchor, which is strictly more informative.
std::vector<PositionBiasPairObservation> extractCrossSensorPositionPairs(
    const std::vector<Track>& tracks,
    Timestamp cycle_time,
    const ISensorBiasProvider* bias_provider,
    SensorBiasPairExtractorConfig cfg = {},
    CrossSensorEligibilityConfig gates = {});

}  // namespace navtracker
