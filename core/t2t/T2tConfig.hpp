#pragma once

// T2tConfig — all knobs for the track-to-track fuser, in one per-instance
// struct. Following the house convention (see RemoteTrackAdapterConfig,
// CpaEvaluatorConfig): brace-init defaults, units baked into field names, one
// doc comment per field, and threaded per-instance through the T2tFuser ctor
// (`T2tFuser(..., T2tConfig cfg = {})`) — never a global, static, or singleton
// (dynamic-library hazard; project rule).
//
// Defaults are deliberately conservative. Anything about correctness (the
// choice of covariance intersection over naive fusion) is NOT a knob — it is
// fixed in the math (docs/algorithms/t2t-fusion.md). These knobs tune only
// timing, gating, and lifecycle.

namespace navtracker::t2t {

struct T2tConfig {
  // --- Time alignment (algorithm doc §1.1) ---

  // White-acceleration process-noise PSD used to predict a stored source
  // track from its last report time to the engine's current time. Same units
  // and form as the core constant-velocity model (m^2 / s^3). Matches the
  // core CV default so a fused track coasts like a native one.
  double process_noise_accel_psd_m2_s3{0.1};

  // A source track with no report newer than this (seconds) stops contributing
  // to its fused track. Stale inputs are never extrapolated forever; the fused
  // track coasts / demotes instead (see fused_delete_age_s).
  double max_report_age_s{10.0};

  // Velocity 1-sigma (m/s) assumed for a POSITION-ONLY source track when
  // predicting it forward to the fusion time. The source states no velocity, so
  // this prior sets how fast its position uncertainty grows between reports. Not
  // used for sources that supply a valid velocity.
  double source_unknown_velocity_std_mps{5.0};

  // --- Track-to-track association (algorithm doc §1.2) ---

  // Squared-Mahalanobis gate on the 2-D position innovation between a fused
  // prediction and a candidate source track, using S = P1 + P2 (cross-
  // correlation ignored -> conservatively wide gate). 9.21 = chi^2 with 2 DoF
  // at probability 0.99. (The repo has no probability->threshold inverse, so
  // this is stored directly as the cutoff, mirroring gate_threshold elsewhere.)
  double gate_chi2_position{9.21};

  // Consistent in-gate assignments before a (fused track, source track)
  // pairing forms. Anti-flicker hysteresis, mirroring the M-of-N spirit used
  // elsewhere in the repo.
  int pair_confirm_hits{3};

  // Consecutive missed/failed assignments a formed pairing survives before it
  // starts breaking. A pairing breaking never deletes the fused track while
  // another source still sustains it.
  int pair_break_misses{3};

  // SOFT bonus subtracted from an assignment cost when both candidate inputs
  // carry the SAME non-empty MMSI (shared MMSI is corroboration). Small
  // relative to the gate so kinematics still dominate.
  double shared_mmsi_cost_bonus{2.0};

  // SOFT penalty added to an assignment cost when the two inputs carry
  // DIFFERENT non-empty MMSIs. Kept below the gate so a very strong kinematic
  // match still wins (invariant 5: identity is evidence, not the key).
  double conflicting_mmsi_cost_penalty{6.0};

  // --- Covariance intersection (algorithm doc §1.3) ---

  // Fixed golden-section iterations for the CI weight search. Fixed (not
  // convergence-gated) so replay is bit-for-bit deterministic.
  int ci_omega_iterations{40};

  // --- Fused identity & lifecycle (algorithm doc §1.5) ---

  // Fused-track birth M-of-N: an unassociated source track births a fused
  // track once seen `fused_confirm_m` times within the last `fused_confirm_n`
  // reports. Single-source fused tracks are legitimate (presence over
  // classification, one level up).
  int fused_confirm_m{2};
  int fused_confirm_n{3};

  // Confirm a fused track immediately when a contributing source reports
  // Confirmed status; otherwise the fused track earns confirmation by its own
  // M-of-N above.
  bool trust_source_status{true};

  // Delete a coasting fused track once no contributing source has reported
  // within this many seconds. Covariance inflates via the CV process noise
  // while coasting; deletion is by age, not by covariance size.
  double fused_delete_age_s{30.0};

  // Drop per-source reports that arrive out of order (older than that source's
  // own high-water timestamp). Per SOURCE, since sources have independent
  // clocks and latencies. Exposed as a drop counter for diagnostics.
  bool reject_stale_reports{true};
};

}  // namespace navtracker::t2t
