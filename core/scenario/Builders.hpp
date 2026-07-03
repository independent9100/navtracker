#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/scenario/Truth.hpp"

namespace navtracker {

Scenario buildStraightLineScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

Scenario buildParallelTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two CV targets on opposing courses, optionally laterally offset. Truth is
// emitted in (A, B) order per step.
Scenario buildCrossingTargetsScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& velocity_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity_b,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two same-direction CV targets with a lateral offset. Truth is emitted in
// (slow, fast) order per step.
Scenario buildOvertakingScenario(
    const Eigen::Vector2d& start_slow,
    const Eigen::Vector2d& velocity_slow,
    const Eigen::Vector2d& start_fast,
    const Eigen::Vector2d& velocity_fast,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// One CV target observed by a sensor at the ENU origin: first sample is a
// Position2D measurement (to seed the track), subsequent samples are
// RangeBearing2D measurements with the supplied noise. Designed to stress
// nonlinear measurement filtering — pass the target close to the origin
// to exercise high curvature of h(x) = (r, atan2(py, px)).
Scenario buildRangeBearingPassScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double range_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// One CV target observed by a bearing-only sensor at the ENU origin. The
// first sample is a Position2D measurement with deliberately WIDE
// covariance to seed a broad prior; subsequent samples are Bearing2D only
// (scalar β = atan2(py, px)). Designed to stress filters: the posterior
// over range stays as wide as the prior, so a Kalman filter's Gaussian
// approximation degrades while a particle filter can represent the
// banana-shaped posterior faithfully.
Scenario buildBearingOnlyScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Target follows three legs: straight at constant velocity from `start` for
// `straight_duration_s`; then a constant-rate turn at `omega_rad_s` for
// `turn_duration_s` (omega > 0 = left turn in ENU); then straight again for
// `straight_duration_s`. Position2D measurements at every `sample_dt_s`.
Scenario buildManeuveringTargetScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    double straight_duration_s,
    double turn_duration_s,
    double omega_rad_s,
    double sample_dt_s,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Two crossing CV targets plus uniform-random clutter measurements per scan.
// All measurements at a given timestamp share that timestamp exactly, so
// `runScenarioBatched` will group them. `n_clutter_per_scan` false alarms
// are drawn uniformly within the box [clutter_min, clutter_max] each scan.
Scenario buildClutterCrossingScenario(
    const Eigen::Vector2d& start_a,
    const Eigen::Vector2d& velocity_a,
    const Eigen::Vector2d& start_b,
    const Eigen::Vector2d& velocity_b,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    int n_clutter_per_scan,
    const Eigen::Vector2d& clutter_min,
    const Eigen::Vector2d& clutter_max,
    std::uint32_t seed = 0);

// Two CV targets crossing at the origin with a brief sensor dropout. Targets
// move toward each other along the x axis at +/- velocity_x, with a small
// y offset (closest approach ~ 2 * y_offset_m). A sensor dropout zeroes the
// emitted measurements for any scan whose timestamp falls within
// [dropout_start_s, dropout_end_s); truth is still emitted so OSPA can be
// computed.
Scenario buildCrossingDropoutScenario(
    double velocity_x_mps,
    double y_offset_m,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    double dropout_start_s,
    double dropout_end_s,
    std::uint32_t seed = 0);

// Along-track speed-change scenario: a target on constant heading that
// suddenly accelerates ("engine surge") and then decelerates ("drift
// after thrust loss"). Truth phases:
//   [0, surge_start)        : constant velocity (initial_velocity)
//   [surge_start, surge_end): constant acceleration surge_accel_mps2
//   [surge_end, end]        : constant deceleration drift_decel_mps2
// No heading change anywhere — this is the niche CV and CT both miss.
// The noisy-CV mode (high accel PSD, no rotational state) is the right
// model for the surge and drift segments. Position2D measurements at
// every truth tick with isotropic Gaussian noise.
Scenario buildSpeedChangeScenario(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& initial_velocity,
    double surge_start_s,
    double surge_duration_s,
    double surge_accel_mps2,
    double drift_decel_mps2,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// Stationary target observed by a bearing-only sensor on a moving platform.
// Sensor starts at sensor_start and moves with sensor_velocity; target sits
// at target_position. Initial sample emits a wide Position2D seed at the
// target with covariance initial_position_std_m^2 (sensor_position_enu set
// to the sensor's initial location). Subsequent samples emit Bearing2D
// measurements whose sensor_position_enu is the sensor's ENU position at
// that timestamp. Range becomes observable via parallax over the run.
Scenario buildBearingOnlyMovingSensorScenario(
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& sensor_start,
    const Eigen::Vector2d& sensor_velocity,
    const std::vector<double>& sample_times_seconds,
    double initial_position_std_m,
    double bearing_std_rad,
    std::uint32_t seed = 0,
    std::uint64_t truth_id = 1);

// N constant-velocity targets on parallel lanes, all sharing `velocity`.
// Lane i (i = 0..n_targets-1) is offset from `start` by i*lane_spacing_m along
// the unit vector perpendicular to `velocity` (rotated +90 deg: (-vy, vx)/|v|).
// Truth ids are 1..n_targets, emitted in lane order each scan. `velocity` must
// be non-zero. Stresses track resolution / merge as spacing shrinks.
Scenario buildParallelLaneScenario(
    int n_targets,
    double lane_spacing_m,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& velocity,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// Two CV targets that both pass through `crossing_point` at the mid-time of
// `sample_times_seconds`, subtending `crossing_angle_deg`. Target A heads +x
// at `speed_mps`; target B heads at `crossing_angle_deg` from +x at the same
// speed. Truth emitted in (A, B) order each scan (ids 1, 2). Sweep the angle
// externally (e.g. 30/60/90) to probe angle-dependent association.
Scenario buildCrossingAngleScenario(
    double crossing_angle_deg,
    double speed_mps,
    const Eigen::Vector2d& crossing_point,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// `n_targets` CV targets in a single lane along +x at `speed_mps`, spaced by
// `gap_m` (member i starts at x = -i*gap_m, y = 0), plus one faster overtaker
// at `overtaker_speed_mps` starting behind the convoy on a parallel track
// 25 m to the side. Truth emitted convoy-first (ids 1..n_targets) then the
// overtaker (id n_targets+1) each scan. Stresses in-line association +
// overtaking.
Scenario buildConvoyScenario(
    int n_targets,
    double gap_m,
    double speed_mps,
    double overtaker_speed_mps,
    const std::vector<double>& sample_times_seconds,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

// A synthetic coastline plus the fixed shore-clutter positions derived from
// it. The same object seeds the stationary returns (addShoreClutter) AND is
// handed to the land model (ScenarioRun::syntheticCoastline) so an A/B of
// use_land_model runs against one shoreline. Deterministic: no RNG.
struct SyntheticShore {
  CoastlineGeometry geometry;
  geo::Datum datum;
  std::vector<Eigen::Vector2d> clutter_enu_points;  // ENU, deep inland
};

// Build a simple synthetic shoreline about `datum_origin`: land occupies
// y >= shore_y_m up to y = shore_y_m + land_depth_m, across x in
// [-extent_m, extent_m], with one rectangular pier of width pier_width_m
// protruding pier_length_m into the water at x = 0. The ENU outline is
// converted to geodetic via the datum and stored as one LandPolygon (outer
// ring, lon/lat). `n_clutter` stationary returns are placed deep inland
// (y = shore_y_m + 0.5*land_depth_m, spread across x) — the hard-gate region.
SyntheticShore buildSyntheticShore(
    const geo::Geodetic& datum_origin,
    double shore_y_m,
    double extent_m,
    double land_depth_m,
    double pier_width_m,
    double pier_length_m,
    int n_clutter,
    const CoastlinePriorParams& params = {});

// Add fixed, recurring returns at `points` (ENU) to `base`, tagged
// SensorKind::ArpaTtm / `source_id`. For each distinct scan timestamp in
// base.measurements, each point emits a Position2D measurement at its fixed
// position plus isotropic Gaussian noise, with probability `detection_prob`
// (seeded Bernoulli). NO TruthSample is created — these are environment /
// structure, not vessels. Sets base.datum = datum; returns measurements and
// truth re-sorted by time. addShoreClutter is a thin wrapper over this with
// source_id "sim_shore".
Scenario addFixedClutter(
    Scenario base, const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& points, const std::string& source_id,
    double detection_prob, double pos_noise_std_m, std::uint32_t seed = 0);

// Add `boat_positions.size()` stationary (anchored) vessels to `base`. Each
// boat is a REAL target: a zero-velocity TruthSample is emitted for it every
// scan (ids truth_id_start, +1, ...), AND — with probability detection_prob —
// a radar-like ArpaTtm / "sim_anchored" Position2D return with isotropic
// noise (a compact watch circle). Truth is emitted even on undetected scans
// (the boat is present). These are the "keep, never suppress" set for the
// live static-occupancy layer. Sets base.datum; returns measurements and truth
// re-sorted by time.
Scenario addAnchoredBoats(
    Scenario base, const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& boat_positions,
    std::uint64_t truth_id_start, double detection_prob, double pos_noise_std_m,
    std::uint32_t seed = 0);

// Add ONE target that transitions from anchored to underway with a STABLE
// truth id (the ADR-0002 amendment rule-3 recovery case). For each scan
// timestamp already in `base`:
//   t <  move_start_s : truth position = anchored_pos, velocity = 0.
//   t >= move_start_s : truth position = anchored_pos + depart_velocity ·
//                       (t − move_start_s), velocity = depart_velocity.
// Truth is emitted every scan under the single id `truth_id` (identity must
// survive the transition). A radar-like ArpaTtm / "sim_stopgo" Position2D
// return is emitted with probability `detection_prob` (seeded Bernoulli). The
// boat is non-cooperative (no AIS) on purpose: while anchored a live-occupancy
// detector may classify + suppress it as a static hazard (accepted degraded
// mode); once underway the vacated cells must decay so it births + confirms as
// a moving track. Sets base.datum; returns measurements + truth re-sorted by
// time.
Scenario addStopGoBoat(
    Scenario base, const geo::Datum& datum, const Eigen::Vector2d& anchored_pos,
    const Eigen::Vector2d& depart_velocity, double move_start_s,
    std::uint64_t truth_id, double detection_prob, double pos_noise_std_m,
    std::uint32_t seed = 0);

// Add `n_per_scan` uniform-random false alarms per scan to `base`, drawn in
// the ENU box [box_min, box_max], tagged ArpaTtm / "sim_clutter". Positions
// are re-drawn every scan (transient — the defining contrast with the fixed
// recurring returns of addFixedClutter). NO TruthSample. Sets base.datum;
// returns measurements and truth re-sorted by time.
Scenario addUniformClutter(
    Scenario base, const geo::Datum& datum, const Eigen::Vector2d& box_min,
    const Eigen::Vector2d& box_max, int n_per_scan, std::uint32_t seed = 0);

// Add stationary shore clutter to `base`. For each distinct scan timestamp in
// base.measurements, each point in `clutter_enu_points` emits a Position2D
// measurement (SensorKind::ArpaTtm, source_id "sim_shore") at its fixed ENU
// position plus isotropic Gaussian noise, with probability `detection_prob`
// (seeded Bernoulli). NO TruthSample is created for clutter. Sets base.datum
// = datum and returns base with measurements re-sorted by time. The same
// nominal positions recur every scan — the defining property versus the
// uniform-Poisson buildClutterCrossingScenario.
Scenario addShoreClutter(
    Scenario base,
    const geo::Datum& datum,
    const std::vector<Eigen::Vector2d>& clutter_enu_points,
    double detection_prob,
    double pos_noise_std_m,
    std::uint32_t seed = 0);

}  // namespace navtracker
