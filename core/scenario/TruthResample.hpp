#pragma once

#include <vector>

#include "core/scenario/Truth.hpp"

namespace navtracker {

// Resample asynchronous per-target truth onto a shared fixed clock.
//
// Math. The evaluation clock is t_k = t_min + k·period_s, k = 0..K,
// covering [t_min, t_max] of the input. Per target (truth_id) and per
// tick:
//   - bracketed tick (a.time ≤ t_k ≤ b.time, consecutive samples a, b
//     with b.time − a.time ≤ max_gap_s): position is linearly
//     interpolated, velocity is the segment finite difference
//     (b.pos − a.pos)/(b.time − a.time);
//   - tick within period_s/2 of the target's first/last sample (or of
//     a gap endpoint when the bracket exceeds max_gap_s): clamped to
//     that sample, velocity from the adjacent segment;
//   - otherwise the target is absent at that tick.
// Output is sorted by (time, truth_id); samples sharing a tick carry
// the identical Timestamp — the grouping invariant BenchRunner's
// truth-step bucketing relies on.
//
// Assumptions. Linear interpolation is adequate between fixes — true
// for harbour AIS cadences (≈ 2–12 s) at harbour speeds, where the
// deviation from a straight segment is well below sensor noise. The
// max_gap_s guard exists precisely for when this assumption breaks
// (minutes-long AIS dropouts must not be bridged by a straight line).
//
// Rationale. Replay scenarios with AIS-as-truth (philos) carry
// per-vessel asynchronous messages with no scan structure: no two raw
// samples share a timestamp, so BenchRunner's exact-time bucketing
// fragments every evaluation step to cardinality 1 and the identity
// metrics count phantom switches — the same harness failure mode the
// AutoFerry per-scan truth fix addressed for scan-structured data
// (evaluation-log 2026-06-10). Resampling onto a shared clock restores
// honest per-step cardinality without touching the measurement stream.
// Single-fix targets are snapped to their nearest tick (one-step
// presence) rather than dropped, so a tracked-but-absent-from-truth
// vessel does not show up as a permanent cardinality error.
//
// Ways to improve / test next. (1) Higher-order interpolation using
// AIS SOG/COG when carried through to TruthSample velocity. (2) A
// presence-weighted lifetime metric for targets whose truth coverage
// is much sparser than the tracker's update rate.
//
// period_s ≤ 0 disables resampling (returns the input unchanged).
std::vector<TruthSample> resampleTruthToClock(
    const std::vector<TruthSample>& samples,
    double period_s,
    double max_gap_s);

}  // namespace navtracker
