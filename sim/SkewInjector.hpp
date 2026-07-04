#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker {

// Per-sensor lag (s, constant) and jitter half-width J (s, uniform on [-J,+J]).
// Index by SensorKind cast to size_t.
struct SkewProfile {
  struct Entry { double lag_s{0.0}; double jitter_s{0.0}; };
  // Size matches the SensorKind enumerator count (…, Cooperative, RemoteTrack).
  // Grow this in step whenever a kind is appended. std::array::at() below is
  // deliberately bounds-CHECKED so a missed resize throws std::out_of_range
  // (fail-loud) instead of silently reading out of bounds (R8.8 lesson).
  std::array<Entry, 9> by_kind{};

  Entry& at(SensorKind k) { return by_kind.at(static_cast<std::size_t>(k)); }
  const Entry& at(SensorKind k) const {
    return by_kind.at(static_cast<std::size_t>(k));
  }
};

// Realistic maritime defaults — see spec §3.2.
SkewProfile defaultMaritimeSkewProfile();

// Re-orders `input` into arrival-time order produced by the SkewProfile.
// Measurement.time (truth timestamp) is NEVER modified — only the emission
// order changes. RNG is std::mt19937_64 seeded with `seed`; jitter is drawn
// from a 2049-tick integer ladder so that draws are bit-exact across STLs
// (std::uniform_real_distribution is not portable).
//
// === Math ===
// arrival_time(m) = m.time + lag_k + jitter_k
//   lag_k:    SkewProfile.at(m.sensor).lag_s
//   jitter_k: ((rng() % (2*N+1)) - N) * (J_k / N), N = 1024
// Output order: stable_sort by arrival_time, ties broken by ingestion index.
//
// === Assumptions ===
//   1. Truth timestamps (Measurement.time) are preserved unchanged downstream.
//   2. Skew is symmetric per sensor (Uniform(-J,+J)); burst-correlated lag is
//      out of scope (see "Ways to improve").
//   3. The injector is deterministic in (seed, input order). Same inputs +
//      same seed -> same arrival sequence on any STL/platform.
//
// === Rationale ===
//   - Stable sort on arrival_time: real packets arrive in jittered order; the
//     buffer is what decides drops, not the injector.
//   - Per-kind lag + uniform jitter: matches the observed envelope of
//     maritime feeds (VDL contention for AIS, frame-bus latency for EO/IR)
//     well enough to surface ordering bugs without overclaiming a calibrated
//     noise model.
//   - Integer jitter ladder: std::uniform_real_distribution differs across
//     libstdc++/libc++/MSVC; quantizing to 2049 ticks gives bit-exact draws.
//
// === Ways to improve / what to test next ===
//   - Burst-correlated lag (Markov idle/burst-of-K) for realistic AIS storms.
//   - Per-source-id keying (two AIS receivers with different lags).
//   - Calibrated profiles from sea-trial logs.
std::vector<Measurement> applySkew(const std::vector<Measurement>& input,
                                   const SkewProfile& profile,
                                   std::uint64_t seed);

}  // namespace navtracker
