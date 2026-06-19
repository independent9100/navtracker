#include "sim/SkewInjector.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <random>

namespace navtracker {

SkewProfile defaultMaritimeSkewProfile() {
  SkewProfile p;
  p.at(SensorKind::Unknown) = {0.0, 0.0};
  p.at(SensorKind::Ais)     = {0.50, 1.00};
  p.at(SensorKind::ArpaTtm) = {0.05, 0.05};
  p.at(SensorKind::ArpaTll) = {0.05, 0.05};
  p.at(SensorKind::EoIr)    = {0.15, 0.05};
  p.at(SensorKind::OwnShip) = {0.00, 0.02};
  p.at(SensorKind::Lidar)   = {0.00, 0.00};
  p.at(SensorKind::Cooperative) = {0.10, 0.05};  // typical fleet-link latency
  return p;
}

namespace {
constexpr int kJitterTicks = 1024;  // half-range; full ladder is 2*N+1.

double drawJitter(std::mt19937_64& rng, double half_width_s) {
  if (half_width_s <= 0.0) return 0.0;
  const std::uint64_t span = 2ULL * kJitterTicks + 1ULL;
  const int tick = static_cast<int>(rng() % span) - kJitterTicks;
  return static_cast<double>(tick) * (half_width_s / static_cast<double>(kJitterTicks));
}
}  // namespace

std::vector<Measurement> applySkew(const std::vector<Measurement>& input,
                                   const SkewProfile& profile,
                                   std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  const std::size_t n = input.size();
  std::vector<std::int64_t> arrival_nanos(n);
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), std::size_t{0});

  for (std::size_t i = 0; i < n; ++i) {
    const auto& entry = profile.at(input[i].sensor);
    const double jitter = drawJitter(rng, entry.jitter_s);
    const double offset_s = entry.lag_s + jitter;
    arrival_nanos[i] =
        input[i].time.nanos() + static_cast<std::int64_t>(offset_s * 1e9);
  }

  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     return arrival_nanos[a] < arrival_nanos[b];
                   });

  std::vector<Measurement> out;
  out.reserve(n);
  for (std::size_t idx : order) out.push_back(input[idx]);
  return out;
}

}  // namespace navtracker
