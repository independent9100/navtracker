#include "sim/AisEmitter.hpp"

#include <utility>

namespace navtracker::sim {

AisEmitter::AisEmitter(AisAdapter& adapter,
                       const geo::Datum& datum,
                       AisEmitterConfig cfg,
                       std::uint32_t seed)
    : adapter_(adapter),
      datum_(datum),
      cfg_(std::move(cfg)),
      rng_(seed),
      noise_(0.0, cfg_.pos_std_m > 0.0 ? cfg_.pos_std_m : 1.0) {}

// Class-A SOTDMA cadence (ITU-R M.1371-5 §3.3.4.1.2). Buckets are inclusive
// on the lower bound: speed in [0, 14) kn -> 10 s, [14, 23) kn -> 6 s,
// [23, +inf) kn -> 2 s. Position at the slot time uses the most recent
// truth sample; v1 does not simulate slot-time fractional drift.
double AisEmitter::cadenceSeconds(double speed_mps) {
  // Class-A SOTDMA buckets, table from spec §5.2.
  constexpr double kKnotsPerMps = 1.9438444924;  // 1 m/s in knots
  const double knots = speed_mps * kKnotsPerMps;
  if (knots < 14.0) return 10.0;
  if (knots < 23.0) return 6.0;
  return 2.0;
}

bool AisEmitter::inDropout(double t_relative_s) const {
  for (const auto& [a, b] : cfg_.dropout_windows_s) {
    if (t_relative_s >= a && t_relative_s < b) return true;
  }
  return false;
}

void AisEmitter::emit(const EmitContext& ctx) {
  if (!initialised_) {
    t0_ = ctx.now;
    for (const auto& te : cfg_.targets) next_emit_[te.truth_id] = ctx.now;
    initialised_ = true;
  }

  // Build a quick lookup from truth_id -> TruthState.
  std::unordered_map<std::uint64_t, TruthState> truths;
  truths.reserve(ctx.targets.size());
  for (const auto& t : ctx.targets) truths.emplace(t.truth_id, t.state);

  for (const auto& te : cfg_.targets) {
    auto it = truths.find(te.truth_id);
    if (it == truths.end()) continue;
    const TruthState& truth = it->second;
    const double speed = truth.velocity.norm();
    const double dt = cadenceSeconds(speed);

    Timestamp& next = next_emit_[te.truth_id];
    while (next <= ctx.now) {
      const double t_rel = next.secondsSince(t0_);
      if (!inDropout(t_rel)) {
        const double nx = cfg_.pos_std_m > 0.0 ? noise_(rng_) : 0.0;
        const double ny = cfg_.pos_std_m > 0.0 ? noise_(rng_) : 0.0;
        const Eigen::Vector3d enu(truth.position.x() + nx,
                                  truth.position.y() + ny,
                                  0.0);
        const geo::Geodetic g = datum_.toGeodetic(enu);

        AisDynamicReport r;
        r.time = next;
        r.mmsi = te.mmsi;
        r.lat_deg = g.lat_deg;
        r.lon_deg = g.lon_deg;
        r.high_accuracy = te.high_accuracy;
        adapter_.ingest(r);
      }
      next = Timestamp::fromSeconds(next.seconds() + dt);
    }
  }
}

}  // namespace navtracker::sim
