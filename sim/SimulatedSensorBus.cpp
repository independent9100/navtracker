#include "sim/SimulatedSensorBus.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <random>
#include <utility>

namespace navtracker::sim {

SimulatedSensorBus::SimulatedSensorBus(SimulatedSensorBusConfig cfg)
    : cfg_(std::move(cfg)) {}

void SimulatedSensorBus::setOwnShip(std::shared_ptr<ITruthTrajectory> trajectory) {
  ownship_ = std::move(trajectory);
}

void SimulatedSensorBus::addTarget(std::uint64_t truth_id,
                                   std::shared_ptr<ITruthTrajectory> trajectory) {
  targets_.emplace_back(truth_id, std::move(trajectory));
}

std::uint32_t SimulatedSensorBus::derive_seed_(const char* emitter_id) const {
  std::array<std::uint32_t, 8> mix{};
  mix[0] = cfg_.seed;
  std::size_t i = 1;
  for (const char* p = emitter_id; *p && i < mix.size(); ++p, ++i)
    mix[i] = static_cast<std::uint32_t>(*p);
  std::seed_seq sseq(mix.begin(), mix.end());
  std::array<std::uint32_t, 1> out{};
  sseq.generate(out.begin(), out.end());
  return out[0];
}

void SimulatedSensorBus::attachOwnShip(OwnShipNmeaAdapter& adapter,
                                       OwnShipEmitterConfig cfg) {
  own_emitter_ = std::make_unique<OwnShipEmitter>(
      adapter, cfg_.datum, *ownship_, std::move(cfg), derive_seed_("ownship"));
}

void SimulatedSensorBus::attachAis(AisAdapter& adapter,
                                   AisEmitterConfig cfg) {
  ais_emitter_ = std::make_unique<AisEmitter>(
      adapter, cfg_.datum, std::move(cfg), derive_seed_("ais"));
  ais_adapter_ = &adapter;
}

void SimulatedSensorBus::attachArpa(ArpaAdapter& adapter,
                                    ArpaEmitterConfig cfg) {
  arpa_emitter_ = std::make_unique<ArpaEmitter>(
      adapter, cfg_.datum, std::move(cfg), derive_seed_("arpa"));
  arpa_adapter_ = &adapter;
}

void SimulatedSensorBus::attachEoIr(EoIrAdapter& adapter,
                                    EoIrEmitterConfig cfg) {
  eo_emitter_ = std::make_unique<EoIrEmitter>(
      adapter, std::move(cfg), derive_seed_("eoir"));
  eo_adapter_ = &adapter;
}

bool SimulatedSensorBus::stepOnce(Scenario& out) {
  if (!ownship_) return false;

  if (!stream_initialised_) {
    // Integer step counting so accumulated floating-point error in dt does not
    // chop the final tick off the run.
    n_steps_ = static_cast<long long>(cfg_.duration_s / cfg_.dt_s + 0.5);
    next_truth_sample_ = cfg_.t0;
    step_idx_ = 0;
    stream_initialised_ = true;
  }

  if (step_idx_ > n_steps_) return false;

  const long long step = step_idx_;
  const Timestamp t = Timestamp::fromSeconds(cfg_.t0.seconds() + step * cfg_.dt_s);
  EmitContext ctx;
  ctx.now = t;
  ctx.ownship_truth = ownship_->eval(t);
  ctx.targets.reserve(targets_.size());
  for (const auto& [tid, traj] : targets_)
    ctx.targets.push_back(TargetTruth{tid, traj->eval(t)});

  // Own-ship first so the OwnShipProvider is current for ARPA/EO-IR.
  if (own_emitter_)  own_emitter_->emit(ctx);
  if (ais_emitter_)  ais_emitter_->emit(ctx);
  if (arpa_emitter_) arpa_emitter_->emit(ctx);
  if (eo_emitter_)   eo_emitter_->emit(ctx);

  if (ais_adapter_) {
    auto v = ais_adapter_->poll();
    out.measurements.insert(out.measurements.end(),
                            std::make_move_iterator(v.begin()),
                            std::make_move_iterator(v.end()));
  }
  if (arpa_adapter_) {
    auto v = arpa_adapter_->poll();
    out.measurements.insert(out.measurements.end(),
                            std::make_move_iterator(v.begin()),
                            std::make_move_iterator(v.end()));
  }
  if (eo_adapter_) {
    auto v = eo_adapter_->poll();
    out.measurements.insert(out.measurements.end(),
                            std::make_move_iterator(v.begin()),
                            std::make_move_iterator(v.end()));
  }

  if (t >= next_truth_sample_) {
    for (const auto& [tid, traj] : targets_) {
      const TruthState s = traj->eval(t);
      TruthSample ts;
      ts.time = t;
      ts.truth_id = tid;
      ts.position = s.position;
      ts.velocity = s.velocity;
      out.truth.push_back(ts);
    }
    next_truth_sample_ = Timestamp::fromSeconds(
        next_truth_sample_.seconds() + cfg_.truth_sample_dt_s);
  }

  ++step_idx_;
  // Return true: a step was performed this call. Subsequent calls past the
  // end will hit the `step_idx_ > n_steps_` guard above and return false.
  return true;
}

Scenario SimulatedSensorBus::run() {
  Scenario out;
  if (!ownship_) return out;

  while (stepOnce(out)) {
    // Body intentionally empty: stepOnce appends to `out`.
  }

  // Safety net: per-tick emitters are dispatched in a fixed order, but a
  // catching-up emitter (AIS dropout recovery) can produce a Measurement
  // backdated relative to a later same-tick one. Stable-sort guarantees
  // strict time ordering on the way out.
  std::stable_sort(out.measurements.begin(), out.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time < b.time;
                   });
  return out;
}

}  // namespace navtracker::sim
