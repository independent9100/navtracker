#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "adapters/ais/AisAdapter.hpp"
#include "adapters/arpa/ArpaAdapter.hpp"
#include "adapters/eoir/EoIrAdapter.hpp"
#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "core/scenario/Truth.hpp"
#include "core/types/Timestamp.hpp"
#include "sim/AisEmitter.hpp"
#include "sim/ArpaEmitter.hpp"
#include "sim/EoIrEmitter.hpp"
#include "sim/OwnShipEmitter.hpp"
#include "sim/SensorEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct SimulatedSensorBusConfig {
  Timestamp t0;
  double duration_s{60.0};
  double dt_s{0.1};
  double truth_sample_dt_s{1.0};
  std::uint32_t seed{2026};
  geo::Datum datum{geo::Geodetic{0.0, 0.0, 0.0}};
};

class SimulatedSensorBus {
 public:
  explicit SimulatedSensorBus(SimulatedSensorBusConfig cfg);

  void setOwnShip(std::shared_ptr<ITruthTrajectory> trajectory);
  void addTarget(std::uint64_t truth_id, std::shared_ptr<ITruthTrajectory> trajectory);

  // Adapter must outlive this SimulatedSensorBus. Bus borrows by reference.
  void attachOwnShip(OwnShipNmeaAdapter& adapter, OwnShipEmitterConfig cfg);
  void attachAis    (AisAdapter& adapter,         AisEmitterConfig cfg);
  void attachArpa   (ArpaAdapter& adapter,        ArpaEmitterConfig cfg);
  void attachEoIr   (EoIrAdapter& adapter,        EoIrEmitterConfig cfg);

  Scenario run();

 private:
  std::uint32_t derive_seed_(const char* emitter_id) const;

  SimulatedSensorBusConfig cfg_;
  std::shared_ptr<ITruthTrajectory> ownship_;
  std::vector<std::pair<std::uint64_t, std::shared_ptr<ITruthTrajectory>>> targets_;

  std::unique_ptr<OwnShipEmitter> own_emitter_;
  std::unique_ptr<AisEmitter>     ais_emitter_;
  std::unique_ptr<ArpaEmitter>    arpa_emitter_;
  std::unique_ptr<EoIrEmitter>    eo_emitter_;

  AisAdapter*  ais_adapter_{nullptr};
  ArpaAdapter* arpa_adapter_{nullptr};
  EoIrAdapter* eo_adapter_{nullptr};
};

}  // namespace navtracker::sim
