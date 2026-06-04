#pragma once

#include <random>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "core/geo/Datum.hpp"
#include "sim/SensorEmitter.hpp"
#include "sim/TruthTrajectory.hpp"

namespace navtracker::sim {

struct OwnShipEmitterConfig {
  double dt_s{1.0};
  double gps_pos_std_m{5.0};
  // When true, the emitter advertises gps_pos_std_m on the published
  // OwnShipPose so ARPA/EO/IR adapters R-inflate accordingly. Default
  // false preserves pre-2026-06-03 behaviour where the noise was
  // injected on lat/lon but the adapter did not budget for it. New
  // tests that want closed-loop GPS-uncertainty modelling opt in.
  bool report_gps_std{false};
  double heading_true_deg{0.0};
  // §14.9 hooks (default zero — deferred per spec).
  double heading_bias_deg{0.0};
  double heading_drift_deg_per_s{0.0};
  double heading_noise_std_deg{0.0};

  // Velocity / RMC controls (Task 4 of 2026-06-04 RMC plan).
  //
  // emit_rmc: when true, the emitter composes and pushes a $GPRMC sentence
  // alongside GGA/HDT each cycle, carrying truth SOG/COG (in knots / deg
  // true) optionally perturbed by sigma_sog_emit / sigma_cog_emit. Default
  // OFF for backward compat — turning RMC on by default would populate
  // pose.velocity_is_valid on every existing sim test and propagate
  // velocity covariance into downstream estimators, risking margin-flip
  // regressions (cf. the report_gps_std fallout at ce61b65). Tests that
  // want closed-loop velocity modelling opt in.
  bool emit_rmc{false};
  // report_velocity is a no-op as of 2026-06-04 — kept as a placeholder so
  // call sites can express intent without a separate sticky-setter path.
  // The current implementation routes velocity reporting through emit_rmc.
  bool report_velocity{false};
  // Sim's truth-noise floors when synthesising RMC SOG/COG. Independent
  // of the adapter's sigma_sog_m_per_s / sigma_cog_deg (those are the
  // adapter's *assumed* RMC noise, used for sigma_v derivation on parse).
  double sigma_sog_emit_m_per_s{0.1};
  double sigma_cog_emit_deg{0.5};
};

class OwnShipEmitter final : public ISensorEmitter {
 public:
  OwnShipEmitter(OwnShipNmeaAdapter& adapter,
                 const geo::Datum& datum,
                 const ITruthTrajectory& ownship_trajectory,
                 OwnShipEmitterConfig cfg,
                 std::uint32_t seed);

  void emit(const EmitContext& ctx) override;

 private:
  OwnShipNmeaAdapter& adapter_;
  const geo::Datum& datum_;
  const ITruthTrajectory& trajectory_;
  OwnShipEmitterConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  bool initialised_{false};
  Timestamp next_emit_{};
  Timestamp t0_{};
};

}  // namespace navtracker::sim
