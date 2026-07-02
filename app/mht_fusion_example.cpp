// app/mht_fusion_example.cpp — "best MHT" fusion for AIS + Radar.
//
// A ready-to-adapt composition root for the canonical `imm_cv_ct_mht`
// tracker (navtracker's reference config), wired for an application that
// cycles at a fixed rate (e.g. 10 Hz), collecting asynchronous,
// mixed-timestamp inputs each tick and processing them as one MHT scan.
//
// Like app/example.cpp this builds but the main() runs no external I/O.
//
// Key facts baked in here:
//   * TTM (range + relative bearing) and TLL (absolute lat/lon) BOTH become
//     Position2D measurements -> ONE tracker config; only the builder differs.
//   * MhtTracker is scan/batch-based: processBatch() once per tick, NOT per
//     measurement. The batch may arrive in ANY time order — the tracker orders
//     it internally (backlog #15), so this loop just hands over whatever it
//     collected this tick. (Cross-tick late data older than the high-water mark
//     is still dropped; wire a ReorderBuffer upstream to recover it.)
//   * Multi-sensor REQUIRES a per-sensor FixedSensorDetectionModel. Sharing one
//     (P_D, lambda_C) across AIS + radar is dimensionally wrong and collapses
//     tracking (see MhtTracker::defaultDetectionModelWarning()).

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity5State.hpp"
#include "core/estimation/CoordinatedTurn.hpp"
#include "core/estimation/ImmEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/output/TrackOutput.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/tracking/SensorDetectionModels.hpp"
#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/MeasurementBuilders.hpp"
#include "core/types/SensorDefaults.hpp"
#include "core/types/Timestamp.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

// ---- The canonical "best" IMM(CV+CT) estimator, UKF inner filter ------------
// Mirrors benchmark::makeImmCvCt(): a 2-mode IMM over a constant-velocity and a
// coordinated-turn model. This is the estimator the repo treats as reference.
std::shared_ptr<IEstimator> makeBestImm() {
  std::vector<std::shared_ptr<IMotionModel>> motions = {
      std::make_shared<ConstantVelocity5State>(/*accel_psd=*/0.5,
                                               /*omega_psd=*/0.01),
      std::make_shared<CoordinatedTurn>(/*accel_psd=*/0.5,
                                        /*omega_psd=*/0.1)};
  Eigen::MatrixXd tpm(2, 2);
  tpm << 0.95, 0.05,
         0.10, 0.90;
  Eigen::VectorXd mu0(2);
  mu0 << 0.5, 0.5;
  return std::make_shared<ImmEstimator>(
      std::move(motions), tpm, mu0,
      /*init_speed_std=*/10.0, /*init_omega_std=*/0.1,
      /*noise=*/nullptr, /*bearing_range_guard=*/false,
      /*use_ukf=*/true);
}

// ---- The canonical MhtTracker::Config (mirrors benchmark::makeMhtConfig) -----
MhtTracker::Config makeBestMhtConfig() {
  MhtTracker::Config cfg;                 // header defaults = IPDA + VIMM lifecycle
  cfg.gate_threshold = 20.0;              // chi-square gate
  cfg.probability_of_detection = 0.9;    // fallback P_D (overridden per-sensor below)
  cfg.clutter_density = 1e-4;             // fallback lambda_C (overridden per-sensor)
  // Everything else is the reference default: Murty K=3, N-scan=3,
  // Bhattacharyya merge 1.0, IPDA confirm 0.9 / demote 0.6 / delete 0.05,
  // VIMM visibility ON. Do not change without an A/B on the metrics harness.
  return cfg;
}

// ---- Per-sensor detection model (THE multi-sensor requirement) --------------
// Distinct (P_D, lambda_C) per (SensorKind, MeasurementModel). AIS is very
// reliable and almost never produces false contacts; radar has real clutter.
//
// NOTE: these are documented STARTING points. Tune them sim-first against the
// metrics harness before trusting them on real data.
std::shared_ptr<ISensorDetectionModel> makeBestDetectionModel() {
  auto model = std::make_shared<FixedSensorDetectionModel>(
      DetectionParams{/*P_D=*/0.9, /*lambda_C=*/1e-4});  // fallback

  // AIS: absolute position, high P_D, negligible clutter.
  model->set(SensorKind::Ais, MeasurementModel::Position2D,
             DetectionParams{/*P_D=*/0.95, /*lambda_C=*/1e-7});
  // Radar as TTM (range+bearing -> projected Position2D).
  model->set(SensorKind::ArpaTtm, MeasurementModel::Position2D,
             DetectionParams{/*P_D=*/0.90, /*lambda_C=*/1e-4});
  // Radar as TLL (absolute lat/lon -> Position2D). Same radar physics as TTM.
  model->set(SensorKind::ArpaTll, MeasurementModel::Position2D,
             DetectionParams{/*P_D=*/0.90, /*lambda_C=*/1e-4});
  return model;
}

// ============================================================================
// MhtFusion — build once at startup, drive from your fixed-rate loop.
// ============================================================================
class MhtFusion {
 public:
  MhtFusion()
      : estimator_(makeBestImm()),
        detection_(makeBestDetectionModel()),
        mht_(*estimator_, makeBestMhtConfig(), detection_),
        defaults_(pessimisticSensorDefaults()) {
    // Keep MHT hypotheses in-frame if own-ship travels far enough to trigger
    // the OwnShipProvider's >30 km auto-recenter. Safe to keep even if you use
    // a single fixed area (it just never fires).
    provider_.registerDatumSink(&datum_sink_);
  }

  // --- Feed own-ship pose (GPS fix). Call before any measurement so the datum
  //     initializes; then call whenever a new fix arrives. -------------------
  void pushOwnShip(const OwnShipPose& pose) { provider_.update(pose); }

  // --- CASE 1: Radar as TTM (range + RELATIVE bearing). Buffered for this tick.
  void pushRadarTtm(const std::string& source_id, Timestamp t,
                    double range_m, double rel_bearing_rad,
                    double range_std_m, double bearing_std_rad,
                    AssociationHints hints = {}) {
    Measurement m = makeMeasurementFromRelativeBearing(
        SensorKind::ArpaTtm, source_id, t, range_m, rel_bearing_rad,
        range_std_m, bearing_std_rad, provider_, hints);
    if (m.value.size() > 0) pending_.push_back(std::move(m));  // else: no pose yet
  }

  // --- CASE 2: Radar as TLL (absolute lat/lon). Buffered for this tick. ------
  void pushRadarTll(const std::string& source_id, Timestamp t,
                    double lat_deg, double lon_deg,
                    const Eigen::Matrix2d& cov_enu = Eigen::Matrix2d::Zero(),
                    AssociationHints hints = {}) {
    pushAbsolute(SensorKind::ArpaTll, source_id, t, lat_deg, lon_deg, cov_enu,
                 hints);
  }

  // --- AIS (absolute lat/lon). MMSI goes in the hints as an identity attribute.
  void pushAis(const std::string& source_id, Timestamp t,
               double lat_deg, double lon_deg, std::uint32_t mmsi,
               const Eigen::Matrix2d& cov_enu = Eigen::Matrix2d::Zero()) {
    pushAbsolute(SensorKind::Ais, source_id, t, lat_deg, lon_deg, cov_enu,
                 AssociationHints{mmsi, std::nullopt});
  }

  // --- Call ONCE per tick after pushing this cycle's inputs. Runs one MHT scan
  //     and clears the buffer. No pre-sort needed — processBatch orders the
  //     batch by time internally (backlog #15). ------------------------------
  void tick() {
    if (pending_.empty()) return;  // nothing arrived this cycle
    mht_.processBatch(pending_);
    pending_.clear();
  }

  // --- Drain confirmed tracks in operator-friendly (lat/lon, SOG/COG) form. --
  std::vector<TrackOutput> confirmedTracks() const {
    std::vector<TrackOutput> out;
    for (const Track& t : mht_.tracks()) {
      if (t.status != TrackStatus::Confirmed) continue;
      out.push_back(toTrackOutput(t, provider_.datum()));
    }
    return out;
  }

  const OwnShipProvider& provider() const { return provider_; }
  MhtTracker& tracker() { return mht_; }  // for setInnovationSink / diagnostics

 private:
  void pushAbsolute(SensorKind kind, const std::string& source_id, Timestamp t,
                    double lat_deg, double lon_deg, const Eigen::Matrix2d& cov,
                    AssociationHints hints) {
    const Eigen::Vector3d enu = provider_.datum().toEnu({lat_deg, lon_deg, 0.0});
    Measurement m = makeMeasurementFromEnuPosition(
        kind, source_id, t, Eigen::Vector2d(enu.x(), enu.y()), cov, hints);
    applyDefaultsIfEmpty(m, defaults_);  // fills covariance if you passed Zero
    pending_.push_back(std::move(m));
  }

  // Forwards the provider's >30 km auto-recenter to the MHT hypothesis state.
  struct DatumSink : IDatumChangeSink {
    MhtTracker* mht;
    explicit DatumSink(MhtTracker* m) : mht(m) {}
    void onDatumRecentered(const geo::Datum& o, const geo::Datum& n) override {
      mht->onDatumRecentered(o, n);
    }
  };

  OwnShipProvider provider_;
  std::shared_ptr<IEstimator> estimator_;          // must outlive mht_
  std::shared_ptr<ISensorDetectionModel> detection_;
  MhtTracker mht_;
  SensorDefaults defaults_;
  DatumSink datum_sink_{&mht_};
  std::vector<Measurement> pending_;               // this tick's scan buffer
};

}  // namespace navtracker

int main() {
  using namespace navtracker;

  MhtFusion fusion;  // build once at startup

  // Prime the datum with the first GPS fix BEFORE any measurement.
  {
    OwnShipPose p0;
    p0.time = Timestamp::fromSeconds(0.0);
    p0.lat_deg = 53.500;
    p0.lon_deg = 8.000;
    p0.heading_true_deg = 45.0;
    p0.position_std_m = 5.0;
    fusion.pushOwnShip(p0);
  }

  // One iteration of your fixed-rate (e.g. 10 Hz) loop. In a real system this
  // is `while (running) { ... }`; here we run a single representative cycle.
  const double t = 0.1;  // this tick's stamp (seconds)

  // Whatever arrived this cycle, in any mix and any order:

  // CASE 1 — radar reported as TTM (range + relative bearing):
  fusion.pushRadarTtm("radar1", Timestamp::fromSeconds(t),
                      /*range_m=*/1500.0, /*rel_bearing_rad=*/0.5,
                      /*range_std_m=*/50.0,
                      /*bearing_std_rad=*/1.0 * 3.14159265358979 / 180.0);

  // CASE 2 — radar reported as TLL (absolute lat/lon):
  fusion.pushRadarTll("radar1", Timestamp::fromSeconds(t),
                      /*lat_deg=*/53.55, /*lon_deg=*/8.05);

  // AIS (absolute lat/lon + MMSI identity hint):
  fusion.pushAis("ais1", Timestamp::fromSeconds(t),
                 /*lat_deg=*/53.55, /*lon_deg=*/8.05, /*mmsi=*/200000001u);

  fusion.tick();  // exactly one MHT scan per cycle

  for (const TrackOutput& out : fusion.confirmedTracks()) {
    std::cout << "Track id=" << out.id.value
              << "  lat=" << out.position.lat_deg
              << "  lon=" << out.position.lon_deg;
    if (out.velocity.is_valid) {
      std::cout << "  sog=" << out.velocity.sog_m_per_s
                << "  cog=" << out.velocity.cog_deg;
    }
    std::cout << "\n";
  }

  return 0;
}
