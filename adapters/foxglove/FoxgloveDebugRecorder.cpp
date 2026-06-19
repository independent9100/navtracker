#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <Eigen/Cholesky>
#include "adapters/foxglove/FoxgloveJson.hpp"
#include "adapters/foxglove/Geometry.hpp"
#include "adapters/foxglove/Schemas.hpp"
#include "core/output/TrackOutput.hpp"

namespace navtracker::foxglove {
using nlohmann::json;

namespace {
std::string loadSchema(const char* file) {
  std::ifstream in(std::string(NAVTRACKER_FOXGLOVE_SCHEMA_DIR) + "/" + file);
  std::stringstream ss; ss << in.rdbuf(); return ss.str();   // empty if missing -> name-only
}
Eigen::Matrix2d pos2(const Eigen::MatrixXd& P) { return P.topLeftCorner<2,2>(); }
Eigen::Vector2d xy(const Eigen::VectorXd& s) { return s.head<2>(); }
}  // namespace

FoxgloveDebugRecorder::FoxgloveDebugRecorder(const std::string& path, const geo::Datum& datum,
                                             const ISensorBiasProvider* bias, RecorderConfig cfg)
    : w_(std::make_unique<McapWriter>(path)), datum_(datum), bias_(bias), cfg_(cfg) {
  registerChannels();
}
FoxgloveDebugRecorder::~FoxgloveDebugRecorder() { close(); }
void FoxgloveDebugRecorder::close() { if (w_) w_->close(); }

void FoxgloveDebugRecorder::registerChannels() {
  const std::string scene = loadSchema("SceneUpdate.json");
  const std::string loc   = loadSchema("LocationFix.json");
  const std::string tf    = loadSchema("FrameTransform.json");
  const std::string log   = loadSchema("Log.json");
  // Per-sensor detection topics (/detections/<source_id>, /map/detections/<source_id>)
  // are registered lazily in recordMeasurement so each sensor is its own layer.
  for (const char* t : {"/tracks/confirmed","/tracks/tentative",
                        "/associations","/gates","/cpa","/ownship"})
    w_->ensureChannel(t, kSceneUpdateSchema, scene);
  for (const char* t : {"/map/tracks/confirmed","/map/tracks/tentative","/map/ownship"})
    w_->ensureChannel(t, kLocationFixSchema, loc);
  w_->ensureChannel("/tf", kFrameTransformSchema, tf);
  w_->ensureChannel("/log", kLogSchema, log);
  for (const char* t : {"/diag/innovation","/diag/track_count","/diag/bias"})
    w_->ensureChannel(t, kDiagSchema, "");
}

void FoxgloveDebugRecorder::ensureRootFrame(Timestamp t) {
  if (root_frame_done_) return;
  root_frame_done_ = true;
  // Identity map->enu so the 3D panel has a frame for the enu-framed entities.
  w_->write("/tf", t, frameTransform(t, "map", kRootFrame, 0.0, 0.0, 0.0, 0.0).dump());
}

void FoxgloveDebugRecorder::onTracks(const std::vector<Track>& tracks, Timestamp now) {
  last_time_ = now;
  ensureRootFrame(now);
  // Split by lifecycle: /tracks/confirmed is the committed OUTPUT (what a
  // consumer would publish, per app/example.cpp's Confirmed-only drain);
  // /tracks/tentative is candidates not yet confirmed. Same split on the Map.
  std::vector<json> conf_entities, tent_entities;
  int confirmed = 0, tentative = 0;
  for (const auto& t : tracks) {
    if (t.state.size() < 2 || t.covariance.rows() < 2 || t.covariance.cols() < 2) continue;
    const bool is_output = (t.status == TrackStatus::Confirmed);
    const Eigen::Vector2d p = xy(t.state);
    const Rgba col = is_output ? Rgba{0.1,0.9,0.1,1.0} : Rgba{0.9,0.9,0.1,1.0};
    const std::string base = "track-" + std::to_string(t.id.value);
    std::vector<json>& bucket = is_output ? conf_entities : tent_entities;
    bucket.push_back(lineEntity(base + "-cov",
        covarianceEllipse(p, pos2(t.covariance), cfg_.ellipse_k), col));
    bucket.push_back(textEntity(base + "-label", {p.x(), p.y(), 0},
        std::to_string(t.id.value), col));
    if (t.velocity_observed && t.state.size() >= 4) {
      const Eigen::Vector2d v = t.state.segment<2>(2);
      bucket.push_back(arrowEntity(base + "-vel", {p.x(),p.y(),0},
          {p.x()+v.x(), p.y()+v.y(), 0}, col));
    }
    (is_output ? confirmed : tentative)++;
    // Map: lat/lon via the canonical helper.
    const auto geo = toGeodeticWithCov(p, pos2(t.covariance), datum_);
    // NED (0=north,1=east) -> row-major ENU LocationFix (0=EE,1=EN,3=NE,4=NN).
    std::array<double,9> cov{};
    cov[0] = geo.position_covariance_m2(1,1); cov[1] = geo.position_covariance_m2(1,0);
    cov[3] = geo.position_covariance_m2(0,1); cov[4] = geo.position_covariance_m2(0,0);
    w_->write(is_output ? "/map/tracks/confirmed" : "/map/tracks/tentative", now,
              locationFix(now, geo.lat_deg, geo.lon_deg, cov).dump());
  }
  if (cfg_.gate_gamma > 0.0) {
    std::vector<json> gate_entities;
    for (const auto& t : tracks) {
      if (t.state.size() < 2) continue;
      auto sit = last_S_.find(t.id.value);
      if (sit == last_S_.end() || sit->second.rows() < 2) continue;
      const Eigen::Vector2d p = xy(t.state);
      gate_entities.push_back(lineEntity("gate-" + std::to_string(t.id.value),
          covarianceEllipse(p, sit->second.topLeftCorner<2,2>(), std::sqrt(cfg_.gate_gamma)),
          Rgba{0.4,0.4,1.0,0.6}));
    }
    w_->write("/gates", now, sceneUpdate(now, gate_entities, cfg_.entity_lifetime_sec).dump());
  }
  std::vector<json> assoc;
  for (const auto& t : tracks) {
    if (t.state.size() < 2) continue;
    const Eigen::Vector2d tp = xy(t.state);
    int k = 0;
    for (const auto& st : t.recent_contributions) {
      Eigen::Vector2d from = st.value_enu;
      if (std::isnan(st.alpha_rad) == false) {        // bearing-only touch: anchor at sensor
        from = st.sensor_position_enu;
      }
      assoc.push_back(lineEntity(
          "assoc-" + std::to_string(t.id.value) + "-" + std::to_string(k++),
          {{from.x(),from.y(),0},{tp.x(),tp.y(),0}},
          colorForSensor(st.sensor, st.source_id)));
    }
  }
  w_->write("/associations", now, sceneUpdate(now, assoc, cfg_.entity_lifetime_sec).dump());
  w_->write("/tracks/confirmed", now, sceneUpdate(now, conf_entities, cfg_.entity_lifetime_sec).dump());
  w_->write("/tracks/tentative", now, sceneUpdate(now, tent_entities, cfg_.entity_lifetime_sec).dump());
  json diag{{"time_ns", now.nanos()}, {"confirmed", confirmed}, {"tentative", tentative},
            {"total", confirmed + tentative}};
  w_->write("/diag/track_count", now, diag.dump());
}

void FoxgloveDebugRecorder::recordMeasurement(const Measurement& m) {
  last_time_ = m.time;
  ensureRootFrame(m.time);
  const Rgba col = colorForSensor(m.sensor, m.source_id);
  std::vector<json> entities;
  const std::string base = "det-" + m.source_id + "-" + std::to_string(m.time.nanos());

  if (m.model == MeasurementModel::Bearing2D) {
    const double alpha = m.value(0);
    const double sigma = std::sqrt(m.covariance(0,0));
    entities.push_back(lineEntity(base + "-ray",
        bearingWedge(m.sensor_position_enu, alpha, sigma, /*length=*/2000.0, cfg_.ellipse_k), col));
    if (bias_) {
      const auto bb = bias_->bearingBias({m.sensor, m.source_id});
      if (bb.is_published) {
        Rgba c2 = col; c2.a = 0.4;
        // Corrected = raw - bias, matching applyBiasCorrection (BiasCorrection.hpp).
        entities.push_back(lineEntity(base + "-ray-corr",
            bearingWedge(m.sensor_position_enu, alpha - bb.bias_rad, sigma, 2000.0, cfg_.ellipse_k), c2));
        w_->write("/diag/bias", m.time,
                  json{{"time_ns", m.time.nanos()}, {"sensor", static_cast<int>(m.sensor)},
                       {"source_id", m.source_id}, {"bearing_bias_rad", bb.bias_rad},
                       {"is_published", true}}.dump());
      }
    }
  } else {  // Position2D / PositionVelocity2D / RangeBearing2D: ENU point + ellipse
    const Eigen::Vector2d p = m.value.head<2>();
    entities.push_back(lineEntity(base + "-cov",
        covarianceEllipse(p, m.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k), col));
    // Only Position2D carries a position bias in the pipeline; PositionVelocity2D
    // gets no correction and RangeBearing2D is corrected on its bearing component
    // (see BiasCorrection.hpp), so we draw the position-bias overlay for Position2D only.
    if (bias_ && m.model == MeasurementModel::Position2D) {
      const auto pb = bias_->positionBias({m.sensor, m.source_id});
      if (pb.is_published) {
        const Eigen::Vector2d pc = p - pb.bias_enu_m;  // corrected = raw - bias
        Rgba c2 = col; c2.a = 0.4;
        entities.push_back(lineEntity(base + "-cov-corr",
            covarianceEllipse(pc, m.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k), c2));
        entities.push_back(arrowEntity(base + "-bias", {p.x(),p.y(),0}, {pc.x(),pc.y(),0}, c2));
        w_->write("/diag/bias", m.time,
                  json{{"time_ns", m.time.nanos()}, {"sensor", static_cast<int>(m.sensor)},
                       {"source_id", m.source_id}, {"bias_e_m", pb.bias_enu_m.x()},
                       {"bias_n_m", pb.bias_enu_m.y()}, {"is_published", true}}.dump());
      }
    }
    const auto geo = toGeodeticWithCov(p, m.covariance.topLeftCorner<2,2>(), datum_);
    // position_covariance_m2 is local NED (0=north,1=east); LocationFix expects
    // row-major ENU (0=EE, 4=NN), so map east<-NED(1,1), north<-NED(0,0).
    std::array<double,9> cov{};
    cov[0]=geo.position_covariance_m2(1,1); cov[4]=geo.position_covariance_m2(0,0);
    const std::string map_topic = "/map/detections/" + m.source_id;
    w_->ensureChannel(map_topic, kLocationFixSchema, "");
    w_->write(map_topic, m.time, locationFix(m.time, geo.lat_deg, geo.lon_deg, cov).dump());
  }
  // Per-sensor detection layer so radar/lidar/EO/IR are independently toggleable.
  const std::string det_topic = "/detections/" + m.source_id;
  w_->ensureChannel(det_topic, kSceneUpdateSchema, "");
  w_->write(det_topic, m.time, sceneUpdate(m.time, entities, cfg_.entity_lifetime_sec).dump());
}

void FoxgloveDebugRecorder::onTrackInitiated(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 1, "lifecycle",
      "track " + std::to_string(e.id.value) + " initiated").dump());
}
void FoxgloveDebugRecorder::onTrackConfirmed(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 2, "lifecycle",
      "track " + std::to_string(e.id.value) + " confirmed").dump());
}
void FoxgloveDebugRecorder::onTrackUpdated(const TrackLifecycleEvent&) { /* high-volume: skip /log */ }
void FoxgloveDebugRecorder::onTrackDeleted(const TrackLifecycleEvent& e) {
  w_->write("/log", e.time, logMsg(e.time, 3, "lifecycle",
      "track " + std::to_string(e.id.value) + " deleted").dump());
}

void FoxgloveDebugRecorder::onInnovation(const InnovationEvent& e) {
  const double nis = (e.residual.transpose() * e.S.ldlt().solve(e.residual)).value();
  last_S_[e.track_id.value] = e.S;
  w_->write("/diag/innovation", e.time,
            json{{"time_ns", e.time.nanos()}, {"track_id", e.track_id.value},
                 {"sensor", static_cast<int>(e.sensor)}, {"source_id", e.source_id},
                 {"nis", nis}, {"dim", e.dim}}.dump());
}

void FoxgloveDebugRecorder::onCollisionRisk(const CollisionRiskEvent& e) {
  const char* kind = e.transition == CollisionRiskTransition::Entered ? "ENTERED"
                   : e.transition == CollisionRiskTransition::Exited  ? "EXITED" : "UPDATED";
  w_->write("/log", e.time, logMsg(e.time, 2, "cpa",
      std::string("CPA ") + kind + " track " + std::to_string(e.other.value) +
      " d=" + std::to_string(e.prediction.cpa_distance_m) +
      "m t=" + std::to_string(e.prediction.tcpa_seconds) + "s").dump());
  // Minimal CPA marker: a text at origin carrying the numbers.
  std::vector<json> ents{ textEntity("cpa-" + std::to_string(e.other.value),
      {0,0,0}, std::string(kind) + " d=" + std::to_string(e.prediction.cpa_distance_m),
      {1.0,0.3,0.3,1.0}) };
  w_->write("/cpa", e.time, sceneUpdate(e.time, ents, cfg_.entity_lifetime_sec).dump());
}

void FoxgloveDebugRecorder::recordOwnShip(const OwnShipPose& pose) {
  last_time_ = pose.time;
  ensureRootFrame(pose.time);
  // ENU position of own-ship relative to the datum.
  // heading_true_deg is clockwise-from-north; ENU yaw (CCW-from-east) = 90 - hdg.
  const Eigen::Vector3d p3 = datum_.toEnu(geo::Geodetic{pose.lat_deg, pose.lon_deg, 0.0});
  const double yaw = (90.0 - pose.heading_true_deg) * M_PI / 180.0;
  w_->write("/tf", pose.time, frameTransform(pose.time, kRootFrame, "own_ship",
      p3.x(), p3.y(), 0.0, yaw).dump());

  // Visible own-ship marker: a diamond + label at the own-ship ENU position,
  // bright white so it stands out from tracks/detections. Stable id -> the
  // marker moves in place. Also a /map/ownship point for the Map panel.
  const double d = 20.0;  // marker half-size, metres
  const Rgba white{1.0, 1.0, 1.0, 1.0};
  std::vector<json> own{
      lineEntity("ownship",
                 {{p3.x()+d, p3.y(), 0}, {p3.x(), p3.y()+d, 0}, {p3.x()-d, p3.y(), 0},
                  {p3.x(), p3.y()-d, 0}, {p3.x()+d, p3.y(), 0}}, white, 2.0),
      textEntity("ownship-label", {p3.x(), p3.y(), 0}, "own-ship", white)};
  w_->write("/ownship", pose.time, sceneUpdate(pose.time, own, cfg_.entity_lifetime_sec).dump());
  std::array<double,9> own_cov{};  // own-ship GPS std (m) -> EE/NN
  own_cov[0] = own_cov[4] = pose.position_std_m * pose.position_std_m;
  w_->write("/map/ownship", pose.time,
            locationFix(pose.time, pose.lat_deg, pose.lon_deg, own_cov).dump());
}

void FoxgloveDebugRecorder::onDatumRecentered(const geo::Datum& /*old_d*/, const geo::Datum& new_d) {
  datum_ = new_d;
  // Mark the discontinuity in the log at the last known event time.
  w_->write("/log", last_time_, logMsg(last_time_, 4, "datum",
      "datum recentered").dump());
}

}  // namespace navtracker::foxglove
