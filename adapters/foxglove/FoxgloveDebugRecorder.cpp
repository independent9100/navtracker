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
  for (const char* t : {"/tracks","/detections","/associations","/gates","/cpa"})
    w_->ensureChannel(t, kSceneUpdateSchema, scene);
  for (const char* t : {"/map/tracks","/map/detections"})
    w_->ensureChannel(t, kLocationFixSchema, loc);
  w_->ensureChannel("/tf", kFrameTransformSchema, tf);
  w_->ensureChannel("/log", kLogSchema, log);
  for (const char* t : {"/diag/innovation","/diag/track_count","/diag/gate_ratio","/diag/bias"})
    w_->ensureChannel(t, kDiagSchema, "");
}

void FoxgloveDebugRecorder::onTracks(const std::vector<Track>& tracks, Timestamp now) {
  last_time_ = now;
  std::vector<json> entities;
  int confirmed = 0, tentative = 0;
  for (const auto& t : tracks) {
    if (t.state.size() < 2 || t.covariance.rows() < 2 || t.covariance.cols() < 2) continue;
    const Eigen::Vector2d p = xy(t.state);
    const Rgba col = (t.status == TrackStatus::Confirmed) ? Rgba{0.1,0.9,0.1,1.0}
                                                          : Rgba{0.9,0.9,0.1,1.0};
    const std::string base = "track-" + std::to_string(t.id.value);
    entities.push_back(lineEntity(base + "-cov",
        covarianceEllipse(p, pos2(t.covariance), cfg_.ellipse_k), col));
    entities.push_back(textEntity(base + "-label", {p.x(), p.y(), 0},
        std::to_string(t.id.value), col));
    if (t.velocity_observed && t.state.size() >= 4) {
      const Eigen::Vector2d v = t.state.segment<2>(2);
      entities.push_back(arrowEntity(base + "-vel", {p.x(),p.y(),0},
          {p.x()+v.x(), p.y()+v.y(), 0}, col));
    }
    (t.status == TrackStatus::Confirmed ? confirmed : tentative)++;
    // Map: lat/lon via the canonical helper.
    const auto geo = toGeodeticWithCov(p, pos2(t.covariance), datum_);
    std::array<double,9> cov{};
    cov[0] = geo.position_covariance_m2(0,0); cov[1] = geo.position_covariance_m2(0,1);
    cov[3] = geo.position_covariance_m2(1,0); cov[4] = geo.position_covariance_m2(1,1);
    w_->write("/map/tracks", now, locationFix(now, geo.lat_deg, geo.lon_deg, cov).dump());
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
    w_->write("/gates", now, sceneUpdate(now, gate_entities).dump());
  }
  w_->write("/tracks", now, sceneUpdate(now, entities).dump());
  json diag{{"time_ns", now.nanos()}, {"confirmed", confirmed}, {"tentative", tentative},
            {"total", confirmed + tentative}};
  w_->write("/diag/track_count", now, diag.dump());
}

void FoxgloveDebugRecorder::recordMeasurement(const Measurement& m) {
  last_time_ = m.time;
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
        entities.push_back(lineEntity(base + "-ray-corr",
            bearingWedge(m.sensor_position_enu, alpha + bb.bias_rad, sigma, 2000.0, cfg_.ellipse_k), c2));
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
    if (bias_) {
      const auto pb = bias_->positionBias({m.sensor, m.source_id});
      if (pb.is_published) {
        const Eigen::Vector2d pc = p + pb.bias_enu_m;
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
    std::array<double,9> cov{};
    cov[0]=geo.position_covariance_m2(0,0); cov[4]=geo.position_covariance_m2(1,1);
    w_->write("/map/detections", m.time, locationFix(m.time, geo.lat_deg, geo.lon_deg, cov).dump());
  }
  w_->write("/detections", m.time, sceneUpdate(m.time, entities).dump());
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
  w_->write("/cpa", e.time, sceneUpdate(e.time, ents).dump());
}

void FoxgloveDebugRecorder::recordOwnShip(const OwnShipPose& pose) {
  last_time_ = pose.time;
  // ENU position of own-ship relative to the datum.
  // heading_true_deg is clockwise-from-north; ENU yaw (CCW-from-east) = 90 - hdg.
  const Eigen::Vector3d p3 = datum_.toEnu(geo::Geodetic{pose.lat_deg, pose.lon_deg, 0.0});
  const double yaw = (90.0 - pose.heading_true_deg) * M_PI / 180.0;
  w_->write("/tf", pose.time, frameTransform(pose.time, kRootFrame, "own_ship",
      p3.x(), p3.y(), 0.0, yaw).dump());
}

void FoxgloveDebugRecorder::onDatumRecentered(const geo::Datum& /*old_d*/, const geo::Datum& new_d) {
  datum_ = new_d;
  // Mark the discontinuity in the log at the last known event time.
  w_->write("/log", last_time_, logMsg(last_time_, 4, "datum",
      "datum recentered").dump());
}

}  // namespace navtracker::foxglove
