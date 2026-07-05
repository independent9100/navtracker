#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <Eigen/Cholesky>
#include "adapters/foxglove/FoxgloveJson.hpp"
#include "adapters/foxglove/Geometry.hpp"
#include "adapters/foxglove/Schemas.hpp"
#include "core/output/TrackOutput.hpp"
#include "core/types/StaticObstacle.hpp"
#include "core/land/CoastlineGeometry.hpp"       // LandPolygon
#include "core/static/LiveOccupancyModel.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "ports/IStaticHazardSink.hpp"           // StaticHazardEvent

namespace navtracker::foxglove {
using nlohmann::json;

namespace {
std::string loadSchema(const char* file) {
  std::ifstream in(std::string(NAVTRACKER_FOXGLOVE_SCHEMA_DIR) + "/" + file);
  std::stringstream ss; ss << in.rdbuf(); return ss.str();   // empty if missing -> name-only
}
Eigen::Matrix2d pos2(const Eigen::MatrixXd& P) { return P.topLeftCorner<2,2>(); }
Eigen::Vector2d xy(const Eigen::VectorXd& s) { return s.head<2>(); }

Rgba colorForObstacleCategory(ObstacleCategory cat) {
  switch (cat) {
    case ObstacleCategory::Rock:        return {0.55, 0.35, 0.20, 1.0};  // brown
    case ObstacleCategory::Wreck:       return {0.80, 0.20, 0.20, 1.0};  // red
    case ObstacleCategory::Obstruction: return {0.85, 0.45, 0.10, 1.0};  // orange
    case ObstacleCategory::Pile:
    case ObstacleCategory::Platform:    return {0.70, 0.70, 0.20, 1.0};  // olive
    case ObstacleCategory::Buoy:
    case ObstacleCategory::Beacon:      return {0.20, 0.70, 0.90, 1.0};  // cyan
    default:                            return {0.75, 0.75, 0.75, 1.0};  // grey
  }
}
const char* obstacleCategoryName(ObstacleCategory cat) {
  switch (cat) {
    case ObstacleCategory::Rock:        return "rock";
    case ObstacleCategory::Wreck:       return "wreck";
    case ObstacleCategory::Obstruction: return "obstruction";
    case ObstacleCategory::Pile:        return "pile";
    case ObstacleCategory::Platform:    return "platform";
    case ObstacleCategory::Buoy:        return "buoy";
    case ObstacleCategory::Beacon:      return "beacon";
    case ObstacleCategory::Other:       return "other";
    default:                            return "obstacle";
  }
}
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
                        "/tracks/imm_modes","/tracks/particles",
                        "/associations","/gates","/cpa","/ownship"})
    w_->ensureChannel(t, kSceneUpdateSchema, scene);
  for (const char* t : {"/map/tracks/confirmed","/map/tracks/tentative","/map/ownship"})
    w_->ensureChannel(t, kLocationFixSchema, loc);
  w_->ensureChannel("/tf", kFrameTransformSchema, tf);
  w_->ensureChannel("/log", kLogSchema, log);
  for (const char* t : {"/diag/innovation","/diag/track_count","/diag/bias",
                        "/diag/mode_prob","/diag/existence"})
    w_->ensureChannel(t, kDiagSchema, "");
}

void FoxgloveDebugRecorder::ensureRootFrame(Timestamp t) {
  if (root_frame_done_) return;
  root_frame_done_ = true;
  // Identity map->enu so the 3D panel has a frame for the enu-framed entities.
  w_->write("/tf", t, frameTransform(t, "map", kRootFrame, 0.0, 0.0, 0.0, 0.0).dump());
}

void FoxgloveDebugRecorder::onTracks(const std::vector<Track>& tracks, Timestamp now) {
  if (!cfg_.enabled) return;
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

  // Estimator internals: per-mode IMM ellipses, particle clouds, and the
  // existence / mode-probability scalars (drawn from fields already on Track).
  std::vector<json> imm_ents;
  std::vector<std::pair<Pt, Rgba>> particle_cells;
  for (const auto& t : tracks) {
    if (t.state.size() < 2) continue;
    const std::string base = "track-" + std::to_string(t.id.value);
    // IMM per-mode ellipses (opacity ∝ mode probability).
    const int K = static_cast<int>(t.imm_mode_probabilities.size());
    if (K > 0 && t.imm_means.cols() >= K && static_cast<int>(t.imm_covariances.size()) >= K) {
      for (int k = 0; k < K; ++k) {
        if (t.imm_means.rows() < 2 || t.imm_covariances[k].rows() < 2) continue;
        const double mp = t.imm_mode_probabilities(k);
        imm_ents.push_back(lineEntity(base + "-mode-" + std::to_string(k),
            covarianceEllipse(t.imm_means.col(k).head<2>(),
                              t.imm_covariances[k].topLeftCorner<2,2>(), cfg_.ellipse_k),
            Rgba{0.95, 0.5, 0.1, 0.2 + 0.7 * std::min(1.0, std::max(0.0, mp))}, 1.0));
      }
      json mp{{"time_ns", now.nanos()}, {"track_id", t.id.value}};
      for (int k = 0; k < K; ++k) mp["m" + std::to_string(k)] = t.imm_mode_probabilities(k);
      w_->write("/diag/mode_prob", now, mp.dump());
    }
    // Particle cloud (opacity ∝ normalized weight).
    if (t.particles.cols() > 0 && t.particles.rows() >= 2) {
      double wmax = 1e-12;
      for (int j = 0; j < t.particle_weights.size(); ++j)
        wmax = std::max(wmax, t.particle_weights(j));
      for (int j = 0; j < t.particles.cols(); ++j) {
        const double w = j < t.particle_weights.size() ? t.particle_weights(j) / wmax : 1.0;
        particle_cells.push_back({{t.particles(0, j), t.particles(1, j), 0.0},
                                  {0.1, 0.9, 0.9, 0.2 + 0.6 * std::min(1.0, w)}});
      }
    }
    // Existence / visibility scalars (IPDA / VIMM).
    w_->write("/diag/existence", now,
              json{{"time_ns", now.nanos()}, {"track_id", t.id.value},
                   {"existence_probability", t.existence_probability},
                   {"visibility_given_exists", t.visibility_given_exists}}.dump());
  }
  w_->write("/tracks/imm_modes", now, sceneUpdate(now, imm_ents, cfg_.entity_lifetime_sec).dump());
  w_->write("/tracks/particles", now,
            sceneUpdate(now, {gridCellsEntity("particles", particle_cells, 3.0)},
                        cfg_.entity_lifetime_sec).dump());

  json diag{{"time_ns", now.nanos()}, {"confirmed", confirmed}, {"tentative", tentative},
            {"total", confirmed + tentative}};
  w_->write("/diag/track_count", now, diag.dump());
}

void FoxgloveDebugRecorder::recordMeasurement(const Measurement& m) {
  if (!cfg_.enabled) return;
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
  if (!cfg_.enabled) return;
  w_->write("/log", e.time, logMsg(e.time, 1, "lifecycle",
      "track " + std::to_string(e.id.value) + " initiated").dump());
}
void FoxgloveDebugRecorder::onTrackConfirmed(const TrackLifecycleEvent& e) {
  if (!cfg_.enabled) return;
  w_->write("/log", e.time, logMsg(e.time, 2, "lifecycle",
      "track " + std::to_string(e.id.value) + " confirmed").dump());
}
void FoxgloveDebugRecorder::onTrackUpdated(const TrackLifecycleEvent&) { /* high-volume: skip /log */ }
void FoxgloveDebugRecorder::onTrackDeleted(const TrackLifecycleEvent& e) {
  if (!cfg_.enabled) return;
  w_->write("/log", e.time, logMsg(e.time, 3, "lifecycle",
      "track " + std::to_string(e.id.value) + " deleted").dump());
}

void FoxgloveDebugRecorder::onInnovation(const InnovationEvent& e) {
  if (!cfg_.enabled) return;
  const double nis = (e.residual.transpose() * e.S.ldlt().solve(e.residual)).value();
  last_S_[e.track_id.value] = e.S;
  w_->write("/diag/innovation", e.time,
            json{{"time_ns", e.time.nanos()}, {"track_id", e.track_id.value},
                 {"sensor", static_cast<int>(e.sensor)}, {"source_id", e.source_id},
                 {"nis", nis}, {"dim", e.dim}}.dump());
}

void FoxgloveDebugRecorder::onCollisionRisk(const CollisionRiskEvent& e) {
  if (!cfg_.enabled) return;
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
  if (!cfg_.enabled) return;
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
  // Keep the datum in sync even when drawing is disabled, so re-enabling
  // mid-run draws in the correct ENU frame; only the /log note is gated.
  datum_ = new_d;
  if (!cfg_.enabled) return;
  // Mark the discontinuity in the log at the last known event time.
  w_->write("/log", last_time_, logMsg(last_time_, 4, "datum",
      "datum recentered").dump());
}

void FoxgloveDebugRecorder::onStaticHazard(const StaticHazardEvent& e) {
  if (!cfg_.enabled) return;
  const char* kind = e.transition == StaticHazardTransition::Entered ? "ENTERED"
                   : e.transition == StaticHazardTransition::Exited  ? "EXITED" : "UPDATED";
  w_->write("/log", e.time, logMsg(e.time, 3, "static_hazard",
      std::string("HAZARD ") + kind + " id=" + std::to_string(e.hazard_id) +
      " d=" + std::to_string(e.distance_m) + "m keep_clear=" +
      std::to_string(e.keep_clear_m) + "m").dump());
  // Marker mirrors /cpa (numbers at origin); the obstacle ring is already drawn
  // on /static_obstacles, so this only flags the crossing.
  w_->ensureChannel("/static_hazard", kSceneUpdateSchema, "");
  std::vector<json> ents{ textEntity("hazard-" + std::to_string(e.hazard_id), {0,0,0},
      std::string(kind) + " id=" + std::to_string(e.hazard_id) +
      " d=" + std::to_string(e.distance_m), {1.0, 0.5, 0.0, 1.0}) };
  w_->write("/static_hazard", e.time, sceneUpdate(e.time, ents, cfg_.entity_lifetime_sec).dump());
}

void FoxgloveDebugRecorder::recordCoastline(const std::vector<LandPolygon>& polys) {
  if (!cfg_.enabled) return;
  ensureRootFrame(last_time_);
  w_->ensureChannel("/land", kSceneUpdateSchema, "");
  const Rgba land{0.55, 0.45, 0.30, 0.9};   // tan/brown outline
  auto ringToEnu = [&](const std::vector<Eigen::Vector2d>& ring) {
    std::vector<Pt> out;
    out.reserve(ring.size());
    for (const auto& lonlat : ring) {        // LandPolygon stores (lon, lat)
      const Eigen::Vector3d e = datum_.toEnu(geo::Geodetic{lonlat.y(), lonlat.x(), 0.0});
      out.push_back({e.x(), e.y(), 0.0});
    }
    return out;
  };
  std::vector<json> ents;
  int i = 0;
  for (const auto& poly : polys) {
    auto outer = ringToEnu(poly.outer);
    if (outer.size() >= 2)
      ents.push_back(lineEntity("land-" + std::to_string(i) + "-outer", outer, land, 2.0));
    int h = 0;
    for (const auto& hole : poly.holes) {
      auto hr = ringToEnu(hole);
      if (hr.size() >= 2)
        ents.push_back(lineEntity("land-" + std::to_string(i) + "-hole-" + std::to_string(h++),
                                  hr, land, 1.0));
    }
    ++i;
  }
  // Persist (static geometry): lifetime 0 so it stays visible across the run.
  w_->write("/land", last_time_, sceneUpdate(last_time_, ents, 0.0).dump());
}

void FoxgloveDebugRecorder::recordStaticObstacles(const std::vector<StaticObstacle>& obstacles) {
  if (!cfg_.enabled) return;
  ensureRootFrame(last_time_);
  w_->ensureChannel("/static_obstacles", kSceneUpdateSchema, "");
  std::vector<json> ents;
  int i = 0;
  for (const auto& o : obstacles) {
    const Eigen::Vector3d e3 = datum_.toEnu(o.position);
    const Eigen::Vector2d c(e3.x(), e3.y());
    const Rgba col = colorForObstacleCategory(o.category);
    const std::string base = "obs-" + std::to_string(i++);
    const double hard = o.footprint_radius_m + o.position_uncertainty_m;  // no-birth core
    if (hard > 0.0) ents.push_back(lineEntity(base + "-footprint", circle(c, hard), col, 2.0));
    if (o.keep_clear_radius_m > 0.0) {
      Rgba soft = col; soft.a = 0.5;
      ents.push_back(lineEntity(base + "-keepclear", circle(c, o.keep_clear_radius_m), soft, 1.0));
    }
    ents.push_back(textEntity(base + "-label", {c.x(), c.y(), 0},
        obstacleCategoryName(o.category), col));
  }
  w_->write("/static_obstacles", last_time_, sceneUpdate(last_time_, ents, 0.0).dump());
}

void FoxgloveDebugRecorder::recordOccupancy(const LiveOccupancyModel& occ, Timestamp now) {
  if (!cfg_.enabled) return;
  last_time_ = now;
  ensureRootFrame(now);
  const double cs = occ.cellSizeMeters();
  // Persistence heatmap (cubes, colored blue->red by normalized persistence).
  {
    std::vector<std::pair<Pt, Rgba>> cells;
    const double peak = std::max(1e-9, occ.peakPersistence());
    for (const auto& [center, val] : occ.persistenceCells()) {
      const double v = std::min(1.0, val / peak);
      cells.push_back({{center.x(), center.y(), 0.0},
                       {v, 0.2, 1.0 - v, 0.25 + 0.5 * v}});
    }
    w_->ensureChannel("/occupancy/persistence", kSceneUpdateSchema, "");
    w_->write("/occupancy/persistence", now,
              sceneUpdate(now, {gridCellsEntity("occ-persist", cells, cs)},
                          cfg_.entity_lifetime_sec).dump());
  }
  // Learned structure hazards (rings) + charted points.
  {
    std::vector<json> ents;
    const auto& obs = occ.obstacles();
    const auto& ctr = occ.structureCenters();
    for (std::size_t i = 0; i < obs.size() && i < ctr.size(); ++i) {
      const Rgba col = occ.obstacleCameraObservedEmpty(i) ? Rgba{1.0, 0.2, 0.2, 0.9}   // eviction candidate
                     : occ.obstacleCorroborated(i)        ? Rgba{0.2, 0.9, 0.2, 0.9}   // chart-confirmed
                                                          : Rgba{0.9, 0.9, 0.2, 0.9};  // uncorroborated
      const double r = obs[i].keep_clear_radius_m > 0.0 ? obs[i].keep_clear_radius_m : cs;
      ents.push_back(lineEntity("occ-struct-" + std::to_string(i), circle(ctr[i], r), col, 2.0));
    }
    const auto& chart = occ.chartedPoints();
    for (std::size_t i = 0; i < chart.size(); ++i)
      ents.push_back(lineEntity("occ-chart-" + std::to_string(i),
                                circle(chart[i], cs * 0.3, 8), Rgba{0.6, 0.6, 0.6, 0.7}));
    w_->ensureChannel("/occupancy/structures", kSceneUpdateSchema, "");
    w_->write("/occupancy/structures", now, sceneUpdate(now, ents, cfg_.entity_lifetime_sec).dump());
  }
  // Camera-observed-empty cells (eviction evidence).
  {
    std::vector<json> ents;
    int i = 0;
    for (const auto& c : occ.cameraObservedEmptyCells())
      ents.push_back(lineEntity("occ-camempty-" + std::to_string(i++),
                                circle(c, cs * 0.4, 8), Rgba{1.0, 0.4, 0.0, 0.8}));
    w_->ensureChannel("/occupancy/camera_empty", kSceneUpdateSchema, "");
    w_->write("/occupancy/camera_empty", now, sceneUpdate(now, ents, cfg_.entity_lifetime_sec).dump());
  }
  // Vessel-fix veto rings.
  {
    std::vector<json> ents;
    int i = 0;
    const double vr = occ.vetoRadiusMeters();
    for (const auto& c : occ.vesselFixPositions())
      ents.push_back(lineEntity("occ-veto-" + std::to_string(i++),
                                circle(c, vr), Rgba{0.2, 0.6, 1.0, 0.6}));
    w_->ensureChannel("/occupancy/veto", kSceneUpdateSchema, "");
    w_->write("/occupancy/veto", now, sceneUpdate(now, ents, cfg_.entity_lifetime_sec).dump());
  }
}

void FoxgloveDebugRecorder::recordPmbmDensity(const pmbm::PmbmDensity& density, Timestamp now) {
  if (!cfg_.enabled) return;
  last_time_ = now;
  ensureRootFrame(now);
  // PPP intensity: weighted covariance ellipse per Poisson component (opacity ~ weight).
  {
    std::vector<json> ents;
    double wmax = 1e-9;
    for (const auto& p : density.ppp) wmax = std::max(wmax, p.weight);
    int i = 0;
    for (const auto& p : density.ppp) {
      if (p.mean.size() < 2 || p.covariance.rows() < 2) continue;
      const double a = 0.15 + 0.5 * std::min(1.0, p.weight / wmax);
      ents.push_back(lineEntity("ppp-" + std::to_string(i++),
          covarianceEllipse(p.mean.head<2>(), p.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k),
          Rgba{0.6, 0.3, 0.9, a}));   // purple, opacity ~ intensity
    }
    w_->ensureChannel("/pmbm/ppp", kSceneUpdateSchema, "");
    w_->write("/pmbm/ppp", now, sceneUpdate(now, ents, cfg_.entity_lifetime_sec).dump());
  }
  // Bernoulli existence ellipses + trajectories, from the top-weight global hypothesis.
  {
    std::vector<json> bern, traj;
    const pmbm::GlobalHypothesis* best = nullptr;
    for (const auto& h : density.mbm)
      if (best == nullptr || h.weight > best->weight) best = &h;
    if (best != nullptr) {
      int i = 0;
      for (const auto& b : best->bernoullis) {
        if (b.mean.size() < 2 || b.covariance.rows() < 2) { ++i; continue; }
        const double r = b.existence_probability;
        const Rgba col{1.0 - r, 0.3 + 0.6 * r, 0.2, 0.3 + 0.6 * r};  // green ramps with r
        bern.push_back(lineEntity("bern-" + std::to_string(i),
            covarianceEllipse(b.mean.head<2>(), b.covariance.topLeftCorner<2,2>(), cfg_.ellipse_k),
            col));
        std::vector<Pt> path;
        for (const auto& tp : b.trajectory)
          if (tp.state.size() >= 2) path.push_back({tp.state(0), tp.state(1), 0.0});
        if (path.size() >= 2)
          traj.push_back(lineEntity("btraj-" + std::to_string(i), path, col, 1.5));
        ++i;
      }
    }
    w_->ensureChannel("/pmbm/bernoulli", kSceneUpdateSchema, "");
    w_->write("/pmbm/bernoulli", now, sceneUpdate(now, bern, cfg_.entity_lifetime_sec).dump());
    w_->ensureChannel("/pmbm/trajectories", kSceneUpdateSchema, "");
    w_->write("/pmbm/trajectories", now, sceneUpdate(now, traj, cfg_.entity_lifetime_sec).dump());
  }
}

void FoxgloveDebugRecorder::recordSensorCoverage(const std::string& source_id, SensorKind sensor,
    const Eigen::Vector2d& sensor_enu, double center_rad, double half_width_rad,
    double range_m, Timestamp now) {
  if (!cfg_.enabled) return;
  last_time_ = now;
  ensureRootFrame(now);
  Rgba col = colorForSensor(sensor, source_id); col.a = 0.25;
  const std::string topic = "/coverage/" + source_id;
  w_->ensureChannel(topic, kSceneUpdateSchema, "");
  auto ent = lineEntity("cov-" + source_id,
      sectorArc(sensor_enu, center_rad, half_width_rad, range_m), col, 1.0);
  w_->write(topic, now, sceneUpdate(now, {ent}, cfg_.entity_lifetime_sec).dump());
}

void FoxgloveDebugRecorder::recordClutterMap(const ClutterMapSensorDetectionModel& clutter,
    const Eigen::Vector2d& origin_enu, Timestamp now) {
  if (!cfg_.enabled) return;
  last_time_ = now;
  ensureRootFrame(now);
  // Position-space clutter heatmap (cubes, blue->red by normalized λ).
  {
    const auto in = clutter.positionClutterCells();
    double lmax = 1e-12;
    for (const auto& [c, l] : in) lmax = std::max(lmax, l);
    std::vector<std::pair<Pt, Rgba>> cells;
    for (const auto& [c, l] : in) {
      const double v = std::min(1.0, l / lmax);
      cells.push_back({{c.x(), c.y(), 0.0}, {v, 0.1, 1.0 - v, 0.2 + 0.6 * v}});
    }
    w_->ensureChannel("/clutter/position", kSceneUpdateSchema, "");
    w_->write("/clutter/position", now,
              sceneUpdate(now, {gridCellsEntity("clutter-pos", cells, clutter.cellSizeMeters())},
                          cfg_.entity_lifetime_sec).dump());
  }
  // Bearing-space azimuth rose: radial ray per touched cell, length ∝ λ.
  {
    const auto rose = clutter.bearingClutterCells();
    double lmax = 1e-12;
    for (const auto& [az, l] : rose) lmax = std::max(lmax, l);
    std::vector<json> ents;
    constexpr double kRefLen = 1000.0;   // ray length (m) at peak λ
    int i = 0;
    for (const auto& [az, l] : rose) {
      const double len = kRefLen * std::min(1.0, l / lmax);
      const Eigen::Vector2d tip = origin_enu + len * Eigen::Vector2d(std::cos(az), std::sin(az));
      ents.push_back(lineEntity("clutter-brg-" + std::to_string(i++),
          {{origin_enu.x(), origin_enu.y(), 0}, {tip.x(), tip.y(), 0}},
          Rgba{0.9, 0.2, 0.2, 0.7}));
    }
    w_->ensureChannel("/clutter/bearing", kSceneUpdateSchema, "");
    w_->write("/clutter/bearing", now, sceneUpdate(now, ents, cfg_.entity_lifetime_sec).dump());
  }
}

}  // namespace navtracker::foxglove
