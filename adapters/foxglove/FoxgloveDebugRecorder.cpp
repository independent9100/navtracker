#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include <fstream>
#include <sstream>
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
  std::vector<json> entities;
  int confirmed = 0, tentative = 0;
  for (const auto& t : tracks) {
    if (t.state.size() < 2) continue;
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
  w_->write("/tracks", now, sceneUpdate(now, entities).dump());
  json diag{{"time_ns", now.nanos()}, {"confirmed", confirmed}, {"tentative", tentative},
            {"total", confirmed + tentative}};
  w_->write("/diag/track_count", now, diag.dump());
}

// Remaining sink methods implemented in later tasks (Tasks 5-8). Provide
// empty bodies now so the class is concrete and links:
void FoxgloveDebugRecorder::recordMeasurement(const Measurement&) {}
void FoxgloveDebugRecorder::recordOwnShip(const OwnShipPose&) {}
void FoxgloveDebugRecorder::onTrackInitiated(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackConfirmed(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackUpdated(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onTrackDeleted(const TrackLifecycleEvent&) {}
void FoxgloveDebugRecorder::onInnovation(const InnovationEvent&) {}
void FoxgloveDebugRecorder::onCollisionRisk(const CollisionRiskEvent&) {}
void FoxgloveDebugRecorder::onDatumRecentered(const geo::Datum&, const geo::Datum&) {}

}  // namespace navtracker::foxglove
