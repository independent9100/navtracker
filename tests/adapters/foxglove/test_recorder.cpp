#include <gtest/gtest.h>
#include <cstdio>
#include <map>
#include <string>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "core/land/CoastlineGeometry.hpp"       // LandPolygon
#include "core/pmbm/PmbmTypes.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/types/StaticObstacle.hpp"
#include "ports/IStaticHazardSink.hpp"
#include "tests/adapters/foxglove/TmpPath.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

static std::map<std::string,int> countByTopic(const std::string& path) {
  mcap::McapReader r; (void)r.open(path);
  std::map<std::string,int> counts;
  auto view = r.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) counts[it->channel->topic]++;
  r.close();
  return counts;
}

static Track makeTrack(std::uint64_t id, double e, double n) {
  Track t; t.id = TrackId{id}; t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(e, n, 1.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 4.0;
  t.velocity_observed = true;
  return t;
}

TEST(Recorder, TracksEmitSceneMapAndCount) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_tracks");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    std::vector<Track> tracks{makeTrack(1, 100, 200), makeTrack(2, -50, -50)};
    rec.onTracks(tracks, Timestamp{1000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  // makeTrack() builds Confirmed tracks -> the confirmed (output) layer.
  EXPECT_EQ(c["/tracks/confirmed"], 1);        // one SceneUpdate per onTracks call
  EXPECT_EQ(c["/tracks/tentative"], 1);        // empty scene, still emitted
  EXPECT_EQ(c["/map/tracks/confirmed"], 2);    // one LocationFix per confirmed track
  EXPECT_EQ(c["/diag/track_count"], 1);
}

static Measurement posMeas(double e, double n) {
  Measurement m; m.time = Timestamp{2000}; m.sensor = SensorKind::Ais; m.source_id = "ais-1";
  m.model = MeasurementModel::Position2D; m.value = Eigen::Vector2d(e, n);
  m.covariance = Eigen::Matrix2d::Identity() * 9.0;
  return m;
}
static Measurement bearingMeas(double alpha) {
  Measurement m; m.time = Timestamp{2001}; m.sensor = SensorKind::EoIr; m.source_id = "cam-1";
  m.model = MeasurementModel::Bearing2D;
  m.value = Eigen::VectorXd::Constant(1, alpha);
  m.covariance = Eigen::MatrixXd::Constant(1,1, 0.01);
  m.sensor_position_enu = Eigen::Vector2d(0,0);
  return m;
}

TEST(Recorder, PositionAndBearingDetectionsEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_det");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    rec.recordMeasurement(posMeas(10, 20));
    rec.recordMeasurement(bearingMeas(0.0));
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  // Per-sensor topics: posMeas is Ais/"ais-1", bearingMeas is EoIr/"cam-1".
  EXPECT_EQ(c["/detections/ais-1"], 1);
  EXPECT_EQ(c["/detections/cam-1"], 1);
  EXPECT_EQ(c["/map/detections/ais-1"], 1);  // only the position meas maps to lat/lon
}

TEST(Recorder, InnovationEmitsNisAndGate) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_innov");
  {
    RecorderConfig cfg; cfg.gate_gamma = 9.21;        // chi2 2dof 99%
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}), nullptr, cfg);
    InnovationEvent e; e.time = Timestamp{3000}; e.track_id = TrackId{1};
    e.sensor = SensorKind::Ais; e.source_id = "ais-1";
    e.residual = Eigen::Vector2d(1.0, 0.0);
    e.S = Eigen::Matrix2d::Identity() * 4.0; e.R = e.S; e.dim = 2;
    rec.onInnovation(e);                              // caches S for track 1
    rec.onTracks({makeTrack(1, 0, 0)}, Timestamp{3001});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/diag/innovation"], 1);
  EXPECT_EQ(c["/gates"], 1);                          // gate drawn because S cached + gamma>0
}

TEST(Recorder, LifecycleCpaOwnshipEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_lifecycle");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    rec.onTrackConfirmed({TrackId{1}, Timestamp{4000}, TrackStatus::Confirmed});
    rec.onTrackDeleted({TrackId{1}, Timestamp{4500}, TrackStatus::Confirmed});
    CollisionRiskEvent ev; ev.transition = CollisionRiskTransition::Entered;
    ev.other = TrackId{1}; ev.time = Timestamp{4100};
    ev.prediction.cpa_distance_m = 50; ev.prediction.tcpa_seconds = 120;
    ev.prediction.probability_below_threshold = 0.8; ev.prediction.d_threshold_m = 100;
    rec.onCollisionRisk(ev);
    OwnShipPose pose; pose.time = Timestamp{4000}; pose.lat_deg = 59.9; pose.lon_deg = 10.7;
    pose.heading_true_deg = 90.0;
    rec.recordOwnShip(pose);
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_GE(c["/log"], 3);     // confirmed + deleted + cpa entered
  EXPECT_EQ(c["/cpa"], 1);
  EXPECT_EQ(c["/tf"], 2);  // static map->enu root frame + the own-ship transform
}

TEST(Recorder, MasterDisableSuppressesAllOutput) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_disabled");
  {
    RecorderConfig cfg; cfg.enabled = false; cfg.gate_gamma = 9.21;
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}), nullptr, cfg);
    // Drive every entry point; with enabled=false none may write a message.
    rec.recordMeasurement(posMeas(10, 20));
    rec.recordMeasurement(bearingMeas(0.0));
    OwnShipPose pose; pose.time = Timestamp{4000}; pose.lat_deg = 59.9; pose.lon_deg = 10.7;
    rec.recordOwnShip(pose);
    InnovationEvent e; e.time = Timestamp{3000}; e.track_id = TrackId{1};
    e.sensor = SensorKind::Ais; e.source_id = "ais-1";
    e.residual = Eigen::Vector2d(1.0, 0.0);
    e.S = Eigen::Matrix2d::Identity() * 4.0; e.R = e.S; e.dim = 2;
    rec.onInnovation(e);
    rec.onTracks({makeTrack(1, 0, 0)}, Timestamp{3001});
    rec.onTrackConfirmed({TrackId{1}, Timestamp{4000}, TrackStatus::Confirmed});
    CollisionRiskEvent ev; ev.transition = CollisionRiskTransition::Entered;
    ev.other = TrackId{1}; ev.time = Timestamp{4100};
    rec.onCollisionRisk(ev);
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  int total = 0;
  for (const auto& kv : c) total += kv.second;
  EXPECT_EQ(total, 0) << "enabled=false must suppress all messages";
}

TEST(Recorder, CoastlineAndStaticObstaclesEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_env");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    LandPolygon poly;
    poly.outer = {{10.70, 59.90}, {10.71, 59.90}, {10.71, 59.91}, {10.70, 59.91}};  // (lon,lat)
    poly.holes = {{{10.703, 59.903}, {10.705, 59.903}, {10.705, 59.905}}};
    rec.recordCoastline({poly});
    StaticObstacle o;
    o.position = geo::Geodetic{59.905, 10.705};
    o.footprint_radius_m = 20.0; o.keep_clear_radius_m = 100.0;
    o.category = ObstacleCategory::Rock;
    rec.recordStaticObstacles({o});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/land"], 1);
  EXPECT_EQ(c["/static_obstacles"], 1);
}

TEST(Recorder, OccupancyLayersEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_occ");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    LiveOccupancyModel occ(geo::Datum(geo::Geodetic{59.9, 10.7}));
    occ.observeVesselFix(1.0, Eigen::Vector2d(50, 50));          // veto anchor
    StaticObstacle chart; chart.position = geo::Geodetic{59.9, 10.7};
    occ.setChartedStructure({chart});                            // charted point
    rec.recordOccupancy(occ, Timestamp{6000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/occupancy/persistence"], 1);
  EXPECT_EQ(c["/occupancy/structures"], 1);
  EXPECT_EQ(c["/occupancy/camera_empty"], 1);
  EXPECT_EQ(c["/occupancy/veto"], 1);
}

TEST(Recorder, PmbmDensityLayersEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_pmbm");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    pmbm::PmbmDensity d;
    pmbm::PoissonComponent pc;
    pc.weight = 0.5; pc.mean = Eigen::Vector4d(10, 20, 0, 0);
    pc.covariance = Eigen::Matrix4d::Identity() * 25.0;
    d.ppp.push_back(pc);
    pmbm::GlobalHypothesis gh; gh.weight = 1.0;
    pmbm::Bernoulli b;
    b.existence_probability = 0.8; b.mean = Eigen::Vector4d(30, 40, 1, 0);
    b.covariance = Eigen::Matrix4d::Identity() * 9.0;
    pmbm::TrajectoryPoint tp0; tp0.state = Eigen::Vector4d(28, 38, 1, 0);
    pmbm::TrajectoryPoint tp1; tp1.state = Eigen::Vector4d(30, 40, 1, 0);
    b.trajectory = {tp0, tp1};
    gh.bernoullis.push_back(b);
    d.mbm.push_back(gh);
    rec.recordPmbmDensity(d, Timestamp{7000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/pmbm/ppp"], 1);
  EXPECT_EQ(c["/pmbm/bernoulli"], 1);
  EXPECT_EQ(c["/pmbm/trajectories"], 1);
}

TEST(Recorder, EstimatorInternalsAndCoverageEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_est");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    Track t = makeTrack(1, 0, 0);
    t.imm_means = Eigen::MatrixXd(4, 2);
    t.imm_means.col(0) = Eigen::Vector4d(0, 0, 1, 0);
    t.imm_means.col(1) = Eigen::Vector4d(1, 1, 1, 0);
    t.imm_covariances = {Eigen::MatrixXd::Identity(4, 4) * 4.0,
                         Eigen::MatrixXd::Identity(4, 4) * 9.0};
    t.imm_mode_probabilities = Eigen::Vector2d(0.7, 0.3);
    t.particles = Eigen::MatrixXd(4, 3);
    t.particles << 0, 1, 2,  0, 1, 2,  1, 1, 1,  0, 0, 0;
    t.particle_weights = Eigen::Vector3d(0.5, 0.3, 0.2);
    rec.onTracks({t}, Timestamp{8000});
    rec.recordSensorCoverage("radar-1", SensorKind::ArpaTtm, Eigen::Vector2d(0, 0),
                             /*center_rad=*/0.0, /*half_width=*/M_PI, /*range=*/3000.0,
                             Timestamp{8000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/tracks/imm_modes"], 1);
  EXPECT_EQ(c["/tracks/particles"], 1);
  EXPECT_EQ(c["/diag/mode_prob"], 1);
  EXPECT_EQ(c["/diag/existence"], 1);
  EXPECT_EQ(c["/coverage/radar-1"], 1);
}

TEST(Recorder, ClutterMapLayersEmit) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_clutter");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    // Empty maps (no observe): the layers still emit one message each so the
    // panel has channels; accessors return empty vectors and don't touch inner_.
    ClutterMapSensorDetectionModel cm(/*inner=*/nullptr);
    rec.recordClutterMap(cm, Eigen::Vector2d(0, 0), Timestamp{9500});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/clutter/position"], 1);
  EXPECT_EQ(c["/clutter/bearing"], 1);
}

TEST(Recorder, StaticHazardEventEmits) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_hazard");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    StaticHazardEvent e;
    e.transition = StaticHazardTransition::Entered; e.hazard_id = 42;
    e.time = Timestamp{9000}; e.distance_m = 80.0; e.keep_clear_m = 100.0;
    rec.onStaticHazard(e);
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/static_hazard"], 1);
  EXPECT_GE(c["/log"], 1);
}

TEST(Recorder, AssociationsLineFromTouchToTrack) {
  const std::string path = navtracker::foxglove::test::tmpMcapPath("recorder_assoc");
  {
    FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
    Track t = makeTrack(1, 100, 100);
    Track::SourceTouch st; st.sensor = SensorKind::Ais; st.source_id = "ais-1";
    st.time = Timestamp{5000}; st.value_enu = Eigen::Vector2d(105, 98);
    // alpha_rad left at default NaN -> position touch (not bearing-only)
    t.recent_contributions.push_back(st);
    rec.onTracks({t}, Timestamp{5000});
    rec.close();
  }
  auto c = countByTopic(path);
  std::remove(path.c_str());
  EXPECT_EQ(c["/associations"], 1);
}
