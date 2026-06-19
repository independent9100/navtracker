#include <gtest/gtest.h>
#include <cstdio>
#include <map>
#include <string>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
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
