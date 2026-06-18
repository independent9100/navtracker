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
  EXPECT_EQ(c["/tracks"], 1);              // one SceneUpdate per onTracks call
  EXPECT_EQ(c["/map/tracks"], 2);          // one LocationFix per track
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
  EXPECT_EQ(c["/detections"], 2);          // one SceneUpdate per measurement
  EXPECT_EQ(c["/map/detections"], 1);      // only the position meas maps to lat/lon
}
