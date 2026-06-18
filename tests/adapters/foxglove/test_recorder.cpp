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
