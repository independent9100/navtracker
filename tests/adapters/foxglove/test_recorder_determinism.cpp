#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <vector>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "tests/adapters/foxglove/TmpPath.hpp"

using namespace navtracker;
using namespace navtracker::foxglove;

namespace {

// Build one Track identical to what the recorder exercises in onTracks.
Track makeDetermTrack() {
  Track t;
  t.id = TrackId{1};
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(100.0, 200.0, 1.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity() * 4.0;
  t.velocity_observed = true;
  return t;
}

// Run the same synthetic sequence into a recorder writing to `path`.
void run(const std::string& path) {
  FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
  Track t = makeDetermTrack();
  rec.onTracks({t}, Timestamp{1000});
  rec.onTracks({t}, Timestamp{2000});
  rec.close();
}

// Read every message from `path` and return "topic|logTime|data" strings.
std::vector<std::string> payloads(const std::string& path) {
  mcap::McapReader reader;
  (void)reader.open(path);
  std::vector<std::string> out;
  auto view = reader.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) {
    const auto& msg = it->message;
    const std::string data(reinterpret_cast<const char*>(msg.data), msg.dataSize);
    out.push_back(it->channel->topic + "|" +
                  std::to_string(msg.logTime) + "|" + data);
  }
  reader.close();
  return out;
}

}  // namespace

TEST(RecorderDeterminism, SameInputIdenticalPayloads) {
  const std::string path_a = navtracker::foxglove::test::tmpMcapPath("determinism_a");
  const std::string path_b = navtracker::foxglove::test::tmpMcapPath("determinism_b");

  run(path_a);
  run(path_b);

  const auto pa = payloads(path_a);
  const auto pb = payloads(path_b);

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());

  ASSERT_FALSE(pa.empty()) << "run() produced no messages";
  EXPECT_EQ(pa, pb);
}
