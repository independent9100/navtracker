#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <vector>
#include <mcap/reader.hpp>
#include "adapters/foxglove/FoxgloveDebugRecorder.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/static/LiveOccupancyModel.hpp"
#include "core/tracking/ClutterMapDetectionModel.hpp"
#include "core/types/StaticObstacle.hpp"
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

// Run the same synthetic sequence into a recorder writing to `path`. Covers the
// original track path plus every environment / PMBM / occupancy / coverage tap
// so the determinism guarantee extends to the new layers.
void run(const std::string& path) {
  FoxgloveDebugRecorder rec(path, geo::Datum(geo::Geodetic{59.9, 10.7}));
  Track t = makeDetermTrack();
  rec.onTracks({t}, Timestamp{1000});

  LandPolygon poly;
  poly.outer = {{10.70, 59.90}, {10.71, 59.90}, {10.71, 59.91}};
  rec.recordCoastline({poly});

  StaticObstacle o;
  o.position = geo::Geodetic{59.905, 10.705};
  o.footprint_radius_m = 20.0; o.keep_clear_radius_m = 100.0;
  o.category = ObstacleCategory::Rock;
  rec.recordStaticObstacles({o});

  LiveOccupancyModel occ(geo::Datum(geo::Geodetic{59.9, 10.7}));
  occ.observeVesselFix(1.0, Eigen::Vector2d(50, 50));
  rec.recordOccupancy(occ, Timestamp{1500});

  pmbm::PmbmDensity d;
  pmbm::PoissonComponent pc;
  pc.weight = 0.5; pc.mean = Eigen::Vector4d(10, 20, 0, 0);
  pc.covariance = Eigen::Matrix4d::Identity() * 25.0;
  d.ppp.push_back(pc);
  rec.recordPmbmDensity(d, Timestamp{1600});

  rec.recordSensorCoverage("radar-1", SensorKind::ArpaTtm, Eigen::Vector2d(0, 0),
                           0.0, M_PI, 3000.0, Timestamp{1700});

  ClutterMapSensorDetectionModel cm(/*inner=*/nullptr);
  rec.recordClutterMap(cm, Eigen::Vector2d(0, 0), Timestamp{1750});

  StaticHazardEvent he;
  he.transition = StaticHazardTransition::Entered; he.hazard_id = 7;
  he.time = Timestamp{1800}; he.distance_m = 80.0; he.keep_clear_m = 100.0;
  rec.onStaticHazard(he);

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
