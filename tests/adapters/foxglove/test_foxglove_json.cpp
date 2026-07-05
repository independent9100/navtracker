#include <gtest/gtest.h>
#include "adapters/foxglove/FoxgloveJson.hpp"
using namespace navtracker;
using namespace navtracker::foxglove;

TEST(FoxgloveJson, TimeSplitsNanos) {
  auto j = timeJson(Timestamp{1'500'000'003LL});   // 1.5s + 3ns  (adjust if Timestamp differs)
  EXPECT_EQ(j["sec"].get<long long>(), 1);
  EXPECT_EQ(j["nsec"].get<long long>(), 500'000'003);
}

TEST(FoxgloveJson, SceneUpdateHasEntityWithLine) {
  auto line = lineEntity("e1", {{0,0,0},{1,1,0}}, {1,0,0,1}, 2.0);
  auto su = sceneUpdate(Timestamp{0}, {line});
  ASSERT_EQ(su["entities"].size(), 1u);
  auto& ent = su["entities"][0];
  EXPECT_EQ(ent["frame_id"], "enu");
  ASSERT_EQ(ent["lines"].size(), 1u);
  EXPECT_EQ(ent["lines"][0]["points"].size(), 2u);
  EXPECT_DOUBLE_EQ(ent["lines"][0]["color"]["r"].get<double>(), 1.0);
}

TEST(FoxgloveJson, GridCellsEntityHasCubePerCell) {
  std::vector<std::pair<Pt, Rgba>> cells{
      {{0, 0, 0}, {1, 0, 0, 0.5}}, {{25, 0, 0}, {0, 1, 0, 0.8}}};
  auto e = gridCellsEntity("occ", cells, /*cell_size=*/25.0, /*height=*/0.0);
  ASSERT_EQ(e["cubes"].size(), 2u);
  EXPECT_DOUBLE_EQ(e["cubes"][0]["size"]["x"].get<double>(), 25.0);
  EXPECT_DOUBLE_EQ(e["cubes"][1]["pose"]["position"]["x"].get<double>(), 25.0);
  EXPECT_DOUBLE_EQ(e["cubes"][0]["color"]["a"].get<double>(), 0.5);
  EXPECT_EQ(e["lines"].size(), 0u);   // heatmap uses cubes, not lines
}

TEST(FoxgloveJson, LocationFixCarriesLatLonAndCov) {
  std::array<double,9> cov{}; cov[0] = 4.0; cov[4] = 9.0;
  auto j = locationFix(Timestamp{0}, 59.9, 10.7, cov);
  EXPECT_DOUBLE_EQ(j["latitude"].get<double>(), 59.9);
  EXPECT_DOUBLE_EQ(j["longitude"].get<double>(), 10.7);
  ASSERT_EQ(j["position_covariance"].size(), 9u);
  EXPECT_DOUBLE_EQ(j["position_covariance"][0].get<double>(), 4.0);
}

TEST(FoxgloveJson, FrameTransformYawToQuaternion) {
  auto j = frameTransform(Timestamp{0}, "enu", "own_ship", 5, 6, 0, 0.0);
  EXPECT_DOUBLE_EQ(j["translation"]["x"].get<double>(), 5.0);
  EXPECT_NEAR(j["rotation"]["w"].get<double>(), 1.0, 1e-12);   // yaw 0 -> identity
  EXPECT_NEAR(j["rotation"]["z"].get<double>(), 0.0, 1e-12);
}
