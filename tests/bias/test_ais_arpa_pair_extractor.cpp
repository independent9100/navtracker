#include "core/bias/AisArpaPairExtractor.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace navtracker {

namespace {

// Default own-ship is a realistic non-zero ENU offset from the datum: the
// extractor now skips a pair whose ARPA touch carries the (0,0) "own-ship
// unset" sentinel (see SkipsWhenArpaOwnShipOriginUnset), so pair-forming tests
// must supply a genuine own-ship position.
Track::SourceTouch makeTouch(SensorKind k, Timestamp t,
                             Eigen::Vector2d v,
                             Eigen::Vector2d own = Eigen::Vector2d(10.0, 20.0)) {
  Track::SourceTouch s;
  s.sensor = k;
  s.time = t;
  s.value_enu = v;
  s.sensor_position_enu = own;
  s.covariance = Eigen::Matrix2d::Identity() * 25.0;
  return s;
}

Timestamp tAt(double s) {
  return Timestamp{static_cast<std::int64_t>(s * 1e9)};
}

}  // namespace

TEST(AisArpaPairExtractorTest, EmitsOnePairWhenBothSourcesContributedThisCycle) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0)));
  const auto pairs = extractPairs({tr}, tAt(10.0));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_NEAR(pairs[0].ais_target_position_enu.x(), 1000.0, 1e-9);
  EXPECT_NEAR(pairs[0].arpa_target_position_enu.y(), 87.0, 1e-9);
}

TEST(AisArpaPairExtractorTest, SkipsWhenOnlyOneSourcePresent) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  EXPECT_TRUE(extractPairs({tr}, tAt(10.0)).empty());
}

TEST(AisArpaPairExtractorTest, IgnoresContributionsOutsideCycleWindow) {
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(0.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0)));
  EXPECT_TRUE(extractPairs({tr}, tAt(10.0)).empty());
}

TEST(AisArpaPairExtractorTest, PropagatesOwnPositionStdFromTouch) {
  Track tr;
  auto ais = makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0));
  auto arpa =
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0));
  arpa.own_position_std_m = 3.0;
  tr.recent_contributions.push_back(ais);
  tr.recent_contributions.push_back(arpa);
  const auto pairs = extractPairs({tr}, tAt(10.0));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_DOUBLE_EQ(pairs[0].own_position_std_m, 3.0);
}

TEST(AisArpaPairExtractorTest, CooperativeGnssActsAsAnchorLikeAis) {
  Track tr;
  tr.recent_contributions.push_back(makeTouch(
      SensorKind::Cooperative, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(10.0), Eigen::Vector2d(995.0, 87.0)));
  const auto pairs = extractPairs({tr}, tAt(10.0));
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_NEAR(pairs[0].ais_target_position_enu.x(), 1000.0, 1e-9);
}

TEST(AisArpaPairExtractorTest, SkipsWhenArpaOwnShipOriginUnset) {
  // W3.3 teeth: an ARPA-TLL fix arriving before the first own-ship pose leaves
  // sensor_position_enu at the (0,0) "unset" sentinel. Forming a pair about the
  // origin would measure the bearing subtended at the datum, not own-ship — the
  // exact bug W3.3 fixes — and those cold-start observations are outlier-gate-
  // exempt. The extractor must SKIP such a pair.
  Track tr;
  tr.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(10.0), Eigen::Vector2d(1000.0, 0.0)));
  tr.recent_contributions.push_back(makeTouch(SensorKind::ArpaTll, tAt(10.0),
                                              Eigen::Vector2d(995.0, 87.0),
                                              Eigen::Vector2d::Zero()));
  EXPECT_TRUE(extractPairs({tr}, tAt(10.0)).empty());

  // Same pair, but with a known own-ship origin → a pair IS formed.
  tr.recent_contributions.back().sensor_position_enu =
      Eigen::Vector2d(10.0, 20.0);
  EXPECT_EQ(extractPairs({tr}, tAt(10.0)).size(), 1u);
}

TEST(AisArpaPairExtractorTest, MultipleTracksEmitMultiplePairs) {
  Track a;
  a.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(5.0), Eigen::Vector2d(1000.0, 0.0)));
  a.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(5.0), Eigen::Vector2d(990.0, 50.0)));
  Track b;
  b.recent_contributions.push_back(
      makeTouch(SensorKind::Ais, tAt(5.0), Eigen::Vector2d(0.0, 2000.0)));
  b.recent_contributions.push_back(
      makeTouch(SensorKind::ArpaTtm, tAt(5.0), Eigen::Vector2d(50.0, 1990.0)));
  EXPECT_EQ(extractPairs({a, b}, tAt(5.0)).size(), 2u);
}

}  // namespace navtracker
