#include <vector>

#include <gtest/gtest.h>
#include "core/association/GnnAssociator.hpp"

using navtracker::GnnAssociator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::Track;

namespace {
Measurement positionMeas(double x, double y) {
  Measurement z;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity();
  return z;
}
Track positionTrack(double x, double y) {
  Track t;
  t.state = Eigen::Vector4d(x, y, 0.0, 0.0);
  t.covariance = Eigen::Matrix4d::Identity();
  return t;
}
}  // namespace

TEST(GnnAssociator, MatchesNearestTracksAndMeasurements) {
  const GnnAssociator assoc(9.21);
  const std::vector<Track> tracks{positionTrack(0.0, 0.0), positionTrack(100.0, 0.0)};
  const std::vector<Measurement> meas{positionMeas(0.5, 0.0), positionMeas(100.5, 0.0)};
  const auto r = assoc.associate(tracks, meas);

  ASSERT_EQ(r.matches.size(), 2u);
  bool m00 = false, m11 = false;
  for (const auto& m : r.matches) {
    if (m.first == 0 && m.second == 0) m00 = true;
    if (m.first == 1 && m.second == 1) m11 = true;
  }
  EXPECT_TRUE(m00);
  EXPECT_TRUE(m11);
  EXPECT_TRUE(r.unmatched_tracks.empty());
  EXPECT_TRUE(r.unmatched_measurements.empty());
}

TEST(GnnAssociator, OutOfGateBecomesUnmatched) {
  const GnnAssociator assoc(9.21);
  const std::vector<Track> tracks{positionTrack(0.0, 0.0)};
  const std::vector<Measurement> meas{positionMeas(1000.0, 0.0)};
  const auto r = assoc.associate(tracks, meas);

  EXPECT_TRUE(r.matches.empty());
  ASSERT_EQ(r.unmatched_tracks.size(), 1u);
  EXPECT_EQ(r.unmatched_tracks[0], 0u);
  ASSERT_EQ(r.unmatched_measurements.size(), 1u);
  EXPECT_EQ(r.unmatched_measurements[0], 0u);
}
