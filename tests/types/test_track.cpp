#include <gtest/gtest.h>
#include "core/types/Track.hpp"

using navtracker::Track;
using navtracker::TrackId;
using navtracker::TrackStatus;

TEST(Track, DefaultsToTentativeWithStableId) {
  Track t;
  EXPECT_EQ(t.status, TrackStatus::Tentative);
  EXPECT_EQ(t.id.value, 0u);
  EXPECT_TRUE(t.contributing_sources.empty());
  EXPECT_FALSE(t.attributes.mmsi.has_value());
}

TEST(Track, HoldsKinematicStateAndProvenance) {
  Track t;
  t.id = TrackId{17};
  t.status = TrackStatus::Confirmed;
  t.state = Eigen::Vector4d(10.0, 20.0, 1.0, -2.0);   // [px, py, vx, vy]
  t.covariance = Eigen::Matrix4d::Identity();
  t.attributes.mmsi = 211234560u;
  t.attributes.name = "MV TEST";
  t.contributing_sources.push_back("ais");
  t.contributing_sources.push_back("radar_fwd");

  EXPECT_EQ(t.id, TrackId{17});
  EXPECT_EQ(t.state.size(), 4);
  EXPECT_EQ(t.state(2), 1.0);
  ASSERT_TRUE(t.attributes.name.has_value());
  EXPECT_EQ(*t.attributes.name, "MV TEST");
  EXPECT_EQ(t.contributing_sources.size(), 2u);
}

TEST(Track, DefaultHasEmptyParticleEnsemble) {
  navtracker::Track t;
  EXPECT_EQ(t.particles.rows(), 0);
  EXPECT_EQ(t.particles.cols(), 0);
  EXPECT_EQ(t.particle_weights.size(), 0);
}
