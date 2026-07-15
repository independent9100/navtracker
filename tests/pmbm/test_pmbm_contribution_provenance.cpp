#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Track.hpp"

// ---------------------------------------------------------------------------
// F2 (pre-release HIGH): contributing_sources must list only sensors that
// ACTUALLY contributed to a track.
//
// The source-touch walk (PmbmTracker.cpp ~L1646) used to match, for every
// Bernoulli in the dominant hypothesis, the nearest measurement whose
// z.time == b.last_update. Its comment claimed misdetected Bernoullis "inherit
// the parent pre-predict timestamp" so an exact-time filter separates
// detections from misses. That is false: PmbmTracker::predict advances
// last_update to the scan time on EVERY Bernoulli (detected or not), so a
// misdetected Bernoulli matches any measurement at the scan time and was
// falsely credited the nearest one — with no distance limit. In a uniform-
// timestamp scan that is every lone measurement.
//
// Consequence pinned here: a track updated only by radar, coasting through a
// scan that carries a foreign vessel's AIS return, was charged an AIS
// SourceTouch it never claimed. The fix keys the walk on
// Bernoulli::last_claimed_meas_index (-1 on a miss), so a misdetected
// Bernoulli gets no touch at all.
// ---------------------------------------------------------------------------
namespace {

using navtracker::EkfEstimator;
using navtracker::ConstantVelocity2D;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::Track;
using navtracker::pmbm::PmbmTracker;

Measurement mkPos(double t, double x, double y, SensorKind kind,
                  const std::string& src,
                  std::optional<std::uint32_t> mmsi = std::nullopt) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = kind;
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(x, y);
  z.covariance = Eigen::Matrix2d::Identity() * 4.0;
  z.hints.mmsi = mmsi;
  return z;
}

const Track* nearestTrack(const PmbmTracker& t, double x, double y) {
  const Track* best = nullptr;
  double best_d = 1e30;
  for (const auto& tr : t.tracks()) {
    if (tr.state.size() < 2) continue;
    const double d =
        (tr.state.head<2>() - Eigen::Vector2d(x, y)).squaredNorm();
    if (d < best_d) { best_d = d; best = &tr; }
  }
  return best;
}

bool listsSource(const Track& tr, const std::string& src) {
  return std::any_of(tr.recent_contributions.begin(),
                     tr.recent_contributions.end(),
                     [&](const Track::SourceTouch& s) {
                       return s.source_id == src;
                     });
}

}  // namespace

TEST(PmbmContributionProvenance, MisdetectedTrackDoesNotInheritForeignSource) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  // Deployment-shaped base: source_aware_misdetection + idle_halflife on (the
  // makePmbmConfig defaults), adaptive birth so radar detections start tracks.
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-6;
  c.survival_probability = 1.0;
  c.adaptive_birth = true;
  c.birth_existence_target = 0.6;
  c.confirm_threshold = 0.5;
  c.source_aware_misdetection = true;
  c.idle_halflife_sec = 10.0;
  c.r_min = 1e-4;
  PmbmTracker tracker(ekf, c);

  // Scans 0..5: radar repeatedly detects one target at the origin.
  for (int k = 0; k <= 5; ++k) {
    tracker.predict(Timestamp::fromSeconds(k));
    tracker.processBatch({mkPos(k, 0.0, 0.0, SensorKind::ArpaTtm, "radar")});
  }
  const Track* radar_track = nearestTrack(tracker, 0.0, 0.0);
  ASSERT_NE(radar_track, nullptr);
  EXPECT_TRUE(listsSource(*radar_track, "radar"))
      << "sanity: the radar-detected track should list radar";
  EXPECT_FALSE(listsSource(*radar_track, "ais"))
      << "no AIS has been seen yet";

  // Scan 6: NO radar return for the target; a DIFFERENT vessel's AIS return
  // appears 7 km away. The origin track is misdetected this scan.
  tracker.predict(Timestamp::fromSeconds(6));
  tracker.processBatch({mkPos(6, 5000.0, 5000.0, SensorKind::Ais, "ais",
                              std::optional<std::uint32_t>{123456789U})});

  const Track* origin_track = nearestTrack(tracker, 0.0, 0.0);
  ASSERT_NE(origin_track, nullptr);
  EXPECT_FALSE(listsSource(*origin_track, "ais"))
      << "a track updated only by radar must never list AIS in "
         "contributing_sources — AIS never contributed to it";
  // The genuine AIS-born track (far away) SHOULD carry the AIS touch.
  const Track* ais_track = nearestTrack(tracker, 5000.0, 5000.0);
  if (ais_track != nullptr && ais_track != origin_track) {
    EXPECT_TRUE(listsSource(*ais_track, "ais"))
        << "the track the AIS return actually created must list AIS";
  }
}
