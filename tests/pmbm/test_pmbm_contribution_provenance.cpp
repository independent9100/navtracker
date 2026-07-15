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

// ---------------------------------------------------------------------------
// W5.2 <-> F2 COMPOSITION (this branch rebased onto master after the F2 cycle):
// the POSITIVE mixed-timestamp case F2's own test does not reach.
//
// F2 re-keyed the source-touch walk onto Bernoulli::last_claimed_meas_index,
// which is independent of last_update. W5.2 (this wave) stamps a detected
// Bernoulli's last_update at the state's PHYSICAL time (t_max, where predict()
// advanced every component), NOT the claimed measurement's earlier timestamp.
// Together, on a MIXED-timestamp two-sensor scan, each target must be attributed
// exactly the sensor it CLAIMED — including the target whose claimed measurement
// arrived at t < t_max. F2's own regression exercises only uniform-timestamp
// scans, so this composition was untested until W5.2 made the divergence
// reachable.
//
// TEETH: reverting the walk to the OLD `z.time == b.last_update` key turns this
// RED. The radar target's last_update is stamped t_max=5.5 by W5.2, so the old
// key would match it to the ONLY same-time measurement — the AIS return at 5.5,
// 4 km away, with no distance bound — falsely crediting the radar track an AIS
// SourceTouch AND dropping its genuine radar@5.0 touch. The index key is immune.
// This is exactly the coupling this wave flagged before the rebase; the rebase
// resolves it and this test pins the resolution.
// ---------------------------------------------------------------------------
TEST(PmbmContributionProvenance, MixedTimestampMultiSensorAttributesEachClaimedSource) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
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

  // Two well-separated targets, each on its own sensor, over uniform scans 0..4:
  //   A at the origin, fed by radar;  B 4 km east, fed by AIS.
  for (int k = 0; k <= 4; ++k) {
    tracker.predict(Timestamp::fromSeconds(k));
    tracker.processBatch(
        {mkPos(k, 0.0, 0.0, SensorKind::ArpaTtm, "radar"),
         mkPos(k, 4000.0, 0.0, SensorKind::Ais, "ais",
               std::optional<std::uint32_t>{987654321U})});
  }

  // The MIXED-timestamp scan: A's radar return arrives at t=5.0 (EARLIER), B's
  // AIS return at t=5.5 (== t_max). processBatch predicts both components to
  // t_max=5.5, so W5.2 stamps A.last_update = 5.5 though A claimed the 5.0 return.
  tracker.predict(Timestamp::fromSeconds(5.5));
  tracker.processBatch(
      {mkPos(5.0, 0.0, 0.0, SensorKind::ArpaTtm, "radar"),
       mkPos(5.5, 4000.0, 0.0, SensorKind::Ais, "ais",
             std::optional<std::uint32_t>{987654321U})});

  const Track* a = nearestTrack(tracker, 0.0, 0.0);
  const Track* b = nearestTrack(tracker, 4000.0, 0.0);
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  ASSERT_NE(a, b) << "the two targets are 4 km apart; they must not fuse";

  // Each target lists only the sensor it actually claimed.
  EXPECT_TRUE(listsSource(*a, "radar"));
  EXPECT_FALSE(listsSource(*a, "ais"))
      << "the radar target claimed the 5.0 radar return, NOT the 5.5 AIS return "
         "4 km away — the old z.time==last_update key misattributes it because "
         "W5.2 stamps its last_update at t_max=5.5";
  EXPECT_TRUE(listsSource(*b, "ais"));
  EXPECT_FALSE(listsSource(*b, "radar"))
      << "the AIS target never saw radar";

  // Positive attribution of the earlier-timestamp claim: A must carry a radar
  // SourceTouch AT the claimed 5.0 time (this mixed scan), not just historical
  // ones — the old key drops it (5.0 != last_update 5.5).
  const bool a_has_radar_at_5 = std::any_of(
      a->recent_contributions.begin(), a->recent_contributions.end(),
      [](const Track::SourceTouch& s) {
        return s.source_id == "radar" && s.time == Timestamp::fromSeconds(5.0);
      });
  EXPECT_TRUE(a_has_radar_at_5)
      << "the radar target's claimed 5.0 return must be recorded as a SourceTouch "
         "at t=5.0";
}
