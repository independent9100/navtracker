// F2 provenance cycle — Rider 2 pin: LIVE T2T pedigree CONTENT is truthful.
//
// Until the F2 source-touch fix, §10 Rider B (docs/baselines/2026-07-11_t2t_
// gates.md) forbade asserting pedigree *content* from a LIVE tracker — only
// handcrafted TrackOutput fixtures (test_navtracker_source) and plumbing
// (test_t2t_full_stack). The stated reason: a live track's provenance could
// carry spurious sensor attributions (the PmbmTracker source-touch bug, F2).
//
// This test lifts that prohibition with EVIDENCE. It drives a live two-sensor
// pipeline (radar + AIS into ONE Tracker) through the T2T self-adapter and
// asserts the emitted pedigree CONTENT is truthful, per track:
//   - a track a sensor genuinely NEVER contributed to must NOT list that sensor
//     (Used); it stays at the Unknown/NotUsed default;
//   - a track both sensors contributed to lists BOTH Used.
//
// Mechanism note (measured in the F2 cycle): the T2T pedigree is filled by
// NavtrackerSource from Track::contributing_sources, which the flat/MHT pipeline
// populates per-update from the associated measurement's source_id — genuine by
// construction. The F2 bug lived in a DIFFERENT channel (PmbmTracker's
// recent_contributions / SourceTouch vector), which does not feed
// contributing_sources; PMBM leaves contributing_sources empty. So the live T2T
// pedigree is truthful on the flat/MHT path and empty-safe (never spurious) on
// PMBM — this test pins the former; the tracker-side regression test
// (PmbmContributionProvenance) guards the latter's source channel.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "adapters/t2t/NavtrackerSource.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/t2t/ExternalTrack.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;
using namespace navtracker::t2t;

namespace {

Measurement posAt(double x, double y, double t_s, double sigma_m,
                  const std::string& src) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;  // kind is irrelevant here; source_id drives pedigree
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (sigma_m * sigma_m);
  return m;
}

}  // namespace

// One live Tracker, two well-separated targets, two sensor streams:
//   target R (around x≈+400): fed by "radar" ONLY — "ais" never contributes;
//   target B (around x≈-400): fed by "radar" AND "ais" (co-located).
// Assert the pedigree NavtrackerSource emits per track reflects exactly that.
TEST(T2tLivePedigreeContent, RadarOnlyTrackNeverListsAisAndBothListsBoth) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator est(motion, 5.0);
  GnnAssociator assoc(100.0);
  TrackManager mgr(1, 4);
  Tracker tracker(est, assoc, mgr, 30.0);

  // Capture the latest ExternalTrack per source_track_id from the self-adapter.
  std::vector<ExternalTrack> emitted;
  NavtrackerSource src("navtracker", mgr,
                       [&](ExternalTrack e) { emitted.push_back(std::move(e)); });
  mgr.setTrackSink(&src);

  // 30 s. Target R near x=+400 (radar only). Target B near x=-400 (radar+ais).
  for (int sec = 1; sec <= 30; ++sec) {
    const double t_s = static_cast<double>(sec);
    tracker.process(posAt(+400.0, 3.0 * t_s, t_s, /*sigma=*/5.0, "radar"));  // R: radar only
    tracker.process(posAt(-400.0, 3.0 * t_s, t_s, /*sigma=*/5.0, "radar"));  // B: radar
    tracker.process(posAt(-400.0, 3.0 * t_s, t_s, /*sigma=*/8.0, "ais"));    // B: ais
  }

  ASSERT_FALSE(emitted.empty()) << "self-adapter emitted no ExternalTracks";

  // Latest emission per source track id, classified by ENU x-sign.
  const ExternalTrack* radar_only = nullptr;  // x > 0
  const ExternalTrack* both = nullptr;        // x < 0
  for (const auto& e : emitted) {
    if (e.position_enu.x() > 0.0) radar_only = &e;
    else both = &e;
  }
  ASSERT_NE(radar_only, nullptr) << "radar-only target track never surfaced";
  ASSERT_NE(both, nullptr) << "both-sensor target track never surfaced";
  ASSERT_TRUE(radar_only->pedigree.has_value());
  ASSERT_TRUE(both->pedigree.has_value());

  // The radar-only track's pedigree is TRUTHFUL: radar Used, AIS absent
  // (Unknown/NotUsed default) — it must NOT spuriously claim AIS.
  EXPECT_EQ(radar_only->pedigree->usageOf("radar"), SensorUsage::Used);
  EXPECT_NE(radar_only->pedigree->usageOf("ais"), SensorUsage::Used)
      << "radar-only track spuriously lists AIS in its live pedigree";

  // The both-sensor track lists BOTH as Used.
  EXPECT_EQ(both->pedigree->usageOf("radar"), SensorUsage::Used);
  EXPECT_EQ(both->pedigree->usageOf("ais"), SensorUsage::Used)
      << "both-sensor track dropped a genuine contributor from its pedigree";
}
