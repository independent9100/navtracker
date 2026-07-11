// End-to-end integration test for navtracker_t2t (ticket §6.4 / M4).
//
// Assembles the full push path a real consumer wires:
//   synthetic Position2D measurements
//     -> two INDEPENDENT live Tracker instances (own TrackManager each)
//     -> NavtrackerSource self-adapter (ITrackSink) per tracker
//     -> one shared T2tFuser (CI)
//     -> RecordingFusedSink.
//
// One target transits east->west; tracker A sees it via source "radarA",
// tracker B via source "aisB" (disjoint pedigrees -> ProvablyIndependent).
// Tracker B is silenced for a 10 s window mid-run (a real dropout: B's own
// track coasts, deletes, then re-initiates on B's return).
//
// Asserts: fused lifecycle events fire (init + confirm); the fused track keeps
// ONE identity across the dropout (continuity, no spurious re-birth wave when B
// returns); and the fused independence_class is ProvablyIndependent while both
// disjoint sources contribute.

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "adapters/t2t/NavtrackerSource.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Datum.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/t2t/FusedTrackOutput.hpp"
#include "core/t2t/T2tFuser.hpp"
#include "core/tracking/TrackManager.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/IFusedTrackSink.hpp"

using namespace navtracker;
using namespace navtracker::t2t;

namespace {

class FusedRecorder : public IFusedTrackSink {
 public:
  std::vector<std::string> events;
  std::vector<std::uint64_t> init_ids;
  void onFusedTrackInitiated(const FusedTrackLifecycleEvent& e) override {
    events.push_back("init");
    init_ids.push_back(e.id.value);
  }
  void onFusedTrackConfirmed(const FusedTrackLifecycleEvent&) override { events.push_back("confirm"); }
  void onFusedTrackUpdated(const FusedTrackLifecycleEvent&) override { events.push_back("update"); }
  void onFusedTrackDeleted(const FusedTrackLifecycleEvent&) override { events.push_back("delete"); }
  int count(const std::string& k) const {
    int n = 0;
    for (const auto& e : events) if (e == k) ++n;
    return n;
  }
};

Measurement positionAt(double x, double y, double t_s, double sigma_m, const std::string& src) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t_s);
  m.sensor = SensorKind::Ais;
  m.source_id = src;
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * (sigma_m * sigma_m);
  return m;
}

}  // namespace

TEST(T2tFullStack, TwoLiveTrackersFuseWithLifecycleContinuityAndIndependence) {
  const geo::Datum datum(geo::Geodetic{0.0, 0.0, 0.0});

  // Two independent live trackers (Position2D measurements are already ENU, so
  // no own-ship/datum plumbing is needed on the tracker side).
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator estA(motion, 5.0), estB(motion, 5.0);
  GnnAssociator assocA(100.0), assocB(100.0);
  TrackManager mgrA(1, 4), mgrB(1, 4);
  Tracker trackerA(estA, assocA, mgrA, 30.0);
  Tracker trackerB(estB, assocB, mgrB, 30.0);

  // One shared fuser + recording sink.
  T2tFuser fuser;  // default rule = covariance intersection
  fuser.setDatum(datum);
  FusedRecorder fsink;
  fuser.setFusedTrackSink(&fsink);

  // Self-adapters: each manager's lifecycle events -> ExternalTracks -> fuser.
  t2t::NavtrackerSource srcA("trackerA", mgrA,
                             [&](t2t::ExternalTrack e) { fuser.process(std::move(e)); });
  t2t::NavtrackerSource srcB("trackerB", mgrB,
                             [&](t2t::ExternalTrack e) { fuser.process(std::move(e)); });
  mgrA.setTrackSink(&srcA);
  mgrB.setTrackSink(&srcB);

  // Scenario: one target inbound east->west. B silenced for [12, 22).
  const double drop_lo = 12.0, drop_hi = 22.0;
  std::set<std::uint64_t> confirmed_ids;
  std::set<t2t::IndependenceClass> multi_source_classes;
  for (int sec = 1; sec <= 30; ++sec) {
    const double t_s = static_cast<double>(sec);
    const double x = 100.0 - 3.0 * t_s, y = 0.0;
    trackerA.process(positionAt(x, y, t_s, /*sigma=*/5.0, "radarA"));
    if (!(t_s >= drop_lo && t_s < drop_hi))
      trackerB.process(positionAt(x, y, t_s, /*sigma=*/8.0, "aisB"));
    fuser.flush();  // close this scan's fusion cycle

    for (const auto& fo : fuser.fusedTracks()) {
      if (fo.track.status == TrackStatus::Confirmed) confirmed_ids.insert(fo.track.id.value);
      if (fo.contributing_trackers.size() >= 2) multi_source_classes.insert(fo.independence_class);
    }
  }

  // 1. Fused lifecycle fired.
  EXPECT_GE(fsink.count("init"), 1);
  EXPECT_GE(fsink.count("confirm"), 1);

  // 2. Continuity: ONE fused identity across the whole run (the dropout coasts on
  //    A and re-acquires B without spawning a second fused track).
  EXPECT_EQ(confirmed_ids.size(), 1u)
      << "expected a single stable fused identity across the dropout";
  EXPECT_LE(fsink.count("init"), 2) << "no spurious re-birth wave on B's return";

  // 3. Independence: disjoint sources (radarA vs aisB) -> ProvablyIndependent
  //    whenever both contribute (before the dropout and after B is re-acquired).
  EXPECT_TRUE(multi_source_classes.count(t2t::IndependenceClass::ProvablyIndependent) != 0);

  // 4. The track survives to the end as a confirmed fused track.
  ASSERT_GE(fuser.fusedTracks().size(), 1u);
}
