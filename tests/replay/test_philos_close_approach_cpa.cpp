// R8.6 item 2 — the first REAL-DATA collision-alarm test.
//
// close_approach (Charles River sailing basin, zero AIS, 80 s): a ~4 m club
// sailing dinghy closes on the berthed own-ship and makes contact at t≈61 s
// (unix ≈1635536620); a second dinghy crosses close ahead at the same moment.
// The radar plots carry returns on the collider down to 17 m (n_cells 4–29,
// amp ≥74). This test feeds those real plots through the full stack
// (EKF + GNN + TrackManager + CpaEvaluator) and asserts the collision alarm:
//   (a) a Confirmed track closes onto the collider during the approach;
//   (b) CpaEvaluator fires Entered BEFORE contact, with usable lead time.
//
// Fixture-gated: skips when the local philos fixture is absent (as the other
// philos replay tests do); the bench driver runs from the project root.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "adapters/replay/OwnshipCsvReader.hpp"
#include "adapters/replay/PlotCsvReplayAdapter.hpp"
#include "core/association/GnnAssociator.hpp"
#include "core/collision/CpaEvaluator.hpp"
#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/geo/Wgs84.hpp"
#include "core/pipeline/Tracker.hpp"
#include "core/tracking/TrackManager.hpp"
#include "ports/ICollisionRiskSink.hpp"

using namespace navtracker;

namespace {

std::string clipDir() {
  return std::string(NAVTRACKER_SOURCE_DIR) +
         "/tests/fixtures/philos/out/close_approach";
}
bool exists(const std::string& p) { return static_cast<bool>(std::ifstream(p)); }

struct RiskRecorder : ICollisionRiskSink {
  std::vector<CollisionRiskEvent> events;
  void onCollisionRisk(const CollisionRiskEvent& e) override {
    events.push_back(e);
  }
};

}  // namespace

TEST(PhilosCloseApproachCpa, CollisionAlarmFiresBeforeContact) {
  const std::string own = clipDir() + "/ownship.csv";
  const std::string plots = clipDir() + "/radar_plots.csv";
  if (!exists(own) || !exists(plots))
    GTEST_SKIP() << "close_approach fixture not reachable from cwd";

  const auto poses = navtracker::replay::loadOwnshipCsv(own);
  ASSERT_FALSE(poses.empty());
  const double clip_start_unix = poses.front().time.seconds();
  // Contact ≈ unix 1635536620 (t≈61 s into the clip); approach window t≈30–65 s.
  const double contact_unix = 1635536620.0;

  // Own-ship is berthed/static (~15 m total): a single provider pre-filled with
  // all poses both projects the body-frame plots AND, since own-ship barely
  // moves, is an accurate CPA reference via provider.latest().
  OwnShipProvider provider(poses.size() + 1);
  navtracker::replay::feedOwnshipHistory(provider, poses);
  const Eigen::Vector3d own_enu3 =
      provider.datum().toEnu(navtracker::geo::Geodetic{
          provider.latest()->lat_deg, provider.latest()->lon_deg, 0.0});
  const Eigen::Vector2d own_enu(own_enu3.x(), own_enu3.y());

  auto meas = navtracker::replay::loadPlotCsvBodyFrame(
      plots, provider, SensorKind::ArpaTtm, "philos_radar");
  ASSERT_FALSE(meas.empty());
  std::sort(meas.begin(), meas.end(),
            [](const Measurement& a, const Measurement& b) {
              return a.time < b.time;
            });

  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator estimator(motion, 5.0);
  GnnAssociator associator(100.0);
  TrackManager manager(/*confirm_hits=*/2, /*delete_misses=*/5);
  Tracker tracker(estimator, associator, manager, /*init_gate_m=*/30.0);

  CpaEvaluatorConfig cfg;
  cfg.d_threshold_m = 50.0;  // the collider closes to 17 m, well inside
  cfg.enter_probability = 0.5;
  cfg.exit_probability = 0.3;
  CpaEvaluator cpa(manager, provider, cfg);
  RiskRecorder risk;
  cpa.setSink(&risk);

  // The COLLIDER is the track that gets closest to own-ship near contact (it
  // physically made contact). Ambient berthed boats stay at their fixed berth
  // range — so keying on "closest near contact" isolates the collider from the
  // dense sailing-basin background (which would otherwise trip Entered trivially).
  std::map<std::uint64_t, double> min_range_near_contact;  // per confirmed id
  double closest_range_in_window = std::numeric_limits<double>::infinity();

  // Feed scan-by-scan (measurements sharing a timestamp are one radar scan),
  // then evaluate CPA at that time.
  std::size_t i = 0;
  while (i < meas.size()) {
    const Timestamp scan_t = meas[i].time;
    std::vector<Measurement> scan;
    while (i < meas.size() && meas[i].time == scan_t) scan.push_back(meas[i++]);
    tracker.processBatch(scan);
    cpa.evaluate(scan_t);

    const double t = scan_t.seconds();
    const bool in_window = t >= clip_start_unix + 30.0 && t <= contact_unix + 5.0;
    const bool near_contact = t >= contact_unix - 8.0 && t <= contact_unix + 3.0;
    for (const Track& tr : manager.tracks()) {
      if (tr.status != TrackStatus::Confirmed || tr.state.size() < 2) continue;
      const double r =
          (Eigen::Vector2d(tr.state(0), tr.state(1)) - own_enu).norm();
      if (in_window) closest_range_in_window = std::min(closest_range_in_window, r);
      if (near_contact) {
        auto it = min_range_near_contact.find(tr.id.value);
        if (it == min_range_near_contact.end())
          min_range_near_contact[tr.id.value] = r;
        else
          it->second = std::min(it->second, r);
      }
    }
  }

  // Identify the collider = the confirmed track closest to own-ship near contact.
  std::uint64_t collider_id = 0;
  double collider_min_range = std::numeric_limits<double>::infinity();
  for (const auto& [id, r] : min_range_near_contact)
    if (r < collider_min_range) { collider_min_range = r; collider_id = id; }

  // The collider's OWN first Entered transition (not an ambient boat's).
  double collider_entered_unix = std::numeric_limits<double>::infinity();
  for (const auto& e : risk.events)
    if (e.transition == CollisionRiskTransition::Entered &&
        e.other.value == collider_id)
      collider_entered_unix = std::min(collider_entered_unix, e.time.seconds());

  std::cout << "\n=== close_approach real-data collision alarm ===\n"
            << "  closest confirmed-track range in window: "
            << closest_range_in_window << " m\n"
            << "  collider id=" << collider_id
            << " min-range-near-contact=" << collider_min_range << " m\n"
            << "  collider Entered lead: "
            << (std::isfinite(collider_entered_unix)
                    ? (contact_unix - collider_entered_unix)
                    : -1.0)
            << " s before contact\n" << std::flush;

  // (a) A confirmed track closes onto the collider during the approach.
  // #24: the old 15 m bar sat BELOW the documented 17 m nearest radar return, so
  // passing required the EKF estimate to lead inside its own last plot — a
  // toolchain/association-sensitive knife-edge. Use a physically-justified bar
  // with real margin: 25 m is well inside the 50 m CPA ring (unambiguously a
  // close pass, distinct from ambient berthed boats) and above the nearest
  // return, so a small estimator/gate shift no longer flips it. Measured ~10 m.
  ASSERT_NE(collider_id, 0u) << "no confirmed track near own-ship at contact";
  EXPECT_LT(collider_min_range, 25.0)
      << "the closest track never actually closed onto own-ship: "
      << collider_min_range << " m";

  // (b) CpaEvaluator fired Entered FOR THE COLLIDER, before contact, with usable
  // lead — the first real-data collision alarm.
  ASSERT_TRUE(std::isfinite(collider_entered_unix))
      << "CpaEvaluator never fired Entered on the closing dinghy";
  EXPECT_LT(collider_entered_unix, contact_unix)
      << "the collider's Entered fired at/after contact — no lead time";
  EXPECT_GE(contact_unix - collider_entered_unix, 2.0)
      << "less than 2 s of collision-alarm lead time on the collider";
}
