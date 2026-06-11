#include <gtest/gtest.h>

#include <memory>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/types/Track.hpp"

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::MhtTracker;
using navtracker::Timestamp;
using navtracker::TrackStatus;

namespace {

Measurement posMeas(double x, double y, double t) {
  Measurement m;
  m.time = Timestamp::fromSeconds(t);
  m.model = MeasurementModel::Position2D;
  m.value = Eigen::Vector2d(x, y);
  m.covariance = Eigen::Matrix2d::Identity() * 1.0;
  m.source_id = "t";
  return m;
}

MhtTracker::Config ipdaConfig() {
  MhtTracker::Config cfg;
  cfg.use_ipda_lifecycle = true;
  cfg.use_visibility = false;  // plain IPDA; VIMM tests enable it
  cfg.ipda_init_existence = 0.5;
  cfg.ipda_confirm_threshold = 0.9;
  cfg.ipda_delete_threshold = 0.05;
  cfg.ipda_persistence = 0.99;
  cfg.ipda_gate_probability_mass = 0.99;
  cfg.probability_of_detection = 0.9;
  cfg.clutter_density = 1e-4;
  // Loosen the score-based delete so we isolate IPDA lifecycle.
  cfg.score_delete_threshold = -1e9;
  return cfg;
}

}  // namespace

// IPDA lifecycle — a consistent run of well-fitting hits drives the
// best leaf's existence_probability above the confirm threshold, so the
// emitted track is reported Confirmed. Catches: (a) the recursion is
// wired (existence updates each scan), and (b) the confirm/tentative
// status flip reads existence (not M-of-N hits).
TEST(MhtIpdaLifecycle, ConsistentHitsDriveConfirmation) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker mht(ekf, ipdaConfig());

  TrackStatus last_status = TrackStatus::Tentative;
  for (int i = 1; i <= 8; ++i) {
    mht.processBatch({posMeas(static_cast<double>(i) * 5.0, 0.0,
                              static_cast<double>(i))});
    ASSERT_EQ(mht.tracks().size(), 1u);
    last_status = mht.tracks()[0].status;
  }
  EXPECT_EQ(last_status, TrackStatus::Confirmed)
      << "IPDA should confirm a clean target via existence ≥ "
         "ipda_confirm_threshold";
}

// IPDA lifecycle — a sustained run of misses (track leaves the gate)
// drops existence below the delete threshold and the tree is dropped.
// Catches: tree-delete gate honours IPDA existence.
TEST(MhtIpdaLifecycle, SustainedMissesDeleteTrack) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg = ipdaConfig();
  // Tighten the gate so the synthetic "missed scan" measurements
  // really miss (otherwise the tracker keeps gating them as hits).
  cfg.gate_threshold = 9.0;
  // Confirm threshold lower so the seed leaf is plausible before
  // we start starving it.
  cfg.ipda_confirm_threshold = 0.6;
  MhtTracker mht(ekf, cfg);

  // Seed two hits to establish the track.
  mht.processBatch({posMeas(0.0, 0.0, 0.0)});
  mht.processBatch({posMeas(0.0, 0.0, 1.0)});
  ASSERT_EQ(mht.treeCount(), 1u);

  // Now feed clutter far outside the gate — no measurement gates to
  // this tree → every scan is a miss for it. Existence should decay.
  for (int i = 0; i < 30; ++i) {
    mht.processBatch({posMeas(50000.0 + i, 50000.0,
                              2.0 + static_cast<double>(i))});
  }
  // Tree should have been deleted by IPDA existence (clutter spawns
  // its own ephemeral trees but none of them is the original).
  // The original track's id was 1; any survivor would re-emit it.
  for (const auto& tr : mht.tracks()) {
    EXPECT_NE(tr.id.value, 1u)
        << "Original track should be IPDA-deleted after a long miss stretch";
  }
}

// VIMM — under sustained misses with visibility on, existence decays
// markedly less than plain IPDA because the (1 − v) "hidden" channel
// soaks up the miss. Direct comparison: same scan sequence, two
// configs.
TEST(MhtVimmRecursion, VisibilityShieldsExistenceDuringObscuration) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);

  MhtTracker::Config cfg_ipda = ipdaConfig();
  MhtTracker::Config cfg_vimm = ipdaConfig();
  cfg_vimm.use_visibility = true;
  cfg_vimm.visibility_init = 1.0;
  cfg_vimm.visibility_persistence = 0.85;  // visible-→-visible
  cfg_vimm.visibility_recovery = 0.2;      // hidden-→-visible
  // Disable the delete gate so we can compare existence trajectories
  // without one config dropping the tree mid-run.
  cfg_ipda.ipda_delete_threshold = 0.0;
  cfg_vimm.ipda_delete_threshold = 0.0;
  // Tight gate so "far" measurements really miss.
  cfg_ipda.gate_threshold = 9.0;
  cfg_vimm.gate_threshold = 9.0;

  MhtTracker tracker_ipda(ekf, cfg_ipda);
  MhtTracker tracker_vimm(ekf, cfg_vimm);

  // Establish each track with a few hits.
  for (int i = 0; i < 3; ++i) {
    const double t = static_cast<double>(i);
    tracker_ipda.processBatch({posMeas(0.0, 0.0, t)});
    tracker_vimm.processBatch({posMeas(0.0, 0.0, t)});
  }

  // Long obscured stretch — clutter measurements far away so every
  // scan is a miss for the target track.
  for (int i = 0; i < 10; ++i) {
    const double t = 3.0 + i;
    tracker_ipda.processBatch({posMeas(50000.0 + i, 0.0, t)});
    tracker_vimm.processBatch({posMeas(50000.0 + i, 0.0, t)});
  }

  // Both trackers must still own the original track (delete gate off).
  ASSERT_FALSE(tracker_ipda.tracks().empty());
  ASSERT_FALSE(tracker_vimm.tracks().empty());

  const auto find_orig = [](const std::vector<navtracker::Track>& trs) {
    for (const auto& tr : trs)
      if (tr.id.value == 1u) return &tr;
    return static_cast<const navtracker::Track*>(nullptr);
  };
  const navtracker::Track* ipda_orig = find_orig(tracker_ipda.tracks());
  const navtracker::Track* vimm_orig = find_orig(tracker_vimm.tracks());
  ASSERT_NE(ipda_orig, nullptr);
  ASSERT_NE(vimm_orig, nullptr);
  // The substantive claim: under the same miss sequence, VIMM's
  // existence > IPDA's existence because the (1−v) hidden channel
  // absorbs miss likelihood. And visibility itself drops below 1.
  EXPECT_GT(vimm_orig->existence_probability,
            ipda_orig->existence_probability)
      << "VIMM should shield existence vs IPDA under sustained obscuration";
  EXPECT_LT(vimm_orig->visibility_given_exists, 1.0)
      << "VIMM visibility should decay across the obscured stretch";
  EXPECT_DOUBLE_EQ(ipda_orig->visibility_given_exists, 1.0)
      << "Plain IPDA (no use_visibility) should leave v at its sentinel";
}

// Hysteresis — a confirmed track whose existence dips below the confirm
// threshold but stays above the demote threshold remains Confirmed;
// only crossing the demote threshold flips it back to Tentative (and
// re-confirmation then requires the full confirm threshold again).
// Rationale: without hysteresis a single shallow evidence dip demotes a
// healthy track for a scan — on real multi-rate data that produces
// lifecycle flicker (cardinality holes) even when the track is never
// in doubt.
TEST(MhtIpdaLifecycle, HysteresisKeepsConfirmedThroughShallowDip) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg = ipdaConfig();
  cfg.gate_threshold = 9.0;
  // P_D = 0.5 → per-miss existence decay is gentle, so r walks down
  // *through* the (demote, confirm) band over several scans instead of
  // jumping across it (P_D = 0.9 goes 0.92 → 0.51 in one miss).
  cfg.probability_of_detection = 0.5;
  cfg.ipda_confirm_threshold = 0.9;
  cfg.ipda_demote_threshold = 0.6;
  MhtTracker mht(ekf, cfg);

  // Establish + confirm with clean 1 Hz hits.
  double t = 0.0;
  for (int i = 0; i < 5; ++i, t += 1.0) {
    mht.processBatch({posMeas(0.0, 0.0, t)});
  }
  const auto find1 = [&]() -> const navtracker::Track* {
    for (const auto& tr : mht.tracks())
      if (tr.id.value == 1u) return &tr;
    return nullptr;
  };
  ASSERT_NE(find1(), nullptr);
  ASSERT_EQ(find1()->status, TrackStatus::Confirmed);

  // Miss scans (far measurement keeps the scan clock ticking). Walk r
  // down into the hysteresis band — Confirmed must hold while
  // r ∈ [demote, confirm).
  bool saw_band_scan = false;
  for (int i = 0; i < 20; ++i, t += 1.0) {
    mht.processBatch({posMeas(50000.0 + 100.0 * i, 50000.0, t)});
    const navtracker::Track* tr = find1();
    ASSERT_NE(tr, nullptr);
    const double r = tr->existence_probability;
    if (r >= 0.6 && r < 0.9) {
      saw_band_scan = true;
      EXPECT_EQ(tr->status, TrackStatus::Confirmed)
          << "scan " << i << ": r=" << r
          << " is in the hysteresis band — must stay Confirmed";
    }
    if (r < 0.6) {
      EXPECT_EQ(tr->status, TrackStatus::Tentative)
          << "scan " << i << ": r=" << r << " below demote — Tentative";
      break;
    }
  }
  EXPECT_TRUE(saw_band_scan) << "test never exercised the band — adjust P_D";

  // Re-acquisition: hits drive r back ≥ confirm → Confirmed again.
  for (int i = 0; i < 4; ++i, t += 1.0) {
    mht.processBatch({posMeas(0.0, 0.0, t)});
  }
  const navtracker::Track* tr = find1();
  ASSERT_NE(tr, nullptr);
  EXPECT_GE(tr->existence_probability, 0.9);
  EXPECT_EQ(tr->status, TrackStatus::Confirmed);
}

// Demote == confirm reproduces the pre-hysteresis behaviour exactly:
// status is a pure threshold readout of r each scan.
TEST(MhtIpdaLifecycle, DemoteEqualToConfirmIsLegacyBehaviour) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  const EkfEstimator ekf(motion, 5.0);
  MhtTracker::Config cfg = ipdaConfig();
  cfg.gate_threshold = 9.0;
  cfg.probability_of_detection = 0.5;
  cfg.ipda_confirm_threshold = 0.9;
  cfg.ipda_demote_threshold = 0.9;  // no band
  MhtTracker mht(ekf, cfg);

  double t = 0.0;
  for (int i = 0; i < 5; ++i, t += 1.0) {
    mht.processBatch({posMeas(0.0, 0.0, t)});
  }
  // Walk r below confirm; with no band the first scan with r < 0.9
  // must already be Tentative.
  for (int i = 0; i < 20; ++i, t += 1.0) {
    mht.processBatch({posMeas(50000.0 + 100.0 * i, 50000.0, t)});
    for (const auto& tr : mht.tracks()) {
      if (tr.id.value != 1u) continue;
      if (tr.existence_probability < 0.9) {
        EXPECT_EQ(tr.status, TrackStatus::Tentative);
        return;  // first sub-confirm scan checked — done
      }
    }
  }
  FAIL() << "r never dropped below confirm";
}
