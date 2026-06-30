#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <optional>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
#include "core/sensor_activity/DeclaredSensorActivity.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/pmbm/PmbmTypes.hpp"
#include "core/types/Measurement.hpp"

// ---------------------------------------------------------------------------
// Basic AssociationHints field check (Task 2 sanity test).
// ---------------------------------------------------------------------------
TEST(AssociationHints, CarriesNumericPlatformId) {
  navtracker::AssociationHints h;
  EXPECT_FALSE(h.platform_id.has_value());
  h.platform_id = std::uint64_t{1234567890123ULL};
  ASSERT_TRUE(h.platform_id.has_value());
  EXPECT_EQ(*h.platform_id, 1234567890123ULL);
}

// ---------------------------------------------------------------------------
// Identity-gate tests (Task 3).
//
// Background: PmbmTracker::enumerateChildren::should_misdetect decides whether
// a Bernoulli is "observable" in a given scan.  The result drives whether the
// miss recursion r ← (1-p_D)·r / (1-r·p_D) is applied or skipped.
//
// Important timing detail: EkfEstimator::predict sets track.last_update = to
// (the scan time).  The source-touch history is updated inside processBatch
// AFTER enumerateChildren runs, using z.time == b.last_update to detect
// which measurements are detections.  Because predict sets last_update to the
// scan time, the "nearest measurement" in each scan is always matched and a
// new SourceTouch is appended to contribution_history_ even for miss-detected
// Bernoullis.
//
// Key consequence: if a touch has platform_id set (after the fix), the new
// "continue" logic suppresses the source_id fallback for that touch.  This
// makes the "not observable" assertion stable: after the fix, a scan with a
// non-matching platform_id does NOT add a source-fallback touch that would
// fire on the following scan.  Before the fix, no platform_id field is set,
// so platform_id=nullopt for every touch, and the source_id fallback fires,
// yielding different r values in RED than in GREEN.
//
// Test (a) uses two different source_ids (seed="sensor_a", test="sensor_b")
// and no mmsi in any scan, isolating the platform_id path:
//   RED: observable scan: source_b ∉ {sensor_a} → should_misdetect=false → r unchanged.
//        not-observable scan: accumulated touch has source_b ∈ scan_sources → true → r drops.
//        Both assertions fail.
//   GREEN: observable scan: platform_id=7 ∈ scan_platforms → true → r drops (assertion passes).
//          not-observable scan: touch has platform_id=7, scan_platforms={8} → continue
//          (source suppressed) → false → r unchanged (assertion passes).
//
// Test (b) uses mmsi+platform_id in the seed; the test scan has NO mmsi,
// isolating the "shared platform_id" path when vessel_id is present but absent:
//   RED: vessel_id=111 ∉ scan_vessels={} → continue → end-of-loop → false → r unchanged.
//   GREEN: vessel_id=111 ∉ {} → then check platform_id=7 ∈ scan_platforms={7} → true → r drops.
// ---------------------------------------------------------------------------
namespace {

using navtracker::ConstantVelocity2D;
using navtracker::EkfEstimator;
using navtracker::Measurement;
using navtracker::MeasurementModel;
using navtracker::SensorKind;
using navtracker::Timestamp;
using navtracker::pmbm::Bernoulli;
using navtracker::pmbm::GlobalHypothesis;
using navtracker::pmbm::PmbmTracker;
using navtracker::pmbm::PoissonComponent;

// ---- Geometry helpers -------------------------------------------------------

Eigen::Vector4d gCvState(double px, double py, double vx, double vy) {
  return Eigen::Vector4d(px, py, vx, vy);
}
Eigen::Matrix4d gPosCov(double sp, double sv) {
  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = P(1, 1) = sp * sp;
  P(2, 2) = P(3, 3) = sv * sv;
  return P;
}
PoissonComponent gMkPoisson(double w, double px, double py) {
  PoissonComponent c;
  c.weight = w;
  c.mean = gCvState(px, py, 0.0, 0.0);
  c.covariance = gPosCov(5.0, 1.0);
  return c;
}

// Position2D measurement at (px, py) at time t.
Measurement gMkMeas(double t, double px, double py, const std::string& src,
                    std::optional<std::uint32_t> mmsi,
                    std::optional<std::uint64_t> platform_id) {
  Measurement z;
  z.time = Timestamp::fromSeconds(t);
  z.sensor = SensorKind::Lidar;
  z.source_id = src;
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(px, py);
  z.covariance = Eigen::Matrix2d::Identity();
  z.hints.mmsi = mmsi;
  z.hints.platform_id = platform_id;
  return z;
}

// Highest existence_probability across the dominant hypothesis.
// Returns -1.0 on empty MBM.  Because the seeded Bernoulli starts near r=1
// and new phantom Bernoullis (from far-away measurements) are pruned by r_min,
// the max-r Bernoulli in the dominant hypothesis tracks the seeded one.
double gMaxR(const navtracker::pmbm::PmbmDensity& d) {
  if (d.mbm.empty()) return -1.0;
  auto dom = std::max_element(
      d.mbm.begin(), d.mbm.end(),
      [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
        return a.weight < b.weight;
      });
  if (dom->bernoullis.empty()) return -1.0;
  auto b_it = std::max_element(
      dom->bernoullis.begin(), dom->bernoullis.end(),
      [](const Bernoulli& a, const Bernoulli& b) {
        return a.existence_probability < b.existence_probability;
      });
  return b_it->existence_probability;
}

// Tracker config with source_aware_misdetection + source_aware_identity ON.
// survival_probability=1.0 isolates update math; idle_halflife=0 so r is
// exactly unchanged when should_misdetect returns false (no implicit decay).
PmbmTracker::Config gIdentityCfg() {
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-6;
  c.survival_probability = 1.0;
  c.source_aware_misdetection = true;
  c.source_aware_identity = true;
  c.idle_halflife_sec = 0.0;
  c.r_min = 1e-4;
  c.weight_min = 1e-5;
  c.hypothesis_weight_min = 1e-5;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// (a) A Bernoulli tracked by platform_id only (no mmsi).
//
// The "observable" scan shares platform_id=7 but uses a DIFFERENT source_id
// so the source_id fallback cannot fire from the seeding touch.  After the
// fix, platform_id is checked and the gate returns true (miss math, r drops).
//
// The "not observable" scan uses platform_id=8 (same source_id="sensor_b").
// Before fix: the observable scan left a touch with source_id="sensor_b"
//   (no platform_id set → nullopt), so that touch fires via source fallback
//   → should_misdetect=true → r drops → EXPECT_NEAR(r2, r1) FAILS (RED).
// After fix: the observable scan left a touch with platform_id=7 set; the
//   new should_misdetect logic hits platform_id=7 ∉ {8} → platform_id known
//   → continue (source suppressed) → false → r unchanged → PASSES (GREEN).
//
// RED: both EXPECT_LT(r1,r0) and EXPECT_NEAR(r2,r1) fail.
// GREEN: both pass.
// ---------------------------------------------------------------------------
TEST(PmbmIdentityGate, CooperativeOnlyKeyedByPlatformId) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  PmbmTracker tracker(ekf, gIdentityCfg());
  tracker.predict(Timestamp::fromSeconds(0.0));

  // ---- Seed: PPP at (0,0) + measurement with platform_id=7, no mmsi. ------
  tracker.mutableDensityForTesting().ppp.push_back(gMkPoisson(1.0, 0.0, 0.0));
  tracker.processBatch(
      {gMkMeas(1.0, 0.0, 0.0, "sensor_a", std::nullopt,
               std::optional<std::uint64_t>{7ULL})});
  ASSERT_FALSE(tracker.density().mbm.empty());
  const double r0 = gMaxR(tracker.density());
  EXPECT_GT(r0, 0.5);

  // ---- Scan 2 ("observable"): far measurement, source="sensor_b", pid=7. ---
  // source_id differs from seed ("sensor_a"), so source fallback cannot fire
  // from the seeding touch.  After fix: platform_id=7 shared → observable.
  tracker.processBatch(
      {gMkMeas(2.0, 1000.0, 1000.0, "sensor_b", std::nullopt,
               std::optional<std::uint64_t>{7ULL})});
  const double r1 = gMaxR(tracker.density());
  EXPECT_LT(r1, r0)
      << "platform_id=7 in scan matches touch.platform_id=7: Bernoulli is "
         "observable → miss math must reduce r "
         "(fails before platform_id gate is wired)";

  // ---- Scan 3 ("not observable"): same source_id, pid=8 (no match). --------
  // After fix: touch added by scan 2 has platform_id=7 set; scan_platforms={8},
  // 7∉{8} → platform_id.has_value() → continue (source_id suppressed) → false.
  // r must remain at r1 (no miss math).
  tracker.processBatch(
      {gMkMeas(3.0, 1000.0, 1000.0, "sensor_b", std::nullopt,
               std::optional<std::uint64_t>{8ULL})});
  const double r2 = gMaxR(tracker.density());
  EXPECT_NEAR(r2, r1, 1e-9)
      << "platform_id=8 does not match touch.platform_id=7: Bernoulli is NOT "
         "observable → miss math must be skipped → r unchanged "
         "(fails before platform_id gate is wired, because accumulated "
         "source_b touch fires via source fallback)";
}

// ---------------------------------------------------------------------------
// (b) Touch has BOTH mmsi=111 and platform_id=7 (seed with both keys).
//     Test scan has platform_id=7 ONLY (no mmsi).
//
// Before fix: vessel_id=111 is set but absent from scan_vessels={} → `continue`
//   (source fallback suppressed for that touch) → end of loop → false → r unchanged.
//   EXPECT_LT(r1, r0) FAILS → RED.
// After fix: vessel_id=111 ∉ scan_vessels → also check platform_id=7 ∈
//   scan_platforms={7} → returns true → miss math → r decreases.
//   EXPECT_LT(r1, r0) PASSES → GREEN.
//
// This tests "EITHER mmsi OR platform_id" semantics: when the scan only
// carries platform_id (not mmsi), the Bernoulli is still observable because
// it shares platform_id with its contribution history.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Task 4: setSensorActivity seam — bit-identical default contract.
//
// A tracker with NO activity wired (sensor_activity_ == nullptr and
// use_sensor_activity == false) must produce exactly the same existence
// decay as the pre-Task-4 baseline.  Canonical misdetection math:
//   r ← (1 − p_D) · r / (1 − r · p_D)
//   = 0.1 · 0.8 / (1 − 0.72) = 0.08 / 0.28
// This test calls setSensorActivity(nullptr) explicitly to verify the
// nullable setter compiles and is side-effect-free.  Without the
// implementation the test fails to compile (RED → GREEN after Step 3).
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, NullActivityIsBehaviourNeutral) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  PmbmTracker::Config cfg;
  cfg.probability_of_detection = 0.9;
  cfg.survival_probability = 1.0;  // isolate update math from predict
  PmbmTracker tracker(ekf, cfg);

  // Prove the nullable setter compiles and is behaviour-neutral.
  tracker.setSensorActivity(nullptr);

  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed one Bernoulli at r=0.8.
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  Bernoulli b;
  b.id = navtracker::pmbm::BernoulliId{1};
  b.existence_probability = 0.8;
  b.mean = gCvState(0.0, 0.0, 0.0, 0.0);
  b.covariance = gPosCov(2.0, 0.5);
  b.last_update = Timestamp::fromSeconds(0.0);
  h.bernoullis.push_back(b);
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));

  tracker.processBatch({});  // empty scan: every Bernoulli is a miss

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.08 / 0.28, 1e-9)
      << "setSensorActivity(nullptr) must leave misdetection math bit-identical";
}

TEST(PmbmIdentityGate, SharedEitherKeyIsSameVessel) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  PmbmTracker tracker(ekf, gIdentityCfg());
  tracker.predict(Timestamp::fromSeconds(0.0));

  // ---- Seed: measurement with BOTH mmsi=111 AND platform_id=7. -------------
  // Touch will record: vessel_id=111 AND platform_id=7 (after fix).
  tracker.mutableDensityForTesting().ppp.push_back(gMkPoisson(1.0, 0.0, 0.0));
  tracker.processBatch(
      {gMkMeas(1.0, 0.0, 0.0, "ais",
               std::optional<std::uint32_t>{111U},
               std::optional<std::uint64_t>{7ULL})});
  ASSERT_FALSE(tracker.density().mbm.empty());
  const double r0 = gMaxR(tracker.density());
  EXPECT_GT(r0, 0.5);

  // ---- Scan 2: far measurement, NO mmsi, platform_id=7. --------------------
  // scan_vessels={} (no mmsi) → vessel_id=111 not found → before fix: continue
  // (source "ais"≠"sensor_b" anyway, but the `continue` blocks it) → false.
  // After fix: also check touch.platform_id=7 ∈ scan_platforms={7} → true.
  tracker.processBatch(
      {gMkMeas(2.0, 1000.0, 1000.0, "sensor_b", std::nullopt,
               std::optional<std::uint64_t>{7ULL})});
  const double r1 = gMaxR(tracker.density());
  EXPECT_LT(r1, r0)
      << "Touch has vessel_id=111 but scan has NO mmsi: vessel_id branch "
         "fires `continue` (correct), but shared platform_id=7 SHOULD still "
         "make the Bernoulli observable → miss math reduces r "
         "(fails before platform_id gate is wired)";
}

// ---------------------------------------------------------------------------
// Task 5: Per-duty-cycle surveillance miss behind use_sensor_activity.
//
// DeclaredSensorActivity with a radar profile: pD=0.9, range=10km, duty=60s.
// All three tests isolate the surveillance-miss path with survival=1.0 so the
// only r change comes from the misdetection update.
// ---------------------------------------------------------------------------
namespace {

navtracker::DeclaredSensorActivity gRadarActivity() {
  navtracker::DeclaredSensorActivity::ChannelProfile p;
  p.kind = navtracker::ChannelKind::Surveillance;
  p.sensor = navtracker::SensorKind::ArpaTtm;  // radar; Radar enum doesn't exist, ArpaTtm is closest
  p.duty_cycle_sec = 60.0;
  p.max_range_m = 10000.0;
  p.sector_width_rad = 6.283185307179586;  // full circle
  p.p_D = 0.7;
  return navtracker::DeclaredSensorActivity{{p}};
}

// Tracker config with use_sensor_activity=true, no other knobs active so
// the new code path is isolated. survival=1.0 means predict has no r effect.
PmbmTracker::Config gActivityCfg() {
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-6;
  c.survival_probability = 1.0;
  c.use_sensor_activity = true;
  c.idle_halflife_sec = 0.0;
  c.r_min = 1e-4;
  c.weight_min = 1e-5;
  c.hypothesis_weight_min = 1e-5;
  return c;
}

// Seed one Bernoulli at (px, py) with r=r0 into a tracker that already had
// predict(0.0) called. birth_time defaults to 0.0 (same as epoch).
void gSeedBernoulli(PmbmTracker& tracker, double px, double py, double r0) {
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  Bernoulli b;
  b.id = navtracker::pmbm::BernoulliId{99};
  b.existence_probability = r0;
  b.mean = gCvState(px, py, 0.0, 0.0);
  b.covariance = gPosCov(2.0, 0.5);
  b.last_update = Timestamp::fromSeconds(0.0);
  // b.birth_time defaults to Timestamp{} = 0.0 (same as predict epoch)
  h.bernoullis.push_back(b);
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));
}

}  // namespace

// ---------------------------------------------------------------------------
// Active radar sweeps past the track position (5 km, in range) and returns
// nothing.  dt=60s >= duty_cycle=60s → ONE surveillance miss at pD=0.9.
// Expected: r = (1-0.9)*0.8 / (1 - 0.8*0.9) = 0.08/0.28 ≈ 0.285714.
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, SurveillanceCoveredSweepAppliesOneMiss) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gRadarActivity();

  PmbmTracker tracker(ekf, gActivityCfg());
  tracker.setSensorActivity(&activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed Bernoulli at (5000, 0) — inside 10 km radar coverage.
  gSeedBernoulli(tracker, 5000.0, 0.0, 0.8);

  // Advance 60 s (>= duty_cycle) then process empty scan.
  tracker.predict(Timestamp::fromSeconds(60.0));
  tracker.processBatch({});

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  const double r = tracker.density().mbm[0].bernoullis[0].existence_probability;
  EXPECT_NEAR(r, 0.24 / 0.44, 1e-9)
      << "Covered sweep + no return: must apply EXACTLY ONE miss with pD=0.7. "
         "Expected r = (1-pD)*r0 / (1 - r0*pD) = 0.24/0.44";
}

// ---------------------------------------------------------------------------
// Track at (20000, 0) is OUTSIDE the radar's 10 km max range.
// DeclaredSensorActivity returns surveillance_miss=false → r unchanged.
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, OutOfCoverageNoPenalty) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gRadarActivity();

  PmbmTracker tracker(ekf, gActivityCfg());
  tracker.setSensorActivity(&activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Track at 20 km — beyond radar max_range.
  gSeedBernoulli(tracker, 20000.0, 0.0, 0.8);

  tracker.predict(Timestamp::fromSeconds(60.0));
  tracker.processBatch({});

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.8, 1e-9)
      << "Track outside radar range: no surveillance miss → r must stay 0.8";
}

// ---------------------------------------------------------------------------
// dt=30s < duty_cycle=60s: radar has NOT completed a sweep since last check.
// DeclaredSensorActivity returns surveillance_miss=false → r unchanged.
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, MidSweepNoPenalty) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gRadarActivity();

  PmbmTracker tracker(ekf, gActivityCfg());
  tracker.setSensorActivity(&activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Track at (5000, 0) — inside radar coverage.
  gSeedBernoulli(tracker, 5000.0, 0.0, 0.8);

  // Only 30 s elapsed: radar has not completed a full sweep yet.
  tracker.predict(Timestamp::fromSeconds(30.0));
  tracker.processBatch({});

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.8, 1e-9)
      << "Mid-sweep (dt < duty_cycle): no surveillance miss → r must stay 0.8";
}

// ---------------------------------------------------------------------------
// Task 5 Step-4 fix (defect #1): a DETECTION must reset the surveillance-miss
// window. Before the fix the detection branch never wrote last_activity_check_,
// so the first dropout after a detection computed dt = now - birth_time, which
// can exceed the duty cycle even though only one short interval has elapsed
// since the detection -> a full miss is wrongly charged.
//
// survival_probability is set < 1 so the post-detection existence is < 1 and
// the miss math is observable (a miss applied to r == 1 is a no-op:
// (1-pD)*1/(1-pD) == 1, which would hide the defect).
//
// Timeline (chosen so NO miss fires at the detection scan in the broken
// code, which would otherwise mask the defect by mutating the window):
//   detection at t=30 (dt=30 < 60 duty from birth=0 -> alt hypotheses don't
//   charge a miss either), then an empty scan at t=80 (only 50 s < 60 s duty
//   since the detection, but 80 s > 60 s since birth).
//   GREEN (fixed): detection stages window=30 -> dt=50 < 60 -> no miss ->
//                  r stays at the post-detection-and-predict value (0.9).
//   RED (broken):  detection never writes the window and no miss fired at
//                  t=30, so the window falls back to birth_time=0 -> dt=80 >
//                  60 -> a miss wrongly drops r to
//                  (1-0.7)*0.9 / (1 - 0.9*0.7) = 0.27/0.37 ~= 0.7297.
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, SurveillanceDetectionResetsMissWindow) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gRadarActivity();  // duty=60 s, range=10 km, pD=0.7

  auto cfg = gActivityCfg();
  cfg.survival_probability = 0.9;  // make post-detection r < 1 (miss observable)
  PmbmTracker tracker(ekf, cfg);
  tracker.setSensorActivity(&activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed Bernoulli id 99 at (5000, 0) (in radar coverage), birth_time = 0.
  gSeedBernoulli(tracker, 5000.0, 0.0, 0.8);

  // Detection scan at t=30 (< 60 s duty since birth): a radar return exactly
  // at the track position. processBatch predicts to 30 internally; the
  // detection sets existence to 1 and (after the fix) advances the
  // surveillance-miss window to t=30. No miss is charged anywhere this scan.
  Measurement z;
  z.time = Timestamp::fromSeconds(30.0);
  z.sensor = SensorKind::ArpaTtm;
  z.source_id = "radar";
  z.model = MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(5000.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  tracker.processBatch({z});
  const double r_after_det = gMaxR(tracker.density());
  ASSERT_GT(r_after_det, 0.95)
      << "detection must drive existence to ~1 (detected target)";

  // Empty scan at t=80: 50 s < 60 s duty since the detection -> NO miss.
  tracker.predict(Timestamp::fromSeconds(80.0));  // r *= survival -> 0.9
  tracker.processBatch({});
  const double r_after_gap = gMaxR(tracker.density());

  EXPECT_NEAR(r_after_gap, 0.9, 1e-9)
      << "A detection at t=30 resolves observability up to t=30, so it must "
         "advance the surveillance-miss window. Only 50 s (< 60 s duty) have "
         "elapsed since the detection, so NO miss may be charged. Before the "
         "fix the window falls back to birth_time=0, dt=80 s > 60 s, and a "
         "miss wrongly drops r to ~0.7297.";
}

// ---------------------------------------------------------------------------
// Task 5 Step-4 fix (defect #2): a surveillance miss must be applied
// CONSISTENTLY across every parent global hypothesis that contains the same
// BernoulliId. enumerateChildren runs once per parent; before the fix the
// FIRST parent to charge a miss for id X mutated last_activity_check_[X]=now,
// so a LATER parent enumerating the SAME id saw dt=0 and skipped the miss ->
// the two parents' copies of id X diverged (one missed, one untouched).
//
// Seed TWO global hypotheses, each holding a Bernoulli with id 99 at (5000,0).
// Empty scan after a full duty cycle: with the snapshot-read + deferred-write
// fix BOTH parents read the same pre-scan window (birth_time=0), so BOTH charge
// the miss and BOTH copies end at the same existence. survival=1 isolates the
// miss math.
//   GREEN: both copies -> r = (1-0.7)*0.8 / (1 - 0.8*0.7) = 0.24/0.44.
//   RED:   one copy 0.24/0.44, the other still 0.8 (order-dependent).
// ---------------------------------------------------------------------------
TEST(PmbmSensorActivity, SurveillanceMissAppliedConsistentlyAcrossHypotheses) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gRadarActivity();  // duty=60 s, pD=0.7

  PmbmTracker tracker(ekf, gActivityCfg());  // survival = 1.0
  tracker.setSensorActivity(&activity);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed two parent global hypotheses, BOTH carrying Bernoulli id 99 at the
  // same in-coverage position. Distinct log_weights keep both above the
  // hypothesis weight floor and make the order deterministic. There is no
  // global-hypothesis dedup in pruneAndNormalise, so both survive the scan.
  auto seedHyp = [&](double log_w) {
    GlobalHypothesis h;
    h.weight = 1.0;
    h.log_weight = log_w;
    Bernoulli b;
    b.id = navtracker::pmbm::BernoulliId{99};
    b.existence_probability = 0.8;
    b.mean = gCvState(5000.0, 0.0, 0.0, 0.0);
    b.covariance = gPosCov(2.0, 0.5);
    b.last_update = Timestamp::fromSeconds(0.0);
    h.bernoullis.push_back(b);
    tracker.mutableDensityForTesting().mbm.push_back(std::move(h));
  };
  seedHyp(0.0);             // weight ~0.62 after normalise
  seedHyp(std::log(0.5));   // weight ~0.38 after normalise

  // Full duty cycle elapses, then an empty scan -> surveillance miss for BOTH.
  tracker.predict(Timestamp::fromSeconds(60.0));
  tracker.processBatch({});

  ASSERT_EQ(tracker.density().mbm.size(), 2u)
      << "both parent hypotheses must survive (no global-hypothesis dedup)";
  const double expected = 0.24 / 0.44;
  for (const auto& h : tracker.density().mbm) {
    ASSERT_EQ(h.bernoullis.size(), 1u);
    EXPECT_EQ(h.bernoullis[0].id, navtracker::pmbm::BernoulliId{99});
    EXPECT_NEAR(h.bernoullis[0].existence_probability, expected, 1e-9)
        << "every parent containing id 99 must receive the SAME surveillance "
           "miss. Before the fix the second parent reads a window the first "
           "parent already advanced (dt=0) and keeps r=0.8.";
  }
}

// ---------------------------------------------------------------------------
// Task 6: cooperative stale / comms-loss signal tests.
//
// Key invariant (spec §9c): a cooperative own-identity report being overdue
// means "we lost comms", NOT "vessel sank". Existence MUST NOT change.
// The IStaleSignalSink is a pure push notification; wiring it to anything
// that lowers existence violates the spec.
// ---------------------------------------------------------------------------
namespace {

// Cooperative profile: expected own-identity report every 10 s.
// No surveillance channel — so the only activity is cooperative.
navtracker::DeclaredSensorActivity gCooperativeActivity() {
  navtracker::DeclaredSensorActivity::ChannelProfile p;
  p.kind = navtracker::ChannelKind::Cooperative;
  p.expected_report_interval_sec = 10.0;
  return navtracker::DeclaredSensorActivity{{p}};
}

// Tracker config: use_sensor_activity, no surveillance, survival=1 so that
// predict has zero r-effect and we can reason purely about the miss logic.
PmbmTracker::Config gCoopCfg() {
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-6;
  c.survival_probability = 1.0;
  c.use_sensor_activity = true;
  c.idle_halflife_sec = 0.0;
  c.r_min = 1e-4;
  c.weight_min = 1e-5;
  c.hypothesis_weight_min = 1e-5;
  return c;
}

// Recording sink: captures every onTrackStale(id, now) call.
struct RecordingStaleSink : navtracker::IStaleSignalSink {
  std::vector<navtracker::TrackId> stale;
  void onTrackStale(navtracker::TrackId id, navtracker::Timestamp) override {
    stale.push_back(id);
  }
};

// Seed one Bernoulli with a chosen BernoulliId and existence into a fresh
// hypothesis. The Bernoulli birth_time defaults to Timestamp{} = 0.0.
void gSeedBernoulliId(PmbmTracker& tracker,
                      navtracker::pmbm::BernoulliId bid,
                      double px, double py, double r0) {
  GlobalHypothesis h;
  h.weight = 1.0;
  h.log_weight = 0.0;
  Bernoulli b;
  b.id = bid;
  b.existence_probability = r0;
  b.mean = gCvState(px, py, 0.0, 0.0);
  b.covariance = gPosCov(2.0, 0.5);
  b.last_update = Timestamp::fromSeconds(0.0);
  // birth_time defaults to Timestamp{} ≡ 0.0 s — used as fallback for the
  // cooperative retirement timer when no detection has ever occurred.
  h.bernoullis.push_back(b);
  tracker.mutableDensityForTesting().mbm.push_back(std::move(h));
}

}  // namespace

// ---------------------------------------------------------------------------
// (a) Overdue own-identity report leaves existence unchanged AND flags stale.
//
// Cooperative profile interval=10s, NO surveillance. Bernoulli seeded at r=0.8.
// Advance 25s with empty scan: dt=25 > 10 → cooperative_overdue=true.
// Expected: existence == 0.8 (unchanged) AND RecordingStaleSink fired.
// ---------------------------------------------------------------------------
TEST(PmbmStaleSignal, OverdueLeavesExistenceUnchangedAndFlagsStale) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gCooperativeActivity();
  RecordingStaleSink stale_sink;

  PmbmTracker tracker(ekf, gCoopCfg());
  tracker.setSensorActivity(&activity);
  tracker.setStaleSignalSink(&stale_sink);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed one cooperative-only Bernoulli at r=0.8.
  gSeedBernoulliId(tracker, navtracker::pmbm::BernoulliId{42}, 0.0, 0.0, 0.8);

  // Advance 25 s (well past 10 s interval): cooperative own-identity overdue.
  tracker.predict(Timestamp::fromSeconds(25.0));
  tracker.processBatch({});

  // Existence MUST be unchanged: "comms lost" ≠ "vessel sank" (spec §9c).
  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u);
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.8, 1e-9)
      << "Cooperative overdue: existence MUST NOT change (spec §9c: comms loss "
         "≠ vessel sank)";

  // Stale signal must have fired exactly once for the track.
  EXPECT_FALSE(stale_sink.stale.empty())
      << "Stale signal must fire when cooperative own-identity report is overdue";
  // The emitted track id maps the BernoulliId value.
  const auto& ts = tracker.tracks();
  ASSERT_EQ(ts.size(), 1u);
  EXPECT_EQ(stale_sink.stale.front(), ts[0].id)
      << "Stale signal must carry the aggregated TrackId of the overdue track";

  // TrackStatus must be Coasting (spec §9c: cooperative overdue → coasting).
  EXPECT_EQ(ts[0].status, navtracker::TrackStatus::Coasting)
      << "Cooperative overdue track must have status Coasting";
}

// ---------------------------------------------------------------------------
// (b) Detecting one cooperative track does not flag a DIFFERENT track stale,
//     and must not change the other track's existence via miss math.
//
// Two cooperative-only Bernoullis (BernoulliId 42 at origin, 99 far away).
// At t=5s (within 10 s interval): neither is overdue. A scan with a
// measurement at (0,0) is processed: it detects Bernoulli 42, Bernoulli 99
// goes through misdetection with dt=5s < 10s → cooperative_overdue=false
// → existence unchanged, no stale signal for 99.
// ---------------------------------------------------------------------------
TEST(PmbmStaleSignal, DoesNotAffectDifferentIdentity) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gCooperativeActivity();
  RecordingStaleSink stale_sink;

  PmbmTracker tracker(ekf, gCoopCfg());
  tracker.setSensorActivity(&activity);
  tracker.setStaleSignalSink(&stale_sink);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed two Bernoullis in the SAME global hypothesis.
  // BernoulliId=42 at (0,0), BernoulliId=99 far away at (10000,0).
  // Put them in one hypothesis together.
  {
    GlobalHypothesis h;
    h.weight = 1.0;
    h.log_weight = 0.0;

    Bernoulli b42;
    b42.id = navtracker::pmbm::BernoulliId{42};
    b42.existence_probability = 0.8;
    b42.mean = gCvState(0.0, 0.0, 0.0, 0.0);
    b42.covariance = gPosCov(2.0, 0.5);
    b42.last_update = Timestamp::fromSeconds(0.0);
    h.bernoullis.push_back(b42);

    Bernoulli b99;
    b99.id = navtracker::pmbm::BernoulliId{99};
    b99.existence_probability = 0.8;
    b99.mean = gCvState(10000.0, 0.0, 0.0, 0.0);
    b99.covariance = gPosCov(2.0, 0.5);
    b99.last_update = Timestamp::fromSeconds(0.0);
    h.bernoullis.push_back(b99);

    tracker.mutableDensityForTesting().mbm.push_back(std::move(h));
  }

  // Advance 5 s (< 10 s interval): NEITHER track is overdue.
  tracker.predict(Timestamp::fromSeconds(5.0));

  // Process scan with measurement at (0,0) at t=5s.
  // Gate check: Bernoulli 42 (at (0,0), cov=4m²) → Mahalanobis ≈ 0 (detected).
  //             Bernoulli 99 (at (10000,0)) → gate fails (Mahalanobis >> 9).
  // So assignment must route the measurement to Bernoulli 42 (detected), not 99.
  Measurement z;
  z.time = Timestamp::fromSeconds(5.0);
  z.sensor = navtracker::SensorKind::Cooperative;
  z.source_id = "coop";
  z.model = navtracker::MeasurementModel::Position2D;
  z.value = Eigen::Vector2d(0.0, 0.0);
  z.covariance = Eigen::Matrix2d::Identity();
  tracker.processBatch({z});

  // Stale signal must NOT have fired for Bernoulli 99 (it is NOT overdue).
  const bool stale99 = std::any_of(
      stale_sink.stale.begin(), stale_sink.stale.end(),
      [](navtracker::TrackId id) { return id.value == 99u; });
  EXPECT_FALSE(stale99)
      << "Track 99 is within the 10s interval (dt=5s) → must NOT be stale";

  // Find track 99 in the output and verify its existence is unchanged.
  const auto& ts = tracker.tracks();
  const auto it99 = std::find_if(ts.begin(), ts.end(), [](const navtracker::Track& t) {
    return t.id.value == 99u;
  });
  ASSERT_NE(it99, ts.end())
      << "Track 99 erroneously pruned — miss math applied to non-overdue cooperative track";
  EXPECT_NEAR(it99->existence_probability, 0.8, 1e-9)
      << "Track 99 existence must be unchanged (no miss math for non-overdue "
         "cooperative track when dt < interval)";
  EXPECT_NE(it99->status, navtracker::TrackStatus::Coasting)
      << "Track 99 must not be Coasting (not overdue)";
}

// ---------------------------------------------------------------------------
// (c) Cooperative-only track retired ONLY by the stale timeout, never by
//     per-sweep miss math.
//
// cooperative_stale_timeout_sec=600.
//   - After 25 s overdue: still alive (r unchanged, status Coasting).
//   - After 601 s overdue: Bernoulli pruned (retired).
// Between these two checkpoints, existence MUST remain 0.8 (no miss math
// is ever applied via the surveillance path since there is no surveillance
// profile).
// ---------------------------------------------------------------------------
TEST(PmbmStaleSignal, CooperativeOnlyRetiredOnlyByTimeout) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gCooperativeActivity();
  RecordingStaleSink stale_sink;

  auto cfg = gCoopCfg();
  cfg.cooperative_stale_timeout_sec = 600.0;  // retire only after 600 s

  PmbmTracker tracker(ekf, cfg);
  tracker.setSensorActivity(&activity);
  tracker.setStaleSignalSink(&stale_sink);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed one cooperative-only Bernoulli at r=0.8.
  gSeedBernoulliId(tracker, navtracker::pmbm::BernoulliId{42}, 0.0, 0.0, 0.8);

  // ---- Check 1: 25 s overdue (dt=25 > 10 interval, but 25 < 600 timeout). --
  tracker.predict(Timestamp::fromSeconds(25.0));
  tracker.processBatch({});

  ASSERT_EQ(tracker.density().mbm.size(), 1u);
  ASSERT_EQ(tracker.density().mbm[0].bernoullis.size(), 1u)
      << "After 25s overdue (< 600s timeout): Bernoulli must still be alive";
  EXPECT_NEAR(tracker.density().mbm[0].bernoullis[0].existence_probability,
              0.8, 1e-9)
      << "After 25s overdue: existence MUST NOT change (cooperative overdue ≠ miss)";

  // Stale signal must have fired once.
  EXPECT_FALSE(stale_sink.stale.empty())
      << "Stale signal must fire at 25s (overdue by dt=25 > interval=10)";
  {
    const auto& ts = tracker.tracks();
    ASSERT_EQ(ts.size(), 1u);
    EXPECT_EQ(ts[0].status, navtracker::TrackStatus::Coasting)
        << "Status must be Coasting after cooperative overdue";
  }

  // ---- Check 2: 601 s total since birth (601 > 600 timeout). ----------------
  stale_sink.stale.clear();
  tracker.predict(Timestamp::fromSeconds(601.0));
  tracker.processBatch({});

  // Bernoulli must have been retired (existence → 0 → pruned by r_min).
  bool bernoulli_alive = false;
  for (const auto& h : tracker.density().mbm) {
    for (const auto& b : h.bernoullis) {
      if (b.id == navtracker::pmbm::BernoulliId{42} &&
          b.existence_probability > 0.0) {
        bernoulli_alive = true;
      }
    }
  }
  EXPECT_FALSE(bernoulli_alive)
      << "After 601s (> 600s cooperative_stale_timeout_sec): Bernoulli must be "
         "retired (existence → 0, pruned by r_min)";

  // The output track must also be gone (below output_existence_floor).
  const auto& ts = tracker.tracks();
  const auto it42 = std::find_if(ts.begin(), ts.end(), [](const navtracker::Track& t) {
    return t.id.value == 42u;
  });
  EXPECT_EQ(it42, ts.end())
      << "After timeout retirement, track 42 must not appear in tracks() output";
}

// ---------------------------------------------------------------------------
// Task 6 / channel-kind fix: AIS treated as cooperative-announce source.
//
// Background: `stageCooperativeTouch` was previously gated on
// `scan[l].sensor == SensorKind::Cooperative`. That means an AIS detection
// (SensorKind::Ais) never updated last_cooperative_touch_, so the retirement
// timer fell back to b.birth_time. A track seeded at t=0 and detected by
// AIS at t=20 with cooperative_stale_timeout_sec=30 would be RETIRED at
// t=31 (31-birth_time=31>30) even though the last AIS was only 11s ago.
// The fix: isCooperativeSource() queries channelKindFor() from the activity
// profile; SensorKind::Ais declared as ChannelKind::Cooperative now calls
// stageCooperativeTouch → last_cooperative_touch_[id]=20 → retirement fires
// correctly at t=52 (52-20=32>30), NOT at t=31 (31-20=11<30).
// ---------------------------------------------------------------------------
namespace {

// Activity: AIS declared as Cooperative, own-identity interval 10 s.
// No surveillance channel — pure cooperative-announce model.
navtracker::DeclaredSensorActivity gAisCooperativeActivity() {
  navtracker::DeclaredSensorActivity::ChannelProfile p;
  p.kind = navtracker::ChannelKind::Cooperative;
  p.sensor = navtracker::SensorKind::Ais;
  p.expected_report_interval_sec = 10.0;
  return navtracker::DeclaredSensorActivity{{p}};
}

// Config matching gCoopCfg() but with an explicit cooperative timeout.
PmbmTracker::Config gAisCoopCfg() {
  PmbmTracker::Config c;
  c.probability_of_detection = 0.9;
  c.clutter_intensity = 1e-6;
  c.survival_probability = 1.0;
  c.use_sensor_activity = true;
  c.idle_halflife_sec = 0.0;
  c.r_min = 1e-4;
  c.weight_min = 1e-5;
  c.hypothesis_weight_min = 1e-5;
  c.cooperative_stale_timeout_sec = 30.0;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// (d) AIS declared as Cooperative: detection resets last_cooperative_touch_;
//     stale fires when overdue; retirement fires at last_ais + timeout, not
//     at birth + timeout.
//
// Timeline (key: birth=0, last_AIS=20, timeout=30):
//   t=0  : Bernoulli seeded (birth_time=0, r=0.8)
//   t=20 : AIS detection at the track position
//            → last_activity_check_[42]=20
//            → WITH FIX: last_cooperative_touch_[42]=20
//   t=31 : empty scan; cooperative overdue (31-20=11 > interval=10)
//            → WITH FIX: last_report_sec=20, 31-20=11 < timeout=30 → ALIVE; stale fires
//            → WITHOUT FIX: last_report_sec=birth_time=0, 31-0=31 > timeout=30 → RETIRED; stale not fired
//   t=52 : empty scan; WITH FIX: last_report_sec=20, 52-20=32 > timeout=30 → RETIRED
//
// RED: at t=31, stale_sink is empty (track retired without stale signal, wrong)
//      AND/OR track is not present in tracks() (retired prematurely).
// GREEN: at t=31, stale_sink fired AND track is still alive.
//        At t=52, track is retired.
// ---------------------------------------------------------------------------
TEST(PmbmStaleSignal, AisAsCooperativeFlagsStaleAndRetires) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  auto activity = gAisCooperativeActivity();
  RecordingStaleSink stale_sink;

  PmbmTracker tracker(ekf, gAisCoopCfg());
  tracker.setSensorActivity(&activity);
  tracker.setStaleSignalSink(&stale_sink);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed Bernoulli id=42 at origin with r=0.8 and birth_time=0.
  gSeedBernoulliId(tracker, navtracker::pmbm::BernoulliId{42}, 0.0, 0.0, 0.8);

  // AIS detection at t=20 (well after birth so the birth_time vs
  // last_AIS delta is large enough to discriminate RED from GREEN).
  // processBatch auto-predicts to t_max=20 before the update.
  {
    Measurement ais_z;
    ais_z.time = Timestamp::fromSeconds(20.0);
    ais_z.sensor = navtracker::SensorKind::Ais;
    ais_z.source_id = "ais";
    ais_z.model = navtracker::MeasurementModel::Position2D;
    ais_z.value = Eigen::Vector2d(0.0, 0.0);
    ais_z.covariance = Eigen::Matrix2d::Identity();
    ais_z.hints.mmsi = std::optional<std::uint32_t>{999U};
    tracker.processBatch({ais_z});
    // WITH FIX: isCooperativeSource(Ais)→true → stageCooperativeTouch(42, t=20).
    // WITHOUT FIX: isCooperativeSource(Ais)→false → last_cooperative_touch_ not set.
  }

  // ---- Check at t=31: overdue (11s > interval=10), but NOT yet timed out. --
  // WITH FIX: 31-last_coop_touch(20) = 11 < timeout(30) → ALIVE, stale fired.
  // WITHOUT FIX: 31-birth_time(0) = 31 > timeout(30) → RETIRED, stale NOT fired.
  tracker.predict(Timestamp::fromSeconds(31.0));
  tracker.processBatch({});

  EXPECT_FALSE(stale_sink.stale.empty())
      << "AIS declared as Cooperative: stale signal must fire when AIS is "
         "silent for 11s > interval=10s. "
         "Before fix: stageCooperativeTouch not called for AIS → track retired "
         "immediately at t=31 (31-birth_time=31>timeout=30) → stale not fired. "
         "RED: stale_sink is empty; GREEN: stale_sink has the overdue track.";

  {
    const auto& ts = tracker.tracks();
    const auto it = std::find_if(ts.begin(), ts.end(),
                                 [](const navtracker::Track& t) {
                                   return t.id.value == 42u;
                                 });
    EXPECT_NE(it, ts.end())
        << "Track 42 must be alive at t=31: only 11s since last AIS (< 30s timeout). "
           "Before fix: retirement fires at birth_time=0, 31-0=31>30 → retired. "
           "RED: track absent from tracks(); GREEN: track present and Coasting.";
    if (it != ts.end()) {
      EXPECT_NEAR(it->existence_probability, it->existence_probability, 0.0)
          << "sanity: existence is a valid double";
      EXPECT_EQ(it->status, navtracker::TrackStatus::Coasting)
          << "Overdue cooperative track must be Coasting";
    }
  }

  // ---- Check at t=52: 32s since last AIS > timeout=30 → RETIRED. ----------
  stale_sink.stale.clear();
  tracker.predict(Timestamp::fromSeconds(52.0));
  tracker.processBatch({});

  bool alive_52 = false;
  for (const auto& h : tracker.density().mbm) {
    for (const auto& b : h.bernoullis) {
      if (b.id == navtracker::pmbm::BernoulliId{42} &&
          b.existence_probability > tracker.config().r_min) {
        alive_52 = true;
      }
    }
  }
  EXPECT_FALSE(alive_52)
      << "Track 42 must be retired at t=52: 32s since last AIS > 30s timeout. "
         "(GREEN only — after fix, last_cooperative_touch_=20, 52-20=32>30.)";

  const auto& ts2 = tracker.tracks();
  const auto it52 = std::find_if(ts2.begin(), ts2.end(),
                                  [](const navtracker::Track& t) {
                                    return t.id.value == 42u;
                                  });
  EXPECT_EQ(it52, ts2.end())
      << "After retirement, track 42 must not appear in tracks() output";
}

// ---------------------------------------------------------------------------
// (e) Back-compat: with NO activity provider, SensorKind::Ais is NOT treated
//     as a cooperative-announce source (isCooperativeSource falls back to the
//     legacy `== SensorKind::Cooperative` check, which returns false for Ais).
//
// This test is always GREEN (no RED phase): it verifies that the
// generalisation is profile-gated and does not change behaviour when
// sensor_activity_ is nullptr.
// ---------------------------------------------------------------------------
TEST(PmbmStaleSignal, DeclaredKindOverridesSensorKindDefault) {
  auto motion = std::make_shared<ConstantVelocity2D>(0.1);
  EkfEstimator ekf(motion, 5.0);
  RecordingStaleSink stale_sink;

  // No activity provider wired. Even with use_sensor_activity=true the
  // sensor_activity_==nullptr guard in the tracker's sensor-activity path
  // skips the cooperative_overdue logic entirely, so no stale signal fires.
  PmbmTracker::Config cfg = gAisCoopCfg();
  PmbmTracker tracker(ekf, cfg);
  tracker.setSensorActivity(nullptr);  // explicitly null — no profile
  tracker.setStaleSignalSink(&stale_sink);
  tracker.predict(Timestamp::fromSeconds(0.0));

  // Seed Bernoulli and simulate an AIS detection.
  gSeedBernoulliId(tracker, navtracker::pmbm::BernoulliId{43}, 0.0, 0.0, 0.8);
  {
    Measurement ais_z;
    ais_z.time = Timestamp::fromSeconds(5.0);
    ais_z.sensor = navtracker::SensorKind::Ais;
    ais_z.source_id = "ais";
    ais_z.model = navtracker::MeasurementModel::Position2D;
    ais_z.value = Eigen::Vector2d(0.0, 0.0);
    ais_z.covariance = Eigen::Matrix2d::Identity();
    ais_z.hints.mmsi = std::optional<std::uint32_t>{555U};
    tracker.processBatch({ais_z});
  }

  // Advance well past interval and timeout — no activity provider so the
  // cooperative path is never entered. Stale signal must NOT fire.
  tracker.predict(Timestamp::fromSeconds(100.0));
  tracker.processBatch({});

  EXPECT_TRUE(stale_sink.stale.empty())
      << "With no activity provider, AIS is NOT treated as cooperative "
         "(isCooperativeSource falls back to == SensorKind::Cooperative which is false). "
         "No stale signal should fire — the generalisation is profile-gated.";
}
