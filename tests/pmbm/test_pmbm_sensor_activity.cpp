#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <optional>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/estimation/EkfEstimator.hpp"
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
