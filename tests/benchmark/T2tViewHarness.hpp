#pragma once

// TEST-ONLY harness for the two-tracker-view T2T evaluation (ticket §6.1).
// Not part of any shipped library — it lives in tests/ and exists only to drive
// scenarios and produce the Checkpoint-2 numbers.
//
// Reuse, not new infrastructure: take one sim_multisensor Scenario, split its
// measurements into per-tracker "views" by SensorKind (in-process, per-arm —
// never the SIMMS_RADAR_ONLY env global), run one MHT instance per view via the
// existing bench driver, feed each view's confirmed tracks to a T2tFuser as
// ExternalTracks, and hand back a fused BenchResult scoreable by the existing
// computeMetrics / computeNees.

#include <algorithm>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/ScenarioRun.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/t2t/ExternalTrack.hpp"
#include "core/t2t/Pedigree.hpp"
#include "core/t2t/T2tConfig.hpp"
#include "core/t2t/T2tFuser.hpp"
#include "ports/IFusionRule.hpp"

namespace navtracker::t2t::bench {

using navtracker::benchmark::ScenarioRun;

// The overconfident control: naive independence-assuming fusion. TEST-ONLY —
// this is the double-counting footgun; it exists here purely to show why CI.
class NaiveFusionRule : public IFusionRule {
 public:
  CiResult fuse(const Eigen::VectorXd& x1, const Eigen::MatrixXd& P1,
                const Eigen::VectorXd& x2, const Eigen::MatrixXd& P2,
                IndependenceClass) const override {
    const Eigen::MatrixXd I1 = P1.inverse();
    const Eigen::MatrixXd I2 = P2.inverse();
    CiResult r;
    r.P = (I1 + I2).inverse();
    r.x = r.P * (I1 * x1 + I2 * x2);
    r.omega = 0.5;
    return r;
  }
  const char* name() const override { return "naive"; }
};

// One tracker view: which sensors it consumes + the pedigree it declares.
struct ArmSpec {
  std::string tracker_id;
  std::set<SensorKind> sensors;
  std::optional<SourcePedigree> pedigree;  // nullopt -> unregistered (all-Unknown)
};

// Keep only measurements from `keep`; share truth + datum.
inline Scenario armView(const Scenario& full, const std::set<SensorKind>& keep) {
  Scenario v;
  v.truth = full.truth;
  v.datum = full.datum;
  for (const auto& m : full.measurements)
    if (keep.count(m.sensor)) v.measurements.push_back(m);
  return v;
}

// TEST-ONLY calibration lever (ticket §10 ruling, Point 2). The shared
// AisCsvReplayAdapter stamps every AIS fix with a 30 m pessimistic σ — right for
// real, stale AIS, but ~150× too loose for THIS sim, whose AIS carries no
// additive position noise, only ~0.19 m lat/lon quantization. That mismatch
// leaves each AIS arm's own NEES ≈ 0.2 (under-confident), so the double-count
// shows only as a ratio, never a χ² band violation. Overriding the AIS
// measurement σ HERE (test-locally, per-arm) so each arm's own NEES ≈ 2 makes
// the gate a real band violation. This touches neither the generator nor the
// frozen fixtures nor the shared loader constant — it rewrites the covariance of
// already-loaded AIS Measurements in one scenario view. sigma_m <= 0 is a no-op.
inline Scenario withAisSigma(Scenario s, double sigma_m) {
  if (sigma_m <= 0.0) return s;
  const double var = sigma_m * sigma_m;
  for (auto& m : s.measurements) {
    if (m.sensor != SensorKind::Ais) continue;
    if (m.covariance.rows() >= 2 && m.covariance.cols() >= 2)
      m.covariance.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() * var;
  }
  return s;
}

// Run one view through the canonical IMM+MHT config (mirrors the sim-ms test).
// ais_sigma_m > 0 rewrites the AIS measurement σ first (calibration lever above).
inline benchmark::BenchResult runArm(ScenarioRun& run, const Scenario& view_in,
                                     double ais_sigma_m = 0.0) {
  const Scenario view = withAisSigma(view_in, ais_sigma_m);
  using namespace navtracker::benchmark;
  const auto configs = defaultConfigs();
  const Config* mht = nullptr;
  for (const auto& c : configs)
    if (c.label == "imm_cv_ct_mht") mht = &c;
  auto est = mht->build_estimator();
  MhtTracker::Config cfg = mht->mht_config();
  auto det = detectionModelFor(run.descriptor(), cfg);
  MhtTracker t(*est, cfg, det);
  return runBenchMht(view, t);
}

inline void feedStep(T2tFuser& fuser, const std::string& tracker_id,
                     const benchmark::BenchStep& step) {
  for (const auto& tr : step.tracks) {
    ExternalTrack e;
    e.source_tracker_id = tracker_id;
    e.source_track_id = std::to_string(tr.id.value);
    e.time = step.time;
    e.position_enu = tr.position;
    e.position_cov = tr.pos_covariance;  // ENU m^2 (the tracker's stated cov)
    // Snapshots carry no velocity covariance -> feed position-only.
    fuser.process(std::move(e));
  }
}

// Fuse two views end to end and return the fused BenchResult (ENU, confirmed
// tracks only, to match the bench snapshot convention). Optionally emits each
// view's own BenchResult (for GOSPA<=best-input comparisons) and the final
// FusedTrackOutput set (for independence_class assertions).
inline benchmark::BenchResult fuseTwoViews(
    ScenarioRun& run, const Scenario& full, const ArmSpec& A, const ArmSpec& B,
    const IFusionRule* rule, T2tConfig cfg,
    benchmark::BenchResult* out_a = nullptr,
    benchmark::BenchResult* out_b = nullptr,
    std::set<IndependenceClass>* out_multi_classes = nullptr,
    double ais_sigma_m = 0.0) {
  using namespace navtracker::benchmark;
  const BenchResult ra = runArm(run, armView(full, A.sensors), ais_sigma_m);
  const BenchResult rb = runArm(run, armView(full, B.sensors), ais_sigma_m);
  if (out_a) *out_a = ra;
  if (out_b) *out_b = rb;

  T2tFuser fuser(cfg, rule);
  if (full.datum) fuser.setDatum(*full.datum);
  if (A.pedigree) fuser.registerSource(A.tracker_id, *A.pedigree);
  if (B.pedigree) fuser.registerSource(B.tracker_id, *B.pedigree);

  BenchResult fused;
  const std::size_t n = std::min(ra.steps.size(), rb.steps.size());
  for (std::size_t k = 0; k < n; ++k) {
    const Timestamp t = ra.steps[k].time;
    // Feed the whole tick (both arms) then close the scan with a single cycle:
    // per-scan lifecycle, no ambient over-counting.
    feedStep(fuser, A.tracker_id, ra.steps[k]);
    feedStep(fuser, B.tracker_id, rb.steps[k]);
    fuser.flush();
    BenchStep fs;
    fs.time = t;
    fs.truth = ra.steps[k].truth;
    for (const auto& e : fuser.fusedTracksEnu()) {
      if (e.status != TrackStatus::Confirmed) continue;
      TrackStateSnapshot s;
      s.id = e.id;
      s.position = e.position;
      s.velocity = e.velocity;
      s.pos_covariance = e.position_cov;
      fs.tracks.push_back(s);
    }
    fused.steps.push_back(std::move(fs));
    // Record the independence verdict of genuinely-fused (>=2 contributor)
    // tracks across the WHOLE run — the final tick alone may be single-source
    // (the other arm has passed), which is a correct SingleSource verdict but
    // not the multi-source case under test.
    if (out_multi_classes && full.datum) {
      for (const auto& fo : fuser.fusedTracks())
        if (fo.contributing_trackers.size() >= 2)
          out_multi_classes->insert(fo.independence_class);
    }
  }
  return fused;
}

// Convenience pedigree builders.
inline SourcePedigree usedStreams(const std::vector<std::string>& streams) {
  SourcePedigree p;
  p.default_usage = SensorUsage::NotUsed;
  for (const auto& s : streams) p.sensors[s] = SensorUsage::Used;
  return p;
}
inline SourcePedigree allUnknown() { return SourcePedigree{}; }

}  // namespace navtracker::t2t::bench
