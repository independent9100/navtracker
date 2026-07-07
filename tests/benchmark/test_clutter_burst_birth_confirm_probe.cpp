// §5.0 BINDING entry probe (clutter/birth campaign, Phase B) — the birth-vs-confirm race.
//
// Ruling 1 (Checkpoint 1): before building any birth-side clutter fix, measure whether
// the sim_ms_clutter_burst over-count is REACHABLE by a birth suppressor. A birth prior
// needs ~1-2 scans of burst returns to learn a region, and it acts only at BIRTH — it
// cannot remove an already-CONFIRMED phantom (the channel-reach wall). So if the burst
// phantoms confirm within ~2 scans of burst onset (t=120 s), the fix's ceiling sits
// ABOVE the target and we STOP + report ("birth-side suppression structurally
// insufficient; the lever must reach existing Bernoullis") — a campaign finding, not a
// detour. If the phantom population ramps up gradually over many scans, a birth prior
// learning from scan ~2 onward can cap it → proceed to build Candidate A.
//
// Method: run the deployed config imm_cv_ct_pmbm_coverage_land (the +3.48 baseline
// config) on sim_ms_clutter_burst with the SAME wiring Sweep uses (per-sensor detection
// table + DeclaredSensorActivity; the scenario declares no coastline/obstacles so land/
// occupancy are not wired), and capture the Confirmed-track count every processBatch via
// a read-only PmbmPostScanHook. Truth cardinality = 2 real vessels (tracked ~perfectly:
// lifetime 0.995, breaks 0 at baseline), so confirmed − 2 ≈ phantom count. The shape of
// that trajectory across the burst window [t0+120, t0+240] answers the race.
//
// Skips gracefully without the git-ignored fixtures (set SIMMS_DIR + generate them).
#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adapters/benchmark/SimMultisensorScenarioRun.hpp"
#include "core/benchmark/BenchRunner.hpp"
#include "core/benchmark/Config.hpp"
#include "core/benchmark/Sweep.hpp"
#include "core/pipeline/MhtTracker.hpp"
#include "core/pmbm/PmbmTracker.hpp"
#include "core/scenario/Truth.hpp"
#include "core/sensor_activity/DeclaredSensorActivity.hpp"

using namespace navtracker;
using namespace navtracker::benchmark;

namespace {
double meanConfirmedIn(const std::vector<std::pair<double, int>>& traj, double lo,
                       double hi) {
  double s = 0;
  int n = 0;
  for (const auto& [t, c] : traj)
    if (t >= lo && t <= hi) {
      s += c;
      ++n;
    }
  return n ? s / n : 0.0;
}
}  // namespace

TEST(ClutterBurstBirthConfirmProbe, Race) {
  auto scenarios = defaultSimMultisensorScenarios();
  ScenarioRun* cb = nullptr;
  for (auto& s : scenarios)
    if (s->descriptor().label == "sim_ms_clutter_burst") cb = s.get();
  ASSERT_NE(cb, nullptr);
  const ScenarioDescriptor desc = cb->descriptor();
  Scenario scen = cb->generate(0);
  if (scen.measurements.empty())
    GTEST_SKIP() << "sim_ms_clutter_burst fixtures absent (set SIMMS_DIR + generate)";

  const auto all = defaultConfigs();
  const Config* c = nullptr;
  for (const auto& cc : all)
    if (cc.label == "imm_cv_ct_pmbm_coverage_land") c = &cc;
  ASSERT_NE(c, nullptr);

  auto est = c->build_estimator();
  pmbm::PmbmTracker::Config cfg =
      c->pmbm_config ? c->pmbm_config() : pmbm::PmbmTracker::Config{};

  MhtTracker::Config carrier;
  carrier.probability_of_detection = cfg.probability_of_detection;
  carrier.clutter_density = cfg.clutter_intensity;
  auto det = detectionModelFor(desc, carrier, c->use_clutter_map);
  pmbm::PmbmTracker tracker(*est, cfg);
  if (det) tracker.setSensorDetectionModel(det);

  // DeclaredSensorActivity — verbatim from Sweep.cpp (coverage_land sets
  // use_sensor_activity_model=true; without this the miss path diverges from baseline).
  constexpr double kArpaDutyCycleSec = 2.5;
  constexpr double kEoIrDutyCycleSec = 1.0;
  constexpr double kLidarDutyCycleSec = 0.1;
  constexpr double kAisReportIntervalSec = 10.0;
  std::shared_ptr<DeclaredSensorActivity> activity;
  if (c->use_sensor_activity_model && !desc.detection_table.empty()) {
    std::vector<DeclaredSensorActivity::ChannelProfile> profiles;
    for (const auto& e : desc.detection_table) {
      DeclaredSensorActivity::ChannelProfile prof;
      prof.sensor = e.sensor;
      switch (e.sensor) {
        case SensorKind::ArpaTtm:
        case SensorKind::EoIr:
        case SensorKind::Lidar:
          prof.kind = ChannelKind::Surveillance;
          prof.max_range_m = e.params.max_range_m;
          prof.sector_center_rad = e.params.sector_center_rad;
          prof.sector_width_rad = e.params.sector_width_rad;
          prof.p_D = e.params.probability_of_detection;
          prof.duty_cycle_sec = (e.sensor == SensorKind::ArpaTtm) ? kArpaDutyCycleSec
                                : (e.sensor == SensorKind::EoIr)  ? kEoIrDutyCycleSec
                                                                  : kLidarDutyCycleSec;
          break;
        case SensorKind::Ais:
        case SensorKind::Cooperative:
          prof.kind = ChannelKind::Cooperative;
          prof.expected_report_interval_sec = kAisReportIntervalSec;
          break;
        default:
          continue;
      }
      profiles.push_back(prof);
    }
    activity = std::make_shared<DeclaredSensorActivity>(std::move(profiles));
    tracker.setSensorActivity(activity.get());
  }

  // Per-processBatch capture (read-only → tracker behaviour is baseline-identical).
  std::vector<std::pair<double, int>> traj;      // (scan time, confirmed count)
  std::map<std::uint64_t, double> first_conf;    // id -> first-confirmed scan time
  std::map<std::uint64_t, Eigen::Vector2d> conf_pos;  // id -> pos at first confirm
  PmbmPostScanHook hook = [&](const pmbm::PmbmTracker& t, Timestamp scan_t) {
    int confirmed = 0;
    for (const Track& tr : t.tracks()) {
      if (tr.status != TrackStatus::Confirmed || tr.state.size() < 2) continue;
      ++confirmed;
      if (!first_conf.count(tr.id.value)) {
        first_conf[tr.id.value] = scan_t.seconds();
        conf_pos[tr.id.value] = Eigen::Vector2d(tr.state(0), tr.state(1));
      }
    }
    traj.emplace_back(scan_t.seconds(), confirmed);
  };
  runBenchPmbm(scen, tracker, hook);

  ASSERT_FALSE(traj.empty());
  const double t0 = traj.front().first;
  const double tend = traj.back().first;
  const double burst = t0 + 120.0;  // burst onset (fixture: t in [120,240] s)
  const double burst_end = t0 + 240.0;

  // Over-count split (phantom = confirmed - 2 reals) across the three windows.
  // Bounds what ANY fix targeting each mechanism could remove. card_err is a
  // whole-run mean, so a window's contribution is its mean-phantom * its
  // time-fraction of the run.
  auto meanPhantom = [&](double lo, double hi) {
    double s = 0;
    int n = 0;
    for (const auto& [t, c] : traj)
      if (t >= lo && t <= hi) { s += (c - 2); ++n; }
    return n ? s / n : 0.0;
  };
  const double run = tend - t0;
  const double ph_pre = meanPhantom(t0, burst);              // compound-K background only
  const double ph_burst = meanPhantom(burst, burst_end);     // burst + background
  const double ph_post = meanPhantom(burst_end, tend);       // background, burst decaying
  const double ph_all = meanPhantom(t0, tend);
  std::printf("\n=== over-count split (phantom = confirmed - 2 reals) ===\n");
  std::printf(" pre-burst  [0,120]s   mean_phantom=%.2f  (%.0f%% of run)\n", ph_pre, 100.0*120.0/run);
  std::printf(" burst      [120,240]s mean_phantom=%.2f  (%.0f%% of run)\n", ph_burst, 100.0*120.0/run);
  std::printf(" post-burst [240,end]s mean_phantom=%.2f  (%.0f%% of run)\n", ph_post, 100.0*(run-240.0)/run);
  std::printf(" whole-run  mean_phantom=%.2f  (== card_err_mean; bench=3.48)\n", ph_all);
  // Honest attribution: the ONLY birth-reachable component is the pre-burst
  // background rate (moving compound-K clumps with turnover). The post-burst
  // window's phantoms are the DECAYING BURST tail (sticky phantoms persisting
  // long after t=240), NOT fresh background — so they are burst-attributable and
  // unreachable by a birth suppressor (channel-reach wall). Everything above the
  // pre-burst rate is burst-attributable.
  std::printf(" birth-REACHABLE ceiling: only the pre-burst background (%.2f phantom) is\n", ph_pre);
  std::printf("   birth-turnover-y — and even that needs a prior that locks MOVING clumps\n");
  std::printf("   (A can't) or an NB that flags a CONCENTRATED burst vs diffuse high-count (B can't).\n");
  std::printf(" burst-ATTRIBUTABLE (confirm<=1 scan, sticky, persists post-burst) ~= %.2f of card_err\n", ph_all - ph_pre);
  std::printf(" => a PERFECT birth fix floors card_err at ~%.2f (still ABOVE MHT +2.51; ideal is +0.9).\n", ph_all - ph_pre);

  // Trajectory around the burst window.
  std::printf("\n=== clutter_burst birth-vs-confirm race (coverage_land) ===\n");
  std::printf(" t-t0   confirmed\n");
  for (const auto& [t, cc] : traj) {
    const double rel = t - t0;
    if (rel >= 110.0 && rel <= 250.0) std::printf(" %5.1f   %d\n", rel, cc);
  }

  const double pre = meanConfirmedIn(traj, burst - 20, burst - 1);      // ~2 reals
  const double at_1scan = meanConfirmedIn(traj, burst, burst + 2.5);    // <=1 radar scan
  const double at_2scan = meanConfirmedIn(traj, burst, burst + 5.0);    // <=2 radar scans
  const double steady = meanConfirmedIn(traj, burst + 40, burst + 115); // plateau
  const double reachable = steady - pre;                                // phantom plateau
  const double by2 = at_2scan - pre;                                    // confirmed by 2 scans

  // Confirm-latency of burst-region phantoms (cross-check; burst ~ ENU (1200,900)).
  const Eigen::Vector2d burst_enu(1200.0, 900.0);
  int burst_phantoms = 0, burst_phantoms_le2 = 0;
  for (const auto& [id, ft] : first_conf) {
    const double lat = ft - burst;
    if ((conf_pos[id] - burst_enu).norm() < 500.0 && lat >= -2.5 && lat <= 130.0) {
      ++burst_phantoms;
      if (lat <= 5.0) ++burst_phantoms_le2;
    }
  }

  std::printf("\n pre-burst confirmed (~2 reals) = %.2f\n", pre);
  std::printf(" confirmed by <=1 scan post-onset = %.2f  (excess %.2f)\n", at_1scan, at_1scan - pre);
  std::printf(" confirmed by <=2 scans post-onset= %.2f  (excess %.2f)\n", at_2scan, by2);
  std::printf(" steady burst plateau            = %.2f  (reachable phantoms %.2f)\n", steady, reachable);
  std::printf(" fraction of plateau present by <=2 scans = %.0f%%\n",
              reachable > 1e-6 ? 100.0 * by2 / reachable : 0.0);
  std::printf(" burst-region phantom ids: %d total, %d confirmed within <=2 scans\n",
              burst_phantoms, burst_phantoms_le2);
  std::printf("\n VERDICT: %s\n",
              (reachable > 1e-6 && by2 / reachable >= 0.7)
                  ? "CEILING ABOVE TARGET — most phantoms confirm <=2 scans; birth-side "
                    "suppression structurally insufficient (STOP + report)"
                  : "REACHABLE — phantom population ramps gradually; a birth prior "
                    "learning from scan ~2 can cap it (PROCEED to build A)");
  std::fflush(stdout);

  // The probe must be measuring the designed over-count, or its verdict is meaningless.
  EXPECT_GT(reachable, 1.5) << "expected the compound-K burst over-count (~+3.5 phantoms)";
}
