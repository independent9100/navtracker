#include "adapters/benchmark/SimScenarioRun.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Wgs84.hpp"
#include "core/scenario/Builders.hpp"
#include "core/scenario/Truth.hpp"
#include "sim/SkewInjector.hpp"

namespace navtracker {
namespace benchmark {

namespace {

constexpr std::uint32_t kSeedCount = 10;

// Honest per-sensor detection tables — scenario *properties*, declared
// the same way the autoferry replays declare their calibrated tables.
// The synthetic generators emit exactly one detection per target per
// scan (outside dropout windows), so P_D ≈ 1; declared 0.95 to keep
// log(1 − P_D) finite and tolerate occasional gating misses. Clutter:
// the clutter-free scenarios have a true λ_C of 0, declared as a 1e-6
// m⁻² floor (λ = 0 degenerates the LLR; 1e-6 ≈ "≤1 false alarm per km²
// per scan"). Scoring them with the legacy global 1e-4 made a gated
// hit on a young (unconverged, large-S) track score as evidence
// *against* existence — the measured 6-scan IPDA confirmation latency
// on crossing (see evaluation-log 2026-06-11).
std::vector<SensorDetectionEntry> cleanAisTable() {
  return {{SensorKind::Ais, MeasurementModel::Position2D,
           DetectionParams{0.95, 1e-6}}};
}

// dense_clutter: 4 uniform false alarms per scan in a 600×200 m box →
// λ_C = 4 / 120000 m² = 3.33e-5 m⁻².
std::vector<SensorDetectionEntry> denseClutterTable() {
  return {{SensorKind::Ais, MeasurementModel::Position2D,
           DetectionParams{0.95, 3.33e-5}}};
}

// non_cooperative: bearing-only camera, no false bearings generated;
// floor in the bearing measurement space (rad⁻¹).
std::vector<SensorDetectionEntry> bearingOnlyTable() {
  return {{SensorKind::EoIr, MeasurementModel::Bearing2D,
           DetectionParams{0.95, 1e-2}}};
}

// Shore-clutter scenarios: AIS-cooperative targets + radar-like (ArpaTtm)
// stationary shore returns. λ_C for the radar channel reflects ~30 fixed
// returns over the ~2.7e6 m² scene (30/2.7e6 ≈ 1.1e-5 m⁻², declared 1e-5).
std::vector<SensorDetectionEntry> shoreClutterTable() {
  return {{SensorKind::Ais, MeasurementModel::Position2D,
           DetectionParams{0.95, 1e-6}},
          {SensorKind::ArpaTtm, MeasurementModel::Position2D,
           DetectionParams{0.95, 1e-5}}};
}

// Uncharted pier: a 120 m line of fixed radar scatterers (13 points, 10 m
// apart) at y = -350, x in [-60, 60]. Deliberately NOT a charted obstacle and
// NOT in the truth set — the extended persistent structure the live
// static-occupancy layer must learn to suppress.
std::vector<Eigen::Vector2d> pierPoints() {
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i <= 12; ++i)
    pts.emplace_back(-60.0 + 10.0 * static_cast<double>(i), -350.0);
  return pts;
}

// Fixed synthetic shoreline shared by every shore-clutter scenario. Pure /
// deterministic, so generate() and syntheticCoastline() agree.
SyntheticShore makeBenchShore() {
  return buildSyntheticShore(navtracker::geo::Geodetic{42.35, -71.05, 0.0},
                             /*shore_y_m=*/500.0, /*extent_m=*/1500.0,
                             /*land_depth_m=*/400.0, /*pier_width_m=*/40.0,
                             /*pier_length_m=*/150.0, /*n_clutter=*/30);
}

ScenarioDescriptor describe(const char* label,
                            std::vector<SensorDetectionEntry> table) {
  ScenarioDescriptor d{label, true, kSeedCount};
  d.detection_table = std::move(table);
  return d;
}

std::vector<double> linearSeconds(int first, int last) {
  std::vector<double> v;
  v.reserve(static_cast<std::size_t>(last - first + 1));
  for (int i = first; i <= last; ++i) v.push_back(static_cast<double>(i));
  return v;
}

class CrossingScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("crossing", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildCrossingTargetsScenario(
        Eigen::Vector2d(-500.0, 10.0),
        Eigen::Vector2d(25.0, 0.0),
        Eigen::Vector2d(500.0, -10.0),
        Eigen::Vector2d(-25.0, 0.0),
        linearSeconds(1, 40),
        8.0,
        static_cast<std::uint32_t>(seed));
  }
};

class OvertakingScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("overtaking", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildOvertakingScenario(
        Eigen::Vector2d(0.0, 30.0),
        Eigen::Vector2d(10.0, 0.0),
        Eigen::Vector2d(-500.0, 0.0),
        Eigen::Vector2d(20.0, 0.0),
        linearSeconds(1, 60),
        5.0,
        static_cast<std::uint32_t>(seed));
  }
};

class HeadOnScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("head_on", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Anti-parallel velocities; small lateral offset (5 m) so the targets
    // don't sit on top of each other at t=0 (they cross at y = +/-5).
    return buildCrossingTargetsScenario(
        Eigen::Vector2d(-500.0, 5.0),
        Eigen::Vector2d(25.0, 0.0),
        Eigen::Vector2d(500.0, -5.0),
        Eigen::Vector2d(-25.0, 0.0),
        linearSeconds(1, 40),
        8.0,
        static_cast<std::uint32_t>(seed));
  }
};

class ParallelTargetsScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("parallel_targets", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildParallelTargetsScenario(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 800.0),
        Eigen::Vector2d(5.0, 0.0),
        linearSeconds(1, 30),
        5.0,
        static_cast<std::uint32_t>(seed));
  }
};

class AisDropoutScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("ais_dropout", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildCrossingDropoutScenario(
        /*velocity_x_mps=*/25.0,
        /*y_offset_m=*/10.0,
        linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0,
        /*dropout_start_s=*/10.0,
        /*dropout_end_s=*/20.0,
        static_cast<std::uint32_t>(seed));
  }
};

class ClockSkewScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("clock_skew", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    Scenario s = buildStraightLineScenario(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(10.0, 0.0),
        linearSeconds(1, 40),
        /*pos_noise_std_m=*/5.0,
        static_cast<std::uint32_t>(seed));
    // Apply skew: 50 ms jitter on AIS measurements (the sensor kind the
    // straight-line builder emits). SkewInjector reorders by arrival time
    // but does NOT mutate Measurement.time, so the determinism test (which
    // compares times across two generate(seed) calls) stays valid.
    SkewProfile profile;
    profile.at(SensorKind::Ais) = {/*lag_s=*/0.0, /*jitter_s=*/0.050};
    s.measurements = applySkew(s.measurements, profile, seed);
    return s;
  }
};

class SpeedChangeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("speed_change", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // 40-second run, 1-second cadence. Target cruising east at 8 m/s
    // (~15 kt), then a 5-second engine surge at 2 m/s² (CV can't fit,
    // CT has no heading change to track), then a 10-second drift
    // decelerating at 1 m/s² (engine cut). The noisy-CV mode is what's
    // designed to absorb both surge and drift.
    return buildSpeedChangeScenario(
        /*start=*/Eigen::Vector2d(0.0, 0.0),
        /*initial_velocity=*/Eigen::Vector2d(8.0, 0.0),
        /*surge_start_s=*/10.0,
        /*surge_duration_s=*/5.0,
        /*surge_accel_mps2=*/2.0,
        /*drift_decel_mps2=*/1.0,
        linearSeconds(1, 40),
        /*pos_noise_std_m=*/5.0,
        static_cast<std::uint32_t>(seed));
  }
};

class DenseClutterScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("dense_clutter", denseClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Two crossing CV targets at the origin + 4 uniform false alarms
    // per scan inside a 600x200 m box covering both target tracks.
    // Forces multi-measurement gating per tree per scan — the canonical
    // setting where Score-Δ K and protected K>1 alternatives differ
    // from a Hungarian K=1 commit-to-best each scan. Targets are well
    // separated in y (closest approach 20 m), so the difficulty is
    // pure clutter-vs-target disambiguation, not target-vs-target.
    // 4-per-scan keeps JPDA's hypothesis enumeration tractable while
    // still creating real multi-gate ambiguity.
    return buildClutterCrossingScenario(
        Eigen::Vector2d(-500.0, 10.0),
        Eigen::Vector2d(25.0, 0.0),
        Eigen::Vector2d(500.0, -10.0),
        Eigen::Vector2d(-25.0, 0.0),
        linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0,
        /*n_clutter_per_scan=*/4,
        /*clutter_min=*/Eigen::Vector2d(-300.0, -100.0),
        /*clutter_max=*/Eigen::Vector2d( 300.0,  100.0),
        static_cast<std::uint32_t>(seed));
  }
};

class CrossingDropoutScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("crossing_dropout", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Two CV targets crossing close to each other (6 m y-offset → 12 m
    // closest approach) with a 4-second dropout straddling the crossing
    // time. On the first post-dropout scan, both targets are near the
    // origin and *which-was-which* is ambiguous from a single
    // measurement pair. JPDA-style trackers can swap IDs here; MHT
    // with protected K>1 keeps both interpretations alive a few scans
    // until divergent motion disambiguates. Closer offsets (3 m) are
    // genuinely degenerate for JPDA's hypothesis enumeration —
    // hypothesis count grows combinatorially and OOMs the bench.
    // 6 m is the sweet spot: ambiguous enough to exercise protected
    // K>1, separable enough for JPDA to stay tractable.
    return buildCrossingDropoutScenario(
        /*velocity_x_mps=*/25.0,
        /*y_offset_m=*/6.0,
        linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0,
        /*dropout_start_s=*/18.0,
        /*dropout_end_s=*/22.0,
        static_cast<std::uint32_t>(seed));
  }
};

class NonCooperativeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("non_cooperative", bearingOnlyTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Bearing-only target (no AIS / no range): stresses filters whose
    // Gaussian posterior approximation degrades when range is unobservable
    // for the duration of the run. Geometry: target moves toward the
    // upper-left across the sensor at the origin so bearing rate is
    // appreciable.
    return buildBearingOnlyScenario(
        Eigen::Vector2d(100.0, 0.0),
        Eigen::Vector2d(-5.0, 5.0),
        linearSeconds(1, 40),
        /*initial_position_std_m=*/200.0,
        /*bearing_std_rad=*/0.05,
        static_cast<std::uint32_t>(seed));
  }
};

class ParallelLanesDenseScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("parallel_lanes_dense", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // 4 lanes, 40 m apart, all heading +x at 10 m/s. Tight spacing stresses
    // track resolution / merge.
    return buildParallelLaneScenario(
        /*n_targets=*/4, /*lane_spacing_m=*/40.0, Eigen::Vector2d(-400.0, 0.0),
        Eigen::Vector2d(10.0, 0.0), linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0, static_cast<std::uint32_t>(seed));
  }
};

class CrossingAngleScenarioRun : public ScenarioRun {
 public:
  CrossingAngleScenarioRun(const char* label, double angle_deg)
      : label_(label), angle_deg_(angle_deg) {}
  ScenarioDescriptor descriptor() const override {
    return describe(label_, cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    return buildCrossingAngleScenario(angle_deg_, /*speed_mps=*/20.0,
                                      Eigen::Vector2d(0.0, 0.0),
                                      linearSeconds(1, 40),
                                      /*pos_noise_std_m=*/8.0,
                                      static_cast<std::uint32_t>(seed));
  }

 private:
  const char* label_;
  double angle_deg_;
};

class ConvoyOvertakeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("convoy_overtake", cleanAisTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // 3 in-line targets 80 m apart at 5 m/s, plus a 15 m/s overtaker.
    return buildConvoyScenario(/*n_targets=*/3, /*gap_m=*/80.0,
                               /*speed_mps=*/5.0, /*overtaker_speed_mps=*/15.0,
                               linearSeconds(1, 60), /*pos_noise_std_m=*/5.0,
                               static_cast<std::uint32_t>(seed));
  }
};

class ShoreClutterOpenScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("shore_clutter_open", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Two real targets crossing in open water + stationary shore clutter. The
    // crossing point is at x = 400 so the vertical (north-bound) target runs up
    // x = 400, well clear of the pier (x in [-20, 20]); both targets stay in
    // open water (land prior c = 0): the along-shore target holds y = 100
    // (400 m offshore), and the north-bound target peaks at y ≈ 392 (≈108 m
    // offshore, beyond the 50 m soft band). No land suppression of either.
    const SyntheticShore shore = makeBenchShore();
    Scenario base = buildCrossingAngleScenario(
        /*crossing_angle_deg=*/90.0, /*speed_mps=*/15.0,
        Eigen::Vector2d(400.0, 100.0), linearSeconds(1, 40),
        /*pos_noise_std_m=*/8.0, static_cast<std::uint32_t>(seed));
    return addShoreClutter(std::move(base), shore.datum,
                           shore.clutter_enu_points, /*detection_prob=*/0.9,
                           /*pos_noise_std_m=*/8.0,
                           static_cast<std::uint32_t>(seed));
  }
  std::optional<CoastlineGeometry> syntheticCoastline() const override {
    return makeBenchShore().geometry;
  }
};

class ShoreClutterNearShoreScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("shore_clutter_nearshore", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // One slow AIS target running parallel to the coast at y = 440 (shore at
    // y = 500, so 60 m offshore — just beyond the soft offshore band
    // offshore_halfwidth_m = 50 m, where the land prior is c = 0), routed in
    // x = [-500, -260] so it stays well clear of the pier (which protrudes to
    // y = 350 only at x in [-20, 20]). Plus stationary shore clutter. This
    // validates that the land model removes the shore clutter WITHOUT
    // collaterally suppressing a legitimate vessel travelling near — but
    // outside — the suppression band.
    //
    // KNOWN LIMITATION (measured): under imm_cv_ct_pmbm_coverage_land the
    // phantom-birth floor equals birth_existence_target (0.1), so the entire
    // soft band (< 50 m from shore, and the area around the pier) is a
    // no-birth zone — a vessel inside it does not initiate. Lowering the floor
    // to revive near-shore births re-admits philos water clutter and regresses
    // the real-data win (gospa 73.1→100.0, card_err +6.9→+36.2), so it is not
    // done. See docs/algorithms/synthetic-clutter-bench.md and the eval log.
    const SyntheticShore shore = makeBenchShore();
    Scenario base = buildStraightLineScenario(
        Eigen::Vector2d(-500.0, 440.0), Eigen::Vector2d(6.0, 0.0),
        linearSeconds(1, 40), /*pos_noise_std_m=*/8.0,
        static_cast<std::uint32_t>(seed), /*truth_id=*/1);
    return addShoreClutter(std::move(base), shore.datum,
                           shore.clutter_enu_points, /*detection_prob=*/0.9,
                           /*pos_noise_std_m=*/8.0,
                           static_cast<std::uint32_t>(seed));
  }
  std::optional<CoastlineGeometry> syntheticCoastline() const override {
    return makeBenchShore().geometry;
  }
};

class HarborCompleteTruthScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return describe("harbor_complete_truth", shoreClutterTable());
  }
  Scenario generate(std::uint64_t seed) override {
    // Honest yardstick: complete, controlled truth. Two moving vessels
    // (AIS, ids 1-2) + three anchored non-AIS boats (zero velocity, ids 3-5,
    // the KEEP set) + one uncharted pier (extended fixed radar structure, NO
    // truth, the SUPPRESS set) + uniform sea clutter (transient, NO truth,
    // the IGNORE set). Chart-free on purpose (no syntheticCoastline) so the
    // pier is an *uncharted* open-water structure. See
    // docs/superpowers/specs/2026-07-01-honest-static-occupancy-stage1b-design.md.
    const auto times = linearSeconds(1, 40);
    const geo::Datum datum(navtracker::geo::Geodetic{42.35, -71.05, 0.0});
    const auto s32 = static_cast<std::uint32_t>(seed);

    Scenario base = buildCrossingTargetsScenario(
        Eigen::Vector2d(-500.0, -150.0), Eigen::Vector2d(20.0, 0.0),
        Eigen::Vector2d(500.0, 150.0), Eigen::Vector2d(-20.0, 0.0), times,
        /*pos_noise_std_m=*/8.0, s32);

    base = addAnchoredBoats(
        std::move(base), datum,
        {{100.0, 300.0}, {250.0, 320.0}, {-50.0, 330.0}},
        /*truth_id_start=*/3, /*detection_prob=*/0.95, /*pos_noise_std_m=*/5.0,
        s32);

    base = addFixedClutter(std::move(base), datum, pierPoints(), "sim_pier",
                           /*detection_prob=*/0.9, /*pos_noise_std_m=*/4.0,
                           s32 + 7u);

    base = addUniformClutter(std::move(base), datum,
                             Eigen::Vector2d(-600.0, -450.0),
                             Eigen::Vector2d(600.0, 450.0),
                             /*n_per_scan=*/5, s32 + 12345u);
    return base;
  }
  // No syntheticCoastline(): chart-free by design.
};

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultSimScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  out.reserve(18);
  out.push_back(std::make_unique<CrossingScenarioRun>());
  out.push_back(std::make_unique<OvertakingScenarioRun>());
  out.push_back(std::make_unique<HeadOnScenarioRun>());
  out.push_back(std::make_unique<ParallelTargetsScenarioRun>());
  out.push_back(std::make_unique<AisDropoutScenarioRun>());
  out.push_back(std::make_unique<ClockSkewScenarioRun>());
  out.push_back(std::make_unique<SpeedChangeScenarioRun>());
  out.push_back(std::make_unique<NonCooperativeScenarioRun>());
  out.push_back(std::make_unique<DenseClutterScenarioRun>());
  out.push_back(std::make_unique<CrossingDropoutScenarioRun>());
  out.push_back(std::make_unique<ParallelLanesDenseScenarioRun>());
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_30", 30.0));
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_60", 60.0));
  out.push_back(std::make_unique<CrossingAngleScenarioRun>("crossing_90", 90.0));
  out.push_back(std::make_unique<ConvoyOvertakeScenarioRun>());
  out.push_back(std::make_unique<ShoreClutterOpenScenarioRun>());
  out.push_back(std::make_unique<ShoreClutterNearShoreScenarioRun>());
  out.push_back(std::make_unique<HarborCompleteTruthScenarioRun>());
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
