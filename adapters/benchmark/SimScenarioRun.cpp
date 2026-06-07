#include "adapters/benchmark/SimScenarioRun.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/scenario/Builders.hpp"
#include "core/scenario/Truth.hpp"
#include "sim/SkewInjector.hpp"

namespace navtracker {
namespace benchmark {

namespace {

constexpr std::uint32_t kSeedCount = 10;

std::vector<double> linearSeconds(int first, int last) {
  std::vector<double> v;
  v.reserve(static_cast<std::size_t>(last - first + 1));
  for (int i = first; i <= last; ++i) v.push_back(static_cast<double>(i));
  return v;
}

class CrossingScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"crossing", true, kSeedCount};
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
    return {"overtaking", true, kSeedCount};
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
    return {"head_on", true, kSeedCount};
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
    return {"parallel_targets", true, kSeedCount};
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
    return {"ais_dropout", true, kSeedCount};
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
    return {"clock_skew", true, kSeedCount};
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

class NonCooperativeScenarioRun : public ScenarioRun {
 public:
  ScenarioDescriptor descriptor() const override {
    return {"non_cooperative", true, kSeedCount};
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

}  // namespace

std::vector<std::unique_ptr<ScenarioRun>> defaultSimScenarios() {
  std::vector<std::unique_ptr<ScenarioRun>> out;
  out.reserve(7);
  out.push_back(std::make_unique<CrossingScenarioRun>());
  out.push_back(std::make_unique<OvertakingScenarioRun>());
  out.push_back(std::make_unique<HeadOnScenarioRun>());
  out.push_back(std::make_unique<ParallelTargetsScenarioRun>());
  out.push_back(std::make_unique<AisDropoutScenarioRun>());
  out.push_back(std::make_unique<ClockSkewScenarioRun>());
  out.push_back(std::make_unique<NonCooperativeScenarioRun>());
  return out;
}

}  // namespace benchmark
}  // namespace navtracker
