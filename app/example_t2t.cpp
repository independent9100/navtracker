// example_t2t.cpp — end-to-end wiring example for navtracker_t2t.
//
// "You already have tracks from one or more OTHER trackers (their input sensors
// often unknown) and you want ONE authoritative fused picture." This is the
// consumer surface for that: feed each external tracker's tracks in as
// ExternalTracks (with an optional pedigree), and drain fused tracks out.
//
// Default fusion rule is covariance intersection (CI): consistent for ANY
// unknown cross-correlation, so two trackers that secretly share a sensor (e.g.
// both fused the same AIS) do NOT produce an overconfident fused covariance.
// Nothing here needs to know what sensors the sources used — pedigree is a
// diagnostic hint (with a first-class Unknown), never the fusion key.
//
// Build:  cmake --build build --target navtracker_t2t_example
// Run:    ./build/navtracker_t2t_example

#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/t2t/ExternalTrack.hpp"
#include "core/t2t/FusedTrackOutput.hpp"
#include "core/t2t/Pedigree.hpp"
#include "core/t2t/T2tFuser.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;
using namespace navtracker::t2t;

namespace {

// A pedigree that declares exactly which sensor streams a source used (and that
// it used nothing else). Two sources that share NO Used stream are provably
// independent; any possible overlap (or any Unknown) is treated as possibly
// correlated and CI keeps the fusion honest.
SourcePedigree used(const std::string& stream) {
  SourcePedigree p;
  p.default_usage = SensorUsage::NotUsed;  // "I used only what I list"
  p.sensors[stream] = SensorUsage::Used;
  return p;
}

}  // namespace

int main() {
  // 1. Composition. Default rule = covariance intersection. The datum lets the
  //    fuser hand you lat/lon out; the fusion math itself is in the local ENU
  //    tangent plane.
  const geo::Datum datum(geo::Geodetic{59.91, 10.75, 0.0});  // Oslo-ish
  T2tFuser fuser;  // == T2tFuser({}, /*CI*/ nullptr)
  fuser.setDatum(datum);

  // 2. Declare what each source tracker used. "nav_radar" fused only radar;
  //    "coastal_ais" fused only AIS -> disjoint -> ProvablyIndependent. (Had
  //    they shared a stream, or had we passed no pedigree at all, the verdict
  //    would be PossiblyCorrelated and CI would still be correct — that is the
  //    whole point: you are safe by default even when you don't know.)
  fuser.registerSource("nav_radar", used("radar"));
  fuser.registerSource("coastal_ais", used("ais"));

  // 3. Feed each tracker's tracks in, over a few scans. Both are watching the
  //    same vessel ~1.2 km NE of the datum, closing slowly. A scan closes when
  //    a strictly-newer timestamp arrives (or on flush()).
  const Eigen::Matrix2d radar_cov = Eigen::Matrix2d::Identity() * (20.0 * 20.0);  // 20 m
  const Eigen::Matrix2d ais_cov = Eigen::Matrix2d::Identity() * (12.0 * 12.0);    // 12 m
  for (int k = 0; k < 4; ++k) {
    const double t = static_cast<double>(k);
    const Eigen::Vector2d p(800.0 + 3.0 * t, 900.0 + 2.0 * t);  // ENU metres
    fuser.process(makeExternalTrackFromEnu("nav_radar", "17", Timestamp::fromSeconds(t),
                                           p + Eigen::Vector2d(4.0, -3.0), radar_cov));
    fuser.process(makeExternalTrackFromEnu("coastal_ais", "MMSI-257000000",
                                           Timestamp::fromSeconds(t),
                                           p + Eigen::Vector2d(-2.0, 5.0), ais_cov));
    fuser.flush();  // close this scan's fusion cycle
  }

  // 4. Drain the fused picture. toTrackOutput-style geodetic output + the T2T
  //    provenance (which trackers contributed, the independence verdict, the
  //    fusion rule, and whether the covariance is a pessimistic default).
  std::printf("Fused tracks: %zu\n", fuser.fusedTracks().size());
  for (const auto& fo : fuser.fusedTracks()) {
    const char* cls = fo.independence_class == IndependenceClass::ProvablyIndependent
                          ? "ProvablyIndependent"
                          : fo.independence_class == IndependenceClass::PossiblyCorrelated
                                ? "PossiblyCorrelated"
                                : "SingleSource";
    std::printf(
        "  fused id=%llu status=%d  lat=%.6f lon=%.6f  sigma_pos=(%.1f,%.1f) m  "
        "rule=%s  independence=%s  contributors=%zu\n",
        static_cast<unsigned long long>(fo.track.id.value),
        static_cast<int>(fo.track.status), fo.track.position.lat_deg,
        fo.track.position.lon_deg,
        fo.track.position.position_covariance_m2(0, 0) > 0
            ? std::sqrt(fo.track.position.position_covariance_m2(0, 0))
            : 0.0,
        fo.track.position.position_covariance_m2(1, 1) > 0
            ? std::sqrt(fo.track.position.position_covariance_m2(1, 1))
            : 0.0,
        fo.fusion_rule.c_str(), cls, fo.contributing_trackers.size());
  }
  return 0;
}
