// Unit tests for T2tFuser (ticket §6.3 lifecycle + determinism bullets):
// birth M-of-N, confirm, coast, delete, id-never-reused, multi-source CI
// fusion, independence verdict, deterministic replay, dropout continuity,
// and regressions for the review-confirmed defects (per-scan M-of-N; batching
// of same-timestamp reports). Banded/structural assertions (#24).

#include "core/t2t/T2tFuser.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/geo/Wgs84.hpp"

namespace navtracker::t2t {
namespace {

geo::Datum testDatum() { return geo::Datum(geo::Geodetic{60.0, 10.0, 0.0}); }

ExternalTrack rep(const std::string& tk, const std::string& id, double tsec,
                  double x, double y, std::optional<SourcePedigree> ped = std::nullopt) {
  return makeExternalTrackFromEnu(tk, id, Timestamp::fromSeconds(tsec),
                                  Eigen::Vector2d(x, y),
                                  Eigen::Matrix2d::Identity() * 400.0, {},
                                  std::move(ped));  // 20 m 1-sigma
}

// Ingest one report and close its scan (run the cycle) — the per-report test
// idiom now that reports are batched by timestamp.
void feed(T2tFuser& f, ExternalTrack e) {
  f.process(std::move(e));
  f.flush();
}

SourcePedigree usedOnly(const std::string& stream) {
  SourcePedigree p;
  p.default_usage = SensorUsage::NotUsed;
  p.sensors[stream] = SensorUsage::Used;
  return p;
}

struct RecordingSink : IFusedTrackSink {
  std::vector<std::string> kinds;
  void onFusedTrackInitiated(const FusedTrackLifecycleEvent&) override { kinds.push_back("init"); }
  void onFusedTrackConfirmed(const FusedTrackLifecycleEvent&) override { kinds.push_back("confirm"); }
  void onFusedTrackUpdated(const FusedTrackLifecycleEvent&) override { kinds.push_back("update"); }
  void onFusedTrackDeleted(const FusedTrackLifecycleEvent&) override { kinds.push_back("delete"); }
};

int countKind(const RecordingSink& s, const std::string& k) {
  return static_cast<int>(std::count(s.kinds.begin(), s.kinds.end(), k));
}

TEST(T2tFuser, BirthRequiresMofNThenConfirms) {
  T2tFuser f;
  f.setDatum(testDatum());
  RecordingSink sink;
  f.setFusedTrackSink(&sink);

  feed(f, rep("A", "1", 0.0, 0, 0));
  EXPECT_EQ(f.size(), 0u);  // one reporting scan < fused_confirm_m

  feed(f, rep("A", "1", 1.0, 0, 0));
  ASSERT_EQ(f.size(), 1u);  // born on the second reporting scan
  EXPECT_EQ(f.fusedTracks()[0].track.status, TrackStatus::Tentative);
  EXPECT_EQ(countKind(sink, "init"), 1);

  feed(f, rep("A", "1", 2.0, 0, 0));  // second contribution -> confirmed
  EXPECT_EQ(f.fusedTracks()[0].track.status, TrackStatus::Confirmed);
  EXPECT_EQ(countKind(sink, "confirm"), 1);
}

TEST(T2tFuser, TrustSourceStatusConfirmsImmediately) {
  T2tFuser f;
  f.setDatum(testDatum());
  feed(f, rep("A", "1", 0.0, 0, 0));
  ExternalTrack r1 = rep("A", "1", 1.0, 0, 0);
  r1.source_status = TrackStatus::Confirmed;  // source says confirmed
  feed(f, std::move(r1));  // births AND confirms (trust_source_status)
  ASSERT_EQ(f.size(), 1u);
  EXPECT_EQ(f.fusedTracks()[0].track.status, TrackStatus::Confirmed);
}

TEST(T2tFuser, CoastsThenDeletesByAgeAndNeverReusesId) {
  T2tFuser f;
  f.setDatum(testDatum());
  feed(f, rep("A", "1", 0.0, 0, 0));
  feed(f, rep("A", "1", 1.0, 0, 0));
  ASSERT_EQ(f.size(), 1u);
  const std::uint64_t id1 = f.fusedTracks()[0].track.id.value;

  f.advanceTo(Timestamp::fromSeconds(45.0));  // well past fused_delete_age_s
  EXPECT_EQ(f.size(), 0u);

  feed(f, rep("B", "9", 100.0, 500, 500));
  feed(f, rep("B", "9", 101.0, 500, 500));
  ASSERT_EQ(f.size(), 1u);
  EXPECT_GT(f.fusedTracks()[0].track.id.value, id1);  // fresh id, never reused
}

TEST(T2tFuser, TwoIndependentSourcesFuseToOneTrack) {
  T2tFuser f;
  f.setDatum(testDatum());
  f.registerSource("A", usedOnly("radar"));
  f.registerSource("B", usedOnly("ais"));

  double t = 0.0;
  for (int i = 0; i < 8; ++i) {
    feed(f, rep("A", "1", t, 200, 200));
    feed(f, rep("B", "1", t + 0.4, 200, 200));
    t += 1.0;
  }
  const auto out = f.fusedTracks();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].contributing_trackers.size(), 2u);
  EXPECT_EQ(out[0].independence_class, IndependenceClass::ProvablyIndependent);
  EXPECT_EQ(out[0].fusion_rule, "CI");
}

TEST(T2tFuser, SingleSourceIsLegitimateAndClassifiedSingleSource) {
  T2tFuser f;
  f.setDatum(testDatum());
  for (int i = 0; i < 4; ++i) feed(f, rep("A", "1", i, 10, 10));
  const auto out = f.fusedTracks();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].independence_class, IndependenceClass::SingleSource);
  EXPECT_EQ(out[0].contributing_trackers.size(), 1u);
}

TEST(T2tFuser, SharedStreamPedigreeIsPossiblyCorrelated) {
  T2tFuser f;
  f.setDatum(testDatum());
  f.registerSource("A", usedOnly("ais:feed"));  // both used the SAME ais stream
  f.registerSource("B", usedOnly("ais:feed"));
  double t = 0.0;
  for (int i = 0; i < 8; ++i) {
    feed(f, rep("A", "1", t, 0, 0));
    feed(f, rep("B", "1", t + 0.4, 0, 0));
    t += 1.0;
  }
  const auto out = f.fusedTracks();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].independence_class, IndependenceClass::PossiblyCorrelated);
}

TEST(T2tFuser, DeterministicReplay) {
  std::vector<ExternalTrack> script;
  double t = 0.0;
  for (int i = 0; i < 8; ++i) {
    script.push_back(rep("A", "1", t, 100 + i, 50));
    script.push_back(rep("B", "1", t + 0.3, 100 + i, 50));
    t += 1.0;
  }
  auto run = [&]() {
    T2tFuser f;
    f.setDatum(testDatum());
    for (const auto& r : script) f.process(r);
    f.flush();
    return f.fusedTracks();
  };
  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].track.id.value, b[i].track.id.value);
    EXPECT_EQ(a[i].track.status, b[i].track.status);
    EXPECT_DOUBLE_EQ(a[i].track.position.lat_deg, b[i].track.position.lat_deg);
    EXPECT_DOUBLE_EQ(a[i].track.position.lon_deg, b[i].track.position.lon_deg);
  }
}

TEST(T2tFuser, ContinuityThroughSourceDropout) {
  T2tFuser f;
  f.setDatum(testDatum());
  f.registerSource("A", usedOnly("radar"));
  f.registerSource("B", usedOnly("ais"));

  double t = 0.0;
  for (int i = 0; i < 5; ++i) {  // establish a fused track from A+B
    feed(f, rep("A", "1", t, 300, 300));
    feed(f, rep("B", "1", t + 0.4, 300, 300));
    t += 1.0;
  }
  ASSERT_EQ(f.size(), 1u);
  const std::uint64_t id = f.fusedTracks()[0].track.id.value;

  // B goes silent long past max_report_age_s (10 s); A alone sustains the track.
  for (int i = 0; i < 15; ++i) {
    feed(f, rep("A", "1", t, 300 + i, 300));
    t += 1.0;
  }
  EXPECT_EQ(f.size(), 1u);
  EXPECT_EQ(f.fusedTracks()[0].track.id.value, id);  // continuity: same id

  // B resumes near the track — no spurious second fused track, same id.
  feed(f, rep("B", "1", t, 300 + 15, 300));
  EXPECT_EQ(f.size(), 1u);
  EXPECT_EQ(f.fusedTracks()[0].track.id.value, id);
}

TEST(T2tFuser, RejectsInvalidAndStaleReports) {
  T2tFuser f;
  f.setDatum(testDatum());
  ExternalTrack bad = rep("A", "1", 1.0, 0, 0);
  bad.source_tracker_id.clear();  // invalid
  EXPECT_FALSE(f.process(bad));
  EXPECT_EQ(f.rejectedCount(), 1u);

  EXPECT_TRUE(f.process(rep("A", "1", 10.0, 0, 0)));
  EXPECT_FALSE(f.process(rep("A", "1", 5.0, 0, 0)));  // stale for source A
  EXPECT_EQ(f.staleDropped(), 1u);
}

TEST(T2tFuser, NoDatumYieldsEmptyPull) {
  T2tFuser f;  // no datum set
  feed(f, rep("A", "1", 0.0, 0, 0));
  feed(f, rep("A", "1", 1.0, 0, 0));
  EXPECT_TRUE(f.fusedTracks().empty());  // no frame to convert into
  EXPECT_EQ(f.size(), 1u);              // ... the fused track exists internally
}

// REGRESSION (review defect: per-cycle vs per-source-report M-of-N). A source
// that reports ONCE must never be promoted to a fused track by another source's
// ongoing traffic. Here B reports every second far away; A reports once. A must
// never birth.
TEST(T2tFuser, SingleReportNotPromotedByAmbientTraffic) {
  T2tFuser f;
  f.setDatum(testDatum());
  feed(f, rep("A", "1", 0.0, 0, 0));            // A: a single report at the origin
  for (int i = 1; i <= 20; ++i)                  // B: sustained traffic far away
    feed(f, rep("B", "9", i, 5000, 5000));
  // Only B may have become a fused track; A's lone report must not have.
  for (const auto& o : f.fusedTracks()) {
    for (const auto& c : o.contributing_trackers)
      EXPECT_NE(c.source_tracker_id, "A") << "single A report was wrongly promoted";
  }
}

// REGRESSION (review defect: same-timestamp reports must batch into one scan).
// Two reports for the same source key at the SAME timestamp (e.g. the
// self-adapter firing Updated+Confirmed at one instant) must count as ONE
// reporting scan, not two — so they cannot satisfy a 2-of-N birth alone.
TEST(T2tFuser, SameTimestampReportsCollapseToOneScan) {
  T2tFuser f;
  f.setDatum(testDatum());
  f.process(rep("A", "1", 0.0, 0, 0));
  f.process(rep("A", "1", 0.0, 0, 0));  // same key, same time
  f.flush();
  EXPECT_EQ(f.size(), 0u);  // one scan -> birth hits=1 < fused_confirm_m
  feed(f, rep("A", "1", 1.0, 0, 0));    // a genuine second reporting scan
  EXPECT_EQ(f.size(), 1u);
}

}  // namespace
}  // namespace navtracker::t2t
