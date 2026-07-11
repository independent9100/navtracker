#pragma once

// T2tFuser — the track-to-track fusion engine. Ingests ExternalTrack reports
// from N source trackers, aligns them in time, associates them, fuses the
// associated estimates with covariance intersection, and maintains a set of
// fused tracks with stable ids and a lifecycle. See docs/algorithms/t2t-fusion.md.
//
// Design notes worth stating up front:
//  * MEMORYLESS fusion. A fused track's estimate at time t is the CI-fusion of
//    its CURRENTLY-contributing source tracks (each predicted to t). The fused
//    track does not recursively filter its own history into itself — that would
//    re-use information the sources already carry and create a double-counting
//    loop one level down (the exact hazard this module prevents; fused-level
//    feedback is explicitly deferred, algorithm doc §4). The fused track only
//    carries its own state forward when it is COASTING (no fresh source), for
//    continuity through dropout.
//  * Time-driven, never wall-clock (invariant 4). The engine advances on report
//    timestamps; replay of the same report order is deterministic.
//  * Per-instance config; no globals.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/estimation/ConstantVelocity2D.hpp"
#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "core/t2t/ExternalTrack.hpp"
#include "core/t2t/FusedTrackOutput.hpp"
#include "core/t2t/Pedigree.hpp"
#include "core/t2t/T2tAssociator.hpp"
#include "core/t2t/T2tConfig.hpp"
#include "core/types/Track.hpp"
#include "ports/IFusedTrackSink.hpp"
#include "ports/IFusionRule.hpp"

namespace navtracker::t2t {

class T2tFuser : public IDatumChangeSink {
 public:
  // `rule` is optional; null uses covariance intersection (the only shipped
  // rule). The bench injects a test-only naive rule here as a control (M3).
  // The rule pointer, if given, must outlive the fuser.
  explicit T2tFuser(T2tConfig cfg = {}, const IFusionRule* rule = nullptr);

  // Register (or replace) the pedigree for a source tracker. Optional: an
  // unregistered source, and a per-report nullopt pedigree, both resolve to
  // all-Unknown. A per-report pedigree override takes precedence.
  void registerSource(const std::string& source_tracker_id, SourcePedigree pedigree);

  // Ingest one report into the current scan. Reports are BATCHED by timestamp:
  // a report with a newer time first flushes the pending scan (running exactly
  // one fusion cycle for it), so lifecycle M-of-N advances per SCAN, not per
  // report — many reports (multiple targets, or the self-adapter firing
  // Updated+Confirmed at one instant) collapse into a single cycle. Returns
  // false if the report was rejected at the edge (invalid, or stale for its
  // source). Pessimistic covariance defaults are applied to accepted reports.
  // Call flush() (or advanceTo) before pulling to run the last buffered scan.
  bool process(ExternalTrack report);

  // Run the pending scan's fusion cycle now (no-op if nothing is buffered).
  // Pull results after this to see the current scan reflected.
  void flush();

  // Flush any pending scan, then advance to time t with no new input: predict,
  // coast, and delete by age. Never moves time backward.
  void advanceTo(Timestamp t);

  // Pull the current fused output. Requires a datum (set via setDatum or an
  // onDatumRecentered event); without one, returns an empty vector.
  std::vector<FusedTrackOutput> fusedTracks() const;

  // Raw ENU fused state — the internal estimate before geodetic conversion.
  // For evaluation/bench scoring against ENU truth (the geodetic/NED output is
  // for consumers; scoring wants the same frame as the truth). Needs no datum.
  struct FusedEnuState {
    TrackId id;
    TrackStatus status;
    Eigen::Vector2d position;      // ENU meters
    Eigen::Vector2d velocity;      // ENU m/s
    Eigen::Matrix2d position_cov;  // ENU m^2 (position block)
  };
  std::vector<FusedEnuState> fusedTracksEnu() const;

  // Push sink for fused lifecycle events. Nullable, non-owning; null = no
  // overhead. The sink, if set, must outlive the fuser.
  void setFusedTrackSink(IFusedTrackSink* sink) { sink_ = sink; }

  // Datum used for geodetic output and to define the ENU frame the fuser caches
  // state in. Set the initial datum once the OwnShipProvider has one.
  void setDatum(const geo::Datum& d) { datum_ = d; }
  bool hasDatum() const { return datum_.has_value(); }

  // IDatumChangeSink: re-express all cached ENU state into the new frame.
  void onDatumRecentered(const geo::Datum& old_d, const geo::Datum& new_d) override;

  // Diagnostics.
  std::size_t staleDropped() const { return stale_guard_.staleDropped(); }
  std::size_t rejectedCount() const { return rejected_; }
  std::size_t size() const { return fused_.size(); }
  Timestamp currentTime() const { return now_; }

 private:
  // Anti-flicker hysteresis state for one (fused track, source-track) pairing.
  struct Pairing {
    int hits = 0;
    int misses = 0;
    bool formed = false;
  };

  // One fused track plus its T2T bookkeeping. `track` holds the ENU estimate
  // (state [px,py,vx,vy], covariance) and reused metadata (id, status,
  // attributes, contributing_sources).
  struct FusedState {
    Track track;
    std::map<std::string, Pairing> pairings;   // source key -> hysteresis
    std::deque<bool> confirm_window;           // per-cycle contributed? (M-of-N)
    Timestamp last_contrib;                    // last time a source contributed
    IndependenceClass independence = IndependenceClass::SingleSource;
    bool pessimistic_default = false;
    bool confirmed_fired = false;              // onFusedTrackConfirmed fired once
    std::vector<ContributingTracker> contributors;  // current, for output
  };

  // A stored source track (latest report, ENU, defaults applied).
  struct StoredSource {
    ExternalTrack latest;
    Timestamp last_report;
  };

  // A source track predicted to the current time, reduced to what the cycle
  // needs.
  struct PredictedSource {
    std::string key;               // (source_tracker_id, source_track_id)
    std::string source_tracker_id;
    std::string source_track_id;
    Timestamp last_report;
    Eigen::Vector4d state;         // [px,py,vx,vy] ENU, predicted to now_
    Eigen::Matrix4d cov;
    bool has_velocity = false;
    std::optional<std::uint32_t> mmsi;
    std::optional<TrackStatus> source_status;
    SourcePedigree pedigree;       // effective (override else registered else all-Unknown)
    bool pessimistic_default = false;
  };

  static std::string keyOf(const ExternalTrack& t);
  SourcePedigree effectivePedigreeFor(const ExternalTrack& t) const;
  PredictedSource predictSource(const StoredSource& s, Timestamp to) const;
  void predictFusedForward(FusedState& f, Timestamp to) const;
  void flushPending();
  // One fusion cycle at time t. `reporters` = source keys that reported in THIS
  // scan (drives birth/confirm/pairing M-of-N); fusion contribution itself uses
  // every fresh source (within max_report_age), reporter or coasted.
  void runCycle(Timestamp t, const std::set<std::string>& reporters);
  void fuseInto(FusedState& f, const std::vector<const PredictedSource*>& contributors);
  GateCandidate gateOfFused(const FusedState& f) const;
  static GateCandidate gateOfSource(const PredictedSource& p);

  T2tConfig cfg_;
  const IFusionRule* rule_;                 // null -> default_rule_
  CovarianceIntersectionRule default_rule_;
  ConstantVelocity2D motion_;               // CV model with cfg_ process noise
  T2tAssociator associator_;

  std::map<std::string, StoredSource> sources_;
  std::map<std::string, SourcePedigree> registered_pedigrees_;
  std::map<std::string, std::deque<bool>> birth_window_;  // per source key: M-of-N birth
  std::vector<FusedState> fused_;
  std::uint64_t next_fused_id_ = 1;         // monotonic, never reused (invariant 5)

  PerSourceStaleGuard stale_guard_;
  std::size_t rejected_ = 0;
  Timestamp now_;
  bool has_time_ = false;
  // Current scan being buffered: the source keys that have reported since the
  // last flush, and that scan's timestamp.
  std::set<std::string> reported_keys_;
  bool has_pending_ = false;
  Timestamp pending_time_;
  std::optional<geo::Datum> datum_;
  IFusedTrackSink* sink_ = nullptr;
};

}  // namespace navtracker::t2t
