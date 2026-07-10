#pragma once

// FusedTrackOutput — the pull-side output of the T2T fuser. It EMBEDS the
// existing TrackOutput (so consumers get the identical lat/lon + NED covariance
// + SOG/COG contract they already know) and adds the fused-layer metadata:
// which source tracks currently feed it, the pedigree independence verdict for
// what is fused, the fusion rule used, and a pessimistic-default diagnostic.
//
// The stable fused id lives in `track.id` and is minted by the fuser, never
// reused (architecture invariant 5). Pull via T2tFuser::fusedTracks();
// lifecycle push goes through IFusedTrackSink (ports/IFusedTrackSink.hpp).

#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/output/TrackOutput.hpp"  // TrackOutput, toTrackOutput
#include "core/t2t/Pedigree.hpp"        // IndependenceClass
#include "core/types/Timestamp.hpp"
#include "core/types/Track.hpp"

namespace navtracker::t2t {

// One source track currently associated into a fused track.
struct ContributingTracker {
  std::string source_tracker_id;
  std::string source_track_id;
  Timestamp last_seen;  // report time of that source's most recent contribution
};

struct FusedTrackOutput {
  // The canonical single-track output (id = fused id, geodetic position + NED
  // covariance, SOG/COG, status, last_update, attributes, contributing_sources,
  // covariance_is_default). Reused verbatim from core/output.
  TrackOutput track;

  // Source tracks feeding this fused track right now.
  std::vector<ContributingTracker> contributing_trackers;

  // Pedigree verdict for what is currently fused (SingleSource /
  // ProvablyIndependent / PossiblyCorrelated). Diagnostic in v1.
  IndependenceClass independence_class{IndependenceClass::SingleSource};

  // The fusion rule that produced `track`. Constant "CI" in v1 (future-
  // proofing: a later independence-exploiting rule would report its own name).
  std::string fusion_rule{"CI"};

  // True when any contributing input's position covariance came from the
  // pessimistic external default rather than a stated covariance (same
  // diagnostic spirit as TrackOutput::covariance_is_default).
  bool covariance_is_pessimistic_default{false};
};

// Drain a fused track (held internally by the fuser as a navtracker::Track in
// the datum-ENU frame) into a FusedTrackOutput. Mirrors toTrackOutput and
// reuses it for the base geodetic/velocity conversion, then attaches the
// fused-layer metadata.
FusedTrackOutput toFusedTrackOutput(
    const Track& fused, const geo::Datum& datum,
    std::vector<ContributingTracker> contributing_trackers,
    IndependenceClass independence_class, bool covariance_is_pessimistic_default);

}  // namespace navtracker::t2t
