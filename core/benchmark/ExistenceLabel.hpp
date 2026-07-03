#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace navtracker {
namespace benchmark {

// Video-derived EXISTENCE/REGION labels for a replay clip (R8, 2026-07-03).
//
// These are NOT kinematic truth. A frame/radar cross-reference gives, per
// region, "a vessel/structure of this kind was present here, in this time
// window" — existence + rough position + a time window, never per-second
// positions. Converting them into TruthSamples would be circular (positions
// derived from the same radar the tracker consumes) and would corrupt GOSPA's
// localisation term, so they are a separate category consumed only by the
// label-aware philos metric decomposition and the binary canary / stop→go
// gates. See docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md
// (ticket R8) and docs/algorithms/comparison-baselines.md.
enum class ExistenceLabelClass {
  KeepVessel,         // a real vessel — a valid tracker MUST hold a track here
  SuppressStructure,  // fixed structure — phantom mass a suppressor should remove
  KeepAnchorage,      // moored craft inside a charted anchorage — KEEP
  Unknown,            // chart/video silent — defaults to KEEP, gated by nothing
};

struct ExistenceLabel {
  std::string region_id;
  std::string source_rank;  // e.g. "rank11" or "39/45/82" — free text, kept as-is
  double lat_deg{0.0};
  double lon_deg{0.0};
  double radius_m{0.0};
  // Time window, RELATIVE to the clip's first ownship timestamp (seconds).
  // covers_whole_clip == true when both bounds are blank in the CSV.
  double t_start_s{0.0};
  double t_end_s{0.0};
  bool covers_whole_clip{false};
  ExistenceLabelClass label{ExistenceLabelClass::Unknown};
  std::string evidence;
  std::string confidence;
  std::string notes;

  // Does absolute unix time `t` (seconds) fall in this label's window, given
  // the clip start? Whole-clip labels always return true.
  bool activeAtUnix(double t_unix, double clip_start_unix) const {
    if (covers_whole_clip) return true;
    const double rel = t_unix - clip_start_unix;
    return rel >= t_start_s && rel <= t_end_s;
  }
};

// Parse the label CSV from `is`. Schema (header row required, order fixed):
//   region_id,source_rank,lat,lon,radius_m,t_start_s,t_end_s,label,evidence,confidence,notes
// Blank t_start_s / t_end_s ⇒ covers_whole_clip. `label` is one of
// KEEP_VESSEL / SUPPRESS_STRUCTURE / KEEP_ANCHORAGE / UNKNOWN (unrecognised →
// Unknown). Lines starting with '#' and blank lines are skipped. The final
// `notes` field absorbs any embedded commas (it is the last column).
std::vector<ExistenceLabel> parseExistenceLabels(std::istream& is);

}  // namespace benchmark
}  // namespace navtracker
