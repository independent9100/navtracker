#pragma once

// Pedigree — "which sensor streams did the source tracker use?" with honest
// ignorance as a first-class answer, plus the pairwise independence rule.
//
// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------
// Each source tracker declares, per named sensor stream `s`, one of three
// usages: Used, NotUsed, Unknown. Streams not listed take a `default_usage`.
// The resolved usage of stream `s` for pedigree A is:
//
//     usage_A(s) = A.sensors[s]        if s is listed in A
//                = A.default_usage      otherwise
//
// Two source trackers A and B could share stream `s` — and therefore have
// correlated errors through it — unless at least one of them definitely did
// NOT use it. Formally, overlap on stream `s` is POSSIBLE iff
//
//     usage_A(s) != NotUsed  AND  usage_B(s) != NotUsed
//
// (Used/Used = certain overlap; Used/Unknown or Unknown/Unknown = possible
// overlap; anything paired with NotUsed = no overlap on `s`.)
//
// A and B are ProvablyIndependent iff overlap is impossible for EVERY stream —
// every stream id appearing in either pedigree, AND the infinite tail of
// unlisted ids governed by the two defaults. Otherwise PossiblyCorrelated.
//
// ---------------------------------------------------------------------------
// Assumptions
// ---------------------------------------------------------------------------
//  * Stream identifiers are exact-string matched. The SAME physical stream
//    MUST be given the SAME id by every source (agreed at integration time),
//    e.g. "ais:region_feed", "radar:own_xband". A typo makes two views of one
//    stream look independent — the classification is only as honest as the ids.
//  * `Unknown` is the honest default. It never makes fusion unsafe (the fusion
//    math is covariance intersection regardless of this verdict, see
//    docs/algorithms/t2t-fusion.md §1); it only forfeits the future option of a
//    tighter, independence-exploiting rule.
//
// ---------------------------------------------------------------------------
// Rationale
// ---------------------------------------------------------------------------
// Three-valued (not boolean) usage is a hard requirement: a real deployment
// often does not know what an upstream tracker fused. Collapsing Unknown into
// either Used or NotUsed would be a lie — Unknown->NotUsed would silently
// permit overconfident fusion, Unknown->Used would forbid every optimization.
// Keeping ignorance explicit lets the classification be conservative (treat
// Unknown as possible overlap) without ever pretending to knowledge we lack.
//
// ---------------------------------------------------------------------------
// Ways to improve / what to test next
// ---------------------------------------------------------------------------
//  * Partial-overlap information: today any shared/unknown stream taints the
//    whole pair as PossiblyCorrelated. A future rule could decorrelate per
//    stream (information-form) and fuse the independent part tightly — this is
//    what the `IFusionRule` port (ports/IFusionRule.hpp) is a hook for.
//  * Weighted / probabilistic usage instead of the three-valued enum.

#include <map>
#include <string>
#include <vector>

namespace navtracker::t2t {

// Whether a source tracker used a given sensor stream. `Unknown` is the honest
// default everywhere the answer is not known.
enum class SensorUsage { Used, NotUsed, Unknown };

// The pedigree verdict for what is currently fused into one fused track.
//  * SingleSource       — only one source contributes; nothing to correlate.
//  * ProvablyIndependent — every contributing pair provably shares no stream.
//  * PossiblyCorrelated  — at least one contributing pair could share a stream.
enum class IndependenceClass { SingleSource, ProvablyIndependent, PossiblyCorrelated };

// One source tracker's declaration of which sensor streams it consumed.
//
// A default-constructed SourcePedigree{} (empty map, default_usage = Unknown)
// IS the "all-Unknown" pedigree. An ABSENT pedigree (a source that declared
// nothing) must be treated identically to this — see independenceOfPair().
struct SourcePedigree {
  // Per-stream usage. Stream ids are free-form strings agreed at integration
  // time; matching is EXACT-STRING.
  std::map<std::string, SensorUsage> sensors;
  // Usage of any stream not present in `sensors`.
  SensorUsage default_usage = SensorUsage::Unknown;

  // Resolved usage of stream `s`: the listed value, else the default.
  SensorUsage usageOf(const std::string& s) const {
    const auto it = sensors.find(s);
    return it == sensors.end() ? default_usage : it->second;
  }
};

// True iff A and B could share stream with these two resolved usages — i.e.
// neither side definitely refrained from using it.
inline bool overlapPossible(SensorUsage a, SensorUsage b) {
  return a != SensorUsage::NotUsed && b != SensorUsage::NotUsed;
}

// Pairwise independence of two source trackers. Returns ProvablyIndependent
// iff no stream (listed or unlisted) could be shared; otherwise
// PossiblyCorrelated. Never returns SingleSource (a pair is two sources).
//
// Pure function, exhaustively unit-tested (tests/t2t/test_pedigree.cpp). This
// classification drives DIAGNOSTICS and attribute handling in v1, never the
// fusion math (covariance intersection is used regardless).
inline IndependenceClass independenceOfPair(const SourcePedigree& a,
                                            const SourcePedigree& b) {
  // Every stream id appearing in either pedigree, resolved on both sides.
  for (const auto& [id, _] : a.sensors) {
    if (overlapPossible(a.usageOf(id), b.usageOf(id)))
      return IndependenceClass::PossiblyCorrelated;
  }
  for (const auto& [id, _] : b.sensors) {
    // ids also in `a` were covered above, but re-checking is harmless and
    // keeps the logic obviously exhaustive.
    if (overlapPossible(a.usageOf(id), b.usageOf(id)))
      return IndependenceClass::PossiblyCorrelated;
  }
  // The infinite tail of unlisted ids: both fall back to their defaults.
  if (overlapPossible(a.default_usage, b.default_usage))
    return IndependenceClass::PossiblyCorrelated;
  return IndependenceClass::ProvablyIndependent;
}

// The fused-track verdict over N contributing sources' pedigrees.
//  * 0 or 1 source  -> SingleSource.
//  * >=2 sources    -> PossiblyCorrelated if ANY pair is possibly correlated,
//                      else ProvablyIndependent (worst-case combine).
inline IndependenceClass fusedIndependence(
    const std::vector<const SourcePedigree*>& sources) {
  if (sources.size() <= 1) return IndependenceClass::SingleSource;
  for (std::size_t i = 0; i < sources.size(); ++i) {
    for (std::size_t j = i + 1; j < sources.size(); ++j) {
      if (independenceOfPair(*sources[i], *sources[j]) ==
          IndependenceClass::PossiblyCorrelated)
        return IndependenceClass::PossiblyCorrelated;
    }
  }
  return IndependenceClass::ProvablyIndependent;
}

}  // namespace navtracker::t2t
