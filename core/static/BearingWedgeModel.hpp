#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "core/output/BearingWedgeOutput.hpp"

namespace navtracker {

/**
 * One live bearing-wedge hazard (backlog #17 option 1): a DIRECTION, not a
 * position. A camera-only contact (Bearing2D, radar-silent) cannot initiate a
 * track, yet ADR 0002 forbids it becoming nothing — so we surface the direction
 * it lies along: a wedge from own-ship (the apex) spanning ±half-width about the
 * detection bearing, out to an optional max range. CPA is NOT computable for a
 * wedge (no position); the operator reading is "keep clear of that line".
 *
 * apex_enu is stored in the model's fixed ANCHOR frame (like LiveOccupancyModel),
 * so an own-ship datum recenter never moves it. The bearing is the internal
 * Bearing2D convention — math angle atan2(dNorth, dEast), CCW from east — carried
 * through verbatim from the measurement; the output converts it to true bearing.
 */
struct BearingWedge {
  std::uint64_t wedge_id{0};
  Eigen::Vector2d apex_enu{Eigen::Vector2d::Zero()};  // ANCHOR frame
  double bearing_math_rad{0.0};   // atan2(dN, dE), CCW from east
  double half_width_rad{0.0};
  std::optional<double> max_range_m;  // nullopt ⇒ unbounded (range unknown)
  double last_update_s{0.0};
  std::string source_id;
};

/** Tuning for BearingWedgeModel. */
struct BearingWedgeParams {
  // Half-width = clamp(half_width_sigma_mult × bearing σ, min_half_width_rad, ∞).
  // The σ passed to observeBearing MUST be the COMPOSED bearing uncertainty
  // (camera σ ⊕ own-ship heading σ) — the fed bearing is relative-bearing +
  // heading, so heading error is part of it (backlog #16). Calibration showed a
  // ~0.45° median but a 1.32° p90 tail, so a floor keeps a wedge from ever being
  // implausibly thin on an optimistic σ.
  double half_width_sigma_mult{2.0};
  double min_half_width_rad{0.0261799};  // ≈ 1.5°
  // Default range extent when observeBearing is not given one. nullopt ⇒
  // unbounded (range genuinely unknown — the defining case of #17).
  std::optional<double> default_max_range_m;
  // A wedge lapses (is pruned) if not refreshed within this window of the
  // current time — the camera stopped seeing the contact. Presence lapses only
  // when the evidence goes quiet (ADR 0002 / occupancy-veto shape).
  double stale_window_s{10.0};
  // Extra angular slack when deciding a confirmed track "claims" a wedge.
  double handover_margin_rad{0.0};
  // Tracks nearer than this to a wedge apex are ignored as claimers (a
  // degenerate near-apex bearing is meaningless). Metres.
  double claim_min_range_m{1.0};
};

/**
 * Standalone bearing-wedge hazard model (not on the PMBM hot path — the consumer
 * or a thin wiring layer feeds it, like AisAdapter / the occupancy layer). Same
 * conservation shape as the occupancy veto: a wedge is SUPPRESSED from output
 * (not deleted) while a confirmed track sits inside its angular claim, and
 * reappears the moment the track leaves — claim is recomputed every drain, never
 * latched. Only staleness (the camera going quiet) actually removes a wedge.
 * This avoids the ADR-0002 forbidden failure where a near vessel crossing the
 * bearing of a far, still-seen camera contact would permanently erase it.
 */
class BearingWedgeModel : public IDatumChangeSink {
 public:
  explicit BearingWedgeModel(geo::Datum anchor, BearingWedgeParams params = {})
      : anchor_(anchor), current_(anchor), params_(params) {}

  /**
   * A camera-only bearing detection (Bearing2D that cannot initiate a track).
   * Creates or refreshes the wedge for (source_id, contact_id). `own_enu` is
   * own-ship position in the tracker's CURRENT working frame; `bearing_math_rad`
   * is the measurement value verbatim (atan2(dN,dE)); `bearing_sigma_rad` is the
   * COMPOSED bearing σ (see BearingWedgeParams). `max_range` overrides the
   * param default. `suspect` (e.g. #20 sensor_track_id_suspect) forces a fresh
   * wedge_id even without a stale gap — a suspected number-reuse must not
   * resurrect the previous contact's identity.
   */
  void observeBearing(double t_s, const Eigen::Vector2d& own_enu,
                      double bearing_math_rad, double bearing_sigma_rad,
                      const std::string& source_id, std::int64_t contact_id,
                      std::optional<double> max_range = std::nullopt,
                      bool suspect = false);

  /**
   * The current confirmed-track ENU positions (tracker CURRENT frame), replacing
   * the previous set. Used to recompute claim SUPPRESSION at drain time. Call
   * once per cycle; pass empty to clear (nothing claims → all live wedges show).
   */
  void observeConfirmedTracks(const std::vector<Eigen::Vector2d>& track_enu);

  /** Remove wedges not refreshed within stale_window_s of `now_s`. */
  void pruneStale(double now_s);

  /** IDatumChangeSink: re-anchor the current transform; apexes (anchor frame)
   *  never move. */
  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    current_ = new_datum;
  }

  /** All live wedges (includes currently-claimed), deterministic order. */
  std::vector<BearingWedge> liveWedges() const;
  /** Live wedges NOT currently claimed by a confirmed track (the drain set). */
  std::vector<BearingWedge> activeWedges() const;
  /** Drainable outputs for the active wedges (geodetic apex via the anchor). */
  std::vector<BearingWedgeOutput> hazardOutputs() const;

  /** True if a confirmed track currently sits inside this wedge's claim. */
  bool isClaimed(const BearingWedge& w) const;

  /** Number of live wedges (claimed or not). */
  std::size_t size() const { return wedges_.size(); }

 private:
  using Key = std::pair<std::string, std::int64_t>;
  Eigen::Vector2d toAnchorEnu(const Eigen::Vector2d& enu_current) const;

  geo::Datum anchor_;
  geo::Datum current_;
  BearingWedgeParams params_;
  std::uint64_t next_wedge_id_{1};
  std::map<Key, BearingWedge> wedges_;              // live, keyed by contact
  std::vector<Eigen::Vector2d> confirmed_tracks_;   // ANCHOR frame
};

}  // namespace navtracker
